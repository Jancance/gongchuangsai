"""不依赖 ROS 和物料特征的 300 mm 转盘外圆检测器。

检测分为两个阶段：

1. 未锁定圆盘时，在缩小图上使用 Hough 大圆检测快速得到粗略种子；
2. 在完整分辨率图像上，沿种子圆的径向批量采样边缘，并用鲁棒
   最小二乘圆拟合恢复亚像素圆心和半径。

锁定以后只执行第二阶段。若局部精修失败，则在同一帧立即回到第一
阶段重新定位，因此转盘在画面中发生大幅位移后也不需要重启程序。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
from statistics import median
import tempfile
import time

import cv2
import numpy as np


class TurntableDetectionError(ValueError):
    """检测配置或输入图像不满足转盘识别要求。"""


@dataclass(frozen=True)
class TurntableDetectorConfig:
    """粗定位、圆弧精修和稳定判定的全部参数。"""

    image_scale: float
    minimum_radius_px: float
    maximum_radius_px: float
    hough_dp: float
    minimum_center_distance_px: float
    canny_threshold: float
    accumulator_threshold: float
    maximum_candidates: int
    radial_search_half_width_px: float
    angular_step_deg: float
    minimum_visible_arc_deg: float
    maximum_fit_rms_px: float
    minimum_edge_strength: float
    saturated_pixel_threshold: int
    local_recheck_confidence_below: float
    local_recheck_rms_above_px: float
    local_recheck_interval_frames: int
    reacquisition_jump_px: float
    window_frames: int
    maximum_center_jitter_px: float
    maximum_radius_jitter_px: float


@dataclass(frozen=True)
class CircleFit:
    """一帧图像中尚未经过时间平滑的外圆拟合结果。"""

    detected: bool
    center_x_px: float = 0.0
    center_y_px: float = 0.0
    radius_px: float = 0.0
    visible_arc_deg: float = 0.0
    fit_rms_px: float = math.inf
    confidence: float = 0.0
    detail: str = ""


@dataclass(frozen=True)
class DebugCircleCandidate:
    """一个 Hough 粗圆及其精修结果，只供调参界面显示。"""

    seed: tuple[float, float, float]
    fit: CircleFit
    sample_points: np.ndarray


@dataclass(frozen=True)
class TurntableDebugResult:
    """单帧只读诊断结果，不会修改检测器种子或稳定窗口。"""

    candidates: tuple[DebugCircleCandidate, ...]
    selected_index: int | None
    processing_ms: float
    detail: str


@dataclass(frozen=True)
class TurntableDetection:
    """供主程序显示以及未来串口输出使用的结构化检测结果。"""

    detected: bool
    stable: bool
    center_x_px: float
    center_y_px: float
    radius_px: float
    visible_arc_deg: float
    fit_rms_px: float
    confidence: float
    processing_ms: float
    tracking_mode: str
    detail: str
    material_centers_px: tuple[tuple[float, float], ...] = ()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise TurntableDetectionError(message)


def build_default_turntable_detector_document() -> dict:
    """返回项目推荐的转盘检测参数字典。"""

    return {
        "format_version": 1,
        "coarse_search": {
            "image_scale": 0.45,
            "minimum_radius_px": 250.0,
            "maximum_radius_px": 450.0,
            "hough_dp": 1.2,
            "minimum_center_distance_px": 160.0,
            "canny_threshold": 40.0,
            "accumulator_threshold": 30.0,
            "maximum_candidates": 5,
        },
        "refinement": {
            "radial_search_half_width_px": 55.0,
            "angular_step_deg": 2.0,
            "minimum_visible_arc_deg": 80.0,
            "maximum_fit_rms_px": 3.0,
            "minimum_edge_strength": 2.0,
            "saturated_pixel_threshold": 90,
            "local_recheck_confidence_below": 0.72,
            "local_recheck_rms_above_px": 2.2,
            "local_recheck_interval_frames": 5,
            "reacquisition_jump_px": 30.0,
        },
        "stability": {
            "window_frames": 5,
            "maximum_center_jitter_px": 8.0,
            "maximum_radius_jitter_px": 5.0,
        },
    }


def turntable_config_from_document(document: dict) -> TurntableDetectorConfig:
    """把内存 JSON 字典转换为严格校验后的不可变配置。"""

    try:
        _require(isinstance(document, dict), "转盘检测配置根节点必须是对象")
        _require(
            int(document["format_version"]) == 1,
            "不支持的转盘检测配置版本",
        )
        coarse = document["coarse_search"]
        refinement = document["refinement"]
        stability = document["stability"]
        config = TurntableDetectorConfig(
            image_scale=float(coarse["image_scale"]),
            minimum_radius_px=float(coarse["minimum_radius_px"]),
            maximum_radius_px=float(coarse["maximum_radius_px"]),
            hough_dp=float(coarse["hough_dp"]),
            minimum_center_distance_px=float(
                coarse["minimum_center_distance_px"]
            ),
            canny_threshold=float(coarse["canny_threshold"]),
            accumulator_threshold=float(
                coarse["accumulator_threshold"]
            ),
            maximum_candidates=int(coarse["maximum_candidates"]),
            radial_search_half_width_px=float(
                refinement["radial_search_half_width_px"]
            ),
            angular_step_deg=float(refinement["angular_step_deg"]),
            minimum_visible_arc_deg=float(
                refinement["minimum_visible_arc_deg"]
            ),
            maximum_fit_rms_px=float(
                refinement["maximum_fit_rms_px"]
            ),
            minimum_edge_strength=float(
                refinement["minimum_edge_strength"]
            ),
            saturated_pixel_threshold=int(
                refinement["saturated_pixel_threshold"]
            ),
            local_recheck_confidence_below=float(
                refinement["local_recheck_confidence_below"]
            ),
            local_recheck_rms_above_px=float(
                refinement["local_recheck_rms_above_px"]
            ),
            local_recheck_interval_frames=int(
                refinement.get("local_recheck_interval_frames", 5)
            ),
            reacquisition_jump_px=float(
                refinement["reacquisition_jump_px"]
            ),
            window_frames=int(stability["window_frames"]),
            maximum_center_jitter_px=float(
                stability["maximum_center_jitter_px"]
            ),
            maximum_radius_jitter_px=float(
                stability["maximum_radius_jitter_px"]
            ),
        )
    except (KeyError, TypeError, ValueError) as error:
        if isinstance(error, TurntableDetectionError):
            raise
        raise TurntableDetectionError("转盘检测配置字段无效") from error

    _require(0.20 <= config.image_scale <= 1.0, "缩放比例必须位于0.20~1.0")
    _require(
        0.0 < config.minimum_radius_px < config.maximum_radius_px,
        "转盘半径范围无效",
    )
    _require(config.hough_dp >= 1.0, "Hough dp必须不小于1")
    _require(
        config.minimum_center_distance_px > 0.0,
        "Hough圆心间距必须大于0",
    )
    _require(
        config.canny_threshold > 0.0
        and config.accumulator_threshold > 0.0,
        "Hough阈值必须大于0",
    )
    _require(
        1 <= config.maximum_candidates <= 20,
        "Hough候选数量必须位于1~20",
    )
    _require(
        config.radial_search_half_width_px > 0.0,
        "径向精修范围必须大于0",
    )
    _require(
        0.5 <= config.angular_step_deg <= 10.0,
        "圆弧采样角度步长必须位于0.5~10度",
    )
    _require(
        0.0 < config.minimum_visible_arc_deg <= 360.0,
        "最小可见圆弧角度无效",
    )
    _require(config.maximum_fit_rms_px > 0.0, "拟合RMS门槛必须大于0")
    _require(config.minimum_edge_strength > 0.0, "边缘强度门槛必须大于0")
    _require(
        0 <= config.saturated_pixel_threshold <= 255,
        "饱和度门槛必须位于0~255",
    )
    _require(
        0.0 <= config.local_recheck_confidence_below <= 1.0,
        "局部复核置信度必须位于0~1",
    )
    _require(
        0.0 < config.local_recheck_rms_above_px
        <= config.maximum_fit_rms_px,
        "局部复核RMS门槛必须位于0到最大RMS之间",
    )
    _require(
        1 <= config.local_recheck_interval_frames <= 30,
        "局部复核间隔必须位于1~30帧",
    )
    _require(
        config.reacquisition_jump_px > 0.0,
        "重新定位跳变门槛必须大于0",
    )
    _require(2 <= config.window_frames <= 30, "稳定窗口必须位于2~30帧")
    _require(
        config.maximum_center_jitter_px > 0.0
        and config.maximum_radius_jitter_px > 0.0,
        "稳定抖动门槛必须大于0",
    )
    return config


def turntable_config_to_document(config: TurntableDetectorConfig) -> dict:
    """把配置数据类转换为可直接写入 JSON 的标准字典。"""

    checked = turntable_config_from_document({
        "format_version": 1,
        "coarse_search": {
            "image_scale": config.image_scale,
            "minimum_radius_px": config.minimum_radius_px,
            "maximum_radius_px": config.maximum_radius_px,
            "hough_dp": config.hough_dp,
            "minimum_center_distance_px": config.minimum_center_distance_px,
            "canny_threshold": config.canny_threshold,
            "accumulator_threshold": config.accumulator_threshold,
            "maximum_candidates": config.maximum_candidates,
        },
        "refinement": {
            "radial_search_half_width_px": config.radial_search_half_width_px,
            "angular_step_deg": config.angular_step_deg,
            "minimum_visible_arc_deg": config.minimum_visible_arc_deg,
            "maximum_fit_rms_px": config.maximum_fit_rms_px,
            "minimum_edge_strength": config.minimum_edge_strength,
            "saturated_pixel_threshold": config.saturated_pixel_threshold,
            "local_recheck_confidence_below": (
                config.local_recheck_confidence_below
            ),
            "local_recheck_rms_above_px": config.local_recheck_rms_above_px,
            "local_recheck_interval_frames": (
                config.local_recheck_interval_frames
            ),
            "reacquisition_jump_px": config.reacquisition_jump_px,
        },
        "stability": {
            "window_frames": config.window_frames,
            "maximum_center_jitter_px": config.maximum_center_jitter_px,
            "maximum_radius_jitter_px": config.maximum_radius_jitter_px,
        },
    })
    # 从 checked 重建可保证整数类型字段不会因外部数据类构造而写成浮点。
    return {
        "format_version": 1,
        "coarse_search": {
            "image_scale": checked.image_scale,
            "minimum_radius_px": checked.minimum_radius_px,
            "maximum_radius_px": checked.maximum_radius_px,
            "hough_dp": checked.hough_dp,
            "minimum_center_distance_px": checked.minimum_center_distance_px,
            "canny_threshold": checked.canny_threshold,
            "accumulator_threshold": checked.accumulator_threshold,
            "maximum_candidates": checked.maximum_candidates,
        },
        "refinement": {
            "radial_search_half_width_px": checked.radial_search_half_width_px,
            "angular_step_deg": checked.angular_step_deg,
            "minimum_visible_arc_deg": checked.minimum_visible_arc_deg,
            "maximum_fit_rms_px": checked.maximum_fit_rms_px,
            "minimum_edge_strength": checked.minimum_edge_strength,
            "saturated_pixel_threshold": checked.saturated_pixel_threshold,
            "local_recheck_confidence_below": (
                checked.local_recheck_confidence_below
            ),
            "local_recheck_rms_above_px": checked.local_recheck_rms_above_px,
            "local_recheck_interval_frames": (
                checked.local_recheck_interval_frames
            ),
            "reacquisition_jump_px": checked.reacquisition_jump_px,
        },
        "stability": {
            "window_frames": checked.window_frames,
            "maximum_center_jitter_px": checked.maximum_center_jitter_px,
            "maximum_radius_jitter_px": checked.maximum_radius_jitter_px,
        },
    }


def load_turntable_detector_document(path: Path | str) -> dict:
    """读取 JSON 并返回经过规范化的配置字典。"""

    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except OSError as error:
        raise TurntableDetectionError(
            f"无法读取转盘检测配置：{resolved}: {error}"
        ) from error
    except json.JSONDecodeError as error:
        raise TurntableDetectionError(
            f"转盘检测配置不是有效JSON：{resolved}"
        ) from error
    try:
        return turntable_config_to_document(
            turntable_config_from_document(document)
        )
    except TurntableDetectionError as error:
        raise TurntableDetectionError(
            f"转盘检测配置字段无效：{resolved}: {error}"
        ) from error


def load_turntable_detector_config(path: Path | str) -> TurntableDetectorConfig:
    """从 JSON 文件加载并严格校验转盘检测参数。"""

    return turntable_config_from_document(
        load_turntable_detector_document(path)
    )


def save_turntable_detector_config(
    path: Path | str, config: TurntableDetectorConfig | dict
) -> None:
    """原子保存转盘配置，避免中断时破坏主程序正在使用的 JSON。"""

    if isinstance(config, TurntableDetectorConfig):
        document = turntable_config_to_document(config)
    else:
        document = turntable_config_to_document(
            turntable_config_from_document(config)
        )
    resolved = Path(path).expanduser().resolve()
    if not resolved.parent.is_dir():
        raise TurntableDetectionError(f"配置目录不存在：{resolved.parent}")
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=resolved.parent,
            prefix=f".{resolved.name}.", suffix=".tmp", delete=False,
        ) as temporary:
            temporary_name = temporary.name
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, resolved)
    except OSError as error:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink(missing_ok=True)
            except OSError:
                pass
        raise TurntableDetectionError(
            f"无法保存转盘检测配置：{resolved}: {error}"
        ) from error


def _fit_circle(points: np.ndarray, weights=None) -> np.ndarray:
    """使用线性最小二乘拟合圆心和半径。"""

    points = np.asarray(points, dtype=np.float64)
    if points.shape[0] < 3:
        raise TurntableDetectionError("圆拟合点不足")
    design = np.column_stack(
        (2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points)))
    )
    target = np.sum(points * points, axis=1)
    if weights is not None:
        scale = np.sqrt(np.asarray(weights, dtype=np.float64))
        design = design * scale[:, None]
        target = target * scale
    center_x, center_y, constant = np.linalg.lstsq(
        design, target, rcond=None
    )[0]
    radius_squared = constant + center_x**2 + center_y**2
    if radius_squared <= 0.0:
        raise TurntableDetectionError("圆拟合半径无效")
    return np.asarray(
        (center_x, center_y, math.sqrt(radius_squared)),
        dtype=np.float64,
    )


def _longest_contiguous_arc_deg(angles: np.ndarray) -> float:
    """返回圆周上最长的连续观测弧，而不是所有零散弧长之和。

    角度按5度分档，并填补夹在两个有效档之间的单个空档。这样真实外圆
    因反光或轻微遮挡漏掉不足5度时仍可连续，同时不会把分散在桌面、文字
    和设备轮廓上的边缘累计成一条很长的假圆弧。
    """

    bins = np.zeros(72, dtype=bool)
    indices = np.mod(
        (np.mod(angles, 2.0 * math.pi) / (2.0 * math.pi) * 72.0).astype(int),
        72,
    )
    bins[indices] = True
    if not np.any(bins):
        return 0.0

    # 只填补左右两侧都有观测的单个5度空档，不跨越成片缺失区域。
    bins = bins | (np.roll(bins, 1) & np.roll(bins, -1))
    longest = current = 0
    # 扫描两圈以正确处理跨越0度的连续圆弧，并把结果限制为一整圈。
    for occupied in np.concatenate((bins, bins)):
        current = current + 1 if occupied else 0
        longest = max(longest, current)
    return float(min(longest, len(bins)) * 5.0)


def _robust_circle(points, strengths, config):
    """反复剔除离群边缘点，得到稳定的圆模型、圆弧覆盖和 RMS。"""

    if len(points) < 20:
        raise TurntableDetectionError("有效圆弧点不足")
    keep = np.ones(len(points), dtype=bool)
    model = _fit_circle(points, strengths)
    for _ in range(6):
        distances = np.linalg.norm(points - model[:2], axis=1)
        residuals = np.abs(distances - model[2])
        active = residuals[keep]
        center = float(np.median(active))
        mad = float(np.median(np.abs(active - center))) + 1e-6
        threshold = max(1.5, min(5.0, center + 3.0 * 1.4826 * mad))
        updated = residuals <= threshold
        if int(np.count_nonzero(updated)) < 20:
            break
        keep = updated
        model = _fit_circle(points[keep], strengths[keep])

    inliers = points[keep]
    inlier_strengths = strengths[keep]
    residuals = np.linalg.norm(inliers - model[:2], axis=1) - model[2]
    rms = float(math.sqrt(float(np.mean(residuals**2))))

    # 使用最长连续圆弧，而不是把圆周各处互不相连的边缘累计起来。
    # 否则桌面接缝、文字和设备外框可能共同拼出一个不存在的大圆。
    angles = np.mod(
        np.arctan2(inliers[:, 1] - model[1], inliers[:, 0] - model[0]),
        2.0 * math.pi,
    )
    visible_arc_deg = _longest_contiguous_arc_deg(angles)
    edge_strength = float(np.median(inlier_strengths))
    return model, visible_arc_deg, rms, edge_strength


class TurntableDetector:
    """维护粗定位种子和多帧平滑状态的实时转盘检测器。"""

    def __init__(self, config: TurntableDetectorConfig, material_seeder=None):
        self.config = config
        # 三物料粗定位器是可选依赖。调参工具不传入时仍保持原先纯外圆
        # 行为；main.py 会加载它，在能看到三件物料时优先快速定位。
        self.material_seeder = material_seeder
        self.samples = deque(maxlen=config.window_frames)
        self.seed = None
        self.last_material_seed_result = None
        self._material_seed_retry_countdown = 0
        self._last_reacquire_tracking_mode = "reacquired"
        # 初始设为可立即复核，便于外部测试或恢复种子时保持安全行为。
        self._frames_since_global_recheck = (
            config.local_recheck_interval_frames
        )

    def reset(self) -> None:
        """清除局部跟踪种子以及上一模式留下的稳定帧。"""

        self.seed = None
        self.samples.clear()
        self.last_material_seed_result = None
        self._material_seed_retry_countdown = 0
        self._last_reacquire_tracking_mode = "reacquired"
        if self.material_seeder is not None:
            self.material_seeder.reset()
        self._frames_since_global_recheck = (
            self.config.local_recheck_interval_frames
        )

    def _coarse_candidates(self, frame) -> tuple:
        """在缩小图上查找半径范围内的大圆，并恢复到原图坐标。"""

        scale = self.config.image_scale
        small = cv2.resize(
            frame,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_AREA,
        )
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 1.5)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=self.config.hough_dp,
            minDist=self.config.minimum_center_distance_px * scale,
            param1=self.config.canny_threshold,
            param2=self.config.accumulator_threshold,
            minRadius=int(math.floor(self.config.minimum_radius_px * scale)),
            maxRadius=int(math.ceil(self.config.maximum_radius_px * scale)),
        )
        if circles is None:
            return ()

        height, width = frame.shape[:2]
        candidates = []
        for center_x, center_y, radius in circles[0]:
            candidate = (
                float(center_x / scale),
                float(center_y / scale),
                float(radius / scale),
            )
            # 当前方案要求圆心仍在画面内部。圆周可以被画面边缘裁掉，
            # 后续圆弧覆盖门槛会判断剩余边缘是否足够精修。
            if not (
                0.0 <= candidate[0] < width
                and 0.0 <= candidate[1] < height
            ):
                continue
            candidates.append(candidate)
            if len(candidates) >= self.config.maximum_candidates:
                break
        return tuple(candidates)

    def _sample_boundary_points(self, frame, seed, config=None):
        """沿180条径向射线批量提取最可能属于外圆的边缘点。"""

        config = self.config if config is None else config
        height, width = frame.shape[:2]
        angles = np.deg2rad(
            np.arange(0.0, 360.0, config.angular_step_deg)
        )
        radial = np.arange(
            seed[2] - config.radial_search_half_width_px,
            seed[2] + config.radial_search_half_width_px + 1.0,
            1.0,
        )
        cosines = np.cos(angles)[:, None]
        sines = np.sin(angles)[:, None]
        map_x = (seed[0] + cosines * radial[None, :]).astype(np.float32)
        map_y = (seed[1] + sines * radial[None, :]).astype(np.float32)
        valid = (
            (map_x >= 3.0)
            & (map_x < width - 3.0)
            & (map_y >= 3.0)
            & (map_y < height - 3.0)
        )

        # 只从完整画面采样180x约111的小矩阵，再沿径向做平滑。旧实现
        # 会先模糊、转换整张1280x720画面，额外消耗数毫秒；对检测而言
        # 未被射线访问的像素没有处理价值。
        colors = cv2.remap(
            frame,
            map_x,
            map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        colors = cv2.GaussianBlur(colors, (5, 1), 1.0).astype(np.float32)
        # 在采样矩阵上直接按BGR最大/最小值计算HSV饱和度等价值，避免
        # 对完整帧执行cvtColor。max=0时分母使用1，不影响黑色的S=0。
        maximum = np.max(colors, axis=2)
        minimum = np.min(colors, axis=2)
        saturations = (
            (maximum - minimum) * 255.0 / np.maximum(maximum, 1.0)
        )
        gradients = np.linalg.norm(
            colors[:, 2:, :] - colors[:, :-2, :], axis=2
        )
        edge_valid = valid[:, 1:-1] & valid[:, :-2] & valid[:, 2:]
        edge_radii = radial[1:-1]

        # 在粗半径附近加高斯先验，避免同一射线上较远的桌面纹理抢占
        # 外圆边缘。55px搜索宽度仍足以吸收Hough粗定位误差。
        prior = np.exp(-0.5 * ((edge_radii - seed[2]) / 24.0) ** 2)
        scores = gradients * prior[None, :]
        scores[
            (~edge_valid)
            | (
                saturations[:, 1:-1]
                >= config.saturated_pixel_threshold
            )
        ] = -np.inf

        indices = np.argmax(scores, axis=1)
        rows = np.arange(len(angles))
        strengths = gradients[rows, indices]
        eligible = (
            (np.count_nonzero(valid, axis=1) >= 10)
            & np.isfinite(scores[rows, indices])
            & (strengths >= config.minimum_edge_strength)
        )
        selected_angles = angles[eligible]
        selected_radii = edge_radii[indices[eligible]]
        points = np.column_stack(
            (
                seed[0] + selected_radii * np.cos(selected_angles),
                seed[1] + selected_radii * np.sin(selected_angles),
            )
        )
        return (
            points.astype(np.float64),
            np.minimum(strengths[eligible], 20.0).astype(np.float64),
        )

    def _refine_with_samples(self, frame, seed, config=None):
        """执行精修并同时返回调试界面可绘制的径向边缘点。"""

        config = self.config if config is None else config
        points = np.empty((0, 2), dtype=np.float64)
        try:
            points, strengths = self._sample_boundary_points(
                frame, seed, config
            )
            model, visible_arc, rms, edge_strength = _robust_circle(
                points, strengths, config
            )
        except (
            TurntableDetectionError,
            cv2.error,
            np.linalg.LinAlgError,
        ) as error:
            return CircleFit(False, detail=str(error)), points

        center_x, center_y, radius = (float(value) for value in model)
        if not config.minimum_radius_px <= radius <= (
            config.maximum_radius_px
        ):
            return CircleFit(
                False,
                center_x,
                center_y,
                radius,
                visible_arc,
                rms,
                detail="拟合半径超出配置范围",
            ), points
        if visible_arc < config.minimum_visible_arc_deg:
            return CircleFit(
                False,
                center_x,
                center_y,
                radius,
                visible_arc,
                rms,
                detail="可见外圆弧不足",
            ), points
        if rms > config.maximum_fit_rms_px:
            return CircleFit(
                False,
                center_x,
                center_y,
                radius,
                visible_arc,
                rms,
                detail="圆弧拟合残差过大",
            ), points

        # 置信度只表达本帧质量，不把圆心位置或某个固定半径作为先验。
        # 因此转盘在250~450px范围内移动或缩放时不会被目标基准压低分数。
        arc_score = min(1.0, visible_arc / 180.0)
        rms_score = max(
            0.0,
            1.0 - rms / (config.maximum_fit_rms_px * 1.5),
        )
        edge_score = min(1.0, edge_strength / 8.0)
        confidence = float(
            0.35 * arc_score + 0.50 * rms_score + 0.15 * edge_score
        )
        return CircleFit(
            True,
            center_x,
            center_y,
            radius,
            visible_arc,
            rms,
            confidence,
            "外圆鲁棒拟合成功",
        ), points

    def _refine(self, frame, seed, config=None) -> CircleFit:
        """对一个粗圆种子执行全分辨率圆弧采样和质量筛选。"""

        fit, _points = self._refine_with_samples(frame, seed, config)
        return fit

    def _material_seed_fit(self, frame, *, force=False):
        """用三物料产生搜索种子，并要求真实外圆完成最终确认。"""

        if self.material_seeder is None:
            return CircleFit(False, detail="未配置三物料粗定位"), None
        if not force and self._material_seed_retry_countdown > 0:
            self._material_seed_retry_countdown -= 1
            return CircleFit(False, detail="等待下一次三物料重试"), None

        result = self.material_seeder.detect(frame)
        self.last_material_seed_result = result
        self._material_seed_retry_countdown = max(
            0, int(self.material_seeder.config.retry_interval_frames) - 1
        )
        if not result.detected or result.seed is None:
            return CircleFit(False, detail=result.detail), result

        widened = replace(
            self.config,
            radial_search_half_width_px=max(
                self.config.radial_search_half_width_px,
                float(
                    self.material_seeder.config
                    .outer_radial_search_half_width_px
                ),
            ),
        )
        fit = self._refine(frame, result.seed, widened)
        if not fit.detected:
            return replace(
                fit,
                detail=(
                    f"{result.detail}；但300mm外圆确认失败：{fit.detail}"
                ),
            ), result
        return replace(
            fit,
            detail=f"{result.detail}；局部300mm外圆拟合成功",
        ), result

    @staticmethod
    def _candidate_key(candidate: CircleFit):
        """多个粗圆都通过时，优先高置信、长圆弧和低残差结果。"""

        return (
            candidate.confidence,
            candidate.visible_arc_deg,
            -candidate.fit_rms_px,
        )

    def _reacquire(self, frame):
        self._last_reacquire_tracking_mode = "reacquired"
        material_fit, _material_result = self._material_seed_fit(frame)
        if material_fit.detected:
            self._last_reacquire_tracking_mode = "three_material"
            return material_fit
        candidates = self._coarse_candidates(frame)
        if not candidates:
            material_detail = material_fit.detail
            return CircleFit(
                False,
                detail=(
                    f"三物料粗定位未通过：{material_detail}；"
                    "全图未找到大圆候选"
                ),
            )
        refined = tuple(self._refine(frame, seed) for seed in candidates)
        valid = tuple(item for item in refined if item.detected)
        if not valid:
            details = "; ".join(item.detail for item in refined[:3])
            return CircleFit(
                False,
                detail=f"大圆候选均未通过精修：{details}",
            )
        return max(valid, key=self._candidate_key)

    def inspect_frame(self, frame) -> TurntableDebugResult:
        """返回一帧完整粗搜/精修过程，且不污染正式检测状态。"""

        started_s = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise TurntableDetectionError("转盘调试要求BGR彩色图像")
        seeds = self._coarse_candidates(frame)
        candidates = tuple(
            DebugCircleCandidate(seed, *self._refine_with_samples(frame, seed))
            for seed in seeds
        )
        valid_indices = tuple(
            index for index, candidate in enumerate(candidates)
            if candidate.fit.detected
        )
        selected_index = None
        if valid_indices:
            selected_index = max(
                valid_indices,
                key=lambda index: self._candidate_key(candidates[index].fit),
            )
            detail = "已找到通过当前参数的精修圆"
        elif not candidates:
            detail = "当前参数下没有Hough大圆候选"
        else:
            reasons = "; ".join(
                candidate.fit.detail for candidate in candidates[:3]
            )
            detail = f"所有候选均被精修拒绝：{reasons}"
        return TurntableDebugResult(
            candidates=candidates,
            selected_index=selected_index,
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            detail=detail,
        )

    def _failed(self, started_s, detail) -> TurntableDetection:
        """失败时清除时间平滑结果，避免画面继续显示旧圆。"""

        self.seed = None
        self.samples.clear()
        return TurntableDetection(
            detected=False,
            stable=False,
            center_x_px=0.0,
            center_y_px=0.0,
            radius_px=0.0,
            visible_arc_deg=0.0,
            fit_rms_px=math.inf,
            confidence=0.0,
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            tracking_mode="searching",
            detail=str(detail),
        )

    def _accepted(
        self,
        started_s: float,
        fit: CircleFit,
        tracking_mode: str,
        material_centers_px=(),
        prime_stability=False,
    ) -> TurntableDetection:
        """把一帧已通过精修的圆加入稳定窗口并生成公开结果。"""

        # 只有真实外圆通过质量检查才会进入这里，因此保存的局部种子
        # 始终来自外圆，而不是三个物料底端的轨迹圆。
        self.seed = (fit.center_x_px, fit.center_y_px, fit.radius_px)
        if tracking_mode in ("hough", "reacquired", "three_material"):
            self._frames_since_global_recheck = 0
        if prime_stability:
            # 完整 ROS 代码认为“三物料几何 + 同帧真实外圆”已完成交叉
            # 验证，会直接预置稳定窗口，避免下一帧退回等待稳定状态。
            self.samples.clear()
            self.samples.extend(fit for _ in range(self.samples.maxlen))
        else:
            self.samples.append(fit)
        center_x = median(item.center_x_px for item in self.samples)
        center_y = median(item.center_y_px for item in self.samples)
        radius = median(item.radius_px for item in self.samples)
        center_jitter = max(
            math.hypot(
                item.center_x_px - center_x,
                item.center_y_px - center_y,
            )
            for item in self.samples
        )
        radius_jitter = max(
            abs(item.radius_px - radius) for item in self.samples
        )
        stable = bool(
            len(self.samples) == self.samples.maxlen
            and center_jitter <= self.config.maximum_center_jitter_px
            and radius_jitter <= self.config.maximum_radius_jitter_px
        )
        if stable and tracking_mode == "three_material":
            detail = "三物料种子与真实外圆交叉确认完成"
        elif stable:
            detail = "连续圆盘观测稳定"
        else:
            detail = f"累计稳定帧 {len(self.samples)}/{self.samples.maxlen}"
        return TurntableDetection(
            detected=True,
            stable=stable,
            center_x_px=float(center_x),
            center_y_px=float(center_y),
            radius_px=float(radius),
            visible_arc_deg=float(
                median(item.visible_arc_deg for item in self.samples)
            ),
            fit_rms_px=float(median(item.fit_rms_px for item in self.samples)),
            confidence=float(
                median(item.confidence for item in self.samples)
            ),
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            tracking_mode=tracking_mode,
            detail=detail,
            material_centers_px=tuple(material_centers_px),
        )

    def detect_with_debug(
        self, frame
    ) -> tuple[TurntableDetection, TurntableDebugResult]:
        """用一次 Hough/精修同时生成检测结果和调试候选。

        该接口专供参数设置页面使用：页面需要每帧看见 Hough 粗圆，若再
        分别调用 ``detect()`` 和 ``inspect_frame()``，未锁定时会重复执行
        Hough 和候选精修。这里直接采用调试结果中的最佳精修圆更新稳定
        窗口，使画圆、失败原因和正式结果来自完全相同的一次计算。
        """

        started_s = time.perf_counter()
        # main.py 在尚未稳定时会调用本接口。三物料已经给出并由外圆
        # 确认过种子后，后续帧直接局部跟踪，避免为了调试候选重复运行
        # 端面 Hough 或全图大圆 Hough。独立调参工具未配置 seeder，仍
        # 保持每帧展示完整 Hough 候选的原行为。
        if (
            self.material_seeder is not None
            and self.seed is not None
        ):
            result = self.detect(frame)
            return result, TurntableDebugResult(
                candidates=(),
                selected_index=None,
                processing_ms=result.processing_ms,
                detail="三物料种子已锁定，使用局部外圆跟踪",
            )

        material_fit, material_result = self._material_seed_fit(
            frame, force=True
        )
        if material_fit.detected:
            centers = (
                () if material_result is None
                else material_result.material_centers_px
            )
            result = self._accepted(
                started_s,
                material_fit,
                "three_material",
                material_centers_px=centers,
                prime_stability=True,
            )
            return result, TurntableDebugResult(
                candidates=(),
                selected_index=None,
                processing_ms=result.processing_ms,
                detail=material_fit.detail,
            )

        debug = self.inspect_frame(frame)
        if debug.selected_index is None:
            detail = debug.detail
            if material_result is not None:
                detail = (
                    f"三物料粗定位未通过：{material_fit.detail}；{detail}"
                )
                debug = replace(debug, detail=detail)
            return self._failed(started_s, detail), debug
        fit = debug.candidates[debug.selected_index].fit
        return self._accepted(started_s, fit, "hough"), debug

    def detect(self, frame) -> TurntableDetection:
        """检测一帧转盘，并返回平滑后的圆心、半径和质量指标。"""

        started_s = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise TurntableDetectionError("转盘检测要求BGR彩色图像")

        # 1. 已有圆心种子时先沿径向找外圆边缘，避免逐帧做全图Hough。
        tracking_mode = "local"
        fit = CircleFit(False, detail="尚未建立局部种子")
        had_local_seed = self.seed is not None
        if had_local_seed:
            fit = self._refine(frame, self.seed)
            self._frames_since_global_recheck = min(
                self.config.local_recheck_interval_frames,
                self._frames_since_global_recheck + 1,
            )

        # 局部射线偶尔可能在旧位置附近把桌面纹理拟合成低质量大圆。
        # 只对低置信或高RMS结果追加一次全图复核，正常稳定帧不付出
        # Hough开销。若全局结果质量更高且明显位于别处，则接受新圆。
        # 2. 低质量局部结果按既定间隔全图复核，防止跟踪到桌面纹理。
        local_quality_needs_recheck = bool(
            fit.detected
            and (
                fit.confidence
                < self.config.local_recheck_confidence_below
                or fit.fit_rms_px
                > self.config.local_recheck_rms_above_px
            )
        )
        if (
            local_quality_needs_recheck
            and self._frames_since_global_recheck
            >= self.config.local_recheck_interval_frames
        ):
            self._frames_since_global_recheck = 0
            global_fit = self._reacquire(frame)
            if not global_fit.detected:
                return self._failed(
                    started_s,
                    "局部圆质量不足，且全图复核失败："
                    f"{global_fit.detail}",
                )
            center_jump = math.hypot(
                global_fit.center_x_px - fit.center_x_px,
                global_fit.center_y_px - fit.center_y_px,
            )
            radius_jump = abs(global_fit.radius_px - fit.radius_px)
            jump = max(center_jump, radius_jump)
            if jump >= self.config.reacquisition_jump_px:
                if global_fit.confidence < fit.confidence:
                    return self._failed(
                        started_s,
                        "低质量局部圆未被全图复核确认",
                    )
                fit = global_fit
                tracking_mode = self._last_reacquire_tracking_mode
                self.samples.clear()

        # 3. 没有种子或局部精修失败，立即重定位；新位置清除旧稳定窗口。
        if not fit.detected:
            tracking_mode = "reacquired"
            if had_local_seed:
                # 新位置不能与旧位置的平滑窗口求中值，否则大幅跳变后的
                # 第一帧会显示在新旧圆心之间，形成实际上不存在的圆。
                self.samples.clear()
            fit = self._reacquire(frame)
            tracking_mode = self._last_reacquire_tracking_mode
        if not fit.detected:
            return self._failed(started_s, fit.detail)

        # 4. 精修结果进入稳定窗口，输出中值圆心、半径和质量指标。
        material_centers = ()
        if (
            tracking_mode == "three_material"
            and self.last_material_seed_result is not None
        ):
            material_centers = (
                self.last_material_seed_result.material_centers_px
            )
        return self._accepted(
            started_s,
            fit,
            tracking_mode,
            material_centers_px=material_centers,
        )


def draw_turntable_detection(frame, result: TurntableDetection):
    """在矫正画面上绘制转盘外圆、圆心和质量数据。"""

    output = frame
    state = "STABLE" if result.stable else (
        "DETECTED" if result.detected else "SEARCHING"
    )
    state_color = (
        (40, 255, 40)
        if result.stable
        else ((0, 220, 255) if result.detected else (40, 40, 255))
    )
    if result.detected:
        center = (
            int(round(result.center_x_px)),
            int(round(result.center_y_px)),
        )
        cv2.circle(
            output,
            center,
            int(round(result.radius_px)),
            state_color,
            3,
            cv2.LINE_8,
        )
        cv2.drawMarker(
            output, center, state_color, cv2.MARKER_CROSS, 26, 3,
            cv2.LINE_8,
        )
        for index, material_center in enumerate(
            result.material_centers_px, 1
        ):
            point = tuple(int(round(value)) for value in material_center)
            cv2.drawMarker(
                output, point, (0, 220, 255), cv2.MARKER_TILTED_CROSS,
                22, 2, cv2.LINE_8,
            )
            cv2.putText(
                output, f"M{index}", (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2,
                cv2.LINE_AA,
            )

    lines = [
        f"Turntable: {state} ({result.tracking_mode})",
        (
            f"Center: ({result.center_x_px:.1f}, {result.center_y_px:.1f})  "
            f"Radius: {result.radius_px:.1f}px"
            if result.detected
            else "Center: --  Radius: --"
        ),
        (
            (
                f"Arc: {result.visible_arc_deg:.0f}deg  "
                f"RMS: {result.fit_rms_px:.2f}px  "
                + f"Confidence: {result.confidence:.3f}  "
                + f"Detect: {result.processing_ms:.1f}ms"
            )
            if result.detected
            else f"Detect: {result.processing_ms:.1f}ms"
        ),
    ]
    # 使用一块黑色信息底板和单层文字。比每行绘制黑色描边再绘制彩色
    # 前景少三次全分辨率抗锯齿操作，可明显降低显示模式的帧耗时。
    cv2.rectangle(output, (10, 58), (910, 164), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        y = 82 + index * 31
        cv2.putText(
            output,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.67,
            state_color,
            2,
            cv2.LINE_AA,
        )
    return output
