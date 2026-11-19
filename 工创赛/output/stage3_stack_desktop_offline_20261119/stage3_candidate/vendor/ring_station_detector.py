"""无 ROS 的三圆环工位识别、稳定判定和视觉对位计算。

本模块只处理一张已经完成畸变矫正的 BGR 图像。它不打开摄像头、不访问
串口，也不控制底盘或机械臂，因此既可由 ``main.py`` 调用，也可单独用于
离线测试。配置中的参考圆心和纠偏矩阵移植自原 ROS 工程。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from itertools import combinations
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


class RingStationDetectionError(ValueError):
    """圆环配置或输入画面无效。"""


@dataclass(frozen=True)
class RingStationConfig:
    """三圆环几何、图像筛选门槛和固定观察基准。"""

    center_spacing_mm: float
    outer_diameter_mm: float
    ring1_side: str
    canny_low: int
    canny_high: int
    min_contour_points: int
    min_minor_axis_px: float
    max_major_axis_px: float
    min_ellipse_aspect: float
    center_cluster_radius_px: float
    diameter_band_gap_px: float
    min_cluster_support: int
    min_diameter_bands: int
    min_station_span_ratio: float
    max_collinearity_error_ratio: float
    roi_margin_px: int
    roi_fallback_after_failures: int
    reference_yaw_deg: float
    reference_centers_px: tuple[tuple[float, float], ...]
    image_to_station_affine: tuple[tuple[float, float], ...]
    minimum_confidence: float
    stable_frame_count: int
    maximum_center_jitter_px: float
    maximum_affine_residual: float
    perspective_ratio_range: tuple[float, float]
    maximum_position_error_mm: float
    maximum_angle_error_deg: float
    chassis_image_to_motion: tuple[tuple[float, float, float], ...] | None


@dataclass(frozen=True)
class RingStationDetection:
    """一帧圆环工位结果；失败时不携带上一帧坐标。"""

    detected: bool = False
    stable: bool = False
    confidence: float = 0.0
    centers_px: tuple[tuple[float, float], ...] = ()
    ellipses: tuple[tuple, ...] = ()
    station_yaw_deg: float = 0.0
    angle_error_deg: float = 0.0
    error_x_mm: float = 0.0
    error_y_mm: float = 0.0
    position_error_mm: float = 0.0
    correction_body_x_mm: float | None = None
    correction_body_y_mm: float | None = None
    correction_yaw_deg: float | None = None
    affine_residual: float = math.inf
    perspective_ratio: float = math.inf
    within_tolerance: bool = False
    processing_ms: float = 0.0
    detection_scope: str = "full"
    detail: str = "尚未识别"


@dataclass
class _EllipseObservation:
    center: np.ndarray
    axes: tuple[float, float]
    angle_deg: float
    major_axis: float
    minor_axis: float


@dataclass
class _RingCandidate:
    center: np.ndarray
    support: int
    diameter_bands: int
    ellipse: _EllipseObservation
    effective_diameter_px: float


def _finite_matrix(raw, shape, name):
    matrix = np.asarray(raw, dtype=float)
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise RingStationDetectionError(
            f"{name} 必须是有限数值的 {shape[0]}x{shape[1]} 矩阵"
        )
    return tuple(tuple(float(value) for value in row) for row in matrix)


def load_ring_station_config(path: Path | str) -> RingStationConfig:
    """从 JSON 读取并严格校验三圆环工位配置。"""

    resolved = Path(path).expanduser().resolve()
    try:
        raw = json.loads(resolved.read_text(encoding="utf-8"))
        if int(raw["format_version"]) != 1:
            raise RingStationDetectionError("圆环配置版本不受支持")
        geometry = raw["geometry"]
        detection = raw["detection"]
        reference = raw["reference"]
        acceptance = raw["acceptance"]
        centers = tuple(
            (float(center[0]), float(center[1]))
            for center in reference["ring_centers_px"]
        )
        affine = _finite_matrix(
            reference["image_to_station_affine"], (2, 2),
            "image_to_station_affine",
        )
        correction_raw = raw.get("correction_calibration", {}).get(
            "chassis_image_to_motion"
        )
        # 3x3底盘标定矩阵把(x毫米, y毫米, 角度)转换为底盘纠偏量。
        correction = None
        if correction_raw is not None:
            correction = _finite_matrix(
                correction_raw, (3, 3), "chassis_image_to_motion"
            )
        config = RingStationConfig(
            center_spacing_mm=float(geometry["center_spacing_mm"]),
            outer_diameter_mm=float(geometry["outer_diameter_mm"]),
            ring1_side=str(geometry["ring1_side"]),
            canny_low=int(detection["canny_low"]),
            canny_high=int(detection["canny_high"]),
            min_contour_points=int(detection["min_contour_points"]),
            min_minor_axis_px=float(detection["min_minor_axis_px"]),
            max_major_axis_px=float(detection["max_major_axis_px"]),
            min_ellipse_aspect=float(detection["min_ellipse_aspect"]),
            center_cluster_radius_px=float(
                detection["center_cluster_radius_px"]
            ),
            diameter_band_gap_px=float(detection["diameter_band_gap_px"]),
            min_cluster_support=int(detection["min_cluster_support"]),
            min_diameter_bands=int(detection["min_diameter_bands"]),
            min_station_span_ratio=float(
                detection["min_station_span_ratio"]
            ),
            max_collinearity_error_ratio=float(
                detection["max_collinearity_error_ratio"]
            ),
            roi_margin_px=int(detection["roi_margin_px"]),
            roi_fallback_after_failures=int(
                detection["roi_fallback_after_failures"]
            ),
            reference_yaw_deg=float(reference["station_yaw_deg"]),
            reference_centers_px=centers,
            image_to_station_affine=affine,
            minimum_confidence=float(reference["minimum_confidence"]),
            stable_frame_count=int(reference["stable_frame_count"]),
            maximum_center_jitter_px=float(
                reference["maximum_center_jitter_px"]
            ),
            maximum_affine_residual=float(
                reference["maximum_affine_residual"]
            ),
            perspective_ratio_range=tuple(
                float(value) for value in reference["perspective_ratio_range"]
            ),
            maximum_position_error_mm=float(
                acceptance["maximum_position_error_mm"]
            ),
            maximum_angle_error_deg=float(
                acceptance["maximum_angle_error_deg"]
            ),
            chassis_image_to_motion=correction,
        )
    except RingStationDetectionError:
        raise
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError,
            IndexError) as error:
        raise RingStationDetectionError(
            f"无法读取圆环配置 {resolved}：{error}"
        ) from error

    if config.center_spacing_mm <= 0 or config.outer_diameter_mm <= 0:
        raise RingStationDetectionError("圆环实际尺寸必须大于0")
    if config.ring1_side not in {"left", "right"}:
        raise RingStationDetectionError("ring1_side 只能是 left 或 right")
    if len(config.reference_centers_px) != 3:
        raise RingStationDetectionError("参考基准必须包含三个圆心")
    if not 0 < config.canny_low < config.canny_high <= 255:
        raise RingStationDetectionError("Canny 阈值无效")
    if not 0.0 < config.min_ellipse_aspect <= 1.0:
        raise RingStationDetectionError("椭圆长短轴比例无效")
    if config.min_cluster_support < 2 or config.min_diameter_bands < 2:
        raise RingStationDetectionError("同心轮廓支持数门槛过低")
    if not 0.0 < config.minimum_confidence <= 1.0:
        raise RingStationDetectionError("置信度门槛无效")
    if config.stable_frame_count < 2:
        raise RingStationDetectionError("稳定窗口至少需要2帧")
    if config.maximum_center_jitter_px <= 0:
        raise RingStationDetectionError("圆心抖动门槛必须大于0")
    if len(config.perspective_ratio_range) != 2:
        raise RingStationDetectionError("透视比例范围必须包含上下界")
    if not 0 < config.perspective_ratio_range[0] < config.perspective_ratio_range[1]:
        raise RingStationDetectionError("透视比例范围无效")
    if config.roi_margin_px < 0 or config.roi_fallback_after_failures < 1:
        raise RingStationDetectionError("ROI 跟踪配置无效")
    return config


def _find(parent, index):
    if parent[index] != index:
        parent[index] = _find(parent, parent[index])
    return parent[index]


def _extract_observations(image, config):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(blurred, config.canny_low, config.canny_high)
    contours, _ = cv2.findContours(
        edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )
    observations = []
    for contour in contours:
        if len(contour) < config.min_contour_points:
            continue
        center, axes, angle = cv2.fitEllipse(contour)
        major, minor = max(axes), min(axes)
        if (
            minor < config.min_minor_axis_px
            or major > config.max_major_axis_px
            or minor / major < config.min_ellipse_aspect
        ):
            continue
        observations.append(_EllipseObservation(
            center=np.asarray(center, dtype=np.float64),
            axes=(float(axes[0]), float(axes[1])),
            angle_deg=float(angle),
            major_axis=float(major),
            minor_axis=float(minor),
        ))
    return observations


def _cluster_observations(observations, config):
    parent = list(range(len(observations)))
    for first in range(len(observations)):
        for second in range(first):
            if np.linalg.norm(
                observations[first].center - observations[second].center
            ) <= config.center_cluster_radius_px:
                first_root = _find(parent, first)
                second_root = _find(parent, second)
                if first_root != second_root:
                    parent[second_root] = first_root

    groups = {}
    for index, observation in enumerate(observations):
        groups.setdefault(_find(parent, index), []).append(observation)

    candidates = []
    for group in groups.values():
        diameters = sorted(item.major_axis for item in group)
        bands = []
        for diameter in diameters:
            if not bands or diameter - bands[-1] > config.diameter_band_gap_px:
                bands.append(diameter)
        if (
            len(group) < config.min_cluster_support
            or len(bands) < config.min_diameter_bands
        ):
            continue
        center = np.median(
            np.asarray([item.center for item in group]), axis=0
        )
        effective_diameter = float(np.median(diameters[-min(4, len(diameters)):]))
        center_tolerance = max(6.0, effective_diameter * 0.08)
        centered = [
            item for item in group
            if np.linalg.norm(item.center - center) <= center_tolerance
        ] or group
        outer = max(centered, key=lambda item: item.major_axis)
        candidates.append(_RingCandidate(
            center=center,
            support=len(group),
            diameter_bands=len(bands),
            ellipse=_EllipseObservation(
                center=center.copy(), axes=outer.axes,
                angle_deg=outer.angle_deg, major_axis=outer.major_axis,
                minor_axis=outer.minor_axis,
            ),
            effective_diameter_px=effective_diameter,
        ))
    return candidates


def _select_triple(candidates, image_width, config):
    best = None
    for triple in combinations(candidates, 3):
        points = np.asarray([item.center for item in triple])
        endpoints = max(
            (float(np.linalg.norm(points[a] - points[b])), a, b)
            for a in range(3) for b in range(a)
        )
        span, first_index, third_index = endpoints
        if span < image_width * config.min_station_span_ratio:
            continue
        middle_index = 3 - first_index - third_index
        line = points[third_index] - points[first_index]
        offset = points[middle_index] - points[first_index]
        fraction = float(np.dot(offset, line) / (span * span))
        perpendicular = abs(float(np.cross(line, offset))) / span
        collinearity = perpendicular / span
        if not 0.2 < fraction < 0.8:
            continue
        if collinearity > config.max_collinearity_error_ratio:
            continue

        endpoint_candidates = [triple[first_index], triple[third_index]]
        ring1, ring3 = sorted(
            endpoint_candidates,
            key=lambda item: item.center[0],
            reverse=config.ring1_side == "right",
        )
        ordered = (ring1, triple[middle_index], ring3)
        spacing_12 = float(np.linalg.norm(ordered[1].center - ordered[0].center))
        spacing_23 = float(np.linalg.norm(ordered[2].center - ordered[1].center))
        balance = abs(math.log(spacing_12 / spacing_23))
        support = sum(item.support for item in ordered)
        diameter_penalty = 0.0
        for item, spacing in zip(
            ordered, (spacing_12, (spacing_12 + spacing_23) / 2, spacing_23)
        ):
            ratio = item.effective_diameter_px / spacing
            if ratio < 0.25:
                diameter_penalty += 0.25 - ratio
            elif ratio > 1.10:
                diameter_penalty += ratio - 1.10
        score = 8.0 * collinearity + 0.35 * balance + 2.0 * diameter_penalty - 0.015 * support
        if best is None or score < best[0]:
            best = (score, ordered, collinearity, support)
    return best


def _raw_detection(image, config):
    observations = _extract_observations(image, config)
    candidates = _cluster_observations(observations, config)
    selection = _select_triple(candidates, image.shape[1], config)
    if selection is None:
        return None
    _, ordered, collinearity, _ = selection
    spacing_12 = float(np.linalg.norm(ordered[1].center - ordered[0].center))
    spacing_23 = float(np.linalg.norm(ordered[2].center - ordered[1].center))
    perspective = spacing_12 / spacing_23
    geometry = math.exp(
        -10.0 * collinearity - 0.5 * abs(math.log(perspective))
    )
    support = min(item.support for item in ordered) / 10.0
    bands = min(item.diameter_bands for item in ordered) / 6.0
    confidence = min(1.0, 0.55 * geometry + 0.25 * min(support, 1.0) + 0.20 * min(bands, 1.0))

    # 使用中间环椭圆和三环中心距建立与旧实现一致的局部仿射质量指标。
    image_x_per_mm = (
        ordered[2].center - ordered[0].center
    ) / (2.0 * config.center_spacing_mm)
    outer = ordered[1].ellipse
    theta = math.radians(outer.angle_deg)
    rotation = np.asarray([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ])
    ellipse_shape = rotation @ np.diag([
        (outer.axes[0] * 0.5) ** 2,
        (outer.axes[1] * 0.5) ** 2,
    ]) @ rotation.T
    radius_mm = config.outer_diameter_mm * 0.5
    remaining = ellipse_shape / (radius_mm * radius_mm) - np.outer(
        image_x_per_mm, image_x_per_mm
    )
    eigenvalues = np.linalg.eigvalsh((remaining + remaining.T) * 0.5)
    largest = max(float(eigenvalues[-1]), 0.0)
    residual = (
        abs(float(eigenvalues[0])) / largest if largest > 1e-9 else math.inf
    )
    return {
        "centers": tuple(tuple(float(v) for v in item.center) for item in ordered),
        "ellipses": tuple((
            tuple(float(v) for v in item.ellipse.center),
            tuple(float(v) for v in item.ellipse.axes),
            float(item.ellipse.angle_deg),
        ) for item in ordered),
        "confidence": float(confidence),
        "yaw": math.degrees(math.atan2(
            float(ordered[2].center[1] - ordered[0].center[1]),
            float(ordered[2].center[0] - ordered[0].center[0]),
        )),
        "perspective": float(perspective),
        "residual": float(residual),
    }


def _normalize_angle(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


class RingStationDetector:
    """全图搜索后转入 ROI，并只由连续直接观测产生稳定结果。"""

    def __init__(self, config: RingStationConfig):
        self.config = config
        self._samples = deque(maxlen=config.stable_frame_count)
        self._tracking_roi = None
        self._roi_failures = 0

    def reset(self):
        """清除 ROI 和稳定窗口，防止跨模式使用旧坐标。"""

        self._samples.clear()
        self._tracking_roi = None
        self._roi_failures = 0

    def _make_roi(self, raw, shape):
        points = np.asarray(raw["centers"])
        half_diameter = max(
            max(ellipse[1]) for ellipse in raw["ellipses"]
        ) / 2.0
        margin = self.config.roi_margin_px + half_diameter
        height, width = shape[:2]
        return (
            max(0, int(math.floor(points[:, 0].min() - margin))),
            max(0, int(math.floor(points[:, 1].min() - margin))),
            min(width, int(math.ceil(points[:, 0].max() + margin))),
            min(height, int(math.ceil(points[:, 1].max() + margin))),
        )

    @staticmethod
    def _offset_raw(raw, x0, y0):
        if raw is None:
            return None
        raw["centers"] = tuple(
            (x + x0, y + y0) for x, y in raw["centers"]
        )
        raw["ellipses"] = tuple(
            ((center[0] + x0, center[1] + y0), axes, angle)
            for center, axes, angle in raw["ellipses"]
        )
        return raw

    def _find(self, frame):
        if self._tracking_roi is None:
            return _raw_detection(frame, self.config), "full"
        x0, y0, x1, y1 = self._tracking_roi
        raw = _raw_detection(frame[y0:y1, x0:x1], self.config)
        raw = self._offset_raw(raw, x0, y0)
        if raw is not None:
            self._roi_failures = 0
            return raw, "roi"
        self._roi_failures += 1
        if self._roi_failures < self.config.roi_fallback_after_failures:
            return None, "roi"
        self._roi_failures = 0
        return _raw_detection(frame, self.config), "full_fallback"

    def detect(self, frame) -> RingStationDetection:
        """检测一帧，返回三环对位误差、纠偏量、质量和耗时。"""

        started = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise RingStationDetectionError("圆环检测要求 BGR 彩色图像")
        # 1. 提取椭圆、合并同心轮廓、筛选三个共线圆环；有ROI时优先局部搜索。
        raw, scope = self._find(frame)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if raw is None:
            self._samples.clear()
            return RingStationDetection(
                processing_ms=elapsed_ms,
                detection_scope=scope,
                detail="没有找到满足几何约束的三个圆环",
            )

        config = self.config
        self._tracking_roi = self._make_roi(raw, frame.shape)
        # 2. 置信度、仿射残差和透视比例共同决定能否累计稳定帧。
        quality_detail = None
        if raw["confidence"] < config.minimum_confidence:
            quality_detail = "三圆环识别置信度不足"
        elif raw["residual"] > config.maximum_affine_residual:
            quality_detail = "局部仿射模型残差过大"
        elif not (
            config.perspective_ratio_range[0]
            <= raw["perspective"]
            <= config.perspective_ratio_range[1]
        ):
            quality_detail = "圆环透视比超出标定适用范围"

        if quality_detail is not None:
            self._samples.clear()
            return RingStationDetection(
                detected=True,
                confidence=raw["confidence"],
                centers_px=raw["centers"],
                ellipses=raw["ellipses"],
                station_yaw_deg=raw["yaw"],
                affine_residual=raw["residual"],
                perspective_ratio=raw["perspective"],
                processing_ms=elapsed_ms,
                detection_scope=scope,
                detail=quality_detail,
            )

        self._samples.append(raw)
        # 连续帧中，每个圆心的X/Y分别取中值；坐标单位为原图像素。
        centers = []
        for ring_index in range(3):
            xs = [sample["centers"][ring_index][0] for sample in self._samples]
            ys = [sample["centers"][ring_index][1] for sample in self._samples]
            centers.append((float(np.median(xs)), float(np.median(ys))))
        centers = tuple(centers)

        maximum_jitter = 0.0
        for sample in self._samples:
            for ring_index in range(3):
                dx = sample["centers"][ring_index][0] - centers[ring_index][0]
                dy = sample["centers"][ring_index][1] - centers[ring_index][1]
                maximum_jitter = max(maximum_jitter, math.hypot(dx, dy))
        stable = bool(
            len(self._samples) == config.stable_frame_count
            and maximum_jitter <= config.maximum_center_jitter_px
        )
        yaw_errors = [
            _normalize_angle(sample["yaw"] - config.reference_yaw_deg)
            for sample in self._samples
        ]
        angle_error = float(np.median(yaw_errors))
        yaw = _normalize_angle(config.reference_yaw_deg + angle_error)

        affine = np.asarray(config.image_to_station_affine)
        # 图像像素偏差乘2x2标定矩阵，得到工位平面上的毫米偏差。
        metric_errors = []
        for center, reference in zip(centers, config.reference_centers_px):
            pixel_error = np.asarray(center) - np.asarray(reference)
            metric_errors.append(affine @ pixel_error)
        metric_errors = tuple(metric_errors)
        error_x, error_y = (float(v) for v in metric_errors[1])
        position_error = max(float(np.linalg.norm(error)) for error in metric_errors)
        within = bool(
            stable
            and position_error <= config.maximum_position_error_mm
            and abs(angle_error) <= config.maximum_angle_error_deg
        )
        # 3x3底盘标定矩阵把(x毫米, y毫米, 角度)转换为底盘纠偏量。
        correction = None
        if config.chassis_image_to_motion is not None:
            correction = np.asarray(config.chassis_image_to_motion) @ np.asarray(
                (error_x, error_y, angle_error)
            )
        if not stable:
            detail = f"等待稳定观测（{len(self._samples)}/{config.stable_frame_count}）"
        elif within:
            detail = "工位视觉残差已达到验收精度"
        else:
            detail = "工位偏差超出验收精度"
        return RingStationDetection(
            detected=True,
            stable=stable,
            confidence=float(np.median([s["confidence"] for s in self._samples])),
            centers_px=centers,
            ellipses=raw["ellipses"],
            station_yaw_deg=yaw,
            angle_error_deg=angle_error,
            error_x_mm=error_x,
            error_y_mm=error_y,
            position_error_mm=position_error,
            correction_body_x_mm=None if correction is None else float(correction[0]),
            correction_body_y_mm=None if correction is None else float(correction[1]),
            correction_yaw_deg=None if correction is None else float(correction[2]),
            affine_residual=float(np.median([s["residual"] for s in self._samples])),
            perspective_ratio=float(np.median([s["perspective"] for s in self._samples])),
            within_tolerance=within,
            processing_ms=elapsed_ms,
            detection_scope=scope,
            detail=detail,
        )


def draw_ring_station_detection(frame, result: RingStationDetection):
    """在原画面上绘制当前帧的三环、误差和检测质量。"""

    if not result.detected:
        cv2.putText(
            frame, "Ring station: SEARCHING", (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2,
            cv2.LINE_AA,
        )
        return frame
    colors = ((0, 180, 255), (0, 255, 0), (255, 160, 0))
    points = []
    for index, (center, ellipse, color) in enumerate(
        zip(result.centers_px, result.ellipses, colors), start=1
    ):
        cv2.ellipse(frame, ellipse, color, 2, cv2.LINE_AA)
        point = tuple(int(round(value)) for value in center)
        points.append(point)
        cv2.drawMarker(frame, point, color, cv2.MARKER_CROSS, 24, 2)
        cv2.putText(
            frame, f"R{index}", (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA,
        )
    cv2.polylines(frame, [np.asarray(points, np.int32)], False, (255, 0, 255), 2)
    cv2.rectangle(frame, (10, 65), (820, 145), (0, 0, 0), -1)
    cv2.putText(
        frame,
        f"Ring: {'STABLE' if result.stable else 'WAITING'}  "
        f"dX={result.error_x_mm:+.2f}mm dY={result.error_y_mm:+.2f}mm "
        f"dYaw={result.angle_error_deg:+.2f}deg",
        (20, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.66,
        (80, 255, 80) if result.within_tolerance else (255, 255, 255),
        2, cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"Confidence={result.confidence:.3f} residual={result.affine_residual:.3f} "
        f"perspective={result.perspective_ratio:.3f} {result.processing_ms:.1f}ms",
        (20, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
        (255, 255, 255), 2, cv2.LINE_AA,
    )
    return frame
