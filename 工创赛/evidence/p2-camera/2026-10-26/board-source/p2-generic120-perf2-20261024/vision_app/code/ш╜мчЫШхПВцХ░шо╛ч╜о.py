#!/usr/bin/env python3
"""转盘外圆参数的实时可视化设置工具。

左侧显示畸变矫正画面、Hough 粗圆、精修圆和可选径向采样点；右侧用
中文滑条修改参数。按 A 会在当前帧上执行一次宽范围自动诊断并填入半径
建议，按 S 才会把当前值原子保存到正式配置。
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import math
from pathlib import Path
import time

import cv2
import numpy as np
from project_camera import DEFAULT_DEVICE, open_project_camera, calibration_maps

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover - 当前开发板已安装 Pillow。
    Image = ImageDraw = ImageFont = None

from turntable_detector import build_default_turntable_detector_document
from turntable_detector import CircleFit
from turntable_detector import DebugCircleCandidate
from turntable_detector import draw_turntable_detection
from turntable_detector import load_turntable_detector_document
from turntable_detector import save_turntable_detector_config
from turntable_detector import TurntableDebugResult
from turntable_detector import TurntableDetectionError
from turntable_detector import TurntableDetector
from turntable_detector import turntable_config_from_document


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "config" / "decxin_camera_unconfigured.yaml"
DEFAULT_CONFIG = SCRIPT_DIR / "config" / "turntable_detector.json"
FRAME_WIDTH = 1280
FRAME_HEIGHT = 960
PANEL_WIDTH = 420
CANVAS_WIDTH = FRAME_WIDTH + PANEL_WIDTH
MASK_REFRESH_INTERVAL_S = 0.10
FONT_PATHS = (
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
)


class TurntableSettingsError(RuntimeError):
    """参数页面无法打开摄像头、字体或标定数据。"""


@dataclass(frozen=True)
class ParameterSpec:
    """一个右侧滑条与 JSON 字段之间的映射。"""

    page: str
    group: str
    key: str
    label: str
    minimum: float
    maximum: float
    step: float
    integer: bool = False

    def format_value(self, value) -> str:
        if self.integer:
            return str(int(value))
        if self.step >= 1.0:
            return f"{float(value):.0f}"
        if self.step >= 0.1:
            return f"{float(value):.1f}"
        return f"{float(value):.2f}"


PARAMETER_SPECS = (
    ParameterSpec("common", "coarse_search", "image_scale",
                  "处理缩放", 0.20, 1.00, 0.01),
    ParameterSpec("common", "coarse_search", "minimum_radius_px",
                  "最小半径 px", 50, 700, 1, True),
    ParameterSpec("common", "coarse_search", "maximum_radius_px",
                  "最大半径 px", 100, 900, 1, True),
    ParameterSpec("common", "coarse_search", "canny_threshold",
                  "边缘阈值", 1, 255, 1, True),
    ParameterSpec("common", "coarse_search", "accumulator_threshold",
                  "Hough 严格度", 5, 100, 1, True),
    ParameterSpec("common", "refinement", "radial_search_half_width_px",
                  "径向搜索宽度", 10, 150, 1, True),
    ParameterSpec("common", "refinement", "minimum_visible_arc_deg",
                  "最小可见圆弧 °", 10, 360, 1, True),
    ParameterSpec("common", "refinement", "maximum_fit_rms_px",
                  "最大拟合 RMS", 0.5, 10.0, 0.1),
    ParameterSpec("advanced", "coarse_search", "hough_dp",
                  "Hough dp", 1.0, 3.0, 0.1),
    ParameterSpec("advanced", "coarse_search", "minimum_center_distance_px",
                  "候选圆心间距", 20, 500, 1, True),
    ParameterSpec("advanced", "coarse_search", "maximum_candidates",
                  "最多候选数量", 1, 20, 1, True),
    ParameterSpec("advanced", "refinement", "angular_step_deg",
                  "采样角度步长 °", 0.5, 10.0, 0.5),
    ParameterSpec("advanced", "refinement", "minimum_edge_strength",
                  "最小边缘强度", 0.5, 20.0, 0.1),
    ParameterSpec("advanced", "refinement", "saturated_pixel_threshold",
                  "彩色像素过滤", 0, 255, 1, True),
    ParameterSpec("advanced", "refinement", "local_recheck_confidence_below",
                  "局部复核置信度", 0.0, 1.0, 0.01),
    ParameterSpec("advanced", "refinement", "local_recheck_rms_above_px",
                  "局部复核 RMS", 0.1, 10.0, 0.1),
    ParameterSpec("advanced", "refinement", "local_recheck_interval_frames",
                  "复核间隔帧", 1, 30, 1, True),
    ParameterSpec("advanced", "refinement", "reacquisition_jump_px",
                  "重定位跳变 px", 1, 200, 1, True),
    ParameterSpec("advanced", "stability", "window_frames",
                  "稳定窗口帧数", 2, 30, 1, True),
    ParameterSpec("advanced", "stability", "maximum_center_jitter_px",
                  "最大圆心抖动", 0.5, 30.0, 0.5),
    ParameterSpec("advanced", "stability", "maximum_radius_jitter_px",
                  "最大半径抖动", 0.5, 30.0, 0.5),
)


@dataclass(frozen=True)
class AutoDiagnosisResult:
    """一次自动搜索的结果；失败时 document 保持输入值。"""

    success: bool
    document: dict
    candidate: DebugCircleCandidate | None
    message: str
    accumulator_threshold: float


def _accumulator_attempts(current: float) -> tuple[float, ...]:
    """从当前严格度开始，只逐步尝试更宽松的阈值。"""

    values = [float(current)]
    for value in (30.0, 26.0, 22.0, 18.0):
        if value < float(current) and value not in values:
            values.append(value)
    return tuple(values)


def auto_diagnose_frame(frame: np.ndarray, document: dict) -> AutoDiagnosisResult:
    """宽范围寻找最高质量圆，并只建议半径范围和必要 Hough 阈值。"""

    original = copy.deepcopy(document)
    current_accumulator = float(
        original["coarse_search"]["accumulator_threshold"]
    )
    failure_details = []
    for accumulator in _accumulator_attempts(current_accumulator):
        trial = copy.deepcopy(original)
        trial["coarse_search"]["minimum_radius_px"] = 120.0
        trial["coarse_search"]["maximum_radius_px"] = 700.0
        trial["coarse_search"]["accumulator_threshold"] = accumulator
        trial["coarse_search"]["maximum_candidates"] = max(
            8, int(trial["coarse_search"]["maximum_candidates"])
        )
        # 这些宽松值仅用于找候选，绝不会写回用户配置。
        trial["refinement"]["radial_search_half_width_px"] = max(
            90.0,
            float(trial["refinement"]["radial_search_half_width_px"]),
        )
        trial["refinement"]["minimum_visible_arc_deg"] = 40.0
        trial["refinement"]["maximum_fit_rms_px"] = 8.0
        trial["refinement"]["local_recheck_rms_above_px"] = min(
            6.0, float(trial["refinement"]["maximum_fit_rms_px"])
        )
        try:
            debug = TurntableDetector(
                turntable_config_from_document(trial)
            ).inspect_frame(frame)
        except (TurntableDetectionError, cv2.error) as error:
            failure_details.append(str(error))
            continue
        if debug.selected_index is None:
            failure_details.append(debug.detail)
            continue

        candidate = debug.candidates[debug.selected_index]
        radius = float(candidate.fit.radius_px)
        proposed = copy.deepcopy(original)
        proposed["coarse_search"]["minimum_radius_px"] = float(
            max(50, math.floor(radius - 60.0))
        )
        proposed["coarse_search"]["maximum_radius_px"] = float(
            min(900, math.ceil(radius + 60.0))
        )
        if accumulator < current_accumulator:
            proposed["coarse_search"]["accumulator_threshold"] = accumulator
        # 最后用正式严格校验检查建议值；任何异常都不污染当前滑条。
        turntable_config_from_document(proposed)
        return AutoDiagnosisResult(
            True, proposed, candidate,
            (
                f"自动找到外圆：中心({candidate.fit.center_x_px:.1f},"
                f"{candidate.fit.center_y_px:.1f})，半径"
                f"{candidate.fit.radius_px:.1f}px"
            ),
            accumulator,
        )
    detail = failure_details[-1] if failure_details else "没有产生诊断结果"
    return AutoDiagnosisResult(
        False, original, None, f"自动诊断失败：{detail}",
        current_accumulator,
    )


class ParameterPanel:
    """维护两页自绘滑条，并把鼠标动作映射到配置字典。"""

    ROW_START = 126
    # 高级页有13项，40px行距可确保最后一条滑杆位于快捷键上方。
    ROW_HEIGHT = 40
    TRACK_LEFT = 18
    TRACK_RIGHT = PANEL_WIDTH - 18

    def __init__(self, document: dict):
        self.document = copy.deepcopy(document)
        self.page = "common"
        self.dragging_index: int | None = None
        self.selected_index: int | None = None
        self.dirty = False
        self.slider_changed = False

    @property
    def specs(self) -> tuple[ParameterSpec, ...]:
        return tuple(spec for spec in PARAMETER_SPECS if spec.page == self.page)

    def replace_document(self, document: dict) -> None:
        turntable_config_from_document(document)
        self.document = copy.deepcopy(document)
        self.dragging_index = None
        self.selected_index = None
        self.dirty = True
        self.slider_changed = False

    def toggle_page(self) -> None:
        self.page = "advanced" if self.page == "common" else "common"
        self.dragging_index = None
        self.selected_index = None

    def value(self, spec: ParameterSpec):
        return self.document[spec.group][spec.key]

    def set_value(self, spec: ParameterSpec, value: float) -> None:
        clipped = min(spec.maximum, max(spec.minimum, float(value)))
        steps = round((clipped - spec.minimum) / spec.step)
        quantized = spec.minimum + steps * spec.step
        quantized = int(round(quantized)) if spec.integer else float(quantized)
        self.document[spec.group][spec.key] = quantized

        coarse = self.document["coarse_search"]
        if spec.key == "minimum_radius_px" and (
            coarse["minimum_radius_px"] >= coarse["maximum_radius_px"]
        ):
            coarse["maximum_radius_px"] = float(
                min(900, coarse["minimum_radius_px"] + 1)
            )
        elif spec.key == "maximum_radius_px" and (
            coarse["maximum_radius_px"] <= coarse["minimum_radius_px"]
        ):
            coarse["minimum_radius_px"] = float(
                max(50, coarse["maximum_radius_px"] - 1)
            )

        refinement = self.document["refinement"]
        if spec.key == "maximum_fit_rms_px" and (
            refinement["local_recheck_rms_above_px"]
            > refinement["maximum_fit_rms_px"]
        ):
            refinement["local_recheck_rms_above_px"] = float(
                refinement["maximum_fit_rms_px"]
            )
        elif spec.key == "local_recheck_rms_above_px" and (
            refinement["local_recheck_rms_above_px"]
            > refinement["maximum_fit_rms_px"]
        ):
            refinement["maximum_fit_rms_px"] = float(
                refinement["local_recheck_rms_above_px"]
            )
        turntable_config_from_document(self.document)
        self.dirty = True
        self.slider_changed = True

    def set_fraction(self, spec: ParameterSpec, fraction: float) -> None:
        self.set_value(
            spec,
            spec.minimum + min(1.0, max(0.0, fraction))
            * (spec.maximum - spec.minimum),
        )

    def mouse_callback(self, event, x_value, y_value, _flags, _parameter):
        # 即使鼠标在左侧预览区松开，也必须结束右侧滑条的拖动状态。
        if event == cv2.EVENT_LBUTTONUP:
            self.dragging_index = None
            return
        if x_value < FRAME_WIDTH:
            return
        panel_x = x_value - FRAME_WIDTH
        row_index = int((y_value - self.ROW_START) // self.ROW_HEIGHT)
        specs = self.specs
        if event == cv2.EVENT_LBUTTONDOWN:
            if 0 <= row_index < len(specs):
                self.dragging_index = row_index
                self.selected_index = row_index
                fraction = (
                    (panel_x - self.TRACK_LEFT)
                    / (self.TRACK_RIGHT - self.TRACK_LEFT)
                )
                self.set_fraction(specs[row_index], fraction)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging_index is not None:
            fraction = (
                (panel_x - self.TRACK_LEFT)
                / (self.TRACK_RIGHT - self.TRACK_LEFT)
            )
            self.set_fraction(specs[self.dragging_index], fraction)


class ChinesePanelRenderer:
    """用 Pillow 在小尺寸侧栏中绘制中文，避免转换整张摄像头图像。"""

    def __init__(self):
        self.font_path = next((path for path in FONT_PATHS if path.is_file()), None)
        self.available = bool(
            Image is not None and ImageDraw is not None
            and ImageFont is not None and self.font_path is not None
        )
        if self.available:
            self.title_font = ImageFont.truetype(str(self.font_path), 22)
            self.text_font = ImageFont.truetype(str(self.font_path), 16)
            self.small_font = ImageFont.truetype(str(self.font_path), 13)

    @staticmethod
    def _wrap(text: str, width: int = 25) -> list[str]:
        value = str(text)
        return [value[index:index + width]
                for index in range(0, len(value), width)] or [""]

    def render(
        self, panel: ParameterPanel, result, debug: TurntableDebugResult,
        message: str, measured_fps: float, frozen: bool, show_points: bool,
    ) -> np.ndarray:
        if not self.available:
            return self._render_ascii(
                panel, result, debug, message, measured_fps, frozen, show_points
            )
        image = Image.new("RGB", (PANEL_WIDTH, FRAME_HEIGHT), (25, 27, 31))
        draw = ImageDraw.Draw(image)
        page_name = "常用参数" if panel.page == "common" else "高级参数"
        draw.text((14, 8), f"转盘参数设置 · {page_name}",
                  font=self.title_font, fill=(245, 245, 245))
        state = "稳定" if result.stable else (
            "已检出" if result.detected else "未检出"
        )
        draw.text(
            (14, 40),
            f"状态：{state}  候选：{len(debug.candidates)}  FPS：{measured_fps:.1f}",
            font=self.text_font, fill=(80, 240, 120) if result.detected
            else (255, 110, 90),
        )
        if result.detected:
            summary = (
                f"半径 {result.radius_px:.1f}  圆弧 {result.visible_arc_deg:.0f}°  "
                f"RMS {result.fit_rms_px:.2f}"
            )
        else:
            summary = debug.detail
        draw.text((14, 66), summary[:38], font=self.small_font,
                  fill=(220, 220, 220))
        for line_index, line in enumerate(self._wrap(message, 29)[:2]):
            draw.text((14, 86 + line_index * 17), line,
                      font=self.small_font, fill=(255, 205, 70))

        for index, spec in enumerate(panel.specs):
            y_value = panel.ROW_START + index * panel.ROW_HEIGHT
            value = float(panel.value(spec))
            fraction = (value - spec.minimum) / (spec.maximum - spec.minimum)
            selected = index == panel.selected_index
            label_color = (255, 220, 90) if selected else (230, 230, 230)
            draw.text(
                (panel.TRACK_LEFT, y_value),
                f"{spec.label}   {spec.format_value(value)}",
                font=self.small_font, fill=label_color,
            )
            track_y = y_value + 26
            draw.rectangle(
                (panel.TRACK_LEFT, track_y,
                 panel.TRACK_RIGHT, track_y + 6), fill=(65, 68, 76)
            )
            knob_x = int(round(
                panel.TRACK_LEFT
                + fraction * (panel.TRACK_RIGHT - panel.TRACK_LEFT)
            ))
            draw.rectangle(
                (panel.TRACK_LEFT, track_y, knob_x, track_y + 6),
                fill=(40, 170, 255),
            )
            draw.ellipse(
                (knob_x - 6, track_y - 4, knob_x + 6, track_y + 10),
                fill=(255, 255, 255),
            )

        flags = f"画面：{'冻结' if frozen else '实时'}  采样点：{'开' if show_points else '关'}"
        draw.text((14, 651), flags, font=self.small_font, fill=(180, 210, 255))
        draw.text((14, 674), "A自动  Tab换页  Space冻结  P采样点",
                  font=self.small_font, fill=(205, 205, 205))
        draw.text((14, 697), "M掩膜  S保存  R重载  D默认  Q退出",
                  font=self.small_font, fill=(205, 205, 205))
        rgb = np.asarray(image)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _render_ascii(
        self, panel, result, debug, message, measured_fps, frozen, show_points
    ):
        output = np.full((FRAME_HEIGHT, PANEL_WIDTH, 3), 26, dtype=np.uint8)
        lines = [
            "Turntable Settings",
            f"Page={panel.page} State={'STABLE' if result.stable else 'SEARCH'}",
            f"Candidates={len(debug.candidates)} FPS={measured_fps:.1f}",
            str(message)[:48],
        ]
        for index, line in enumerate(lines):
            cv2.putText(output, line, (12, 28 + index * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (230, 230, 230), 1, cv2.LINE_AA)
        for index, spec in enumerate(panel.specs):
            y_value = panel.ROW_START + index * panel.ROW_HEIGHT
            value = float(panel.value(spec))
            fraction = (value - spec.minimum) / (spec.maximum - spec.minimum)
            cv2.putText(
                output, f"{spec.key}: {spec.format_value(value)}",
                (12, y_value + 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (220, 220, 220), 1, cv2.LINE_AA,
            )
            knob_x = int(panel.TRACK_LEFT + fraction * (
                panel.TRACK_RIGHT - panel.TRACK_LEFT
            ))
            cv2.line(output, (panel.TRACK_LEFT, y_value + 28),
                     (panel.TRACK_RIGHT, y_value + 28), (80, 80, 80), 5)
            cv2.circle(output, (knob_x, y_value + 28), 7, (255, 255, 255), -1)
        return output


def _read_matrix(storage: cv2.FileStorage, name: str):
    matrix = storage.getNode(name).mat()
    if matrix is None or matrix.size == 0:
        raise TurntableSettingsError(f"标定文件缺少矩阵：{name}")
    return matrix


def load_camera_calibration(path: Path):
    resolved = path.expanduser().resolve()
    storage = cv2.FileStorage(str(resolved), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise TurntableSettingsError(f"无法打开摄像头标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = _read_matrix(storage, "camera_matrix")
        distortion = _read_matrix(storage, "distortion_coefficients")
        new_camera_matrix = _read_matrix(storage, "new_camera_matrix")
    finally:
        storage.release()
    if (width, height) != (FRAME_WIDTH, FRAME_HEIGHT):
        raise TurntableSettingsError(
            f"工具要求{FRAME_WIDTH}x{FRAME_HEIGHT}标定，"
            f"当前为{width}x{height}"
        )
    return (width, height), camera_matrix, distortion, new_camera_matrix


def open_camera(device: str, fps: float):
    return open_project_camera(device, 1280, 960, fps)


def _best_display_candidate(debug: TurntableDebugResult):
    if debug.selected_index is not None:
        return debug.candidates[debug.selected_index]
    usable = [
        candidate for candidate in debug.candidates
        if candidate.fit.radius_px > 0.0
        and math.isfinite(candidate.fit.fit_rms_px)
    ]
    if not usable:
        return None
    return max(
        usable,
        key=lambda item: (
            item.fit.visible_arc_deg,
            -item.fit.fit_rms_px,
        ),
    )


def draw_debug_overlay(
    frame: np.ndarray, debug: TurntableDebugResult, show_points: bool
) -> np.ndarray:
    """黄色画粗圆、绿色画通过圆、红色画被拒绝圆。"""

    best = _best_display_candidate(debug)
    for index, candidate in enumerate(debug.candidates, start=1):
        seed_x, seed_y, seed_radius = candidate.seed
        cv2.circle(
            frame, (int(round(seed_x)), int(round(seed_y))),
            int(round(seed_radius)), (0, 220, 255), 1, cv2.LINE_8,
        )
        fit = candidate.fit
        if fit.radius_px > 0.0 and math.isfinite(fit.radius_px):
            color = (40, 255, 40) if fit.detected else (30, 30, 255)
            thickness = 3 if candidate is best else 1
            center = (
                int(round(fit.center_x_px)), int(round(fit.center_y_px))
            )
            cv2.circle(frame, center, int(round(fit.radius_px)),
                       color, thickness, cv2.LINE_8)
            cv2.putText(
                frame, f"C{index}", center, cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2, cv2.LINE_AA,
            )
    if show_points and best is not None:
        for point in best.sample_points[::2]:
            cv2.circle(
                frame, (int(round(point[0])), int(round(point[1]))),
                2, (255, 0, 255), -1,
            )
    return frame


def make_turntable_edge_mask(
    frame: np.ndarray, document: dict
) -> np.ndarray:
    """生成与 Hough 粗搜索预处理一致的二值边缘掩膜。

    OpenCV ``HoughCircles`` 会在内部使用 ``param1`` 作为 Canny 高阈值，
    低阈值约为它的一半。这里用相同灰度、模糊、缩放和阈值显式生成
    掩膜，再放大回 1280×960，便于观察外圆边缘是否连续以及背景纹理
    是否过多。返回值保持单通道 ``uint8``，可供测试或后续工具复用。
    """

    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        raise TurntableSettingsError("边缘掩膜要求BGR三通道图像")
    config = turntable_config_from_document(document)
    small = cv2.resize(
        frame, None, fx=config.image_scale, fy=config.image_scale,
        interpolation=cv2.INTER_AREA,
    )
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 1.5)
    high = max(1.0, float(config.canny_threshold))
    edges = cv2.Canny(
        gray, high * 0.5, high, apertureSize=3, L2gradient=True
    )
    return cv2.resize(
        edges, (frame.shape[1], frame.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )


def draw_mask_preview(
    mask: np.ndarray,
    document: dict,
    debug: TurntableDebugResult | None = None,
) -> np.ndarray:
    """在二值边缘上叠加 Hough 粗圆和当前通过的精修圆。"""

    config = turntable_config_from_document(document)
    preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if debug is not None:
        for candidate in debug.candidates:
            center_x, center_y, radius = candidate.seed
            cv2.circle(
                preview,
                (int(round(center_x)), int(round(center_y))),
                int(round(radius)),
                (0, 220, 255),
                3,
                cv2.LINE_AA,
            )
        if debug.selected_index is not None:
            fit = debug.candidates[debug.selected_index].fit
            cv2.circle(
                preview,
                (int(round(fit.center_x_px)), int(round(fit.center_y_px))),
                int(round(fit.radius_px)),
                (40, 255, 40),
                3,
                cv2.LINE_AA,
            )
    edge_ratio = 100.0 * float(np.count_nonzero(mask)) / max(1, mask.size)
    candidate_count = 0 if debug is None else len(debug.candidates)
    cv2.rectangle(preview, (8, 8), (890, 52), (0, 0, 0), -1)
    cv2.putText(
        preview,
        (
            f"Hough Edge Mask  Scale={config.image_scale:.2f}  "
            f"Canny={config.canny_threshold:.0f}  Edge={edge_ratio:.2f}%  "
            f"Circles={candidate_count}"
        ),
        (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.68,
        (0, 220, 255), 2, cv2.LINE_AA,
    )
    return preview


def open_mask_window(window_name: str) -> None:
    """创建不会被参数主窗口完全遮住的掩膜窗口。

    GNOME 会在点击滑条时把主窗口提到最前面。两个 OpenCV 窗口若都由
    窗口管理器居中，掩膜窗口会被 1700x960 的主窗口整个盖住，看起来
    就像已经退出。这里把掩膜窗口缩小后放到屏幕下方并保持置顶。
    """

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 480, 270)
    cv2.moveWindow(window_name, 10, 755)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="可视化设置转盘外圆识别参数")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--undistort", action="store_true", help="提供自己的有效内参后开启；默认原图")
    return parser.parse_args()


def run(args: argparse.Namespace) -> None:
    document = load_turntable_detector_document(args.config)
    panel = ParameterPanel(document)
    detector = TurntableDetector(turntable_config_from_document(document))
    maps = calibration_maps(args.calibration, args.undistort)
    camera = open_camera(args.device, args.fps)
    renderer = ChinesePanelRenderer()
    window_name = "Turntable Parameter Settings"
    mask_window_name = "Turntable Edge Mask"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, CANVAS_WIDTH, FRAME_HEIGHT)
    cv2.moveWindow(window_name, 0, 0)
    cv2.setMouseCallback(window_name, panel.mouse_callback)
    open_mask_window(mask_window_name)

    frozen = False
    frozen_frame = None
    show_points = False
    show_mask = True
    latest_frame = None
    debug = TurntableDebugResult((), None, 0.0, "等待第一帧")
    message = "按 A 自动诊断，或拖动右侧滑条"
    frame_times = []
    measured_fps = 0.0
    result = None
    mask_preview = None
    mask_updated_s = 0.0
    try:
        while True:
            if not frozen or frozen_frame is None:
                ok, raw = camera.read()
                if not ok or raw is None:
                    raise TurntableSettingsError("设置过程中摄像头读取失败")
                latest_frame = (cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR) if maps else raw.copy())
                if frozen:
                    frozen_frame = latest_frame.copy()
            source = frozen_frame if frozen and frozen_frame is not None else latest_frame
            if source is None:
                continue

            configuration_changed = panel.dirty
            if configuration_changed:
                detector = TurntableDetector(
                    turntable_config_from_document(panel.document)
                )
                panel.dirty = False
                if panel.slider_changed:
                    message = "参数已应用到预览，检测历史已清空"
                panel.slider_changed = False

            # 冻结画面不能伪装成多张新鲜帧去填满稳定窗口。参数发生变化
            # 时只在冻结帧上重新计算一次，此后保持该单帧结果。
            if not frozen or result is None or configuration_changed:
                # 调参页面必须展示 Hough 候选；合并接口只做一次粗搜和
                # 精修，同时更新稳定结果，避免 detect/inspect 重复计算。
                result, debug = detector.detect_with_debug(source)
            preview = source.copy()
            draw_debug_overlay(preview, debug, show_points)
            draw_turntable_detection(preview, result)

            now = time.monotonic()
            frame_times.append(now)
            if len(frame_times) > 45:
                del frame_times[0]
            if len(frame_times) >= 2:
                elapsed = frame_times[-1] - frame_times[0]
                if elapsed > 0.0:
                    measured_fps = (len(frame_times) - 1) / elapsed
            side = renderer.render(
                panel, result, debug, message, measured_fps,
                frozen, show_points,
            )
            canvas = np.hstack((preview, side))
            cv2.imshow(window_name, canvas)
            if show_mask:
                # 掩膜仅用于肉眼观察，10 FPS 已足够流畅。降低刷新频率
                # 可以把更多 CPU 留给每帧 Hough 搜索和摄像头显示。
                if (
                    mask_preview is None
                    or configuration_changed
                    or now - mask_updated_s >= MASK_REFRESH_INTERVAL_S
                ):
                    edge_mask = make_turntable_edge_mask(
                        source, panel.document
                    )
                    mask_preview = draw_mask_preview(
                        edge_mask, panel.document, debug
                    )
                    mask_updated_s = now
                cv2.imshow(mask_window_name, mask_preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == 9:  # Tab
                panel.toggle_page()
            elif key == 32:  # Space
                frozen = not frozen
                frozen_frame = source.copy() if frozen else None
                message = "画面已冻结" if frozen else "已恢复实时画面"
            elif key in (ord("p"), ord("P")):
                show_points = not show_points
                message = "已显示径向采样点" if show_points else "已隐藏采样点"
            elif key in (ord("m"), ord("M")):
                show_mask = not show_mask
                if show_mask:
                    open_mask_window(mask_window_name)
                    message = "已显示Hough边缘掩膜窗口"
                else:
                    cv2.destroyWindow(mask_window_name)
                    message = "已隐藏Hough边缘掩膜窗口"
            elif key in (ord("a"), ord("A")):
                diagnosis = auto_diagnose_frame(source, panel.document)
                message = diagnosis.message
                if diagnosis.success:
                    panel.replace_document(diagnosis.document)
            elif key in (ord("r"), ord("R")):
                panel.replace_document(
                    load_turntable_detector_document(args.config)
                )
                message = "已重新读取磁盘中最后保存的参数"
            elif key in (ord("d"), ord("D")):
                panel.replace_document(
                    build_default_turntable_detector_document()
                )
                message = "已恢复推荐默认值；按 S 才会写入磁盘"
            elif key in (ord("s"), ord("S")):
                # 按用户选择始终允许保存；未稳定只做醒目提醒。
                save_turntable_detector_config(args.config, panel.document)
                if result.stable:
                    message = "参数已保存，请重启 main.py"
                else:
                    message = "警告：当前尚未稳定；参数仍已保存，请重启main.py"
                print(message)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main() -> int:
    args = parse_args()
    try:
        run(args)
    except (TurntableSettingsError, TurntableDetectionError, cv2.error) as error:
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
