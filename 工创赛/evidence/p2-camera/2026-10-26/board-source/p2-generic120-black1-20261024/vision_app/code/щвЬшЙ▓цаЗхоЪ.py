#!/usr/bin/env python3
"""独立的六色 OpenCV HSV 实时标定工具。

运行方式::

    python3 code/颜色标定.py

数字键 1~6 选择颜色，随后在物料内部按住鼠标左键拖出采样区域。
同一种颜色可以累积多个区域，工具会自动统计顶面、侧面和阴影的 HSV
分布。窗口下方的六个滑条可以继续人工微调，按 S 才会保存 JSON。

本文件不导入也不修改主视觉程序。后续识别模块可以直接导入
``load_color_config`` 和 ``make_color_mask`` 使用同一份配置。
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable

import cv2
import numpy as np
from project_camera import DEFAULT_DEVICE, open_project_camera, calibration_maps


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "config" / "decxin_camera_unconfigured.yaml"
DEFAULT_CONFIG = SCRIPT_DIR / "config" / "material_color_thresholds.json"
WINDOW_NAME = "HSV Color Calibrator"
SLIDER_NAMES = (
    "H_LOW",
    "H_HIGH",
    "S_LOW",
    "S_HIGH",
    "V_LOW",
    "V_HIGH",
)
SLIDER_MAXIMA = (179, 179, 255, 255, 255, 255)
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960
PANEL_WIDTH = 360
CANVAS_WIDTH = IMAGE_WIDTH + PANEL_WIDTH
SLIDER_X_MIN = IMAGE_WIDTH + 30
SLIDER_X_MAX = CANVAS_WIDTH - 30
SLIDER_Y_POSITIONS = (150, 220, 290, 360, 430, 500)

# 颜色编号与旧视觉移植包保持一致。BGR 只用于界面画框和文字。
COLOR_SPECS = {
    1: ("red", (0, 0, 255)),
    2: ("yellow", (0, 255, 255)),
    3: ("blue", (255, 0, 0)),
    4: ("green", (0, 255, 0)),
    5: ("black", (80, 80, 80)),
    6: ("light_blue", (255, 255, 0)),
}

# 这些预设来自 2026-09-01 当前相机、当前灯光下六件物料的矫正画面。
# reset 操作使用不可变常量重新构造，避免被运行时滑条修改污染。
PRESET_BOUNDS = {
    1: (172, 8, 150, 255, 90, 255),
    2: (20, 32, 170, 255, 120, 255),
    3: (104, 116, 160, 255, 80, 230),
    4: (63, 78, 110, 255, 80, 220),
    5: (0, 179, 0, 130, 0, 60),
    6: (94, 104, 130, 255, 100, 255),
}


class ColorCalibrationError(ValueError):
    """颜色配置、摄像头或用户采样不满足标定要求。"""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ColorCalibrationError(message)


def bounds_to_ranges(bounds: Iterable[int]) -> list[dict]:
    """把六个滑条值转换成一个或两个可供 ``inRange`` 使用的区间。

    Hue 是长度 180 的圆环。当 H_LOW 大于 H_HIGH 时，范围跨越红色的
    179→0 边界，因此拆成 ``0..H_HIGH`` 与 ``H_LOW..179`` 两段。
    """

    values = tuple(int(value) for value in bounds)
    _require(len(values) == 6, "HSV滑条值必须正好包含6个整数")
    h_low, h_high, s_low, s_high, v_low, v_high = values
    _require(0 <= h_low <= 179 and 0 <= h_high <= 179, "Hue必须位于0~179")
    _require(0 <= s_low <= s_high <= 255, "S上下限无效")
    _require(0 <= v_low <= v_high <= 255, "V上下限无效")
    if h_low <= h_high:
        return [{
            "lower": [h_low, s_low, v_low],
            "upper": [h_high, s_high, v_high],
        }]
    return [
        {
            "lower": [0, s_low, v_low],
            "upper": [h_high, s_high, v_high],
        },
        {
            "lower": [h_low, s_low, v_low],
            "upper": [179, s_high, v_high],
        },
    ]


def ranges_to_bounds(ranges: list[dict]) -> tuple[int, ...]:
    """把配置区间恢复为滑条值，并严格拒绝无法表达的组合。"""

    _require(isinstance(ranges, list) and 1 <= len(ranges) <= 2,
             "每种颜色必须包含1或2个HSV区间")

    def triplet(hsv_range: dict, key: str) -> tuple[int, int, int]:
        try:
            values = tuple(int(value) for value in hsv_range[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ColorCalibrationError("HSV区间字段格式无效") from error
        _require(len(values) == 3, "HSV上下限必须正好包含3个整数")
        return values

    if len(ranges) == 1:
        lower = triplet(ranges[0], "lower")
        upper = triplet(ranges[0], "upper")
        return (lower[0], upper[0], lower[1], upper[1], lower[2], upper[2])

    low_hue_part, high_hue_part = ranges
    first_lower = triplet(low_hue_part, "lower")
    first_upper = triplet(low_hue_part, "upper")
    second_lower = triplet(high_hue_part, "lower")
    second_upper = triplet(high_hue_part, "upper")
    _require(first_lower[0] == 0 and second_upper[0] == 179,
             "双HSV区间必须表示跨越Hue边界的范围")
    _require(second_lower[0] > first_upper[0],
             "双HSV区间必须在Hue边界两侧且不能互相重叠")
    _require(first_lower[1:] == second_lower[1:]
             and first_upper[1:] == second_upper[1:],
             "跨Hue边界的两个区间必须使用相同S/V范围")
    return (
        second_lower[0], first_upper[0],
        first_lower[1], first_upper[1],
        first_lower[2], first_upper[2],
    )


def build_default_color_config(
    camera_device: str = "/dev/video0",
    calibration_file: str = "config/decxin_camera_unconfigured.yaml",
) -> dict:
    """用当前六件物料的预设值构造一份全新配置。"""

    colors = []
    for color_id, (name, display_bgr) in COLOR_SPECS.items():
        colors.append({
            "color_id": color_id,
            "name": name,
            "display_bgr": list(display_bgr),
            "ranges": bounds_to_ranges(PRESET_BOUNDS[color_id]),
        })
    return {
        "format_version": 1,
        "color_space": "OpenCV_HSV",
        "camera_device": str(camera_device),
        "image_width": 1280,
        "image_height": 960,
        "calibration_file": str(calibration_file),
        "colors": colors,
    }


def validate_color_config(document: dict) -> dict:
    """严格校验公共配置格式，返回原字典供调用方继续使用。"""

    try:
        _require(isinstance(document, dict), "颜色配置根节点必须是对象")
        _require(int(document["format_version"]) == 1,
                 "不支持的颜色配置版本")
        _require(document["color_space"] == "OpenCV_HSV",
                 "颜色空间必须为OpenCV_HSV")
        _require(int(document["image_width"]) == 1280
                 and int(document["image_height"]) == 960,
                 "颜色配置分辨率必须为1280x960")
        _require(bool(str(document["camera_device"])), "摄像头节点不能为空")
        _require(bool(str(document["calibration_file"])), "标定路径不能为空")
        colors = document["colors"]
        _require(isinstance(colors, list) and len(colors) == 6,
                 "颜色配置必须正好包含6种颜色")
        ids = {int(item["color_id"]) for item in colors}
        _require(ids == set(COLOR_SPECS), "六色编号不完整或重复")
        for item in colors:
            color_id = int(item["color_id"])
            expected_name = COLOR_SPECS[color_id][0]
            _require(item["name"] == expected_name,
                     f"颜色{color_id}名称必须为{expected_name}")
            display_bgr = item["display_bgr"]
            _require(len(display_bgr) == 3 and all(
                0 <= int(value) <= 255 for value in display_bgr
            ), f"{expected_name}显示颜色无效")
            # ranges_to_bounds 同时验证双区间是否是合法的Hue环绕。
            bounds = ranges_to_bounds(item["ranges"])
            bounds_to_ranges(bounds)
            for hsv_range in item["ranges"]:
                lower = tuple(int(value) for value in hsv_range["lower"])
                upper = tuple(int(value) for value in hsv_range["upper"])
                _require(len(lower) == 3 and len(upper) == 3,
                         f"{expected_name} HSV区间长度无效")
                limits = (179, 255, 255)
                _require(all(
                    0 <= lower[index] <= upper[index] <= limits[index]
                    for index in range(3)
                ), f"{expected_name} HSV区间越界")
    except (IndexError, KeyError, TypeError, ValueError) as error:
        if isinstance(error, ColorCalibrationError):
            raise
        raise ColorCalibrationError("颜色配置字段格式无效") from error
    return document


def load_color_config(path: Path | str) -> dict:
    """读取并校验一份六色 HSV JSON 配置。"""

    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except OSError as error:
        raise ColorCalibrationError(f"无法读取颜色配置：{resolved}: {error}") from error
    except json.JSONDecodeError as error:
        raise ColorCalibrationError(f"颜色配置不是有效JSON：{resolved}") from error
    return validate_color_config(document)


def save_color_config(path: Path | str, document: dict) -> None:
    """在目标目录中原子保存配置，避免程序中断留下半个文件。"""

    validate_color_config(document)
    resolved = Path(path).expanduser().resolve()
    _require(resolved.parent.is_dir(), f"配置目录不存在：{resolved.parent}")
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=resolved.parent,
            prefix=f".{resolved.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temporary_name = temporary.name
            json.dump(document, temporary, ensure_ascii=False, indent=2)
            temporary.write("\n")
            temporary.flush()
            os.fsync(temporary.fileno())
        # NamedTemporaryFile 默认权限通常是0600。配置需要供同机其他
        # 视觉进程读取，因此在替换前统一设置为普通源码配置权限。
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, resolved)
    except OSError as error:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink(missing_ok=True)
            except OSError:
                pass
        raise ColorCalibrationError(f"无法保存颜色配置：{resolved}: {error}") from error


def _find_color(document: dict, color: int | str) -> dict:
    if isinstance(color, str):
        matches = [item for item in document["colors"] if item["name"] == color]
    else:
        matches = [
            item for item in document["colors"]
            if int(item["color_id"]) == int(color)
        ]
    if len(matches) != 1:
        raise ColorCalibrationError(f"未知颜色：{color}")
    return matches[0]


def _mask_from_hsv(hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for hsv_range in ranges:
        lower = np.asarray(hsv_range["lower"], dtype=np.uint8)
        upper = np.asarray(hsv_range["upper"], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    return mask


def make_color_mask(
    frame: np.ndarray, color: int | str, config: dict
) -> np.ndarray:
    """使用公共配置从 BGR 图像生成指定颜色的单通道蒙版。"""

    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             "颜色识别要求BGR三通道图像")
    item = _find_color(config, color)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return _mask_from_hsv(hsv, item["ranges"])


def make_all_color_masks(frame: np.ndarray, config: dict) -> dict[int, np.ndarray]:
    """只做一次 BGR→HSV 转换并生成全部六色蒙版。

    后续实时识别需要同时处理六种物料时应优先调用此接口，避免连续
    六次调用 ``make_color_mask`` 重复转换整张1280x960画面。
    """

    validate_color_config(config)
    _require(frame is not None and frame.ndim == 3 and frame.shape[2] == 3,
             "颜色识别要求BGR三通道图像")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {
        int(item["color_id"]): _mask_from_hsv(hsv, item["ranges"])
        for item in config["colors"]
    }


def estimate_hsv_bounds(
    sample_arrays: Iterable[np.ndarray], color_id: int
) -> tuple[int, ...]:
    """根据同色多个 ROI 的 HSV 像素稳健估计六个滑条值。"""

    arrays = [np.asarray(item, dtype=np.float64).reshape(-1, 3)
              for item in sample_arrays if np.asarray(item).size]
    _require(bool(arrays), "没有可用于统计的HSV样本")
    values = np.concatenate(arrays, axis=0)
    _require(len(values) >= 100, "有效HSV样本至少需要100个像素")

    saturation = values[:, 1]
    value = values[:, 2]
    if int(color_id) == 5:
        # 黑色的Hue不稳定且没有分类意义，使用全Hue并重点约束亮度。
        s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 20)
        v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
        return (0, 179, 0, s_high, 0, v_high)

    hue = values[:, 0]
    angles = hue * (2.0 * math.pi / 180.0)
    mean_angle = math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    center_hue = (mean_angle * 180.0 / (2.0 * math.pi)) % 180.0
    # 把Hue展开到圆形均值附近，再做普通百分位，红色不会被错误扩成全环。
    delta = ((hue - center_hue + 90.0) % 180.0) - 90.0
    h_low_unwrapped = math.floor(np.percentile(delta, 2)) + center_hue - 3
    h_high_unwrapped = math.ceil(np.percentile(delta, 98)) + center_hue + 3
    if h_high_unwrapped - h_low_unwrapped >= 179.0:
        h_low, h_high = 0, 179
    else:
        h_low = int(math.floor(h_low_unwrapped)) % 180
        h_high = int(math.ceil(h_high_unwrapped)) % 180

    s_low = max(0, int(math.floor(np.percentile(saturation, 2))) - 15)
    s_high = min(255, int(math.ceil(np.percentile(saturation, 98))) + 15)
    v_low = max(0, int(math.floor(np.percentile(value, 2))) - 20)
    v_high = min(255, int(math.ceil(np.percentile(value, 98))) + 20)
    return (h_low, h_high, s_low, s_high, v_low, v_high)


class HSVCalibrationSession:
    """不依赖窗口的多颜色采样、撤销、重置和阈值状态。"""

    MAX_SAMPLES_PER_COLOR = 20
    MAX_PIXELS_PER_SAMPLE = 50000

    def __init__(self, document: dict):
        self.document = deepcopy(validate_color_config(document))
        self.samples = {color_id: [] for color_id in COLOR_SPECS}
        self.baseline_bounds = {color_id: None for color_id in COLOR_SPECS}
        self.dirty = False

    def _item(self, color_id: int) -> dict:
        return _find_color(self.document, int(color_id))

    def bounds(self, color_id: int) -> tuple[int, ...]:
        return ranges_to_bounds(self._item(color_id)["ranges"])

    def set_bounds(self, color_id: int, bounds: Iterable[int]) -> None:
        ranges = bounds_to_ranges(bounds)
        if ranges != self._item(color_id)["ranges"]:
            self._item(color_id)["ranges"] = ranges
            self.dirty = True

    def _recalculate(self, color_id: int) -> None:
        self.set_bounds(
            color_id,
            estimate_hsv_bounds(self.samples[color_id], color_id),
        )

    def add_sample(self, color_id: int, hsv_pixels: np.ndarray) -> None:
        color_id = int(color_id)
        _require(len(self.samples[color_id]) < self.MAX_SAMPLES_PER_COLOR,
                 f"每种颜色最多{self.MAX_SAMPLES_PER_COLOR}个样本")
        pixels = np.asarray(hsv_pixels, dtype=np.uint8).reshape(-1, 3)
        _require(len(pixels) >= 100, "框选区域太小，有效像素不足100")
        if len(pixels) > self.MAX_PIXELS_PER_SAMPLE:
            step = int(math.ceil(len(pixels) / self.MAX_PIXELS_PER_SAMPLE))
            pixels = pixels[::step]
        if not self.samples[color_id]:
            self.baseline_bounds[color_id] = self.bounds(color_id)
        self.samples[color_id].append(pixels)
        self._recalculate(color_id)

    def undo_sample(self, color_id: int) -> bool:
        color_id = int(color_id)
        if not self.samples[color_id]:
            return False
        self.samples[color_id].pop()
        if self.samples[color_id]:
            self._recalculate(color_id)
        else:
            baseline = self.baseline_bounds[color_id]
            if baseline is not None:
                self.set_bounds(color_id, baseline)
            self.baseline_bounds[color_id] = None
        return True

    def clear_samples(self, color_id: int) -> None:
        color_id = int(color_id)
        if self.samples[color_id]:
            baseline = self.baseline_bounds[color_id]
            self.samples[color_id].clear()
            if baseline is not None:
                self.set_bounds(color_id, baseline)
        self.baseline_bounds[color_id] = None

    def reset_to_preset(self, color_id: int) -> None:
        color_id = int(color_id)
        self.samples[color_id].clear()
        self.baseline_bounds[color_id] = None
        self.set_bounds(color_id, PRESET_BOUNDS[color_id])

    def mark_saved(self) -> None:
        self.dirty = False


def _read_calibration(path: Path):
    resolved = path.expanduser().resolve()
    _require(resolved.is_file(), f"标定文件不存在：{resolved}")
    storage = cv2.FileStorage(str(resolved), cv2.FILE_STORAGE_READ)
    _require(storage.isOpened(), f"无法读取标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
        new_camera_matrix = storage.getNode("new_camera_matrix").mat()
    finally:
        storage.release()
    _require((width, height) == (1280, 960), "标定分辨率必须为1280x960")
    _require(all(item is not None and item.size for item in (
        camera_matrix, distortion, new_camera_matrix
    )), "标定文件缺少相机矩阵")
    return (width, height), camera_matrix, distortion, new_camera_matrix


def _open_camera(device: str, fps: float):
    return open_project_camera(device, 1280, 960, fps)


class ColorCalibratorWindow:
    """把标定会话连接到 OpenCV 摄像头窗口、鼠标和滑条。"""

    def __init__(self, session: HSVCalibrationSession, config_path: Path):
        self.session = session
        self.config_path = config_path.expanduser().resolve()
        self.selected_color_id = 1
        self.current_frame = None
        self.frozen_frame = None
        self.drag_start = None
        self.drag_current = None
        self.active_slider_index = None
        self.sample_rectangles = {color_id: [] for color_id in COLOR_SPECS}
        self.status_message = "Select color 1-6, then drag inside material"
        self.slider_values = self.session.bounds(self.selected_color_id)

    def create_window(self) -> None:
        # 相机画面保持原始1280x960，额外控制栏拼接在右侧。这样鼠标
        # 框选坐标与矫正图像像素一一对应，滑条也不会占据窗口顶部。
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self._on_mouse)

    def _sync_trackbars(self) -> None:
        self.slider_values = self.session.bounds(self.selected_color_id)

    def _read_trackbars(self) -> tuple[int, ...]:
        return tuple(int(value) for value in self.slider_values)

    def _trackbars_valid(self, values: tuple[int, ...]) -> bool:
        return values[2] <= values[3] and values[4] <= values[5]

    def update_manual_thresholds(self) -> bool:
        values = self._read_trackbars()
        self.slider_values = values
        if not self._trackbars_valid(values):
            self.status_message = "INVALID: S_LOW<=S_HIGH and V_LOW<=V_HIGH"
            return False
        self.session.set_bounds(self.selected_color_id, values)
        return True

    def _clamp_point(self, x: int, y: int) -> tuple[int, int]:
        return (
            max(0, min(IMAGE_WIDTH - 1, int(x))),
            max(0, min(IMAGE_HEIGHT - 1, int(y))),
        )

    def _slider_index_at(self, x: int, y: int):
        """返回鼠标命中的右侧滑条编号，未命中时返回 ``None``。"""

        if not SLIDER_X_MIN - 12 <= x <= SLIDER_X_MAX + 12:
            return None
        for index, slider_y in enumerate(SLIDER_Y_POSITIONS):
            if abs(int(y) - slider_y) <= 22:
                return index
        return None

    def _set_slider_from_x(self, slider_index: int, x: int) -> None:
        """把右栏中的水平坐标映射成对应 H/S/V 整数值。"""

        x = max(SLIDER_X_MIN, min(SLIDER_X_MAX, int(x)))
        ratio = (x - SLIDER_X_MIN) / (SLIDER_X_MAX - SLIDER_X_MIN)
        maximum = SLIDER_MAXIMA[slider_index]
        values = list(self.slider_values)
        values[slider_index] = int(round(ratio * maximum))
        self.slider_values = tuple(values)
        if self.update_manual_thresholds():
            self.status_message = (
                f"{SLIDER_NAMES[slider_index]}={values[slider_index]}"
            )

    def _normalised_rectangle(self):
        if self.drag_start is None or self.drag_current is None:
            return None
        x1, y1 = self.drag_start
        x2, y2 = self.drag_current
        return min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)

    def _on_mouse(self, event, x, y, _flags, _userdata) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            slider_index = self._slider_index_at(x, y)
            if slider_index is not None:
                self.active_slider_index = slider_index
                self._set_slider_from_x(slider_index, x)
                return
        if self.active_slider_index is not None:
            if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONUP):
                self._set_slider_from_x(self.active_slider_index, x)
            if event == cv2.EVENT_LBUTTONUP:
                self.active_slider_index = None
            return

        # 右侧控制栏除滑条以外的区域不参与图像框选。
        if x >= IMAGE_WIDTH and self.drag_start is None:
            return
        point = self._clamp_point(x, y)
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            self.frozen_frame = self.current_frame.copy()
            self.drag_start = point
            self.drag_current = point
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_current = point
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_current = point
            rectangle = self._normalised_rectangle()
            try:
                self._accept_rectangle(rectangle)
            except ColorCalibrationError as error:
                self.status_message = f"Sample rejected: {error}"
            finally:
                self.drag_start = None
                self.drag_current = None
                self.frozen_frame = None

    def _accept_rectangle(self, rectangle) -> None:
        x1, y1, x2, y2 = rectangle
        _require(x2 - x1 >= 10 and y2 - y1 >= 10,
                 "ROI width and height must be at least 10px")
        roi = self.frozen_frame[y1:y2, x1:x2]
        hsv_pixels = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        self.session.add_sample(self.selected_color_id, hsv_pixels)
        self.sample_rectangles[self.selected_color_id].append(rectangle)
        self._sync_trackbars()
        count = len(self.session.samples[self.selected_color_id])
        self.status_message = f"Sample accepted: {count}/20"

    def select_color(self, color_id: int) -> None:
        self.selected_color_id = int(color_id)
        self._sync_trackbars()
        name = COLOR_SPECS[self.selected_color_id][0]
        self.status_message = f"Selected {self.selected_color_id} {name}"

    def undo(self) -> None:
        if self.session.undo_sample(self.selected_color_id):
            self.sample_rectangles[self.selected_color_id].pop()
            self._sync_trackbars()
            self.status_message = "Last sample removed"
        else:
            self.status_message = "No sample to undo"

    def clear(self) -> None:
        self.session.clear_samples(self.selected_color_id)
        self.sample_rectangles[self.selected_color_id].clear()
        self._sync_trackbars()
        self.status_message = "Current color samples cleared"

    def reset(self) -> None:
        self.session.reset_to_preset(self.selected_color_id)
        self.sample_rectangles[self.selected_color_id].clear()
        self._sync_trackbars()
        self.status_message = "Current color restored to preset"

    def save(self, device: str, calibration_path: Path) -> None:
        if not self.update_manual_thresholds():
            raise ColorCalibrationError("滑条上下限无效，配置尚未保存")
        self.session.document["camera_device"] = str(device)
        self.session.document["calibration_file"] = str(
            calibration_path.expanduser().resolve()
        )
        self.session.document["saved_at_utc"] = datetime.now(
            timezone.utc
        ).isoformat()
        save_color_config(self.config_path, self.session.document)
        self.session.mark_saved()
        self.status_message = f"Saved: {self.config_path.name}"

    def _preview_ranges(self) -> list[dict]:
        if not self._trackbars_valid(self.slider_values):
            return []
        return bounds_to_ranges(self.slider_values)

    def render(self, frame: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = _mask_from_hsv(hsv, self._preview_ranges())
        # 未匹配区域只保留25%亮度，匹配像素保持原色，直观看出漏选和串色。
        image_output = (frame.astype(np.float32) * 0.25).astype(np.uint8)
        image_output[mask > 0] = frame[mask > 0]
        color = COLOR_SPECS[self.selected_color_id][1]
        # 黑色本身适合做蒙版标签色，却不适合在黑色信息板上画文字和框。
        interface_color = (
            (220, 220, 220) if self.selected_color_id == 5 else color
        )
        for rectangle in self.sample_rectangles[self.selected_color_id]:
            x1, y1, x2, y2 = rectangle
            cv2.rectangle(
                image_output, (x1, y1), (x2, y2), interface_color, 2
            )
        active_rectangle = self._normalised_rectangle()
        if active_rectangle is not None:
            x1, y1, x2, y2 = active_rectangle
            cv2.rectangle(
                image_output, (x1, y1), (x2, y2), (255, 255, 255), 2
            )

        name = COLOR_SPECS[self.selected_color_id][0].upper()
        h_low, h_high, s_low, s_high, v_low, v_high = self.slider_values
        hue_text = f"H={h_low}..{h_high}" + (
            " WRAP" if h_low > h_high else ""
        )
        state = "DIRTY" if self.session.dirty else "SAVED"
        canvas = np.full(
            (IMAGE_HEIGHT, CANVAS_WIDTH, 3), (26, 26, 26), dtype=np.uint8
        )
        canvas[:, :IMAGE_WIDTH] = image_output
        cv2.line(
            canvas,
            (IMAGE_WIDTH, 0),
            (IMAGE_WIDTH, IMAGE_HEIGHT - 1),
            (90, 90, 90),
            2,
        )

        title_lines = [
            f"Color {self.selected_color_id}: {name}",
            (f"Samples={len(self.session.samples[self.selected_color_id])}/20  "
             f"State={state}"),
            self.status_message[:30],
        ]
        for index, text in enumerate(title_lines):
            cv2.putText(
                canvas,
                text,
                (IMAGE_WIDTH + 18, 30 + index * 31),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255) if index else interface_color,
                2,
                cv2.LINE_AA,
            )

        for index, (slider_name, value, maximum, slider_y) in enumerate(zip(
            SLIDER_NAMES,
            self.slider_values,
            SLIDER_MAXIMA,
            SLIDER_Y_POSITIONS,
        )):
            cv2.putText(
                canvas,
                f"{slider_name}: {value}",
                (SLIDER_X_MIN, slider_y - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
            cv2.rectangle(
                canvas,
                (SLIDER_X_MIN, slider_y - 6),
                (SLIDER_X_MAX, slider_y + 8),
                (70, 70, 70),
                -1,
            )
            knob_x = int(round(
                SLIDER_X_MIN
                + value / maximum * (SLIDER_X_MAX - SLIDER_X_MIN)
            ))
            cv2.rectangle(
                canvas,
                (SLIDER_X_MIN, slider_y - 6),
                (knob_x, slider_y + 8),
                interface_color,
                -1,
            )
            cv2.circle(
                canvas, (knob_x, slider_y + 1), 9,
                (255, 255, 255), -1, cv2.LINE_AA,
            )

        help_lines = [
            hue_text,
            f"S={s_low}..{s_high}  V={v_low}..{v_high}",
            "1-6: select color",
            "U: undo   C: clear   R: preset",
            "S: save   Q/Esc: quit",
            "Drag mouse inside material to sample",
        ]
        for index, text in enumerate(help_lines):
            cv2.putText(
                canvas,
                text,
                (IMAGE_WIDTH + 18, 555 + index * 27),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (225, 225, 225),
                1,
                cv2.LINE_AA,
            )
        return canvas


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="独立六色HSV框选与滑条标定工具"
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--calibration", type=Path, default=DEFAULT_CALIBRATION
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--undistort", action="store_true", help="提供自己的有效内参后开启；默认原图")
    return parser.parse_args()


def run(args: argparse.Namespace) -> None:
    maps = calibration_maps(args.calibration, args.undistort)
    if args.config.expanduser().exists():
        document = load_color_config(args.config)
    else:
        document = build_default_color_config(
            args.device, str(args.calibration)
        )
    session = HSVCalibrationSession(document)
    interface = ColorCalibratorWindow(session, args.config)
    camera = _open_camera(args.device, args.fps)
    try:
        interface.create_window()
        print("HSV calibrator controls:")
        print("  1-6 select color; drag left mouse inside material")
        print("  U undo; C clear; R preset; S save; Q/Esc quit")
        while True:
            ok, raw = camera.read()
            if not ok or raw is None:
                raise ColorCalibrationError("标定过程中摄像头读取失败")
            corrected = (cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR) if maps else raw.copy())
            interface.current_frame = corrected
            interface.update_manual_thresholds()
            displayed_frame = (
                interface.frozen_frame
                if interface.frozen_frame is not None
                else corrected
            )
            cv2.imshow(WINDOW_NAME, interface.render(displayed_frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                if session.dirty:
                    print("未保存的HSV修改已放弃")
                break
            if ord("1") <= key <= ord("6"):
                interface.select_color(key - ord("0"))
            elif key in (ord("u"), ord("U")):
                interface.undo()
            elif key in (ord("c"), ord("C")):
                interface.clear()
            elif key in (ord("r"), ord("R")):
                interface.reset()
            elif key in (ord("s"), ord("S")):
                try:
                    interface.save(args.device, args.calibration)
                    print(f"颜色配置已保存：{interface.config_path}")
                except ColorCalibrationError as error:
                    interface.status_message = f"Save failed: {error}"
                    print(f"保存失败：{error}")
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main() -> int:
    args = parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n已停止HSV标定工具")
        return 0
    except (ColorCalibrationError, cv2.error) as error:
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
