#!/usr/bin/env python3
"""高位转盘物料活动区和左/右/下抓取区标定工具。

运行示例：

    python3 code/物料抓取区域标定.py

按顺序用鼠标左键拖框：整个物料活动区、左抓取区、右抓取区、下抓取区。
拖框期间画面冻结，松开鼠标后恢复实时画面。按 S 原子保存配置，U 撤销
最后一区域，R 清空重来，Q 或 Esc 退出。
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import cv2
import numpy as np
from project_camera import DEFAULT_DEVICE, open_project_camera, calibration_maps

from material_pickup_detector import build_default_material_pickup_config
from material_pickup_detector import load_material_pickup_config
from material_pickup_detector import MaterialPickupError
from material_pickup_detector import save_material_pickup_config


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CALIBRATION = SCRIPT_DIR / "config" / "decxin_camera_unconfigured.yaml"
DEFAULT_CONFIG = SCRIPT_DIR / "config" / "material_pickup_detector.json"
REGION_ORDER = ("workspace", "left", "right", "down")
REGION_COLORS = {
    "workspace": (180, 180, 180),
    "left": (0, 220, 255),
    "right": (255, 180, 0),
    "down": (180, 0, 255),
}


class MaterialSlotCalibrationError(RuntimeError):
    """摄像头、内参或交互标定失败。"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="框选高位转盘活动区和左/右/下三个抓取区"
    )
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--undistort", action="store_true", help="提供自己的有效内参后开启；默认原图")
    return parser.parse_args()


def _read_matrix(storage: cv2.FileStorage, name: str):
    matrix = storage.getNode(name).mat()
    if matrix is None or matrix.size == 0:
        raise MaterialSlotCalibrationError(f"标定文件缺少矩阵：{name}")
    return matrix


def load_camera_calibration(path: Path):
    resolved = path.expanduser().resolve()
    storage = cv2.FileStorage(str(resolved), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise MaterialSlotCalibrationError(f"无法打开摄像头标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = _read_matrix(storage, "camera_matrix")
        distortion = _read_matrix(storage, "distortion_coefficients")
        new_camera_matrix = _read_matrix(storage, "new_camera_matrix")
    finally:
        storage.release()
    if (width, height) != (1280, 960):
        raise MaterialSlotCalibrationError(
            f"高位区域标定要求1280x960内参，当前为{width}x{height}"
        )
    return (width, height), camera_matrix, distortion, new_camera_matrix


def open_camera(device: str, fps: float):
    return open_project_camera(device, 1280, 960, fps)


class RegionSelector:
    """保存拖框状态，并保证一次拖动始终使用同一张冻结帧。"""

    def __init__(self, document: dict):
        self.document = copy.deepcopy(document)
        self.rectangles: list[tuple[str, dict]] = []
        if bool(document.get("calibrated")):
            self.rectangles = [
                (name, copy.deepcopy(document["regions"][name]))
                for name in REGION_ORDER
            ]
        self.latest_frame = None
        self.frozen_frame = None
        self.drag_start = None
        self.drag_end = None
        self.message = "Select four regions in order"

    @property
    def complete(self) -> bool:
        return len(self.rectangles) == len(REGION_ORDER)

    @property
    def current_name(self) -> str | None:
        if self.complete:
            return None
        return REGION_ORDER[len(self.rectangles)]

    @staticmethod
    def _rectangle(first, second):
        x1, y1 = first
        x2, y2 = second
        left, top = min(x1, x2), min(y1, y2)
        return {
            "x": int(left), "y": int(top),
            "width": int(abs(x2 - x1)), "height": int(abs(y2 - y1)),
        }

    def mouse_callback(self, event, x_value, y_value, _flags, _parameter):
        point = (
            max(0, min(1279, int(x_value))),
            max(0, min(719, int(y_value))),
        )
        if event == cv2.EVENT_LBUTTONDOWN and not self.complete:
            self.drag_start = point
            self.drag_end = point
            if self.latest_frame is not None:
                self.frozen_frame = self.latest_frame.copy()
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_end = point
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            self.drag_end = point
            rectangle = self._rectangle(self.drag_start, self.drag_end)
            name = self.current_name
            if rectangle["width"] >= 10 and rectangle["height"] >= 10:
                self.rectangles.append((name, rectangle))
                self.message = f"Saved {name}; select the next region"
            else:
                self.message = "Selection is too small; drag again"
            self.drag_start = None
            self.drag_end = None
            self.frozen_frame = None

    def undo(self):
        if self.drag_start is not None:
            self.drag_start = None
            self.drag_end = None
            self.frozen_frame = None
        elif self.rectangles:
            name, _ = self.rectangles.pop()
            self.message = f"Undid {name}"

    def reset(self):
        self.rectangles.clear()
        self.drag_start = None
        self.drag_end = None
        self.frozen_frame = None
        self.message = "Cleared; start again from WORKSPACE"

    def save(self, path: Path):
        if not self.complete:
            self.message = "All four regions are required before saving"
            return False
        document = copy.deepcopy(self.document)
        document["calibrated"] = True
        document["regions"] = {
            name: copy.deepcopy(rectangle)
            for name, rectangle in self.rectangles
        }
        try:
            save_material_pickup_config(path, document)
        except MaterialPickupError as error:
            self.message = f"Save failed: {error}"
            return False
        self.document = document
        self.message = f"Saved: {path.expanduser().resolve()}"
        return True

    def draw(self, live_frame: np.ndarray) -> np.ndarray:
        self.latest_frame = live_frame
        base = (
            self.frozen_frame.copy()
            if self.frozen_frame is not None else live_frame.copy()
        )
        for name, rectangle in self.rectangles:
            first = (rectangle["x"], rectangle["y"])
            second = (
                rectangle["x"] + rectangle["width"],
                rectangle["y"] + rectangle["height"],
            )
            cv2.rectangle(base, first, second, REGION_COLORS[name], 2)
            cv2.putText(
                base, name.upper(), (first[0] + 4, first[1] + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, REGION_COLORS[name], 2,
                cv2.LINE_AA,
            )
        if self.drag_start is not None and self.drag_end is not None:
            name = self.current_name
            rectangle = self._rectangle(self.drag_start, self.drag_end)
            cv2.rectangle(
                base, (rectangle["x"], rectangle["y"]),
                (rectangle["x"] + rectangle["width"],
                 rectangle["y"] + rectangle["height"]),
                REGION_COLORS[name], 2,
            )

        next_item = self.current_name or "DONE - PRESS S"
        cv2.rectangle(base, (8, 8), (1270, 86), (0, 0, 0), -1)
        cv2.putText(
            base,
            f"Next: {next_item.upper()} | Drag=select U=undo R=reset S=save Q=quit",
            (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.67,
            (255, 255, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            base, self.message[:100], (18, 70),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 1,
            cv2.LINE_AA,
        )
        return base


def run(args: argparse.Namespace) -> None:
    try:
        document = load_material_pickup_config(args.config)
    except MaterialPickupError:
        document = build_default_material_pickup_config()
    maps = calibration_maps(args.calibration, args.undistort)
    camera = open_camera(args.device, args.fps)
    selector = RegionSelector(document)
    window_name = "Material Pickup Region Calibrator"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 960)
    cv2.setMouseCallback(window_name, selector.mouse_callback)
    print("框选顺序：整个活动区 -> 左抓取区 -> 右抓取区 -> 下抓取区")
    try:
        while True:
            if selector.frozen_frame is None:
                ok, raw = camera.read()
                if not ok or raw is None:
                    raise MaterialSlotCalibrationError("标定过程中摄像头读取失败")
                corrected = (cv2.remap(raw, maps[0], maps[1], cv2.INTER_LINEAR) if maps else raw.copy())
            else:
                corrected = selector.frozen_frame
            cv2.imshow(window_name, selector.draw(corrected))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("u"), ord("U")):
                selector.undo()
            elif key in (ord("r"), ord("R")):
                selector.reset()
            elif key in (ord("s"), ord("S")):
                if selector.save(args.config):
                    print(selector.message)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main() -> int:
    args = parse_args()
    try:
        run(args)
    except (MaterialSlotCalibrationError, MaterialPickupError, cv2.error) as error:
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
