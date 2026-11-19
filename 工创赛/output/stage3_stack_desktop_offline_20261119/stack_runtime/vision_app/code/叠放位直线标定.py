#!/usr/bin/env python3
"""叠放位外圈和上下基准线搜索标定，不改工位零点、不打开串口。

python3 code/叠放位直线标定.py
左键选中环，S确认外圈/直线并保存，R重新寻找，Q/Esc退出。
"""

import argparse
from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np
from project_camera import DEFAULT_DEVICE, open_project_camera, calibration_maps

from main import (CameraError, DEFAULT_CALIBRATION, DEFAULT_CAMERA_PROCESSING_CONFIG,
                  DEFAULT_RING_STATION_CONFIG, DEFAULT_STACKED_RING_CONFIG,
                  load_calibration, load_undistortion_enabled, open_camera)
from ring_station_detector import load_ring_station_config, RingStationDetectionError
from stacked_ring_detector import (
    StackedRingDetectionError, camera_geometry_signature, load_stacked_ring_config,
    find_calibration_outer_ring, detect_scaled_reference_lines,
    calibration_samples_stable, make_search_calibration, save_search_calibration,
    draw_search_calibration, stacked_edge_image, has_both_reference_lines,
)


# 只有预览和选点状态；检测及配置保存由stacked_ring_detector统一实现。
WINDOW = "Stacking line calibration"
selection = None
samples = deque()
current_observation = None
current_frame_size = None
status_message = "Click middle ring if automatic selection fails"


def parse_args():
    parser = argparse.ArgumentParser(description="以95mm外圈和上下75mm基准线标定叠放位搜索范围")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--camera-processing-config", type=Path, default=DEFAULT_CAMERA_PROCESSING_CONFIG)
    parser.add_argument("--config", type=Path, default=DEFAULT_STACKED_RING_CONFIG)
    parser.add_argument("--ring-config", type=Path, default=DEFAULT_RING_STATION_CONFIG)
    parser.add_argument("--diameter-mm", type=float, required=True, help="填写自己工位实测外圈直径mm")
    parser.add_argument("--line-offset-mm", type=float, required=True, help="填写自己工位实测基准线偏置mm")
    return parser.parse_args()


def clear_observation():
    global current_observation
    samples.clear()
    current_observation = None


def reset_selection():
    global selection, status_message
    selection = None
    clear_observation()
    status_message = "Click middle ring if automatic selection fails"


def handle_mouse(event, x, y, _flags, _userdata):
    global selection, status_message
    if event != cv2.EVENT_LBUTTONDOWN or current_frame_size is None:
        return
    width, height = current_frame_size
    if not (0 <= x < width and 0 <= y < height):
        return
    # HighGUI已经将缩放窗口坐标映射回imshow原图，不要再次按比例缩放。
    selection = (float(x), float(y))
    clear_observation()
    status_message = "Selected ring - checking outer boundary and lines"


def save_current(args, signature, ring_config):
    global status_message
    if (not has_both_reference_lines(current_observation)
            or not all(has_both_reference_lines(sample) for sample in samples)
            or not calibration_samples_stable(samples, ring_config)):
        status_message = "Cannot save: wait for STABLE with BOTH lines"
        return False
    data = make_search_calibration(current_observation, signature,
                                   args.diameter_mm, args.line_offset_mm)
    save_search_calibration(args.config, data)
    status_message = "SAVED - restart main.py to load calibration"
    print(f"已保存叠放位搜索标定：{args.config.resolve()}；工位零点及纠偏矩阵未修改")
    return True


def draw_preview(frame, observation, stable, message, diameter_mm, offset_mm):
    preview = frame.copy()
    if observation is not None:
        draw_search_calibration(preview, observation)
    lines = [
        "Stacking calibration: " + ("STABLE" if stable else "SEARCHING"),
        f"Outer={diameter_mm:g}mm  Lines=+/-{offset_mm:g}mm  Cyan=ROI  Magenta=fit",
        "Click: select ring | S: confirm & save | R: reset | Q/Esc: quit",
        message,
    ]
    if observation is not None:
        axes = observation["ellipse"][1]
        lines.append(f"Outer axes={axes[0]:.1f} x {axes[1]:.1f}px  "
                     f"Yaw={observation.get('yaw', 0):+.2f}deg - verify GREEN is outermost")
    cv2.rectangle(preview, (0, 0), (preview.shape[1] - 1, 25 * len(lines) + 12), (0, 0, 0), -1)
    for index, line in enumerate(lines):
        cv2.putText(preview, line, (10, 25 + index * 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 240, 0) if stable else (0, 220, 255), 1, cv2.LINE_AA)
    return preview


def run(args):
    cv2.ocl.setUseOpenCL(False)
    global samples, selection, current_observation, current_frame_size, status_message
    if (not np.isfinite([args.diameter_mm, args.line_offset_mm, args.fps]).all()
            or args.diameter_mm <= 0 or args.line_offset_mm <= args.diameter_mm / 2
            or args.width <= 0 or args.height <= 0 or args.fps <= 0):
        raise StackedRingDetectionError("相机尺寸、帧率和物理尺寸必须有效，基准线应位于圆环外")
    config = load_stacked_ring_config(args.config)
    ring_config = load_ring_station_config(args.ring_config)
    enabled = load_undistortion_enabled(args.camera_processing_config)
    signature = camera_geometry_signature(args.width, args.height, enabled, args.calibration)
    maps = None
    if enabled:
        size, matrix, distortion, new_matrix = load_calibration(args.calibration)
        if size != (args.width, args.height):
            raise CameraError("分辨率与相机内参不匹配")
        maps = cv2.initUndistortRectifyMap(matrix, distortion, None, new_matrix, size, cv2.CV_16SC2)
    samples = deque(maxlen=ring_config.stable_frame_count)
    reset_selection()
    current_frame_size = None
    camera = None
    retry_at = 0.0
    previous_error = None
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, args.width, args.height)
        cv2.setMouseCallback(WINDOW, handle_mouse)
        while True:
            frame = None
            if camera is None and time.monotonic() >= retry_at:
                try:
                    camera = open_camera(args.device, args.width, args.height, args.fps)
                except (CameraError, cv2.error) as error:
                    print(f"摄像头等待重连：{error}")
                    retry_at = time.monotonic() + 1.0
            if camera is not None:
                try:
                    ok, frame = camera.read()
                except cv2.error as error:
                    print(f"摄像头读取失败：{error}")
                    ok, frame = False, None
                if not ok or frame is None or frame.shape[:2] != (args.height, args.width):
                    camera.release()
                    camera = None
                    frame = None
                    retry_at = time.monotonic() + 1.0
            stable = False
            debug = {}
            if frame is None:
                clear_observation()
                current_frame_size = None
                display_frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                status_message = "Camera disconnected - waiting for reconnect"
            else:
                display_frame = cv2.remap(frame, *maps, cv2.INTER_LINEAR) if maps is not None else frame
                current_frame_size = (args.width, args.height)
                try:
                    edges = stacked_edge_image(display_frame, config)
                    ellipse, _ = find_calibration_outer_ring(
                        display_frame, ring_config, selection, edge_image=edges)
                    observation = detect_scaled_reference_lines(
                        display_frame, ellipse, config, args.diameter_mm, args.line_offset_mm, debug=debug, edge_image=edges, require_both=True)
                    current_observation = observation
                    samples.append(observation)
                    stable = calibration_samples_stable(samples, ring_config)
                    if not status_message.startswith("SAVED"):
                        status_message = "Check GREEN outermost boundary and both lines before S"
                    previous_error = None
                except (StackedRingDetectionError, cv2.error, np.linalg.LinAlgError) as error:
                    clear_observation()
                    status_message = "Check ring selection, outer boundary and both lines; see terminal"
                    if str(error) != previous_error:
                        print(f"标定等待：{error}")
                        previous_error = str(error)
            preview = draw_preview(display_frame, current_observation or debug or None, stable,
                                   status_message, args.diameter_mm, args.line_offset_mm)
            cv2.imshow(WINDOW, preview)
            key = cv2.waitKey(20) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if key in (ord('r'), ord('R')):
                reset_selection()
            elif key in (ord('s'), ord('S')):
                try:
                    save_current(args, signature, ring_config)
                except (OSError, ValueError) as error:
                    status_message = "Save failed - original configuration preserved"
                    print(f"保存失败：{error}")
            # 本机GTK3不支持可见性查询，窗口打开时也会返回-1。
            # 只有明确返回0才判定关闭；不支持查询时使用Q/Esc退出。
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) == 0:
                break
    finally:
        clear_observation()
        current_frame_size = None
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


def main():
    try:
        run(parse_args())
    except (CameraError, StackedRingDetectionError, RingStationDetectionError,
            OSError, ValueError, cv2.error) as error:
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
