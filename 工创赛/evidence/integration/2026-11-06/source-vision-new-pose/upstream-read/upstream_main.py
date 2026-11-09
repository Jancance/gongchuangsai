#!/usr/bin/env python3
"""USB摄像头视觉主程序。

阅读顺序：配置和状态变量 → 模式/测量函数 → 各模式处理 → 摄像头和界面
→ initialize()初始化 → run()主循环。

常用修改位置：
- 改快捷键：handle_key()；改串口命令对应模式：handle_serial_command()。
- 改某模式的发送条件：process_turntable/material_pickup/ring_station/ring_stacking()。
- 改识别阈值：config目录；改检测步骤：对应detector模块的detect()。
全局变量只用于这一个摄像头程序，修改它们的函数会显式声明global。
"""

import argparse
from collections import deque
from enum import Enum
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from 颜色标定 import ColorCalibrationError
from 颜色标定 import load_color_config
from material_pickup_detector import draw_material_pickup_detection
from material_pickup_detector import load_material_pickup_config
from material_pickup_detector import MaterialPickupDetector
from material_pickup_detector import MaterialPickupError
from ring_station_detector import draw_ring_station_detection
from ring_station_detector import load_ring_station_config
from ring_station_detector import RingStationDetectionError
from ring_station_detector import RingStationDetector
from stacked_ring_detector import draw_stacked_ring_detection
from stacked_ring_detector import load_stacked_ring_config
from stacked_ring_detector import StackedRingDetectionError
from stacked_ring_detector import format_line_angle, format_line_status
from stacked_ring_detector import StackedRingDetector
from stacked_ring_detector import camera_geometry_signature
from turntable_serial import DEFAULT_SERIAL_DEVICE
from turntable_serial import DEFAULT_SERIAL_BAUD
from turntable_serial import DEFAULT_SERIAL_RECONNECT_INTERVAL
from turntable_serial import TurntableReportMode
from turntable_serial import TurntableSerialCommand
from turntable_serial import TurntableSerialError
from turntable_serial import TurntableSerialLink
from turntable_detector import draw_turntable_detection
from turntable_detector import load_turntable_detector_config
from turntable_detector import TurntableDetectionError
from turntable_detector import TurntableDetector
from turntable_material_seed import load_three_material_seed_config
from turntable_material_seed import ThreeMaterialSeedError
from turntable_material_seed import ThreeMaterialTurntableSeeder

# ---------- 1. 配置路径与运行状态 ----------
CONFIG_DIR = Path(__file__).resolve().parent / "config"
DEFAULT_CALIBRATION = CONFIG_DIR / "realtek_rgb_camera.yaml"
DEFAULT_CAMERA_PROCESSING_CONFIG = CONFIG_DIR / "camera_processing.json"
DEFAULT_TURNTABLE_CONFIG = CONFIG_DIR / "turntable_detector.json"
DEFAULT_TURNTABLE_MATERIAL_SEED_CONFIG = CONFIG_DIR / "turntable_material_seed.json"
DEFAULT_RING_STATION_CONFIG = CONFIG_DIR / "ring_station_detector.json"
DEFAULT_STACKED_RING_CONFIG = CONFIG_DIR / "stacked_ring_detector.json"
DEFAULT_MATERIAL_PICKUP_CONFIG = CONFIG_DIR / "material_pickup_detector.json"
DEFAULT_MATERIAL_COLOR_CONFIG = CONFIG_DIR / "material_color_thresholds.json"


class CameraError(RuntimeError):
    """相机或配置错误，交给main()统一显示。"""


class Mode(Enum):
    """模式名字，与串口功能码分开；映射见handle_serial_command()。"""

    IDLE = "idle"
    MEASUREMENT = "measurement"
    MATERIAL_PICKUP = "material_pickup"
    RING_STATION = "ring_station"
    RING_STACKING = "ring_stacking"
    TURNTABLE = "turntable"


# 模式和检测器：initialize()创建检测器，set_mode()清理历史。
current_mode = Mode.IDLE
turntable_detector = None
ring_station_detector = None
stacked_ring_detector = None
material_pickup_detector = None
last_turntable_detection = None
last_turntable_debug = None
last_ring_station_detection = None
last_stacked_ring_detection = None
last_material_pickup_detection = None

# 串口上报：None表示进入模式后静默；请求只在成功发送后清除。
serial_link = None
turntable_report_mode = None
pending_center_request = False
ring_report_mode = None
pending_ring_request = False
material_cycle_pending = False
material_reported = False

# 两点测量：0点实时，1/2点冻结，第三击清空并等待新帧。
measurement_points = []
measurement_frame = None

# 相机与显示：断线时camera=None，达到重试时间后再打开。
settings = None
camera = None
camera_next_retry = 0.0
camera_last_error = ""
undistort_enabled = True
map1 = None
map2 = None
window_name = "Undistorted Camera"
task_window_name = "Task Code"


# ---------- 2. 模式切换、串口命令和测量 ----------
def clear_reporting():
    """切换模式时停止旧上报，清除旧请求和物料周期。"""
    global turntable_report_mode, pending_center_request
    global ring_report_mode, pending_ring_request
    global material_cycle_pending, material_reported

    turntable_report_mode = None
    pending_center_request = False
    ring_report_mode = None
    pending_ring_request = False
    material_cycle_pending = False
    material_reported = False


def reset_detection_history():
    """清除所有依赖连续画面的状态，但保留模式和串口请求。

    摄像头断开后不能继续使用断开前的圆心种子或稳定窗口；不过当前
    视觉模式、连续上报策略和尚未回复的单次请求属于底盘状态，重连
    后仍应继续。因此这里与 ``set_mode`` 分开，不清理上报状态。
    """
    global last_turntable_detection, last_turntable_debug
    global last_ring_station_detection, last_stacked_ring_detection
    global last_material_pickup_detection

    turntable_detector.reset()
    ring_station_detector.reset()
    material_pickup_detector.reset()
    stacked_ring_detector.reset()
    last_turntable_detection = None
    last_turntable_debug = None
    last_ring_station_detection = None
    last_stacked_ring_detection = None
    last_material_pickup_detection = None
    clear_measurement()


def set_mode(mode):
    """切换模式并清理旧模式的跟踪历史。

    键盘和串口共同调用这个入口。这样可以确保无论模式由
    哪种输入触发，都不会把上一轮的圆心种子或稳定帧带到下一轮。
    """
    global current_mode

    requested = mode if isinstance(mode, Mode) else Mode(mode)
    changed = requested != current_mode
    reset_detection_history()
    clear_reporting()
    current_mode = requested
    if changed:
        print(f"Mode changed: {current_mode.name}")


def enter_material_pickup():
    """进入高位物料识别，并启动一个全新的一次性上报周期。"""
    global material_cycle_pending, material_reported

    # 即使已经处于物料模式，重复 0x02 也必须清除旧低速证据和
    # REPORTED 锁存，让下一次抓取从新鲜画面重新开始。
    set_mode(Mode.MATERIAL_PICKUP)
    material_cycle_pending = True
    material_reported = False


def handle_serial_command(command):
    """把串口功能码转换成视觉模式和转盘上报子模式。"""
    global turntable_report_mode, pending_center_request
    global ring_report_mode, pending_ring_request

    if command == TurntableSerialCommand.ENTER_IDLE:
        # 与键盘 I 共用同一入口，离开转盘模式时会同时清除检测器
        # 种子、连续上报状态以及尚未完成的单次应答请求。
        set_mode(Mode.IDLE)
        return
    if command == TurntableSerialCommand.ENTER_MATERIAL_PICKUP:
        enter_material_pickup()
        return
    if command == TurntableSerialCommand.ENTER_TURNTABLE:
        set_mode(Mode.TURNTABLE)
        return
    if command == TurntableSerialCommand.ENTER_RING_STATION:
        set_mode(Mode.RING_STATION)
        return
    if command == TurntableSerialCommand.ENTER_RING_STACKING:
        set_mode(Mode.RING_STACKING)
        return
    if current_mode in (Mode.RING_STATION, Mode.RING_STACKING):
        if command == TurntableSerialCommand.ENABLE_CONTINUOUS:
            ring_report_mode = TurntableReportMode.CONTINUOUS
            pending_ring_request = False
        elif command == TurntableSerialCommand.REQUEST_ONCE:
            ring_report_mode = TurntableReportMode.ON_REQUEST
            # 重复请求合并为一个完整的三圆心应答批次。
            pending_ring_request = True
        return
    if current_mode == Mode.MATERIAL_PICKUP:
        # 物料上报固定为每次 0x02 一次；0x14/0x15 在此模式忽略。
        return
    # 0x14/0x15 作用于当前视觉任务；Idle 中直接忽略。
    if current_mode != Mode.TURNTABLE:
        return
    if command == TurntableSerialCommand.ENABLE_CONTINUOUS:
        turntable_report_mode = TurntableReportMode.CONTINUOUS
        pending_center_request = False
    elif command == TurntableSerialCommand.REQUEST_ONCE:
        turntable_report_mode = TurntableReportMode.ON_REQUEST
        # bool 状态天然合并中心稳定前收到的重复查询。
        pending_center_request = True


def get_report_status():
    """返回用于终端状态行的上报模式名称。"""

    if current_mode in (Mode.RING_STATION, Mode.RING_STACKING):
        if ring_report_mode is None:
            return (
                "STACKED_WAITING"
                if current_mode == Mode.RING_STACKING else "RING_WAITING"
            )
        prefix = "STACKED" if current_mode == Mode.RING_STACKING else "RING"
        return f"{prefix}_{ring_report_mode.name}"
    if current_mode == Mode.MATERIAL_PICKUP:
        return (
            "MATERIAL_REPORTED"
            if material_reported else "MATERIAL_ARMED"
        )
    if current_mode != Mode.TURNTABLE:
        return "INACTIVE"
    if turntable_report_mode is None:
        return "WAITING"
    return turntable_report_mode.name


def clear_measurement():
    """清除选点与冻结画面，等待下一张有效实时画面。"""
    global measurement_frame

    measurement_points.clear()
    measurement_frame = None


def prepare_measurement_frame(corrected):
    """缓存实时画面或复用冻结画面，返回供界面叠加使用的独立副本。"""
    global measurement_frame

    if current_mode != Mode.MEASUREMENT:
        return corrected
    if not measurement_points:
        measurement_frame = corrected.copy()
    return measurement_frame.copy()


def calculate_measurement():
    """返回第二点相对第一点的有符号像素差和直线距离。"""

    if len(measurement_points) != 2:
        return None
    (x1, y1), (x2, y2) = measurement_points
    dx, dy = x2 - x1, y2 - y1
    return dx, dy, math.hypot(dx, dy)


def add_measurement_point(x, y):
    """前两次点击选点；第三次只清空，新的有效画面到达后可重测。"""

    if current_mode != Mode.MEASUREMENT or measurement_frame is None:
        return False
    height, width = measurement_frame.shape[:2]
    pixel_x, pixel_y = int(x), int(y)
    if not (0 <= pixel_x < width and 0 <= pixel_y < height):
        return False
    if len(measurement_points) == 2:
        clear_measurement()
        print("Measurement cleared; returning to live preview")
        return True
    measurement_points.append((pixel_x, pixel_y))
    print(f"Measurement P{len(measurement_points)}: "
          f"X={pixel_x}, Y={pixel_y}")
    result = calculate_measurement()
    if result is not None:
        dx, dy, distance = result
        print(f"Measurement: DX={dx:+d}px DY={dy:+d}px "
              f"Distance={distance:.2f}px")
    return True


# ---------- 3. 每种模式：检测 → 绘制 → 判断发送 → 成功后更新状态 ----------
def process_turntable(frame):
    """定位真实转盘外圆；稳定后根据当前上报策略发送圆心。"""
    global last_turntable_detection, last_turntable_debug, pending_center_request

    # 未稳定时优先三物料粗定位，再验证真实外圆；稳定后局部跟踪。
    if last_turntable_detection is None or not last_turntable_detection.stable:
        result, last_turntable_debug = turntable_detector.detect_with_debug(frame)
    else:
        result = turntable_detector.detect(frame)
        last_turntable_debug = None
    last_turntable_detection = result
    draw_turntable_detection(frame, result)

    if not result.detected or not result.stable:
        return result
    continuous = turntable_report_mode == TurntableReportMode.CONTINUOUS
    requested = (turntable_report_mode == TurntableReportMode.ON_REQUEST
                 and pending_center_request)
    if continuous or requested:
        if serial_link.send_center(result.center_x_px, result.center_y_px):
            if requested:
                pending_center_request = False
    return result


def send_ring_if_ready(result):
    """两种圆环模式共用同一应答：2号圆心和实际连线角度。"""
    global pending_ring_request

    if (not result.detected or not result.stable
            or len(result.centers_px) != 3 or result.station_yaw_deg is None):
        return
    continuous = ring_report_mode == TurntableReportMode.CONTINUOUS
    requested = ring_report_mode == TurntableReportMode.ON_REQUEST and pending_ring_request
    if continuous or requested:
        if serial_link.send_ring_alignment(result.centers_px[1], result.station_yaw_deg):
            if requested:
                pending_ring_request = False


def process_ring_station(frame):
    """直接检测三圆环，再按策略上报。"""
    global last_ring_station_detection

    result = ring_station_detector.detect(frame)
    last_ring_station_detection = result
    draw_ring_station_detection(frame, result)
    send_ring_if_ready(result)
    return result


def process_ring_stacking(frame):
    """检测中环和上下长边，再按同一圆环协议上报。"""
    global last_stacked_ring_detection

    result = stacked_ring_detector.detect(frame)
    last_stacked_ring_detection = result
    draw_stacked_ring_detection(frame, result)
    send_ring_if_ready(result)
    return result


def process_material_pickup(frame, timestamp):
    """物料检测器确认转动和低速；每次0x02周期只允许成功发送一次。"""
    global last_material_pickup_detection, material_cycle_pending, material_reported

    result = material_pickup_detector.detect(frame, timestamp=timestamp)
    last_material_pickup_detection = result
    draw_material_pickup_detection(frame, result, material_pickup_detector.config)
    if (material_cycle_pending and not material_reported
            and result.detected and result.ready):
        if serial_link.send_material_slots(*result.slot_colors):
            material_cycle_pending = False
            material_reported = True
    return result


# ---------- 4. 配置读取和摄像头 ----------
def parse_args():
    """解析命令行参数。

    所有参数都提供了当前硬件对应的默认值。保留命令行参数是为了后续
    更换设备节点或进行对比测试时，不必直接修改源代码。
    """
    parser = argparse.ArgumentParser(
        description="按配置选择原始或畸变矫正画面，显示视觉结果和实际帧率"
    )
    # V4L2 主视频节点；同一摄像头的 video1 是元数据节点，不能取图。
    parser.add_argument("--device", default="/dev/video0")
    # 开启矫正时，分辨率必须与标定文件中的宽高完全一致。
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    # 这是向驱动请求的目标帧率，画面上的 FPS 是程序实际测得的帧率。
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=DEFAULT_CALIBRATION,
    )
    parser.add_argument(
        "--camera-processing-config",
        type=Path,
        default=DEFAULT_CAMERA_PROCESSING_CONFIG,
        help="摄像头图像处理配置，控制是否开启畸变矫正",
    )
    parser.add_argument(
        "--turntable-config",
        type=Path,
        default=DEFAULT_TURNTABLE_CONFIG,
        help="转盘粗定位、圆弧精修和稳定判定配置",
    )
    parser.add_argument(
        "--turntable-material-seed-config",
        type=Path,
        default=DEFAULT_TURNTABLE_MATERIAL_SEED_CONFIG,
        help="三物料端面粗定位转盘圆心的配置",
    )
    parser.add_argument(
        "--ring-station-config",
        type=Path,
        default=DEFAULT_RING_STATION_CONFIG,
        help="三圆环检测、固定观察基准和底盘纠偏矩阵配置",
    )
    parser.add_argument(
        "--stacked-ring-config",
        type=Path,
        default=DEFAULT_STACKED_RING_CONFIG,
        help="中环与上下长边码垛检测配置",
    )
    parser.add_argument(
        "--material-pickup-config",
        type=Path,
        default=DEFAULT_MATERIAL_PICKUP_CONFIG,
        help="高位物料活动区、三个抓取区、形态和低速判定配置",
    )
    parser.add_argument(
        "--material-color-config",
        type=Path,
        default=DEFAULT_MATERIAL_COLOR_CONFIG,
        help="六种物料的OpenCV HSV阈值配置",
    )
    parser.add_argument(
        "--camera-reconnect-interval",
        type=float,
        default=1.0,
        help="摄像头断开或打开失败后的重连间隔（秒）",
    )
    parser.add_argument(
        "--serial-device",
        default=DEFAULT_SERIAL_DEVICE,
        help="底盘通信串口设备节点",
    )
    parser.add_argument(
        "--serial-baud",
        type=int,
        default=DEFAULT_SERIAL_BAUD,
        help="底盘通信串口波特率",
    )
    parser.add_argument(
        "--serial-reconnect-interval",
        type=float,
        default=DEFAULT_SERIAL_RECONNECT_INTERVAL,
        help="串口故障后的重连间隔（秒）",
    )
    return parser.parse_args()


def load_undistortion_enabled(path):
    """读取必须明确填写为JSON布尔值的畸变矫正开关。"""

    resolved = path.expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise CameraError(f"无法读取图像处理配置 {resolved}：{error}") from error
    if not isinstance(document, dict) or type(document.get("undistort_enabled")) is not bool:
        raise CameraError(
            f"图像处理配置 {resolved} 的 undistort_enabled 必须为 true 或 false"
        )
    return document["undistort_enabled"]


def read_matrix(storage, name):
    """从已经打开的 OpenCV FileStorage 中读取一个非空矩阵。

    OpenCV 在字段不存在时通常不会直接抛出异常，而是返回空节点或
    ``None``。这里统一做显式检查，避免后续创建矫正映射时出现含义
    不清楚的 OpenCV 底层错误。

    Args:
        storage: 已成功打开的 OpenCV YAML 文件。
        name: YAML 中的矩阵字段名称。

    Returns:
        OpenCV/NumPy 表示的矩阵。
    """
    matrix = storage.getNode(name).mat()
    if matrix is None or matrix.size == 0:
        raise CameraError(f"标定文件缺少矩阵：{name}")
    return matrix


def load_calibration(path):
    """加载畸变矫正所需的分辨率和三个矩阵。

    ``camera_matrix`` 是原始相机内参，``distortion_coefficients`` 是
    plumb_bob 模型的畸变系数，``new_camera_matrix`` 是矫正输出采用的
    新内参。三个参数必须配套使用，不能只替换其中一个。

    Returns:
        ``((width, height), camera_matrix, distortion, new_camera_matrix)``。
    """
    # expanduser 支持命令行中的 ~/...；resolve 便于错误信息显示绝对路径。
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise CameraError(f"标定文件不存在：{resolved}")

    # 该 YAML 使用 !!opencv-matrix 标记，必须使用 FileStorage 读取，
    # 不能把它当作普通 YAML 字典直接解析。
    storage = cv2.FileStorage(str(resolved), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise CameraError(f"无法读取标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        camera_matrix = read_matrix(storage, "camera_matrix")
        distortion = read_matrix(storage, "distortion_coefficients")
        new_camera_matrix = read_matrix(storage, "new_camera_matrix")
    finally:
        # 即使某个字段读取失败，也要关闭底层文件句柄。
        storage.release()
    return (width, height), camera_matrix, distortion, new_camera_matrix


def open_camera(device, width, height, fps):
    """按指定格式打开摄像头，并确认设备能够返回正确尺寸的画面。

    这里显式指定 ``CAP_V4L2``，保证在 Linux 上走 V4L2 后端。MJPG
    在 1280x720 下支持 30 FPS；若使用未压缩 YUYV，该摄像头在相同
    分辨率下只能提供约 5 FPS。

    Returns:
        已打开并完成预热的 ``cv2.VideoCapture`` 对象。
    """
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise CameraError(f"无法打开摄像头：{device}")

    # 先指定像素格式，再设置分辨率和帧率，让驱动协商正确的模式。
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)
    # 单缓冲在当前设备上会把 OpenCV 采集降到约 15 FPS。
    # 四缓冲能稳定读取 30 FPS，并且不需要额外的采集线程。
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    # 摄像头刚启动时，自动曝光/白平衡尚未稳定，而且第一帧可能暂时
    # 不可用。最多读取十次，既完成预热，也验证设备确实能输出图像。
    for _ in range(10):
        ok, frame = camera.read()
        if ok and frame is not None:
            break
    else:
        camera.release()
        raise CameraError(f"摄像头已打开，但无法读取画面：{device}")

    # VideoCapture.set() 只是向驱动提出请求，并不保证请求一定被接受。
    # 因此必须用实际取到的帧再次核对尺寸，防止标定参数被错误套用。
    actual_height, actual_width = frame.shape[:2]
    if (actual_width, actual_height) != (width, height):
        camera.release()
        raise CameraError(
            f"请求 {width}x{height}，实际输出 "
            f"{actual_width}x{actual_height}"
        )
    return camera


def close_camera():
    """释放当前摄像头，允许之后按重连逻辑重新打开。"""
    global camera

    if camera is not None:
        try:
            camera.release()
        except (OSError, cv2.error):
            pass
    camera = None


def schedule_camera_retry(reason):
    """记录断线原因，延迟重试，避免每帧重复打开设备。"""
    global camera_last_error, camera_next_retry

    close_camera()
    camera_last_error = str(reason)
    camera_next_retry = time.monotonic() + settings.camera_reconnect_interval
    print(f"Camera unavailable ({settings.device}): {camera_last_error}; waiting for reconnect")


def read_camera():
    """返回新帧；无设备、等待重连或读取失败时返回None。"""
    global camera, camera_last_error

    if camera is None:
        if time.monotonic() < camera_next_retry:
            return None
        try:
            camera = open_camera(settings.device, settings.width, settings.height, settings.fps)
        except (CameraError, OSError, cv2.error) as error:
            schedule_camera_retry(error)
            return None
        camera_last_error = ""
        print(f"Camera connected: {settings.device}, {settings.width}x{settings.height}, {settings.fps:.1f} FPS")
    try:
        ok, frame = camera.read()
    except (OSError, cv2.error) as error:
        schedule_camera_retry(error)
        return None
    if not ok or frame is None:
        schedule_camera_retry("采集过程中摄像头读取失败")
        return None
    height, width = frame.shape[:2]
    if (width, height) != (settings.width, settings.height):
        schedule_camera_retry(f"摄像头输出尺寸变为 {width}x{height}")
        return None
    return frame


# ---------- 5. 界面绘制和键盘鼠标 ----------
def draw_camera_waiting(frame, detail=''):
    """在占位画面中央显示摄像头断开和等待重连状态。"""

    height, width = frame.shape[:2]
    title = "CAMERA DISCONNECTED"
    subtitle = "Waiting for camera connection..."
    for index, (text, scale, color) in enumerate((
        (title, 1.15, (0, 80, 255)),
        (subtitle, 0.78, (255, 255, 255)),
    )):
        text_size = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2
        )[0]
        x = max(10, (width - text_size[0]) // 2)
        y = height // 2 - 20 + index * 55
        cv2.putText(
            frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
            scale, color, 2, cv2.LINE_AA,
        )
    if detail:
        short_detail = str(detail)[:90]
        cv2.putText(
            frame, short_detail, (30, height - 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (180, 180, 180), 1, cv2.LINE_AA,
        )
    return frame


def draw_task_code_window(task_code, size=(760, 320)):
    """生成只含大号任务码的独立窗口画面。"""

    width, height = (int(value) for value in size)
    if width < 480 or height < 240:
        raise ValueError("任务码窗口尺寸至少为480x240")
    panel = np.full((height, width, 3), (24, 24, 24), dtype=np.uint8)
    if task_code is None:
        return panel

    # 尽量使用大字体，同时根据窗口宽度自动缩小，确保完整任务码不被裁切。
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 4
    font_scale = 2.2
    (text_width, text_height), baseline = cv2.getTextSize(
        task_code.text, font, font_scale, thickness
    )
    maximum_width = width - 40
    if text_width > maximum_width:
        font_scale *= maximum_width / text_width
        (text_width, text_height), baseline = cv2.getTextSize(
            task_code.text, font, font_scale, thickness
        )
    text_x = max(20, (width - text_width) // 2)
    text_y = (height + text_height - baseline) // 2
    cv2.putText(
        panel, task_code.text, (text_x, text_y), font, font_scale,
        (80, 255, 80), thickness, cv2.LINE_AA,
    )
    return panel


def draw_status(frame, mode, measured_fps, frame_count):
    """绘制模式、实时帧率、累计帧数，以及IDLE模式的按键提示。

    使用黑色信息底板和单层白字，既保证亮暗背景下都清晰，也避免
    双层抗锯齿文字占用过多实时处理时间。
    """
    text = (
        f"Mode: {mode.name}  FPS: {measured_fps:5.1f}  "
        f"Frames: {frame_count}"
    )
    status_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)[0][0]
    cv2.rectangle(
        frame, (10, 10), (max(700, status_width + 30), 55), (0, 0, 0), -1
    )
    cv2.putText(
        frame,
        text,
        (20, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if mode == Mode.IDLE:
        # 正常预览和摄像头等待画面共用提示；使用与现有界面一致的英文。
        shortcuts = (
            "Keyboard controls",
            "D : Two-point measurement",
            "A : Turntable",
            "M : Material pickup",
            "R : Three-ring alignment",
            "S : Middle ring + lines (stacking)",
            "I : Idle",
            "Q / Esc : Quit",
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.65
        line_height = 29
        panel_width = max(
            cv2.getTextSize(label, font, scale, 2)[0][0]
            for label in shortcuts
        ) + 20
        cv2.rectangle(
            frame, (10, 65),
            (10 + panel_width, 75 + line_height * len(shortcuts)),
            (0, 0, 0), -1,
        )
        for index, label in enumerate(shortcuts):
            cv2.putText(
                frame, label, (20, 90 + index * line_height),
                font, scale,
                (0, 255, 255) if index == 0 else (255, 255, 255),
                2, cv2.LINE_AA,
            )
    return frame


def draw_measurement(frame):
    """在显示副本上绘制测量标记和黑底结果面板，不修改冻结原图。"""

    if current_mode != Mode.MEASUREMENT:
        return frame
    height, width = frame.shape[:2]
    points = measurement_points
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    colors = ((0, 255, 255), (255, 255, 0))
    marker_bounds = []
    if len(points) == 2:
        cv2.line(frame, points[0], points[1], (0, 255, 0), 2, cv2.LINE_AA)
    for index, (x, y) in enumerate(points):
        color = colors[index]
        cv2.drawMarker(
            frame, (x, y), color, cv2.MARKER_CROSS, 28, 2, cv2.LINE_8
        )
        label = f"P{index + 1}: ({x}, {y})"
        (text_width, text_height), baseline = cv2.getTextSize(
            label, font, scale, 2
        )
        label_x = x + 14
        if label_x + text_width + 10 >= width:
            label_x = max(5, x - text_width - 14)
        # 两点很近或重合时，分别放在点的上方和下方。
        label_y = y - 16 if index == 0 else y + text_height + 16
        label_y = max(text_height + 6, min(height - baseline - 6, label_y))
        marker_bounds.append((
            min(x - 14, label_x - 5), min(y - 14, label_y - text_height - 5),
            max(x + 14, label_x + text_width + 5), max(y + 14, label_y + baseline + 5),
        ))
        cv2.rectangle(
            frame, (label_x - 5, label_y - text_height - 5),
            (label_x + text_width + 5, label_y + baseline + 5),
            (0, 0, 0), -1,
        )
        cv2.putText(
            frame, label, (label_x, label_y), font, scale, color, 2, cv2.LINE_AA
        )

    if measurement_frame is None:
        title = "MEASUREMENT | WAITING FOR LIVE FRAME"
        hint = "Wait for camera image before selecting P1"
    elif not points:
        title = "MEASUREMENT | LIVE"
        hint = "Left click: select P1 and freeze image"
    else:
        title = "MEASUREMENT | FROZEN"
        hint = (
            "Left click: select P2"
            if len(points) == 1 else "Third click: clear and resume live image"
        )
    lines = [(title, colors[0]), (hint, (255, 255, 255))]
    for index in range(2):
        value = str(points[index]) if index < len(points) else "--"
        lines.append((f"P{index + 1}: {value}", colors[index]))
    result = calculate_measurement()
    if result is None:
        lines.append(("DX: -- px   DY: -- px   Distance: -- px", (255, 255, 255)))
    else:
        dx, dy, distance = result
        lines.append((
            f"DX: {dx:+d} px   DY: {dy:+d} px   Distance: {distance:.2f} px",
            (0, 255, 0),
        ))
    lines.append(("D: restart   I: Idle   Q / Esc: quit", (200, 200, 200)))
    panel_width = max(
        cv2.getTextSize(label, font, scale, 2)[0][0] for label, _ in lines
    ) + 20
    # 优先左下角，遇到选点或标签时换到其他角落；留出底部断线设备提示。
    panel_height = 10 + 29 * len(lines)
    candidates = (
        (10, height - 50 - panel_height),
        (width - 10 - panel_width, height - 50 - panel_height),
        (10, 65),
        (width - 10 - panel_width, 65),
    )

    panel_x, panel_y = candidates[0]
    fewest_overlaps = len(marker_bounds) + 1
    for px, py in candidates:
        overlaps = 0
        for left, top, right, bottom in marker_bounds:
            if (px < right and px + panel_width > left
                    and py < bottom and py + panel_height > top):
                overlaps += 1
        if overlaps < fewest_overlaps:
            panel_x, panel_y = px, py
            fewest_overlaps = overlaps
    cv2.rectangle(
        frame, (panel_x, panel_y),
        (panel_x + panel_width, panel_y + panel_height), (0, 0, 0), -1
    )
    for index, (label, color) in enumerate(lines):
        cv2.putText(
            frame, label, (panel_x + 10, panel_y + 24 + 29 * index),
            font, scale, color, 2, cv2.LINE_AA,
        )
    return frame


def handle_measurement_mouse_click(event, mouse_x, mouse_y):
    """接收HighGUI已映射到原图的坐标，仅测量模式接受左键选点。"""

    if event != cv2.EVENT_LBUTTONDOWN:
        return False
    return add_measurement_point(mouse_x, mouse_y)


def handle_mouse(event, x, y, _flags, userdata):
    """OpenCV回调坐标已经过窗口缩放映射，不再按窗口尺寸二次缩放。"""

    handle_measurement_mouse_click(event, x, y)


def handle_key(key):
    """处理一次预览窗口按键；返回 ``False`` 表示退出主循环。

    按键只负责把用户意图转换为模式请求，实际状态清理仍由
    ``set_mode`` 完成。串口命令经handle_serial_command()调用相同入口。
    """

    if key in (ord("q"), ord("Q"), 27):
        return False
    if key in (ord("a"), ord("A")):
        set_mode(Mode.TURNTABLE)
    elif key in (ord("d"), ord("D")):
        set_mode(Mode.MEASUREMENT)
    elif key in (ord("r"), ord("R")):
        set_mode(Mode.RING_STATION)
    elif key in (ord("s"), ord("S")):
        set_mode(Mode.RING_STACKING)
    elif key in (ord("m"), ord("M")):
        enter_material_pickup()
    elif key in (ord("i"), ord("I")):
        set_mode(Mode.IDLE)
    return True


def print_runtime_status(result, measured_fps, frame_count):
    """每秒调用一次，输出当前模式结果和完整串口统计。"""
    turntable_result = result if current_mode == Mode.TURNTABLE else None
    ring_result = result if current_mode == Mode.RING_STATION else None
    stacked_ring_result = result if current_mode == Mode.RING_STACKING else None
    material_result = result if current_mode == Mode.MATERIAL_PICKUP else None
    if current_mode in (Mode.RING_STATION, Mode.RING_STACKING):
        last_sent = serial_link.last_sent_ring_alignment
    elif current_mode == Mode.MATERIAL_PICKUP:
        last_sent = serial_link.last_sent_material_slots
    else:
        last_sent = serial_link.last_sent_center
    serial_status = (
        f"Serial={serial_link.status} "
        f"Report={get_report_status()} "
        f"Pending={pending_center_request or pending_ring_request or material_cycle_pending} "
        f"TurntableSent={serial_link.sent_count} "
        f"RingSent={serial_link.sent_ring_batches} "
        f"MaterialSent={serial_link.sent_material_batches} "
        f"Task={serial_link.last_task_code.text if serial_link.last_task_code else '--'} "
        f"TaskRx={serial_link.received_task_count} "
        f"TaskInvalid={serial_link.invalid_task_count} "
        f"Last={last_sent or '--'}"
    )
    if turntable_result is not None:
        if turntable_result.detected:
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} "
                f"Center=({turntable_result.center_x_px:.1f},"
                f"{turntable_result.center_y_px:.1f}) "
                f"Radius={turntable_result.radius_px:.1f}px "
                f"Arc={turntable_result.visible_arc_deg:.0f}deg "
                f"RMS={turntable_result.fit_rms_px:.2f}px "
                f"Confidence={turntable_result.confidence:.3f} "
                f"Detect={turntable_result.processing_ms:.1f}ms "
                f"Track={turntable_result.tracking_mode} "
                f"Stable={turntable_result.stable} "
                f"Detail={turntable_result.detail} "
                f"{serial_status}"
            )
        else:
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} "
                f"State=SEARCHING "
                f"Detect={turntable_result.processing_ms:.1f}ms "
                f"Track={turntable_result.tracking_mode} "
                f"Detail={turntable_result.detail} "
                f"{serial_status}"
            )
    elif ring_result is not None:
        if ring_result.detected:
            correction = (
                "--"
                if ring_result.correction_body_x_mm is None
                else (
                    f"({ring_result.correction_body_x_mm:+.2f},"
                    f"{ring_result.correction_body_y_mm:+.2f},"
                    f"{ring_result.correction_yaw_deg:+.2f})"
                )
            )
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} "
                f"Centers={tuple((round(x, 1), round(y, 1)) for x, y in ring_result.centers_px)} "
                f"Error=({ring_result.error_x_mm:+.2f},"
                f"{ring_result.error_y_mm:+.2f},"
                f"{ring_result.angle_error_deg:+.2f}) "
                f"Correction={correction} "
                f"Confidence={ring_result.confidence:.3f} "
                f"Detect={ring_result.processing_ms:.1f}ms "
                f"Stable={ring_result.stable} "
                f"Aligned={ring_result.within_tolerance} "
                f"{serial_status}"
            )
        else:
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} State=SEARCHING "
                f"Detect={ring_result.processing_ms:.1f}ms "
                f"Detail={ring_result.detail} {serial_status}"
            )
    elif stacked_ring_result is not None:
        if stacked_ring_result.detected:
            correction = (
                "--"
                if stacked_ring_result.correction_body_x_mm is None
                else (
                    f"({stacked_ring_result.correction_body_x_mm:+.2f},"
                    f"{stacked_ring_result.correction_body_y_mm:+.2f},"
                    f"{stacked_ring_result.correction_yaw_deg:+.2f})"
                )
            )
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} "
                f"Middle=({stacked_ring_result.middle_center_px[0]:.1f},"
                f"{stacked_ring_result.middle_center_px[1]:.1f}) "
                f"{format_line_status(stacked_ring_result)} "
                f"Top={format_line_angle(stacked_ring_result.top_line_angle_deg)} "
                f"Bottom={format_line_angle(stacked_ring_result.bottom_line_angle_deg)} "
                f"Error=({stacked_ring_result.error_x_mm:+.2f},"
                f"{stacked_ring_result.error_y_mm:+.2f},"
                f"{stacked_ring_result.angle_error_deg:+.2f}) "
                f"Correction={correction} "
                f"Confidence={stacked_ring_result.confidence:.3f} "
                f"Detect={stacked_ring_result.processing_ms:.1f}ms "
                f"Stable={stacked_ring_result.stable} "
                f"Aligned={stacked_ring_result.within_tolerance} "
                f"{serial_status}"
            )
        else:
            print(
                f"Mode={current_mode.name} "
                f"FPS={measured_fps:5.1f} State=SEARCHING "
                f"Detect={stacked_ring_result.processing_ms:.1f}ms "
                f"Detail={stacked_ring_result.detail} "
                f"{serial_status}"
            )
    elif material_result is not None:
        print(
            f"Mode={current_mode.name} "
            f"FPS={measured_fps:5.1f} "
            f"State={material_result.state} "
            f"Count={material_result.material_count} "
            f"Slots={material_result.slot_colors} "
            f"Speed={material_result.max_speed_px_s:.1f}px/s "
            f"Motion={material_result.motion_confirmed} "
            f"Move={material_result.moving_frames}/"
            f"{material_result.required_moving_frames} "
            f"Low={material_result.low_speed_frames}/"
            f"{material_result.required_low_speed_frames} "
            f"Detect={material_result.processing_ms:.1f}ms "
            f"Ready={material_result.ready} "
            f"Detail={material_result.detail} "
            f"{serial_status}"
        )
    else:
        print(
            f"Mode={current_mode.name} "
            f"FPS={measured_fps:5.1f} Frames={frame_count} "
            f"{serial_status}"
        )


def print_waiting_status():
    """断线期间继续显示待请求状态和任务码统计。"""
    print(
        f"Mode={current_mode.name} Camera=WAITING "
        f"Detail={camera_last_error or '等待连接'} "
        f"Serial={serial_link.status} "
        f"Report={get_report_status()} "
        f"Pending={pending_center_request or pending_ring_request or material_cycle_pending} "
        f"Task={serial_link.last_task_code.text if serial_link.last_task_code else '--'} "
        f"TaskRx={serial_link.received_task_count} "
        f"TaskInvalid={serial_link.invalid_task_count}"
    )


# ---------- 6. 初始化与主循环 ----------
def initialize(args):
    """只在启动时读取配置、创建检测器和串口，摄像头在read_camera中按需打开。"""
    global settings, undistort_enabled, map1, map2, serial_link
    global turntable_detector, ring_station_detector
    global material_pickup_detector, stacked_ring_detector
    global current_mode, camera_next_retry, camera_last_error, window_name

    settings = args
    close_camera()
    camera_next_retry = 0.0
    camera_last_error = ""
    if args.camera_reconnect_interval <= 0.0:
        raise CameraError("摄像头重连间隔必须大于0")
    undistort_enabled = load_undistortion_enabled(args.camera_processing_config)
    if undistort_enabled:
        # 仅在需要矫正时读取标定；关闭后无需标定文件，也不创建映射。
        calibrated_size, camera_matrix, distortion, new_camera_matrix = (
            load_calibration(args.calibration)
        )
        requested_size = (args.width, args.height)
        if requested_size != calibrated_size:
            raise CameraError(
                f"标定参数适用于 {calibrated_size[0]}x"
                f"{calibrated_size[1]}，不能用于 "
                f"{args.width}x{args.height}"
            )

        # 映射在启动时生成一次，逐帧remap复用；采用定点映射提高速度。
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            distortion,
            None,
            new_camera_matrix,
            calibrated_size,
            cv2.CV_16SC2,
        )
    # 各检测器在摄像头打开前完成配置校验。配置错误时，
    # 程序不会无意义地占用 /dev/video0。
    turntable_config = load_turntable_detector_config(
        args.turntable_config
    )
    turntable_material_seed_config = load_three_material_seed_config(
        args.turntable_material_seed_config
    )
    ring_station_config = load_ring_station_config(
        args.ring_station_config
    )
    stacked_ring_config = load_stacked_ring_config(
        args.stacked_ring_config
    )
    material_pickup_config = load_material_pickup_config(
        args.material_pickup_config
    )
    material_color_config = load_color_config(
        args.material_color_config
    )
    material_seeder = ThreeMaterialTurntableSeeder(
        turntable_material_seed_config, material_color_config
    )
    turntable_detector = TurntableDetector(turntable_config, material_seeder)
    ring_station_detector = RingStationDetector(ring_station_config)
    material_pickup_detector = MaterialPickupDetector(material_pickup_config, material_color_config)
    stacked_ring_detector = StackedRingDetector(stacked_ring_config, ring_station_config)
    stacked_ring_detector.set_camera_geometry(camera_geometry_signature(
        args.width, args.height, undistort_enabled, args.calibration))
    serial_link = TurntableSerialLink(
        device=args.serial_device, baudrate=args.serial_baud,
        reconnect_interval_s=args.serial_reconnect_interval,
    )
    current_mode = Mode.IDLE
    reset_detection_history()
    clear_reporting()
    window_name = "Undistorted Camera" if undistort_enabled else "Raw Camera"


def run(args):
    """主流程：命令 → 相机 → 图像处理 → 模式分派 → 显示 → 按键。"""
    initialize(args)
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)
        cv2.namedWindow(task_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(task_window_name, 760, 320)
        displayed_task_code = serial_link.last_task_code
        cv2.imshow(task_window_name, draw_task_code_window(displayed_task_code))
        cv2.setMouseCallback(window_name, handle_mouse)

        frame_times = deque(maxlen=90)
        frame_count = 0
        measured_fps = 0.0
        camera_was_connected = False
        last_terminal_output = time.monotonic()
        print(
            f"Camera: {args.device}, {args.width}x{args.height}, "
            f"MJPG, requested {args.fps:.1f} FPS; reconnect every "
            f"{args.camera_reconnect_interval:.1f}s"
        )
        print(f"Undistortion: {'ON' if undistort_enabled else 'OFF (raw image)'}")
        if undistort_enabled:
            print(f"Calibration: {args.calibration.expanduser().resolve()}")
        print(
            f"Serial: {args.serial_device}, {args.serial_baud} baud; "
            "RX control: AA 01/02/03/04/05/14/15 BB; "
            "task: AA 06 <15 ASCII bytes> BB"
        )
        print(
            "Controls: A = turntable, M = material pickup, R = ring station, "
            "S = stacked ring, D = measurement, I = idle, Q or Esc = quit"
        )
        print(
            "Turntable tracking: Hough until stable, then local radial tracking"
        )


        while True:
            # 1. 先接收命令；即使相机断线或测量冻结，也持续响应串口。
            for command in serial_link.poll_commands():
                handle_serial_command(command)
            if serial_link.last_task_code != displayed_task_code:
                displayed_task_code = serial_link.last_task_code
                cv2.imshow(task_window_name, draw_task_code_window(displayed_task_code))

            # 2. 获取新帧。断线只清检测历史，保留模式和待发送请求。
            raw = read_camera()
            now = time.monotonic()
            if raw is None:
                if camera_was_connected:
                    reset_detection_history()
                    frame_times.clear()
                    measured_fps = 0.0
                camera_was_connected = False
                waiting = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                draw_status(waiting, current_mode, measured_fps, frame_count)
                draw_camera_waiting(waiting, f"Device: {args.device} | retry every {args.camera_reconnect_interval:.1f}s")
                if current_mode == Mode.MEASUREMENT:
                    draw_measurement(waiting)
                cv2.imshow(window_name, waiting)
                if now - last_terminal_output >= 1.0:
                    print_waiting_status()
                    last_terminal_output = now
                if not handle_key(cv2.waitKey(50) & 0xFF):
                    break
                continue

            if not camera_was_connected:
                frame_times.clear()
                measured_fps = 0.0
            camera_was_connected = True

            # 3. 所有模式使用同一画面，叠加绘制不会改动相机原始缓冲区。
            if undistort_enabled:
                frame = cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
            else:
                frame = raw.copy()

            # 4. 要修改某个模式，直接查看对应process函数。
            result = None
            if current_mode == Mode.IDLE:
                pass  # 只显示画面和快捷键
            elif current_mode == Mode.MEASUREMENT:
                frame = prepare_measurement_frame(frame)
            elif current_mode == Mode.TURNTABLE:
                result = process_turntable(frame)
            elif current_mode == Mode.MATERIAL_PICKUP:
                result = process_material_pickup(frame, now)
            elif current_mode == Mode.RING_STATION:
                result = process_ring_station(frame)
            elif current_mode == Mode.RING_STACKING:
                result = process_ring_stacking(frame)

            # 5. 更新帧率、显示画面，每秒打印一次状态。
            frame_count += 1
            now = time.monotonic()
            frame_times.append(now)
            if len(frame_times) >= 2:
                elapsed = frame_times[-1] - frame_times[0]
                if elapsed > 0.0:
                    measured_fps = (len(frame_times) - 1) / elapsed
            draw_status(frame, current_mode, measured_fps, frame_count)
            if current_mode == Mode.MEASUREMENT:
                draw_measurement(frame)
            cv2.imshow(window_name, frame)
            if now - last_terminal_output >= 1.0:
                print_runtime_status(result, measured_fps, frame_count)
                last_terminal_output = now

            # 6. waitKey同时处理鼠标事件；冻结仅影响显示，不阻塞循环。
            if not handle_key(cv2.waitKey(1) & 0xFF):
                break
    finally:
        serial_link.close()
        close_camera()
        cv2.destroyAllWindows()


def main():
    """程序入口：把运行异常转换为清楚的提示和进程退出码。"""
    args = parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n已停止摄像头预览")
        return 0
    except (
        CameraError,
        TurntableSerialError,
        TurntableDetectionError,
        ThreeMaterialSeedError,
        RingStationDetectionError,
        StackedRingDetectionError,
        MaterialPickupError,
        ColorCalibrationError,
        cv2.error,
    ) as error:
        # 返回非零退出码，便于未来的启动脚本判断摄像头是否正常。
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
