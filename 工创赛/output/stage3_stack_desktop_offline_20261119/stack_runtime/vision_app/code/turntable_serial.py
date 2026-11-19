"""视觉程序与底盘之间的非阻塞串口协议。

入站模式控制帧为 ``AA 功能码 BB``。任务码使用独立的18字节 ``AA 06
15字节ASCII正文 BB`` 帧。物料抓取使用功能码 ``02``，三圆环使用 ``03``，
中环加长边码垛使用 ``04``，转盘中心使用 ``05``。本模块不依赖视觉模式
枚举，只负责解析、编码和串口断线重连。圆环类结果使用一个 ``03`` 帧
同时发送2号圆心和float32实际连线角度。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from numbers import Integral
import struct
import time
from typing import Callable, Optional

try:
    import serial
except ImportError:
    # 桌面只读识别/协议编解码不要求安装pyserial。显式打开串口时再报错。
    class _MissingSerial:
        SerialException = OSError
        @staticmethod
        def Serial(*args, **kwargs):
            raise OSError('pyserial未安装；当前仅能使用离线协议和桌面视觉')
    serial = _MissingSerial()


# 默认串口参数统一在这里修改；main.py也引用这些值。
# 启动时传入 --serial-device / --serial-baud 等参数可临时覆盖。
DEFAULT_SERIAL_DEVICE = "/dev/ttyS7"
DEFAULT_SERIAL_BAUD = 115200
DEFAULT_SERIAL_RECONNECT_INTERVAL = 1.0


class TurntableSerialError(ValueError):
    """串口配置、接收帧或待发送数据不符合协议要求。"""


class TurntableReportMode(Enum):
    """转盘和圆环类视觉结果共用的两种上报策略。"""

    CONTINUOUS = "continuous"
    ON_REQUEST = "on_request"


class TurntableSerialCommand(IntEnum):
    """当前阶段支持的视觉模式和上报策略命令。"""

    ENTER_IDLE = 0x01
    ENTER_MATERIAL_PICKUP = 0x02
    ENTER_RING_STATION = 0x03
    ENTER_RING_STACKING = 0x04
    ENTER_TURNTABLE = 0x05
    ENABLE_CONTINUOUS = 0x14
    REQUEST_ONCE = 0x15


@dataclass(frozen=True)
class TaskCode:
    """已通过完整协议校验的15字符任务码。

    四组值保留扫码顺序。颜色编号为1～6；圆环编号为1～3。任务码与视觉
    工作模式相互独立，模式切换和串口重连均不会清除本对象。
    """

    text: str
    first_colors: tuple[int, int, int]
    first_rings: tuple[int, int, int]
    second_colors: tuple[int, int, int]
    second_rings: tuple[int, int, int]


class TurntableSerialLink:
    """维护一个可自动重连的非阻塞全双工串口。

    Args:
        device: Linux 串口设备节点。
        baudrate: 串口波特率。
        reconnect_interval_s: 打开或写入失败后的重试间隔。
        serial_factory: 创建串口对象的工厂；参数主要用于无硬件测试。
        clock: 单调时钟函数；参数主要用于可重复的重连测试。
    """

    RX_HEADER = 0xAA
    RX_TAIL = 0xBB
    RX_TASK_FUNCTION = 0x06
    TASK_CODE_BODY_SIZE = 15
    TASK_CODE_FRAME_SIZE = 18
    TX_HEADER = 0xCC
    TX_MATERIAL_FUNCTION = 0x02
    TX_RING_FUNCTION = 0x03
    TX_FUNCTION = 0x05
    TX_TAIL = 0xDD
    CENTER_FRAME_SIZE = 7
    MATERIAL_FRAME_SIZE = 6

    def __init__(
        self,
        device: str = DEFAULT_SERIAL_DEVICE,
        baudrate: int = DEFAULT_SERIAL_BAUD,
        reconnect_interval_s: float = DEFAULT_SERIAL_RECONNECT_INTERVAL,
        serial_factory: Callable = serial.Serial,
        clock: Callable[[], float] = time.monotonic,
    ):
        if not device:
            raise TurntableSerialError("串口设备节点不能为空")
        if baudrate <= 0:
            raise TurntableSerialError("串口波特率必须大于0")
        if reconnect_interval_s <= 0.0:
            raise TurntableSerialError("串口重连间隔必须大于0")

        self.device = str(device)
        self.baudrate = int(baudrate)
        self.reconnect_interval_s = float(reconnect_interval_s)
        self._serial_factory = serial_factory
        self._clock = clock
        self._port = None
        self._receive_buffer = bytearray()
        self._next_reconnect_s = 0.0

        # 这些统计值只用于每秒一次的运行状态输出，不参与协议判断。
        self.sent_count = 0
        self.last_sent_center: Optional[tuple[int, int]] = None
        self.sent_ring_batches = 0
        self.last_sent_ring_alignment: Optional[
            tuple[tuple[int, int], float]
        ] = None
        self.sent_material_batches = 0
        self.last_sent_material_slots: Optional[tuple[int, int, int]] = None
        # 任务码只驻留内存：有效新帧整体替换，模式切换/串口重连均保留。
        self.last_task_code: Optional[TaskCode] = None
        self.received_task_count = 0
        self.invalid_task_count = 0
        self.last_task_error = ""
        self.last_error = ""

    @property
    def connected(self) -> bool:
        """当前是否持有一个仍处于打开状态的串口对象。"""

        return bool(
            self._port is not None
            and getattr(self._port, "is_open", True)
        )

    @property
    def status(self) -> str:
        """返回适合终端显示的简短连接状态。"""

        return "CONNECTED" if self.connected else "RETRYING"

    def _disconnect(self, error: Exception | str) -> None:
        """关闭失效句柄，并把下次重连安排到指定间隔以后。"""

        message = str(error)
        if self._port is not None:
            try:
                self._port.close()
            except (OSError, serial.SerialException):
                pass
        self._port = None
        # 半帧不能跨越物理断线继续解析，否则重连后的第一个字节可能与
        # 旧连接残留数据错误拼成一条命令。
        self._receive_buffer.clear()
        self._next_reconnect_s = self._clock() + self.reconnect_interval_s
        self.last_error = message
        # 失败后最多每个重连周期打印一次，不会按摄像头帧率刷屏。
        print(f"Serial unavailable ({self.device}): {message}")

    def _ensure_connected(self) -> bool:
        """在允许的重试时刻打开串口，且永不阻塞等待接收数据。"""

        if self.connected:
            return True
        if self._clock() < self._next_reconnect_s:
            return False
        try:
            self._port = self._serial_factory(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0,
            )
        except (OSError, serial.SerialException) as error:
            self._disconnect(error)
            return False

        self.last_error = ""
        print(f"Serial connected: {self.device}, {self.baudrate} baud")
        return True

    def _consume_received_bytes(
        self, data: bytes
    ) -> tuple[TurntableSerialCommand, ...]:
        """解析任意分段的字节流，并从噪声或坏帧处重新同步。

        一个串口 ``read`` 可能只返回半帧，也可能一次返回多帧。解析器
        因此保留不完整尾部。普通控制帧按3字节解析；看到 ``AA 06`` 后
        必须等待18字节任务帧收满，任务码只更新缓存且不产生命令。
        """

        # 1. 新字节追加到缓存；一条命令可能跨多次read到达。
        self._receive_buffer.extend(data)
        commands = []
        while self._receive_buffer:
            try:
                start = self._receive_buffer.index(self.RX_HEADER)
            except ValueError:
                # 缓冲区完全没有帧头，所有数据都可判定为噪声。
                self._receive_buffer.clear()
                break
            if start:
                del self._receive_buffer[:start]
            if len(self._receive_buffer) < 2:
                break

            # 2. 任务码是18字节，其余模式命令是3字节，分别处理。
            function_code = self._receive_buffer[1]
            if function_code == self.RX_TASK_FUNCTION:
                if len(self._receive_buffer) < self.TASK_CODE_FRAME_SIZE:
                    break
                candidate = bytes(
                    self._receive_buffer[:self.TASK_CODE_FRAME_SIZE]
                )
                try:
                    task_code = self.decode_task_code_frame(candidate)
                except TurntableSerialError as error:
                    # 坏任务帧不能覆盖旧任务；只滑过当前候选帧头，随后
                    # 重新搜索AA，避免吞掉紧随其后的合法粘包。
                    self.invalid_task_count += 1
                    self.last_task_error = str(error)
                    del self._receive_buffer[0]
                    continue

                del self._receive_buffer[:self.TASK_CODE_FRAME_SIZE]
                self.last_task_code = task_code
                self.received_task_count += 1
                self.last_task_error = ""
                print(f"Task code received: {task_code.text}")
                continue

            if len(self._receive_buffer) < 3:
                break
            if self._receive_buffer[2] != self.RX_TAIL:
                # 当前 AA 不是有效帧头，只丢一个字节以便寻找下一个 AA。
                del self._receive_buffer[0]
                continue

            # 3. 消费一个完整控制帧，再继续解析后面的粘包。
            del self._receive_buffer[:3]
            try:
                commands.append(TurntableSerialCommand(function_code))
            except ValueError:
                # 未定义功能码保留给后续视觉模式，本阶段直接忽略。
                continue
        return tuple(commands)

    @classmethod
    def decode_task_code_frame(cls, frame: bytes) -> TaskCode:
        """严格校验并解析一个18字节任务码同步帧。

        正文必须是 ``ddd+ddd+ddd+ddd``。第一、三组是颜色序列，组内
        不重复且颜色集合一致；第二、四组都必须是1/2/3的排列。
        """

        if len(frame) != cls.TASK_CODE_FRAME_SIZE:
            raise TurntableSerialError(
                f"任务码帧长度应为18字节，实际为{len(frame)}"
            )
        if frame[0] != cls.RX_HEADER or frame[1] != cls.RX_TASK_FUNCTION:
            raise TurntableSerialError("任务码帧头或功能码无效")
        if frame[-1] != cls.RX_TAIL:
            raise TurntableSerialError("任务码帧尾不是0xBB")
        try:
            body = frame[2:-1].decode("ascii")
        except UnicodeDecodeError as error:
            raise TurntableSerialError("任务码正文不是ASCII") from error
        if len(body) != cls.TASK_CODE_BODY_SIZE:
            raise TurntableSerialError("任务码正文长度不是15字符")
        if body[3] != "+" or body[7] != "+" or body[11] != "+":
            raise TurntableSerialError("任务码分隔符位置无效")

        groups = (body[0:3], body[4:7], body[8:11], body[12:15])
        first_colors, first_rings, second_colors, second_rings = groups
        for name, colors in (
            ("第一批颜色", first_colors),
            ("第二批颜色", second_colors),
        ):
            if any(value not in "123456" for value in colors):
                raise TurntableSerialError(f"{name}必须为1～6")
            if len(set(colors)) != 3:
                raise TurntableSerialError(f"{name}不能重复")
        if set(first_colors) != set(second_colors):
            raise TurntableSerialError("两批颜色集合不一致")
        for name, rings in (
            ("第一批圆环", first_rings),
            ("第二批圆环", second_rings),
        ):
            if set(rings) != {"1", "2", "3"}:
                raise TurntableSerialError(f"{name}必须是1/2/3的不重复排列")

        parsed = tuple(tuple(int(value) for value in group) for group in groups)
        return TaskCode(body, parsed[0], parsed[1], parsed[2], parsed[3])

    def poll_commands(self) -> tuple[TurntableSerialCommand, ...]:
        """非阻塞读取当前已到达的全部字节并返回完整控制命令。"""

        if not self._ensure_connected():
            return ()
        try:
            available = int(getattr(self._port, "in_waiting", 0))
            if available <= 0:
                return ()
            data = self._port.read(available)
        except (OSError, serial.SerialException) as error:
            self._disconnect(error)
            return ()
        return self._consume_received_bytes(data or b"")

    @classmethod
    def encode_center_frame(
        cls, center_x_px: float, center_y_px: float
    ) -> bytes:
        """把平滑圆心编码成参考代码兼容的 7 字节帧。

        ``int`` 与参考代码一致，直接截去小数部分而不是四舍五入。
        负坐标仅在发送时归零，不修改检测结果；超过uint16上限仍拒绝编码。
        """

        if not (
            math.isfinite(center_x_px) and math.isfinite(center_y_px)
        ):
            raise TurntableSerialError("转盘中心必须是有限数值")
        center_x = max(0, int(center_x_px))
        center_y = max(0, int(center_y_px))
        if not (0 <= center_x <= 0xFFFF and 0 <= center_y <= 0xFFFF):
            raise TurntableSerialError(
                f"转盘中心超出uint16范围：({center_x}, {center_y})"
            )
        return struct.pack(
            "<BBHHB",
            cls.TX_HEADER,
            cls.TX_FUNCTION,
            center_x,
            center_y,
            cls.TX_TAIL,
        )

    @classmethod
    def encode_ring_alignment(cls, center_px, station_yaw_deg) -> bytes:
        """把2号圆心像素和实际连线角度编码到同一个11字节帧。

        帧格式为 ``CC 03 X_UINT16 Y_UINT16 ANGLE_FLOAT32 DD``。
        角度单位为度，按IEEE-754小端float32编码。
        """

        try:
            x_value, y_value = center_px
            x_float, y_float = float(x_value), float(y_value)
        except (TypeError, ValueError) as error:
            raise TurntableSerialError("2号圆环中心格式无效") from error
        if not (math.isfinite(x_float) and math.isfinite(y_float)):
            raise TurntableSerialError("2号圆环中心必须是有限数值")
        center_x, center_y = int(x_float), int(y_float)
        if not (0 <= center_x <= 0xFFFF and 0 <= center_y <= 0xFFFF):
            raise TurntableSerialError(
                f"2号圆环中心超出uint16范围：({center_x}, {center_y})"
            )

        try:
            angle = float(station_yaw_deg)
        except (TypeError, ValueError) as error:
            raise TurntableSerialError("实际连线角度格式无效") from error
        if not math.isfinite(angle):
            raise TurntableSerialError("实际连线角度必须是有限数值")
        try:
            return struct.pack(
                "<BBHHfB",
                cls.TX_HEADER,
                cls.TX_RING_FUNCTION,
                center_x,
                center_y,
                angle,
                cls.TX_TAIL,
            )
        except (OverflowError, struct.error) as error:
            raise TurntableSerialError(
                f"实际连线角度无法编码为float32：{angle}"
            ) from error

    @classmethod
    def encode_material_slots(cls, left, right, down) -> bytes:
        """编码左、右、下三个抓取位置的颜色编号。

        颜色编号 ``1～6`` 与六色 HSV 配置一致，``0`` 表示该位置为空。
        每个位置固定占一个字节，因此帧长度始终为 6 字节。
        """

        values = []
        for name, value in zip(("left", "right", "down"),
                               (left, right, down)):
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise TurntableSerialError(f"{name}颜色编号必须是整数")
            color_id = int(value)
            if not 0 <= color_id <= 6:
                raise TurntableSerialError(
                    f"{name}颜色编号超出0～6范围：{color_id}"
                )
            values.append(color_id)
        return bytes((
            cls.TX_HEADER,
            cls.TX_MATERIAL_FUNCTION,
            values[0], values[1], values[2],
            cls.TX_TAIL,
        ))

    def _write_frame(self, frame):
        """完整写入才算成功；短写或异常断开串口，供下一次调用重连。"""
        if not self._ensure_connected():
            return False
        try:
            written = self._port.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"串口只写入 {written}/{len(frame)} 字节"
                )
        except (OSError, serial.SerialException) as error:
            self._disconnect(error)
            return False

        return True

    def send_center(self, center_x_px: float, center_y_px: float) -> bool:
        """尝试发送一个中心帧；成功写入完整帧时返回 ``True``。"""

        try:
            frame = self.encode_center_frame(center_x_px, center_y_px)
        except TurntableSerialError as error:
            self.last_error = str(error)
            print(f"Serial center rejected: {error}")
            return False
        if not self._write_frame(frame):
            return False

        self.sent_count += 1
        # 从已成功发送的帧中读取实际坐标，确保记录也反映负数归零。
        self.last_sent_center = struct.unpack("<HH", frame[2:6])
        return True

    def send_ring_alignment(self, center_px, station_yaw_deg) -> bool:
        """发送2号圆心与实际连线角度，完整写入时返回 ``True``。"""

        try:
            frame = self.encode_ring_alignment(
                center_px, station_yaw_deg
            )
        except TurntableSerialError as error:
            self.last_error = str(error)
            print(f"Serial ring alignment rejected: {error}")
            return False
        if not self._write_frame(frame):
            return False

        self.sent_ring_batches += 1
        self.last_sent_ring_alignment = (
            (int(center_px[0]), int(center_px[1])),
            float(station_yaw_deg),
        )
        return True

    def send_material_slots(self, left, right, down) -> bool:
        """发送一次左/右/下颜色批次，完整写入成功时返回 ``True``。"""

        try:
            frame = self.encode_material_slots(left, right, down)
        except TurntableSerialError as error:
            self.last_error = str(error)
            print(f"Serial material slots rejected: {error}")
            return False
        if not self._write_frame(frame):
            return False

        self.sent_material_batches += 1
        self.last_sent_material_slots = (
            int(left), int(right), int(down)
        )
        return True

    def close(self) -> None:
        """关闭串口；程序退出路径调用本方法，不再安排重连。"""

        if self._port is not None:
            try:
                self._port.close()
            except (OSError, serial.SerialException):
                pass
        self._port = None
        self._receive_buffer.clear()
