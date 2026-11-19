"""原料区毫米观测候选接口：离线编解码，无串口打开或运动发送。

不读取旧像素串口帧，不进行像素→毫米猜测。线上协议尚未接入主程序。
"""
from dataclasses import dataclass, asdict
import binascii
import math

@dataclass(frozen=True)
class MetricRequest:
    request_id: int
    run_id: int
    stage: int
    start_id: int
    map_version: int
    goal_cell: int
    timeout_ms: int
    camera_id: int
    calibration_id: int
    mount_id: int
    height_id: int
    target_id: int
    max_frame_age_ms: int
    min_confidence_permille: int
    max_delta_milli_mm: int
    max_yaw_mdeg: int
    nominal_x_milli_mm: int
    nominal_y_milli_mm: int

    def validate(self):
        if any(type(v) is not int or not 0 <= v < 2**32 for v in asdict(self).values()):
            raise ValueError('request integer field range')
        if (not all((self.request_id, self.run_id, self.map_version, self.camera_id,
                     self.calibration_id, self.mount_id, self.height_id, self.target_id)) or
                self.stage != 2 or self.start_id not in (1, 2) or self.goal_cell != 14 or
                not 100 <= self.timeout_ms <= 1000 or not 1 <= self.max_frame_age_ms <= 200 or
                not 1 <= self.min_confidence_permille <= 1000 or
                not 1 <= self.max_delta_milli_mm <= 100000 or not 1 <= self.max_yaw_mdeg <= 10000 or
                not 0 <= self.nominal_x_milli_mm < 2400000 or not 0 <= self.nominal_y_milli_mm < 2400000):
            raise ValueError('request session/profile invalid')
        return self


def decode_request(frame: bytes) -> MetricRequest:
    if not isinstance(frame, bytes) or len(frame) > 256 or not frame.endswith(b'\r\n') or frame.count(b'\n') != 1:
        raise ValueError('request framing')
    body, sep, crc = frame[:-2].rpartition(b',')
    if (not sep or len(crc) != 4 or any(c not in b'0123456789ABCDEF' for c in crc) or
            binascii.crc_hqx(body+b',', 0xffff) != int(crc, 16)):
        raise ValueError('request CRC')
    tag, *fields = body.decode('ascii').split(',')
    if tag != 'MR1' or len(fields) != 18 or any(not s or not s.isascii() or not s.isdigit() for s in fields):
        raise ValueError('request schema')
    return MetricRequest(*map(int, fields)).validate()

@dataclass(frozen=True)
class MetricObservation:
    request_id: int
    run_id: int
    stage: int
    start_id: int
    map_version: int
    goal_cell: int
    camera_id: int
    calibration_id: int
    mount_id: int
    height_id: int
    target_id: int
    frame_sequence: int
    capture_after_request_ms: int
    frame_age_ms: int
    confidence_permille: int
    units: int
    dx_milli_mm: int
    dy_milli_mm: int
    yaw_mdeg: int

    def validate(self):
        for name, value in asdict(self).items():
            if type(value) is not int:
                raise ValueError(f'{name}: integer required')
            if name in ('dx_milli_mm', 'dy_milli_mm', 'yaw_mdeg'):
                if not -(2**31) <= value < 2**31:
                    raise ValueError(f'{name}: int32 range')
            elif not 0 <= value < 2**32:
                raise ValueError(f'{name}: uint32 range')
        if self.stage != 2 or self.start_id not in (1, 2) or self.goal_cell != 14:
            raise ValueError('stage/start/goal mismatch')
        if not all((self.request_id,self.run_id,self.map_version,self.camera_id,
                    self.calibration_id,self.mount_id,self.height_id,self.target_id,self.frame_sequence)):
            raise ValueError('missing session/profile identity')
        if self.units != 1:
            raise ValueError('metric chassis frame required; pixels forbidden')
        if not 0 <= self.confidence_permille <= 1000:
            raise ValueError('confidence range')
        return self

def encode_observation(observation: MetricObservation) -> bytes:
    observation.validate()
    body = ('MA1,' + ','.join(str(v) for v in asdict(observation).values()) + ',').encode('ascii')
    return body + f'{binascii.crc_hqx(body, 0xffff):04X}\r\n'.encode('ascii')

def decode_observation(frame: bytes) -> MetricObservation:
    if len(frame) > 256 or not frame.endswith(b'\r\n') or frame.count(b'\n') != 1:
        raise ValueError('framing')
    body, sep, crc = frame[:-2].rpartition(b',')
    if not sep or len(crc) != 4 or binascii.crc_hqx(body+b',',0xffff) != int(crc,16):
        raise ValueError('CRC')
    tag, *fields = body.decode('ascii').split(',')
    if tag != 'MA1' or len(fields) != 19:
        raise ValueError('schema')
    if any(not s or (s[0]=='-' and not s[1:].isdigit()) or
           (s[0]!='-' and not s.isdigit()) for s in fields):
        raise ValueError('integer syntax')
    return MetricObservation(*map(int, fields)).validate()

def calibrated_delta_to_wire(*, units: str, calibrated: bool, dx: float, dy: float,
                             yaw_deg: float) -> tuple[int,int,int]:
    if units != 'chassis_mm' or calibrated is not True:
        raise ValueError('verified metric transform required')
    if not all(math.isfinite(x) for x in (dx,dy,yaw_deg)):
        raise ValueError('nonfinite delta')
    if abs(dx)>100 or abs(dy)>100 or abs(yaw_deg)>10:
        raise ValueError('candidate observation envelope exceeded')
    return round(dx*1000),round(dy*1000),round(yaw_deg*1000)
