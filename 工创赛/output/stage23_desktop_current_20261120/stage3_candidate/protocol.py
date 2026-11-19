"""Third-stage observation wire format. No hardware imports."""
from dataclasses import dataclass
import math


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(fields):
    payload = (','.join(map(str, fields)) + ',').encode('ascii')
    return payload + f'{crc16(payload):04X}\r\n'.encode('ascii')


def decode(data):
    if not data.endswith(b'\r\n') or len(data) > 256:
        raise ValueError('framing')
    payload, checksum = data[:-2].rsplit(b',', 1)
    if len(checksum) != 4 or checksum != f'{crc16(payload + b","):04X}'.encode('ascii'):
        raise ValueError('CRC')
    return payload.decode('ascii').split(',')


def integer(text, low=0, high=0xFFFFFFFF):
    value = int(text)
    if str(value) != text or not low <= value <= high:
        raise ValueError('integer range/canonical format')
    return value


@dataclass(frozen=True)
class Status:
    run: int
    state: int
    pose: int
    item: int
    color0: int
    color1: int
    color2: int
    align_done: int
    config_ready: int

    @property
    def context(self):
        return self.run, self.pose, self.state


def parse_status(data):
    fields = decode(data)
    if len(fields) != 10 or fields[0] != 'S31':
        raise ValueError('status fields')
    values = [integer(value) for value in fields[1:]]
    if values[1] not in (0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11):
        raise ValueError('state')
    if any(v not in (0, 1) for v in values[-2:]):
        raise ValueError('status flags')
    return Status(*values)


def observation(run, pose, seq, valid, motion, age_ms, errors=(0., 0.)):
    if valid not in (False, True, 0, 1):
        raise ValueError('valid')
    integer(str(run)); integer(str(pose)); integer(str(seq))
    if len(motion) != 3 or not all(math.isfinite(float(v)) for v in motion):
        raise ValueError('motion')
    values = tuple(round(float(v) * 1000) for v in motion) if valid else (0, 0, 0)
    for value in values:
        integer(str(value), -0x80000000, 0x7FFFFFFF)
    if len(errors) != 2 or not all(math.isfinite(float(v)) for v in errors) or errors[0] < 0:
        raise ValueError('residual')
    residuals = tuple(round(float(v)*1000) for v in errors) if valid else (0, 0)
    integer(str(residuals[0]), 0, 0x7FFFFFFF)
    integer(str(residuals[1]), -0x80000000, 0x7FFFFFFF)
    integer(str(age_ms), 0, 0xFFFFFFFF)
    return frame(('O31', run, pose, seq, int(valid), *values, *residuals, age_ms))


class Session:
    """A frame must begin acquisition after the current observe epoch arrived."""
    def __init__(self):
        self.status = None
        self.fence = None
        self.last_status_at = None
        self.last_frame = -1
        self.sequence = 0

    def accept(self, status, now):
        changed = self.status is None or self.status.context != status.context
        if changed:
            self.fence = now
            self.last_frame = -1
        self.status = status
        self.last_status_at = now
        return changed

    def encode(self, frame_id, acquired_after, completed_at, now, verified, motion, errors=(0.,0.)):
        if self.status is None:
            return None
        if frame_id <= self.last_frame:
            return None
        self.last_frame = frame_id
        # Include blocking acquisition and processing, using read-start conservatively.
        age = max(0, math.ceil((now - acquired_after) * 1000 - 1e-9))
        status = self.status
        fresh = (status.state == 2
                 and acquired_after > self.fence and 0 <= now - self.last_status_at <= 0.2
                 and 0 <= completed_at - acquired_after <= now - acquired_after
                 and age <= 200)
        self.sequence += 1
        return observation(status.run, status.pose, self.sequence,
                           bool(verified and fresh), motion, age, errors)
