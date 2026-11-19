"""Stage3 centre-ring view, using the supplied existing ring detector.

Only the present middle-ring centre is compared with the current frame centre.
Historical reference pixels or the source-disk H are not copied into this zone.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RingView:
    detected: bool
    stable: bool
    center_px: tuple | None
    error_px: tuple | None
    yaw_error_deg: float | None
    body_delta_mm: tuple | None
    within_tolerance: bool
    reason: str


class RingAligner:
    def __init__(self, detector, zone_parameters, tuning):
        self.detector = detector;self.zone = zone_parameters;self.tuning = tuning

    def process(self, frame):
        detection = self.detector.detect(frame)
        if not detection.detected or len(detection.centers_px) != 3:
            return RingView(False, False, None, None, None, None, False, '等待当前三个圆环，不能用预测圆心替代')
        middle = tuple(map(float, detection.centers_px[1]));height, width = frame.shape[:2]
        error = np.asarray(middle)-[width/2, height/2]
        if not np.isfinite(error).all():
            return RingView(False, False, None, None, None, None, False, '圆心无效')
        yaw = (float(detection.station_yaw_deg)-self.zone['desired_line_angle_deg']+180)%360-180
        aligned = bool(detection.stable and np.linalg.norm(error) <= self.tuning['alignment_pixel_tolerance'] and
                       abs(yaw) <= self.tuning['alignment_yaw_tolerance_deg'])
        matrix = self.zone.get('pixel_to_body_mm');delta = None
        reason = '像素观察，等待本区实际像素到车体方向/尺度绑定'
        if matrix is not None and self.zone.get('pixel_to_body_verified'):
            matrix = np.asarray(matrix, dtype=float)
            if matrix.shape == (2, 2) and np.isfinite(matrix).all():
                delta = tuple(map(float, matrix@error));reason = '当前区域已绑定的纠偏观测'
        if not detection.stable:reason = '圆环尚未稳定，继续观察，不锁整车故障'
        return RingView(True, bool(detection.stable), middle, tuple(map(float,error)), yaw, delta, aligned, reason)
