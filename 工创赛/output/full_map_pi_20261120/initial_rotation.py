"""Observed net phase before first pickup; no timer, UART or actuator API."""
import math
import numpy as np


class InitialRotation:
    def __init__(self, config):
        self.base_center = np.asarray(config['center_px'], dtype=float)
        self.reference_rim = np.asarray(config['calibration_rim_center_px'], dtype=float)
        self.whitening = np.asarray(config['whitening'], dtype=float)
        self.max_interval = float(config['maximum_frame_interval_s'])
        if (self.base_center.shape != (2,) or self.reference_rim.shape != (2,)
                or self.whitening.shape != (2, 2)
                or not all(np.isfinite(v).all() for v in
                           (self.base_center, self.reference_rim, self.whitening))
                or not math.isfinite(self.max_interval) or self.max_interval <= 0):
            raise ValueError('Invalid saved observation geometry')
        self.reset()

    def reset(self):
        self.center = self.base_center.copy()
        self.previous = None
        self.progress = 0.0
        self.opened = False

    def register_rim(self, center):
        current = np.asarray(center, dtype=float)
        if current.shape != (2,) or not np.isfinite(current).all():
            raise ValueError('Current rim required')
        self.reset()
        self.center = self.base_center + current - self.reference_rim

    def pause(self):
        # Never infer the phase traversed while detection was unavailable.
        self.previous = None
        if not self.opened:
            self.progress = 0.0

    def update(self, detection, observed_s, first_color):
        if self.opened:
            return True
        old = self.previous
        self.previous = None
        if not detection.get('calibrated') or not math.isfinite(observed_s):
            self.pause()
            return False
        regions = detection.get('regions', [])
        colors = [r['color_id'] for r in regions]
        if colors.count(first_color) != 1:
            self.pause()
            return False
        item = regions[colors.index(first_color)]
        try:
            point = np.asarray([float(item['center_x_px']), float(item['center_y_px'])])
        except (KeyError, ValueError, TypeError):
            self.pause()
            return False
        q = self.whitening @ (point - self.center)
        if not np.isfinite(q).all() or not np.any(q):
            self.pause()
            return False
        phase = math.atan2(q[1], q[0])
        self.previous = (observed_s, phase)
        if old is None:
            return False
        if not 0 < observed_s-old[0] <= self.max_interval:
            self.progress = 0.0
            return False
        delta = phase-old[1]
        self.progress += math.atan2(math.sin(delta), math.cos(delta))
        self.opened = abs(self.progress) >= math.radians(120)
        return self.opened

    def diagnostic(self):
        return dict(required_deg=120, observed_net_deg=math.degrees(self.progress),
                    ready=self.opened, fresh_reference=self.previous is not None,
                    physical_angle_calibrated=False)
