"""Two fresh frames with saved fixed-view orbit geometry; no full-turn learning.

Offline candidate only. Orbit geometry must be verified at the current camera
pose. This module neither opens a UART nor issues actuator commands.
"""
import math
import numpy as np

from .continuous_turntable import ContinuousTurntable


class FastTurntable:
    def __init__(self, config):
        self.config = config
        self.base_center = np.asarray(config['center_px'], dtype=float)
        self.whitening = np.asarray(config['whitening'], dtype=float)
        self.tool_points = np.asarray(config['target_tool_points_px'], dtype=float)
        self.leads = np.asarray(config['lead_ms_by_region'], dtype=float) / 1000
        self.max_interval = float(config['maximum_frame_interval_s'])
        if (self.base_center.shape != (2,) or self.whitening.shape != (2, 2)
                or self.tool_points.shape != (3, 2) or self.leads.shape != (3,)
                or not all(np.isfinite(x).all() for x in
                           (self.base_center, self.whitening, self.tool_points, self.leads))
                or np.linalg.eigvalsh(self.whitening).min() <= 0
                or (self.leads <= 0).any() or not math.isfinite(self.max_interval)
                or self.max_interval <= 0):
            raise ValueError('Invalid fixed-view orbit configuration')
        self.reset()

    def reset(self):
        self.center = self.base_center.copy()
        self.previous = None

    def pause(self):
        # Restart CURRENT velocity measurement when returning from arm motion.
        self.previous = None

    def register_rim(self, center_px):
        """Translate the saved fixed-view geometry using the current rim fit."""
        current = np.asarray(center_px, dtype=float)
        if current.shape != (2,) or not np.isfinite(current).all():
            raise ValueError('Current rim centre required')
        self.center = self.base_center + current - np.asarray(
            self.config['calibration_rim_center_px'], dtype=float)
        self.pause()

    def _phase(self, point):
        q = self.whitening @ (np.asarray(point) - self.center)
        return math.atan2(q[1], q[0])

    def update(self, regions, observed_s, now_s, target_color, remaining_colors):
        if not all(map(math.isfinite, (observed_s, now_s))) or now_s < observed_s:
            self.pause()
            return None
        positions = {}
        for item in regions:
            color = item['color_id']
            if color not in remaining_colors or color in positions:
                self.pause()
                return None
            try:
                point = (float(item['center_x_px']), float(item['center_y_px']))
            except (ValueError, TypeError):
                self.pause()
                return None
            if not all(map(math.isfinite, point)):
                self.pause()
                return None
            positions[color] = point
        if target_color not in positions:
            self.pause()
            return None
        old = self.previous
        self.previous = (observed_s, positions)
        if old is None:
            return None
        dt = observed_s - old[0]
        if dt <= 0 or dt > self.max_interval:
            return None
        velocities = []
        for color in positions.keys() & old[1].keys():
            delta = self._phase(positions[color]) - self._phase(old[1][color])
            velocities.append(math.atan2(math.sin(delta), math.cos(delta)) / dt)
        if not velocities:
            return None
        # Cross-colour median for this frame pair, no temporal smoothing.
        omega = float(np.median(velocities))
        if not math.isfinite(omega) or omega == 0:
            return None
        theta = self._phase(positions[target_color]) + omega * (now_s - observed_s)
        targets = np.asarray([self._phase(p) for p in self.tool_points])
        period = 2 * math.pi / abs(omega)
        angle = ((targets - theta) if omega > 0 else (theta - targets)) % (2 * math.pi)
        waits = angle / abs(omega)
        waits += np.maximum(0, np.ceil((self.leads - waits) / period)) * period
        delays = (waits - self.leads) * 1000
        if not np.isfinite(delays).all() or (delays > 0x7fffffff).any():
            return None
        return dict(target_color=target_color, start_delay_ms=delays.tolist(),
                    computed_s=now_s, period_ms=period * 1000,
                    angular_speed_rad_s=omega, learned_one_turn=False,
                    fresh_frame_count=2, actual_grasp_verified=False,
                    physical_geometry_verified=False)

    delays_at_send = staticmethod(ContinuousTurntable.delays_at_send)
