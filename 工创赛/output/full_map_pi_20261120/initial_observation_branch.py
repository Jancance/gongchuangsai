"""First fresh state-6 sequence selects moving bypass or observed 120 degrees.

No camera observation is claimed at the exact buzzer instant. Missing samples
discard continuity, never infer motion across an unknown interval.
"""
import math
try:
    from .initial_rotation import InitialRotation
except ImportError:  # Standalone Pi bundle.
    from initial_rotation import InitialRotation


class InitialObservationRotation:
    def __init__(self, orbit_config, pickup_config):
        self.rotation = InitialRotation(orbit_config)
        self.motion = pickup_config['motion']
        self.reset()

    def reset(self):
        self.rotation.reset()
        self.branch = None
        self._clear_samples()

    def _clear_samples(self):
        self.previous = None
        self.stable = []
        self.moving = 0

    def pause(self):
        self.rotation.pause()
        # A new observation pose must not inherit an already-open phase gate.
        # Keep the registered rim translation while discarding the old phase.
        self.rotation.progress = 0.0
        self.rotation.opened = False
        self.branch = None
        self._clear_samples()

    def register_rim(self, center):
        self.rotation.register_rim(center)
        self.branch = None
        self._clear_samples()

    def update(self, detection, observed_s, first_color, remaining):
        # Always advance the observed phase, even while classifying motion.
        rotation_ready = self.rotation.update(detection, observed_s, first_color)
        if self.branch is not None:
            return self.branch == 'initial_moving' or rotation_ready
        try:
            regions = detection.get('regions', [])
            colors = [r['color_id'] for r in regions]
            centers = {r['color_id']: (float(r['center_x_px']), float(r['center_y_px']))
                       for r in regions}
            if (not detection.get('calibrated') or not math.isfinite(observed_s)
                    or not colors or len(colors) != len(set(colors))
                    or not set(colors).issubset(remaining)
                    or not all(math.isfinite(v) for xy in centers.values() for v in xy)):
                raise ValueError('No fresh matched observation')
        except (KeyError, ValueError, TypeError):
            self._clear_samples()
            return False
        old = self.previous
        if old is not None:
            dt = observed_s - old[0]
            if dt <= 0 or dt > self.motion['maximum_frame_interval_s']:
                self._clear_samples()
                old = None
            elif dt < self.motion['minimum_frame_interval_s']:
                return False
            elif centers.keys() != old[1].keys():
                # Do not turn visibility changes into a zero-speed sample.
                self._clear_samples()
                old = None
        self.previous = (observed_s, centers)
        if old is None:
            self.stable = [(observed_s, centers)]
            return False
        speed = max(math.dist(centers[c], old[1][c]) / dt for c in centers)
        if speed >= self.motion['moving_speed_px_s']:
            self.moving += 1
            self.stable = [(observed_s, centers)]
            if self.moving >= self.motion['required_moving_frames']:
                self.branch = 'initial_moving'
        elif speed <= self.motion['near_stop_speed_px_s']:
            self.moving = 0
            if any(math.dist(centers[c], points[c]) / (observed_s - stamp)
                   > self.motion['near_stop_speed_px_s']
                   for stamp, points in self.stable for c in centers):
                self.stable = []
            self.stable.append((observed_s, centers))
            self.stable = self.stable[-5:]
            if len(self.stable) >= 5:
                self.branch = 'initial_stopped'
        else:
            self.moving = 0
            self.stable = [(observed_s, centers)]
        return self.branch == 'initial_moving' or (self.branch == 'initial_stopped' and rotation_ready)

    def diagnostic(self):
        result = self.rotation.diagnostic()
        result.update(initial_branch=self.branch or 'classifying',
                      initial_stable_frames=len(self.stable), initial_moving_frames=self.moving,
                      required_stopped_frames=5,
                      ready=self.branch == 'initial_moving' or
                      (self.branch == 'initial_stopped' and self.rotation.opened))
        return result
