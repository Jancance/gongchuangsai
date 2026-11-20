"""Existing ROI rules plus configurable fresh low-speed frames before committing."""
from collections import deque
import math


class StoppedFrameTrigger:
    def __init__(self, config, required_frames=3):
        self.config = config
        self.required_frames = int(required_frames)
        if self.required_frames < 2 or self.required_frames != required_frames:
            raise ValueError("required_frames must be an integer >= 2")
        self.history = deque(maxlen=self.required_frames)
        self.reset()

    def reset(self):
        self.history.clear()
        self.context = None

    pause = reset

    def diagnostic(self):
        return dict(required_frames=self.required_frames, stable_frames=len(self.history),
                    near_stop_speed_px_s=self.config['motion']['near_stop_speed_px_s'])

    def update(self, detection, timestamp, target, remaining):
        if not detection.get('calibrated') or not math.isfinite(timestamp):
            self.reset(); return None
        regions = detection.get('regions', [])
        colors = [r['color_id'] for r in regions]
        if (not colors or len(colors) > len(remaining) or len(set(colors)) != len(colors)
                or not set(colors).issubset(remaining)):
            self.reset(); return None
        slots = [0, 0, 0]; target_region = None; centers = {}
        margin = self.config['detection']['slot_interior_margin_px']
        for r in regions:
            x = float(r['center_x_px']); y = float(r['center_y_px'])
            if not math.isfinite(x) or not math.isfinite(y):
                self.reset(); return None
            centers[r['color_id']] = (x, y)
            matches = []
            for i, name in enumerate(('left', 'right', 'down')):
                box = self.config['regions'][name]
                if (box['x']+margin <= x <= box['x']+box['width']-margin
                        and box['y']+margin <= y <= box['y']+box['height']-margin):
                    matches.append(i)
            if len(matches) > 1:
                self.reset(); return None
            if matches:
                i = matches[0]
                if slots[i]:
                    self.reset(); return None
                slots[i] = r['color_id']
                if r['color_id'] == target: target_region = i
        if target_region is None:
            self.reset(); return None
        context = (target, tuple(remaining), target_region, tuple(slots), tuple(sorted(colors)))
        if context != self.context:
            self.reset(); self.context = context
        motion = self.config['motion']
        if self.history:
            dt = timestamp-self.history[-1][0]
            if dt <= 0 or dt > motion['maximum_frame_interval_s']:
                self.history.clear()
            elif dt < motion['minimum_frame_interval_s']:
                return None
            elif any(math.hypot(xy[0]-old[c][0], xy[1]-old[c][1])/(timestamp-stamp)
                     > motion['near_stop_speed_px_s']
                     for stamp, old in self.history for c, xy in centers.items()):
                self.history.clear()
        self.history.append((timestamp, centers))
        if len(self.history) < self.required_frames: return None
        delays = [1, 1, 1]; delays[target_region] = 0
        return dict(target_color=target, region=target_region, colors=slots,
                    start_delay_ms=delays, prediction_used=False, confirmation_frames=self.required_frames,
                    stopped=True, stop_speed_limit_px_s=motion['near_stop_speed_px_s'])
