"""Source state2 host-frame gate; raw rim center stays unfiltered.

Read-completion metadata cannot prove sensor exposure time or zero USB backlog.
An unready gate emits only valid=false observations; it creates no MCU fault.
"""
from collections import deque
import math


class SourceAlignmentSettle:
    def __init__(self, config):
        self.dwell_s = float(config['settle_ms']) / 1000.0
        self.required = int(config['required_frames'])
        self.max_pair_px = float(config['maximum_pair_distance_px'])
        if not (math.isfinite(self.dwell_s) and self.dwell_s >= 0 and self.required >= 2 and
                math.isfinite(self.max_pair_px) and self.max_pair_px > 0):
            raise ValueError('Invalid source alignment settle configuration')
        if config.get('count_frames_after_dwell') is not True:
            raise ValueError('Source alignment frames must follow the dwell')
        self.key = None
        self.invalidate('not_state2')

    def invalidate(self, reason='inactive'):
        self.key = None
        self.epoch = None
        self.generation = None
        self.fence = self.last_seq = self.last_stamp = None
        self.centers = deque(maxlen=self.required)
        self.ready = False
        self.reason = reason
        self.max_pair_seen = None

    def _begin(self, key, now, reason):
        self.invalidate(reason)
        self.key, self.epoch = key, now

    def receive(self, status, received, link_was_expired=False):
        if not isinstance(received, (int, float)) or not math.isfinite(received):
            self.invalidate('unknown_status_time')
            return
        if status.get('state') != 2:
            self.invalidate('not_state2')
            return
        key = (status['run'], status['pose'], status['item'])
        if self.key != key or link_was_expired:
            self._begin(key, received, 'await_post_pose_read')

    def reject_fit(self):
        self.centers.clear()
        self.ready = False
        self.max_pair_seen = None
        self.reason = 'rim_fit_invalid'

    def frame(self, metadata, now, center, fit_valid):
        self.ready = False
        if self.key is None:
            self.reason = 'not_state2'
            return False
        if not math.isfinite(now):
            self.invalidate('unknown_host_time')
            return False
        if now < self.epoch:
            self._begin(self.key, now, 'host_clock_reset')
            return False
        try:
            generation, seq, stamp = metadata['generation'], metadata['sequence'], metadata['completed']
            if (generation is None or type(seq) is not int or seq < 1 or
                    not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp > now):
                raise ValueError('Invalid capture metadata')
        except (TypeError, KeyError, ValueError):
            self._begin(self.key, now, 'unknown_capture_metadata')
            return False
        if self.generation is not None and generation != self.generation:
            self._begin(self.key, now, 'capture_generation_changed')
        self.generation = generation
        if stamp <= self.epoch:
            self.reason = 'host_read_before_pose'
            return False
        if self.fence is None:
            self.fence = self.last_seq = seq
            self.last_stamp = stamp
            self.reason = 'first_post_pose_read_discarded'
            return False
        if seq <= self.last_seq or stamp <= self.last_stamp:
            self.reason = 'duplicate_or_out_of_order_capture'
            return False
        self.last_seq, self.last_stamp = seq, stamp
        if now - stamp > .200:
            self.reject_fit()
            self.reason = 'existing_200ms_host_age_exceeded'
            return False
        try:
            x, y = center
            if not fit_valid or not all(math.isfinite(v) for v in (x, y)):
                raise ValueError('Invalid rim fit')
        except (TypeError, ValueError):
            self.reject_fit()
            return False
        if stamp - self.epoch < self.dwell_s:
            self.reason = 'post_pose_dwell'
            return False
        self.centers.append((float(x), float(y)))
        if len(self.centers) < self.required:
            self.reason = 'collecting_post_dwell_frames'
            return False
        self.max_pair_seen = max(math.dist(a, b) for a in self.centers for b in self.centers)
        self.ready = self.max_pair_seen <= self.max_pair_px
        self.reason = 'stable_raw_centers' if self.ready else 'raw_center_still_moving'
        return self.ready

    def diagnostic(self):
        return dict(ready=self.ready, reason=self.reason, pose=self.key,
                    pose_received_monotonic_s=self.epoch, settle_ms=self.dwell_s * 1000,
                    required_frames=self.required, accepted_frames=len(self.centers),
                    maximum_pair_distance_px=self.max_pair_px, measured_max_pair_distance_px=self.max_pair_seen,
                    generation=self.generation, fence_sequence=self.fence, last_sequence=self.last_seq,
                    last_read_completed_monotonic_s=self.last_stamp,
                    sensor_exposure_time_verified=False)
