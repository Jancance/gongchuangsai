"""Learn actual indexed-disc phase; prediction is advice, never a motor trigger.

The caller supplies one event per visually confirmed stop after real motion.
Region coordinates must already belong to the current camera pose. No image
coordinates, physical inventory, or successful grip is invented here.
"""
from dataclasses import dataclass
import math
from statistics import median


@dataclass(frozen=True)
class StopEvent:
    sequence: int
    observed_s: float
    colors: tuple[int, int, int]


@dataclass(frozen=True)
class PhasePrediction:
    stop_start_s: float
    colors: tuple[int, int, int]
    observed_period_s: float
    observed_timing_spread_s: float
    starts_grasp: bool = False


def _valid_colors(colors, remaining):
    if len(colors) != 3 or not remaining or len(set(remaining)) != len(remaining):
        return False
    if any(type(c) is not int or not 1 <= c <= 6 for c in remaining):
        return False
    present = [c for c in colors if c != 0]
    return (bool(present) and all(type(c) is int and c in remaining for c in present)
            and len(set(present)) == len(present))


class IndexedDiscPhase:
    """A small, context-bound model of observed source-region permutations."""

    def __init__(self, context_id):
        if not context_id:
            raise ValueError('需要当前盘面区域坐标系标识')
        self.context_id = context_id
        self.events = []
        self.permutation = None
        self._votes = {}
        self._periods = []

    def reset(self, context_id):
        self.__init__(context_id)

    def record_stop(self, event, *, context_id, remaining_colors, after_motion):
        """Accept an actual new stop event; stationary repeated frames are not cycles."""
        if (context_id != self.context_id or not after_motion or
                type(event.sequence) is not int or event.sequence <= 0 or
                not math.isfinite(event.observed_s) or
                not _valid_colors(event.colors, remaining_colors)):
            return False
        if self.events:
            previous = self.events[-1]
            if event.sequence <= previous.sequence or event.observed_s <= previous.observed_s:
                return False
            visible_before = tuple(c if c in remaining_colors else 0 for c in previous.colors)
            if visible_before == event.colors:
                return False  # Repeated stable frames do not demonstrate another indexed stop.
            # A gap in event numbering may contain an unseen turn. Re-anchor
            # without treating the gap as one measured period/direction vote.
            if event.sequence == previous.sequence + 1:
                before = visible_before
                matches = [p for p in ((1, 2, 0), (2, 0, 1))
                           if tuple(before[i] for i in p) == event.colors]
                if len(matches) == 1:
                    direction = matches[0]
                    self._votes[direction] = self._votes.get(direction, 0) + 1
                    if self.permutation is not None and direction != self.permutation:
                        self._votes = {direction: 1}
                        self._periods.clear()
                        self.permutation = None
                    self._periods.append(event.observed_s - previous.observed_s)
                    # Same rolling five-stop/four-period window as the MATLAB replay.
                    self._periods = self._periods[-4:]
                    if self._votes[direction] >= 3:
                        self.permutation = direction
                else:
                    self._periods.clear()
                    self._votes.clear()
                    self.permutation = None
            else:
                self._periods.clear()
        self.events.append(event)
        self.events = self.events[-8:]
        return True

    def predict_next(self, now_s, *, remaining_colors):
        if (self.permutation is None or len(self._periods) < 3 or not self.events
                or not math.isfinite(now_s)):
            return None
        last = self.events[-1]
        period = median(self._periods)
        spread = max(abs(p - period) for p in self._periods)
        next_start = last.observed_s + period
        # Once the expected next stop has passed, an unconfirmed old phase is
        # not extrapolated indefinitely. Wait for another actual stop event.
        if now_s < last.observed_s or now_s > next_start + max(spread, .20):
            return None
        colors = tuple(c if c in remaining_colors else 0 for c in last.colors)
        return PhasePrediction(next_start, tuple(colors[i] for i in self.permutation),
                               period, spread)


def confirmed_target_region(*, target_color, observed_colors, remaining_colors,
                            observed_s, now_s, stationary, pose_id, observed_pose_id,
                            maximum_age_s=.20):
    """Only a present, fresh, stopped target can confirm an advised approach."""
    if (not pose_id or pose_id != observed_pose_id or not stationary or
            not all(math.isfinite(v) for v in (observed_s, now_s, maximum_age_s)) or
            maximum_age_s <= 0 or not 0 <= now_s - observed_s <= maximum_age_s or
            target_color not in remaining_colors or
            not _valid_colors(observed_colors, remaining_colors)):
        return None
    regions = [i for i, color in enumerate(observed_colors) if color == target_color]
    return regions[0] if len(regions) == 1 else None


def actual_rise_clearance(*, pick_position_deg, current_position_deg,
                          feedback_age_ms, down_sign=1, pulses_per_mm=1600):
    """Return actual rise and strict >65mm clearance from fresh drive feedback."""
    values = (pick_position_deg, current_position_deg, feedback_age_ms, pulses_per_mm)
    if (not all(math.isfinite(v) for v in values) or down_sign not in (-1, 1)
            or pulses_per_mm <= 0 or not 0 <= feedback_age_ms <= 150):
        return None, False
    rise_mm = (pick_position_deg - current_position_deg) * down_sign * (3200 / 360) / pulses_per_mm
    return rise_mm, rise_mm > 65
