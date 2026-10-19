from __future__ import annotations

from dataclasses import dataclass
import math

from .models import MotionState


@dataclass(frozen=True)
class Detection2D:
    x: float
    y: float
    label: str
    confidence: float = 1.0


@dataclass(frozen=True)
class TrackResult:
    track_id: str
    detection: Detection2D
    motion_state: MotionState
    speed_px_s: float | None


@dataclass
class _Track:
    track_id: str
    detection: Detection2D
    timestamp_ns: int
    vx: float = 0.0
    vy: float = 0.0
    stationary_frames: int = 0
    missed: int = 0


class MultiFrameTracker:
    """Small deterministic tracker with velocity prediction and explicit UNKNOWN."""

    def __init__(
        self,
        *,
        max_distance_px: float = 80.0,
        stationary_speed_px_s: float = 8.0,
        moving_speed_px_s: float = 20.0,
        stable_frames: int = 3,
        max_missed: int = 3,
    ) -> None:
        self.max_distance_px = max_distance_px
        self.stationary_speed_px_s = stationary_speed_px_s
        self.moving_speed_px_s = moving_speed_px_s
        self.stable_frames = stable_frames
        self.max_missed = max_missed
        self._tracks: dict[str, _Track] = {}
        self._next_id = 1

    def update(self, detections: list[Detection2D], timestamp_ns: int) -> list[TrackResult]:
        unmatched = set(range(len(detections)))
        output: list[TrackResult] = []
        for track in sorted(self._tracks.values(), key=lambda item: item.track_id):
            dt = max((timestamp_ns - track.timestamp_ns) / 1e9, 1e-6)
            px = track.detection.x + track.vx * dt
            py = track.detection.y + track.vy * dt
            candidates = [
                (math.hypot(detections[i].x - px, detections[i].y - py), i)
                for i in unmatched
                if detections[i].label == track.detection.label
            ]
            if not candidates or min(candidates)[0] > self.max_distance_px:
                track.missed += 1
                continue
            _, index = min(candidates)
            unmatched.remove(index)
            detection = detections[index]
            vx = (detection.x - track.detection.x) / dt
            vy = (detection.y - track.detection.y) / dt
            speed = math.hypot(vx, vy)
            if speed <= self.stationary_speed_px_s:
                track.stationary_frames += 1
            else:
                track.stationary_frames = 0
            if speed >= self.moving_speed_px_s:
                state = MotionState.MOVING
            elif track.stationary_frames >= self.stable_frames:
                state = MotionState.STATIONARY
            else:
                state = MotionState.UNKNOWN
            track.detection = detection
            track.timestamp_ns = timestamp_ns
            track.vx, track.vy, track.missed = vx, vy, 0
            output.append(TrackResult(track.track_id, detection, state, speed))

        for index in sorted(unmatched):
            detection = detections[index]
            track_id = f"trk-{self._next_id:04d}"
            self._next_id += 1
            self._tracks[track_id] = _Track(track_id, detection, timestamp_ns)
            output.append(TrackResult(track_id, detection, MotionState.UNKNOWN, None))

        for track_id in [key for key, value in self._tracks.items() if value.missed > self.max_missed]:
            del self._tracks[track_id]
        return output
