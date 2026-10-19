"""Metric moving-target prediction for a future P1 planner interface.

This module has no camera, serial, motor or grasp side effects. A prediction
does not establish gripper reachability, collision safety or grasp success.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TargetMeasurement:
    track_id: str
    source_id: str
    calibration_id: str
    timestamp_ns: int
    field_xy_mm: tuple[float, float] | None
    position_error_bound_mm: float | None
    coordinate_frame: str = "UNKNOWN"
    timestamp_source: str = "UNKNOWN"
    ego_motion_compensated: bool = False
    identity_valid: bool = False
    visible: bool = False


@dataclass(frozen=True)
class PredictorLimits:
    max_host_age_ms: float
    min_interval_s: float
    max_interval_s: float
    max_target_speed_mm_s: float
    max_acceleration_mm_s2: float
    max_horizon_s: float
    max_position_error_bound_mm: float

    def __post_init__(self) -> None:
        values = vars(self)
        if any(not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError("all predictor limits must be finite and positive")
        if self.min_interval_s >= self.max_interval_s:
            raise ValueError("min_interval_s must be below max_interval_s")


@dataclass(frozen=True)
class TargetGuidance:
    status: str
    reason: str
    track_id: str | None = None
    observed_field_xy_mm: tuple[float, float] | None = None
    predicted_field_xy_mm: tuple[float, float] | None = None
    velocity_field_mm_s: tuple[float, float] | None = None
    horizon_s: float | None = None
    estimated_error_bound_mm: float | None = None


class MovingTargetPredictor:
    """Fail-closed three-observation predictor; one target identity per instance."""

    def __init__(self, limits: PredictorLimits) -> None:
        self.limits = limits
        self._recent: deque[TargetMeasurement] = deque(maxlen=3)

    def reset(self) -> None:
        self._recent.clear()

    def _reject(self, reason: str, *, reset: bool = True) -> TargetGuidance:
        if reset:
            self.reset()
        return TargetGuidance("UNKNOWN", reason)

    def update(
        self,
        measurement: TargetMeasurement,
        *,
        now_ns: int,
        contact_horizon_s: float | None,
        workspace_bounds_mm: tuple[float, float, float, float] | None,
    ) -> TargetGuidance:
        l = self.limits
        if not measurement.visible or not measurement.identity_valid or not measurement.track_id:
            return self._reject("TARGET_NOT_VISIBLE_OR_IDENTITY_UNCERTAIN")
        if measurement.coordinate_frame != "field" or not measurement.ego_motion_compensated:
            return self._reject("METRIC_FRAME_OR_EGO_COMPENSATION_MISSING")
        if measurement.timestamp_source != "sensor":
            return self._reject("SENSOR_TIMESTAMP_NOT_VERIFIED")
        if not measurement.calibration_id or not measurement.source_id:
            return self._reject("SOURCE_OR_CALIBRATION_MISSING")
        p = measurement.field_xy_mm
        sigma = measurement.position_error_bound_mm
        if p is None or len(p) != 2 or not all(math.isfinite(x) for x in p):
            return self._reject("METRIC_POSITION_MISSING")
        if sigma is None or not math.isfinite(sigma) or sigma <= 0:
            return self._reject("POSITION_ERROR_BOUND_MISSING")
        if now_ns < measurement.timestamp_ns or (now_ns - measurement.timestamp_ns) / 1e6 > l.max_host_age_ms:
            return self._reject("STALE_OR_FUTURE_MEASUREMENT")
        if contact_horizon_s is None or not math.isfinite(contact_horizon_s) or not 0 < contact_horizon_s <= l.max_horizon_s:
            return self._reject("CONTACT_TIME_NOT_MEASURED")
        if workspace_bounds_mm is None or len(workspace_bounds_mm) != 4 or not all(math.isfinite(x) for x in workspace_bounds_mm):
            return self._reject("WORKSPACE_BOUNDS_MISSING")
        xmin, ymin, xmax, ymax = workspace_bounds_mm
        if xmin >= xmax or ymin >= ymax:
            return self._reject("WORKSPACE_BOUNDS_INVALID")

        if self._recent:
            prev = self._recent[-1]
            if (measurement.track_id != prev.track_id or measurement.source_id != prev.source_id
                    or measurement.calibration_id != prev.calibration_id):
                self.reset()
            else:
                dt = (measurement.timestamp_ns - prev.timestamp_ns) / 1e9
                if dt <= 0:
                    return self._reject("NONINCREASING_TIMESTAMP")
                if dt < l.min_interval_s or dt > l.max_interval_s:
                    self.reset()
        self._recent.append(measurement)
        if len(self._recent) < 3:
            return TargetGuidance("UNKNOWN", "NEED_THREE_FRESH_OBSERVATIONS")

        a, b, c = self._recent
        dt1 = (b.timestamp_ns - a.timestamp_ns) / 1e9
        dt2 = (c.timestamp_ns - b.timestamp_ns) / 1e9
        v1 = tuple((b.field_xy_mm[i] - a.field_xy_mm[i]) / dt1 for i in range(2))
        v2 = tuple((c.field_xy_mm[i] - b.field_xy_mm[i]) / dt2 for i in range(2))
        velocity = tuple((c.field_xy_mm[i] - a.field_xy_mm[i]) / (dt1 + dt2) for i in range(2))
        if math.hypot(*velocity) > l.max_target_speed_mm_s:
            return self._reject("SPEED_OUT_OF_RANGE")
        observed_accel = math.dist(v1, v2) / ((dt1 + dt2) / 2)
        if observed_accel > l.max_acceleration_mm_s2:
            return self._reject("MOTION_MODEL_INCONSISTENT")

        h = contact_horizon_s
        predicted = tuple(c.field_xy_mm[i] + velocity[i] * h for i in range(2))
        # Development bound, conditional on measured input uncertainty and
        # acceleration limit. It is NOT a validated physical confidence level.
        error = (max(x.position_error_bound_mm for x in self._recent)
                 + 2 * max(x.position_error_bound_mm for x in self._recent) * h / (dt1 + dt2)
                 + 0.5 * l.max_acceleration_mm_s2 * h * h)
        if error > l.max_position_error_bound_mm:
            return self._reject("PREDICTION_ERROR_TOO_LARGE")
        if not (xmin + error <= predicted[0] <= xmax - error
                and ymin + error <= predicted[1] <= ymax - error):
            return self._reject("PREDICTION_OUTSIDE_WORKSPACE")

        return TargetGuidance(
            "GUIDANCE_ONLY", "FRESH_COMPENSATED_METRIC_TRACK",
            track_id=c.track_id,
            observed_field_xy_mm=c.field_xy_mm,
            predicted_field_xy_mm=predicted,
            velocity_field_mm_s=velocity,
            horizon_s=h,
            estimated_error_bound_mm=error,
        )
