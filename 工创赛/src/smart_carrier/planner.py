from __future__ import annotations

import time
from typing import Iterable

from .models import Observations, PlanAction, RobotState, SafePlan


class LocalAvoidancePlanner:
    def __init__(
        self,
        chassis_width_mm: float,
        boundary_margin_mm: float,
        max_observation_age_ms: float = 150.0,
        minimum_confidence: float = 0.65,
        cruise_speed_mm_s: float = 250.0,
    ) -> None:
        self.half_width = chassis_width_mm / 2.0
        self.margin = boundary_margin_mm
        self.max_age_ms = max_observation_age_ms
        self.minimum_confidence = minimum_confidence
        self.cruise_speed = cruise_speed_mm_s

    def plan(self, observations: Observations, state: RobotState, now_ns: int | None = None) -> SafePlan:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        age_ms = (now_ns - observations.timestamp_ns) / 1_000_000
        if age_ms < 0 or age_ms > self.max_age_ms:
            return self._stop("observation is stale")
        if observations.rejection_reason or observations.confidence < self.minimum_confidence:
            return self._stop(observations.rejection_reason or "observation confidence is too low")
        if state.stm32_heartbeat_age_ms >= 500:
            return self._stop("STM32 heartbeat timeout")
        if state.tof_min_mm is not None and state.tof_min_mm <= 180:
            return self._stop("ToF emergency stop threshold")

        center_min = observations.lane_left_mm + self.half_width + self.margin
        center_max = observations.lane_right_mm - self.half_width - self.margin
        if center_min > center_max:
            return self._stop("robot cannot fit inside lane with configured margin")

        candidates = self._candidate_offsets(center_min, center_max)
        feasible = [x for x in candidates if self._is_clear(x, observations.obstacles)]
        if not feasible:
            return SafePlan(PlanAction.GLOBAL_REPLAN, state.lateral_mm, 0.0, "no collision-free local corridor")

        target = min(feasible, key=lambda value: (abs(value - state.lateral_mm), abs(value)))
        speed = self.cruise_speed
        if state.stm32_heartbeat_age_ms >= 200 or (state.tof_min_mm is not None and state.tof_min_mm <= 350):
            speed *= 0.35
        return SafePlan(
            PlanAction.FOLLOW,
            target,
            speed,
            "clear local corridor",
            ((state.lateral_mm, 0.0), (target, 350.0), (target, 800.0)),
        )

    @staticmethod
    def _candidate_offsets(minimum: float, maximum: float, step: float = 10.0) -> list[float]:
        count = max(1, int((maximum - minimum) // step))
        values = [minimum + i * step for i in range(count + 1)] + [maximum, 0.0]
        return sorted({round(v, 6) for v in values if minimum <= v <= maximum})

    def _is_clear(self, lateral_mm: float, obstacles: Iterable) -> bool:
        for obstacle in obstacles:
            if obstacle.y_mm < -100 or obstacle.y_mm > 1200:
                continue
            required = self.half_width + self.margin + obstacle.radius_mm
            if abs(lateral_mm - obstacle.x_mm) < required:
                return False
        return True

    @staticmethod
    def _stop(reason: str) -> SafePlan:
        return SafePlan(PlanAction.STOP, 0.0, 0.0, reason)

