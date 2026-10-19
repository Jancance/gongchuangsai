import unittest

from smart_carrier.models import Obstacle, Observations, PlanAction, RobotMode, RobotState
from smart_carrier.planner import LocalAvoidancePlanner


NOW = 10_000_000_000


def state(**changes):
    values = dict(timestamp_ns=NOW, mode=RobotMode.RUNNING, stm32_heartbeat_age_ms=0, tof_min_mm=1000, lateral_mm=0)
    values.update(changes)
    return RobotState(**values)


def observations(*obstacles, timestamp_ns=NOW, confidence=1.0, rejection_reason=None):
    return Observations(1, timestamp_ns, "cal-v1", confidence, tuple(obstacles), -200, 200, rejection_reason)


class PlannerTests(unittest.TestCase):
    def test_clear_lane_follows(self):
        plan = LocalAvoidancePlanner(240, 20).plan(observations(), state(), NOW)
        self.assertIs(plan.action, PlanAction.FOLLOW)
        self.assertEqual(plan.target_lateral_mm, 0)

    def test_center_obstacle_with_wide_chassis_requests_global_replan(self):
        plan = LocalAvoidancePlanner(240, 20).plan(observations(Obstacle(0, 500, 25, 1)), state(), NOW)
        self.assertIs(plan.action, PlanAction.GLOBAL_REPLAN)

    def test_stale_observation_stops(self):
        planner = LocalAvoidancePlanner(240, 20, max_observation_age_ms=100)
        self.assertIs(planner.plan(observations(timestamp_ns=NOW - 200_000_000), state(), NOW).action, PlanAction.STOP)

    def test_tof_and_heartbeat_fail_closed(self):
        planner = LocalAvoidancePlanner(240, 20)
        self.assertIs(planner.plan(observations(), state(tof_min_mm=100), NOW).action, PlanAction.STOP)
        self.assertIs(planner.plan(observations(), state(stm32_heartbeat_age_ms=501), NOW).action, PlanAction.STOP)


if __name__ == "__main__":
    unittest.main()
