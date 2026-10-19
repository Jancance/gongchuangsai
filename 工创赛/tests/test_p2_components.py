import unittest

import numpy as np

from smart_carrier.models import (
    ActionResult,
    ActionStatus,
    MotionState,
    Obstacle,
    Pose2D,
)
from smart_carrier.navigation import (
    GrayLaneDetector,
    IntersectionHysteresis,
    TopologyPlanner,
    all_corners_inside_zone,
    conservative_obstacle_fusion,
)
from smart_carrier.p2_state_machine import P2Phase, P2StateMachine
from smart_carrier.quality import FrameQualityGate
from smart_carrier.tracking import Detection2D, MultiFrameTracker
from smart_carrier.verification import EvidenceGate


class P2ComponentTests(unittest.TestCase):
    def test_frame_quality_gate_rejects_stale_and_blurry_frames(self):
        gate = FrameQualityGate(
            max_age_ms=100.0,
            min_sharpness=20.0,
            max_clipped_ratio=0.9,
        )
        flat = np.full((60, 80, 3), 100, dtype=np.uint8)
        result = gate.evaluate(flat, timestamp_ns=0, now_ns=200_000_000)
        self.assertFalse(result.accepted)
        self.assertIn("STALE_FRAME", result.reasons)
        self.assertIn("BLUR", result.reasons)

    def test_tracker_uses_three_motion_states(self):
        tracker = MultiFrameTracker(
            max_distance_px=30.0,
            stationary_speed_px_s=2.0,
            moving_speed_px_s=10.0,
            stable_frames=2,
        )
        first = tracker.update([Detection2D(10.0, 10.0, "red")], timestamp_ns=0)[0]
        self.assertEqual(first.motion_state, MotionState.UNKNOWN)

        second = tracker.update([Detection2D(10.0, 10.0, "red")], timestamp_ns=1_000_000_000)[0]
        self.assertEqual(second.motion_state, MotionState.UNKNOWN)
        third = tracker.update([Detection2D(10.0, 10.0, "red")], timestamp_ns=2_000_000_000)[0]
        self.assertEqual(third.motion_state, MotionState.STATIONARY)

        moving = tracker.update([Detection2D(30.0, 10.0, "red")], timestamp_ns=3_000_000_000)[0]
        self.assertEqual(moving.motion_state, MotionState.MOVING)

    def test_lane_detector_and_intersection_hysteresis(self):
        image = np.zeros((100, 160, 3), dtype=np.uint8)
        image[:, 40:120] = 128
        detector = GrayLaneDetector(
            mm_per_pixel=1.0,
            min_l=40,
            max_l=220,
            max_chroma=10,
            min_area_ratio=0.1,
        )
        obs = detector.detect(image)
        self.assertGreater(obs.confidence, 0.5)
        self.assertAlmostEqual(obs.lateral_error_mm, 0.0, delta=2.0)

        hysteresis = IntersectionHysteresis(enter_frames=2, exit_frames=2)
        self.assertEqual(hysteresis.update(("LEFT", "STRAIGHT", "RIGHT"))[0], ())
        self.assertEqual(
            hysteresis.update(("LEFT", "STRAIGHT", "RIGHT"))[0],
            ("LEFT", "RIGHT", "STRAIGHT"),
        )
        self.assertEqual(hysteresis.update(())[0], ("LEFT", "RIGHT", "STRAIGHT"))
        self.assertEqual(hysteresis.update(())[0], ())

    def test_topology_planner_and_home_zone(self):
        planner = TopologyPlanner(
            {
                "A": {"B": 1.0, "C": 5.0},
                "B": {"A": 1.0, "C": 1.0},
                "C": {"A": 5.0, "B": 1.0},
            }
        )
        self.assertEqual(planner.route("A", "C"), ("A", "B", "C"))
        self.assertTrue(
            all_corners_inside_zone(
                ((1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0)),
                (0.0, 0.0, 10.0, 10.0),
            )
        )
        self.assertFalse(
            all_corners_inside_zone(
                ((-1.0, 1.0), (9.0, 1.0)),
                (0.0, 0.0, 10.0, 10.0),
            )
        )

    def test_obstacle_safety_is_conservative(self):
        obstacle = Obstacle(0.0, 100.0, 25.0, 0.9)
        self.assertEqual(
            conservative_obstacle_fusion((obstacle,), (500.0, 500.0, 500.0), stop_threshold_mm=150.0),
            (True, "VISUAL_OBSTACLE"),
        )
        self.assertEqual(
            conservative_obstacle_fusion((), (500.0, 100.0, 500.0), stop_threshold_mm=150.0),
            (True, "TOF_STOP"),
        )
        self.assertEqual(
            conservative_obstacle_fusion((), (500.0, 500.0, 500.0), stop_threshold_mm=150.0),
            (False, "CLEAR"),
        )
        self.assertEqual(
            conservative_obstacle_fusion((), (None, 500.0, 500.0), stop_threshold_mm=150.0),
            (True, "TOF_UNKNOWN"),
        )

    def test_evidence_gate_requires_action_and_current_sensor_evidence(self):
        gate = EvidenceGate()
        action = ActionResult(seq=7, status=ActionStatus.DONE)
        accepted = gate.verify_tray_load(
            action,
            visual_present=True,
            tray_sensor=True,
        )
        self.assertTrue(accepted.accepted)
        rejected = gate.verify_tray_load(
            ActionResult(seq=8, status=ActionStatus.DONE),
            visual_present=True,
            tray_sensor=False,
        )
        self.assertFalse(rejected.accepted)
        self.assertEqual(rejected.reason, "TRAY_SENSOR_NOT_OCCUPIED")

    def test_state_machine_has_bounded_recovery(self):
        sm = P2StateMachine(max_retries=2)
        sm.transition(P2Phase.NAVIGATE, "task accepted")
        sm.perception_failure("camera_lost", elapsed_ms=100)
        self.assertEqual(sm.phase, P2Phase.RECOVER)
        sm.perception_failure("camera_lost", elapsed_ms=200)
        self.assertEqual(sm.phase, P2Phase.RECOVER)
        sm.perception_failure("camera_lost", elapsed_ms=300)
        self.assertEqual(sm.phase, P2Phase.SAFE_STOP)


if __name__ == "__main__":
    unittest.main()
