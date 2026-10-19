import cv2
import numpy as np
import unittest

from smart_carrier.models import Frame
from smart_carrier.models import Observations
from smart_carrier.vision import DarkObstacleDetector, TemporalObstacleFilter, parse_task_code, project_point


def detector():
    return DarkObstacleDetector(
        np.eye(3),
        "front-v1",
        (0, 0, 1, 1),
        dark_l_max=70,
        min_ground_l_median=70,
        max_dark_fraction=0.35,
        min_area_px=50,
        kernel_px=3,
    )


class VisionTests(unittest.TestCase):
    def test_task_code_parser(self):
        self.assertEqual(parse_task_code("156+123+516+231")[0], (1, 5, 6))
        with self.assertRaises(ValueError):
            parse_task_code("999+123")

    def test_projection_identity(self):
        self.assertEqual(project_point(np.eye(3), 12, 34), (12, 34))

    def test_dark_component_is_detected_on_gray_ground(self):
        image = np.full((240, 320, 3), 155, dtype=np.uint8)
        cv2.rectangle(image, (130, 100), (180, 180), (5, 5, 5), -1)
        obs, mask = detector().process(Frame(image, 1_000, 3, "front", "front-v1"))
        self.assertEqual(len(obs.obstacles), 1)
        self.assertAlmostEqual(obs.obstacles[0].x_mm, 155, delta=2)
        self.assertAlmostEqual(obs.obstacles[0].y_mm, 181, delta=2)
        self.assertGreater(int(mask.sum()), 0)

    def test_calibration_mismatch_rejects_frame(self):
        image = np.full((100, 100, 3), 155, dtype=np.uint8)
        obs, _ = detector().process(Frame(image, 1_000, 1, "front", "wrong"))
        self.assertEqual(obs.confidence, 0)
        self.assertTrue(obs.rejection_reason)

    def test_obscured_black_frame_is_not_clear_path(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        obs, _ = detector().process(Frame(image, 1_000, 1, "front", "front-v1"))
        self.assertEqual(obs.confidence, 0)
        self.assertIn("obscured", obs.rejection_reason)

    def test_temporal_filter_requires_three_consistent_frames(self):
        temporal = TemporalObstacleFilter(3, 10)
        observation = Observations(1, 1_000, "front-v1", 1.0)
        self.assertIsNotNone(temporal.update(observation).rejection_reason)
        self.assertIsNotNone(temporal.update(observation).rejection_reason)
        self.assertIsNone(temporal.update(observation).rejection_reason)


if __name__ == "__main__":
    unittest.main()
