import unittest

import numpy as np

from smart_carrier.calibration import estimate_ground_homography


class CalibrationTests(unittest.TestCase):
    def test_recovers_known_affine_homography(self):
        image = np.array([[0, 0], [100, 0], [100, 100], [0, 100], [50, 50]], dtype=float)
        ground = image * np.array([2.0, 3.0]) + np.array([-20.0, 10.0])
        matrix, rms, inliers = estimate_ground_homography(image, ground)
        self.assertLess(rms, 1e-6)
        self.assertTrue(inliers.all())
        point = matrix @ np.array([50.0, 50.0, 1.0])
        point /= point[2]
        np.testing.assert_allclose(point[:2], [80.0, 160.0], atol=1e-6)

    def test_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            estimate_ground_homography(np.zeros((3, 2)), np.zeros((3, 2)))


if __name__ == "__main__":
    unittest.main()
