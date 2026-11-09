import unittest

import cv2
import numpy as np

from smart_carrier.source_markers import fit_circle_in_plane, project, tcp_relative_offsets


class SourceTcpGeometryTests(unittest.TestCase):
    def test_projected_ellipse_center_cannot_replace_plane_circle_fit(self):
        angle = np.linspace(0, 2*np.pi, 360, endpoint=False)
        physical = [30, 20] + np.column_stack((np.cos(angle), np.sin(angle))) * 25
        plane_to_image = np.array([[5, 0.4, 900], [0.2, 4, 400], [0.007, 0.002, 1]])
        pixels = project(plane_to_image, physical)
        image_to_plane = np.linalg.inv(plane_to_image)
        fitted = fit_circle_in_plane(image_to_plane, pixels)
        np.testing.assert_allclose(fitted["center_plane_mm"], [30, 20], atol=1e-8)
        self.assertAlmostEqual(fitted["radius_mm"], 25)
        image_center = cv2.fitEllipse(pixels.astype(np.float32))[0]
        naive = project(image_to_plane, [image_center])[0]
        self.assertGreater(np.linalg.norm(naive - [30, 20]), 1)
        self.assertFalse(fitted["motion_allowed"])

    def test_arc_coverage_and_degenerate_line_are_reported_or_rejected(self):
        angle = np.linspace(0, np.pi/2, 40)
        arc = np.column_stack((np.cos(angle), np.sin(angle))) * 20
        report = fit_circle_in_plane(np.eye(3), arc)
        self.assertAlmostEqual(report["angular_coverage_deg"], 90)
        self.assertFalse(report["motion_allowed"])
        with self.assertRaises(ValueError):
            fit_circle_in_plane(np.eye(3), [[i, 0] for i in range(20)])

    def test_tcp_local_axes_follow_measured_extension_and_preserve_input(self):
        matrix = np.array([[0.25, -0.02, 4], [0.02, 0.25, 3], [0, 0, 1]])
        reference = [960, 540]
        origin, tip = project(matrix, [reference, [960, 539]])
        forward = (tip - origin) / np.linalg.norm(tip-origin)
        right = np.array([-forward[1], forward[0]])
        target = origin + 12*right + 77*forward
        direction = [0, -1]
        report = tcp_relative_offsets(matrix, reference, direction, [target])
        np.testing.assert_allclose(report["right_forward_mm"], [[12, 77]], atol=1e-8)
        self.assertEqual(direction, [0, -1])
        self.assertFalse(report["motion_allowed"])
        self.assertFalse(report["physical_tcp_verified"])
        with self.assertRaises(ValueError):
            tcp_relative_offsets(matrix, reference, [0, 0], [target])


if __name__ == "__main__":
    unittest.main()
