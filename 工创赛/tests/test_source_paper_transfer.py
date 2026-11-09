import copy
import unittest

import numpy as np

from smart_carrier.source_markers import check_paper_transfer, fit_paper, preview_points, project
from test_source_paper import fixture


def transfer_fixture():
    calibration = fit_paper(fixture())
    # 用原纸外、旋转后的另一张矩形独立检查；无需知道纸移动的绝对距离。
    rotation = np.array([[0.8, -0.6], [0.6, 0.8]])
    mm = np.array([[0, 0], [100, 0], [100, 80], [0, 80]]) @ rotation.T + [220, 30]
    pixels = project(np.linalg.inv(calibration["homography_image_to_plane"]), mm)
    data = {"calibration_id": calibration["calibration_id"],
            "context": copy.deepcopy(calibration["context"]), "length_error_limit_mm": 0.01,
            "views": [{"label": "other-region", "measured_paper_mm": [100, 80],
                       "corners_ABCD_px": pixels.tolist()}]}
    return calibration, data


class PaperTransferTests(unittest.TestCase):
    def test_rotated_rectangle_outside_hull_checks_lengths_without_refitting(self):
        calibration, data = transfer_fixture()
        original = copy.deepcopy(calibration)
        report = check_paper_transfer(calibration, data)
        self.assertTrue(report["length_consistency_passed"])
        self.assertTrue(report["views"][0]["outside_original_fit_hull"])
        self.assertEqual(calibration, original)
        for key in ("motion_allowed", "operational_extrapolation_enabled",
                    "absolute_position_verified", "field_calibration_accepted"):
            self.assertFalse(report[key])
        observation = {"calibration_id": data["calibration_id"], "context": data["context"],
                       "targets": [{"id": "outside", "kind": "physical_marker",
                                    "image_px": data["views"][0]["corners_ABCD_px"][0]}]}
        with self.assertRaises(ValueError):
            preview_points(calibration, observation)

    def test_wrong_scale_fails_even_though_shape_is_rectangular(self):
        calibration, data = transfer_fixture()
        data["views"][0]["measured_paper_mm"] = [110, 88]
        self.assertFalse(check_paper_transfer(calibration, data)["length_consistency_passed"])

    def test_absolute_translation_is_unobservable_from_lengths(self):
        calibration, data = transfer_fixture()
        first = check_paper_transfer(calibration, data)
        mm = np.array(first["views"][0]["diagnostic_corners_paper_mm"]) + [5, -3]
        data["views"][0]["corners_ABCD_px"] = project(
            np.linalg.inv(calibration["homography_image_to_plane"]), mm).tolist()
        second = check_paper_transfer(calibration, data)
        self.assertTrue(second["length_consistency_passed"])
        self.assertFalse(second["absolute_position_verified"])

    def test_context_versions_missing_corner_and_invalid_geometry_reject(self):
        changes = (
            lambda c, d: d["context"].update(arm_pose_id="moved"),
            lambda c, d: d.update(calibration_id="old"),
            lambda c, d: c.update(independent_check_passed=False),
            lambda c, d: d.update(length_error_limit_mm=float("nan")),
            lambda c, d: d.update(views=[]),
            lambda c, d: d["views"][0]["corners_ABCD_px"].pop(),
            lambda c, d: d["views"][0]["corners_ABCD_px"][0].__setitem__(0, -1),
            lambda c, d: d["views"][0]["corners_ABCD_px"].__setitem__(
                slice(1, 3), d["views"][0]["corners_ABCD_px"][1:3][::-1]),
        )
        for change in changes:
            with self.subTest(change=change):
                calibration, data = transfer_fixture()
                change(calibration, data)
                with self.assertRaises(ValueError):
                    check_paper_transfer(calibration, data)


if __name__ == "__main__":
    unittest.main()
