import copy
import unittest

import cv2
import numpy as np

from smart_carrier.source_markers import fit_paper, preview_points, project


def fixture():
    context = {"camera_id": "synthetic", "mount_id": "synthetic", "arm_pose_id": "held",
               "plane_id": "paper", "plane_z_mm": 85.5, "image_size": [1920, 1080],
               "pixel_space": "raw", "lens_profile_id": None,
               "plane_origin": "A", "axes": "x_along_ab_y_toward_ad_mm"}
    physical = [[0, 0], [200, 0], [200, 198], [0, 198], [100, 99], [50, 49.5], [150, 49.5]]
    pixel_corners = [[400, 100], [1300, 130], [1250, 850], [350, 830]]
    inverse = cv2.getPerspectiveTransform(np.float32(physical[:4]), np.float32(pixel_corners))
    pixels = project(inverse, physical)
    return {"calibration_id": "synthetic-paper-only", "width_mm": 200, "height_mm": 198,
            "context": context, "limits": {"fit_max_mm": 0.01, "validation_max_mm": 0.01},
            "markers": [{"id": i, "image_px": p.tolist()} for i, p in zip("ABCDMEF", pixels)]}


class PaperTests(unittest.TestCase):
    def test_perspective_rectangle_and_explicit_local_frame(self):
        result = fit_paper(fixture())
        self.assertTrue(result["independent_check_passed"])
        self.assertEqual(result["coordinate_frame"], "paper_local")
        self.assertNotIn("homography_image_to_chassis", result)
        self.assertFalse(result["motion_allowed"])
        self.assertEqual(result["validation"]["count"], 3)

    def test_holdout_error_fails_without_changing_fit(self):
        data = fixture()
        before = fit_paper(data)
        data["markers"][-1]["image_px"][0] += 25
        after = fit_paper(data)
        self.assertFalse(after["independent_check_passed"])
        np.testing.assert_allclose(before["homography_image_to_plane"], after["homography_image_to_plane"])

    def test_invalid_dimensions_order_and_frame(self):
        for change in (
            lambda d: d.update(width_mm=None),
            lambda d: d.update(width_mm=float("nan")),
            lambda d: d.update(height_mm=0),
            lambda d: d["context"].update(axes="x_right_y_forward_mm"),
            lambda d: d["markers"].pop(),
            lambda d: d["markers"].__setitem__(slice(1, 3), list(reversed(d["markers"][1:3]))),
        ):
            data = fixture()
            change(data)
            # 最后一个case仅重排列表不会改变ID对应关系，是允许的输入。
            if data["markers"][1]["id"] == "C" and data["markers"][2]["id"] == "B":
                self.assertTrue(fit_paper(data)["independent_check_passed"])
            else:
                with self.assertRaises(ValueError):
                    fit_paper(data)
        data = fixture()
        data["markers"][1]["image_px"], data["markers"][2]["image_px"] = data["markers"][2]["image_px"], data["markers"][1]["image_px"]
        with self.assertRaises(ValueError):
            fit_paper(data)

    def test_local_preview_never_produces_chassis_coordinates_or_alignment(self):
        data = fixture()
        calibration = fit_paper(data)
        observation = {"calibration_id": data["calibration_id"], "context": copy.deepcopy(data["context"]),
                       "targets": [{"id": "M", "kind": "physical_marker", "image_px": data["markers"][4]["image_px"]}]}
        result = preview_points(calibration, observation)
        self.assertNotIn("chassis_mm", result["targets"][0])
        np.testing.assert_allclose(result["targets"][0]["plane_mm"], [100, 99], atol=0.001)
        observation["alignment"] = {"center_marker_id": "M", "desired_center_chassis_mm": [0, 0], "max_translation_mm": 50}
        with self.assertRaises(ValueError):
            preview_points(calibration, observation)


if __name__ == "__main__":
    unittest.main()
