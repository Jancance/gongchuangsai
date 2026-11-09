"""合成几何与失败入口验证；不代表真实机构或相机通过。"""
import copy
import unittest

import numpy as np

from smart_carrier.source_markers import fit_markers, preview_points, project


def fixture():
    pixels = [[100, 100], [900, 100], [900, 700], [100, 700],
              [300, 300], [700, 300], [500, 550]]
    # 含透视，防止测试退化为固定像素比例。
    matrix = [[0.4, 0.02, -160], [0.01, -0.35, 600], [0.0002, -0.0001, 1]]
    mm = project(matrix, pixels)
    return {"calibration_id": "synthetic-only", "context": {
        "camera_id": "synthetic", "mount_id": "test", "arm_pose_id": "held-pose-test",
        "plane_id": "surface", "plane_z_mm": 90, "chassis_origin": "test origin",
        "axes": "x_right_y_forward_mm", "image_size": [1920, 1080],
        "pixel_space": "raw", "lens_profile_id": None},
        "limits": {"fit_max_mm": 0.01, "validation_max_mm": 0.01},
        "markers": [{"id": str(i), "role": "fit" if i < 4 else "validation",
                     "image_px": p, "chassis_mm": m.tolist()} for i, (p, m) in enumerate(zip(pixels, mm))]}


class MarkerTests(unittest.TestCase):
    def test_independent_projective_check(self):
        result = fit_markers(fixture())
        self.assertTrue(result["independent_check_passed"])
        self.assertLess(result["validation"]["max_mm"], 0.001)
        self.assertEqual(result["validation"]["count"], 3)
        self.assertFalse(result["motion_allowed"])

    def test_bad_holdout_does_not_change_fit_or_pass(self):
        data = fixture()
        before = fit_markers(data)
        data["markers"][-1]["chassis_mm"][0] += 8
        after = fit_markers(data)
        self.assertFalse(after["independent_check_passed"])
        np.testing.assert_allclose(before["homography_image_to_chassis"], after["homography_image_to_chassis"])

    def test_bad_datasets_rejected(self):
        cases = []
        for change in (
            lambda d: d["context"].update(plane_z_mm=None),
            lambda d: d["context"].update(arm_pose_id=""),
            lambda d: d["context"].update(pixel_space="undistorted"),
            lambda d: d["limits"].update(validation_max_mm=float("nan")),
            lambda d: d["markers"][0].update(image_px=[1920, 0]),
            lambda d: d["markers"][0].update(image_px=[float("nan"), 0]),
            lambda d: d["markers"][-1].update(id="0"),
            lambda d: d["markers"][-1].update(image_px=d["markers"][0]["image_px"]),
            lambda d: d["markers"][-1].update(chassis_mm=d["markers"][0]["chassis_mm"]),
            lambda d: d["markers"][-1].update(role="fit"),
            lambda d: d["markers"][-1].update(image_px=[1000, 900]),
            lambda d: [d["markers"][i].update(image_px=[100 + 100*i, 100]) for i in range(4)],
        ):
            data = fixture()
            change(data)
            cases.append(data)
        for i, data in enumerate(cases):
            with self.subTest(case=i), self.assertRaises(ValueError):
                fit_markers(data)

    def observation(self):
        data = fixture()
        return {"calibration_id": data["calibration_id"], "context": copy.deepcopy(data["context"]),
                "targets": [{"id": "center", "kind": "physical_marker", "image_px": [500, 400]},
                            {"id": "position1", "kind": "feature_on_calibrated_plane", "image_px": [300, 300]},
                            {"id": "position2", "kind": "feature_on_calibrated_plane", "image_px": [700, 300]},
                            {"id": "position3", "kind": "feature_on_calibrated_plane", "image_px": [500, 550]}]}

    def test_preview_three_targets_and_translation_sign(self):
        calibration = fit_markers(fixture())
        observation = self.observation()
        center = project(calibration["homography_image_to_chassis"], [[500, 400]])[0]
        observation["alignment"] = {"center_marker_id": "center",
                                    "desired_center_chassis_mm": (center - [10, -5]).tolist(),
                                    "max_translation_mm": 15}
        result = preview_points(calibration, observation)
        np.testing.assert_allclose(result["translation_preview_mm"], [10, -5], atol=1e-8)
        self.assertEqual(len(result["targets"]), 4)
        self.assertFalse(result["motion_allowed"])
        self.assertFalse(result["yaw_observable"])

    def test_stale_pose_plane_size_or_version_rejected(self):
        calibration = fit_markers(fixture())
        for key, value in (("arm_pose_id", "changed"), ("plane_id", "material_top"),
                           ("plane_z_mm", 150), ("image_size", [1280, 720]), ("mount_id", "changed")):
            observation = self.observation()
            observation["context"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                preview_points(calibration, observation)
        observation = self.observation()
        observation["calibration_id"] = "old"
        with self.assertRaises(ValueError):
            preview_points(calibration, observation)

    def test_extrapolation_and_image_circle_center_rejected(self):
        calibration = fit_markers(fixture())
        for change in (
            lambda o: o["targets"][0].update(image_px=[950, 900]),
            lambda o: o["targets"][0].update(kind="fitted_circle_center"),
            lambda o: o["targets"][0].update(image_px=[float("inf"), 0]),
        ):
            observation = self.observation()
            change(observation)
            with self.assertRaises(ValueError):
                preview_points(calibration, observation)
        calibration["independent_check_passed"] = False
        with self.assertRaises(ValueError):
            preview_points(calibration, self.observation())

    def test_excessive_correction_and_wrong_center_kind_rejected(self):
        calibration = fit_markers(fixture())
        observation = self.observation()
        observation["alignment"] = {"center_marker_id": "center", "desired_center_chassis_mm": [0, 0],
                                    "max_translation_mm": 1}
        with self.assertRaises(ValueError):
            preview_points(calibration, observation)
        observation["alignment"]["max_translation_mm"] = 1000
        observation["targets"][0]["kind"] = "feature_on_calibrated_plane"
        with self.assertRaises(ValueError):
            preview_points(calibration, observation)

    def test_projective_horizon_rejected(self):
        with self.assertRaises(ValueError):
            project([[1, 0, 0], [0, 1, 0], [1, 0, -100]], [[100, 20]])


if __name__ == "__main__":
    unittest.main()
