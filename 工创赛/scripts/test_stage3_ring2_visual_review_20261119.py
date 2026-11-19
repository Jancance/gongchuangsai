"""Regression fixtures for the offline review utility; no camera or robot."""

import contextlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import stage3_ring2_visual_review_20261119 as review

REPOWER_CAPTURE = review.EVIDENCE / "ring2-empty-after-repower-capture-20261119-032221-244344.json"


class VisualReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        capture_path = review.EVIDENCE / f"{review.EMPTY_PREFIX}-after-{review.EMPTY_STAMP}.png"
        _, cls.empty = review.read_frame(capture_path)
        cls.colors = review.load_color_config(review.COLOR_PROFILE)

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()

    def save(self, name, frame):
        ok, encoded = cv2.imencode(".png", frame)
        self.assertTrue(ok)
        path = self.root / name
        path.write_bytes(encoded.tobytes())
        return str(path)

    def cli(self, *args):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            review.main(args)
        return json.loads(stdout.getvalue())

    def rejects(self, text, *args):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as cm:
            self.cli(*args)
        self.assertEqual(cm.exception.code, 2)
        self.assertIn(text, stderr.getvalue())

    def test_empty_pair_matches_pixel_archive(self):
        result = self.cli("ring2")
        ref = result["reference"]
        self.assertEqual(result["status"], "AWAITING_LOADED_FRAME")
        self.assertNotEqual(ref["frames"][0]["decoded_sha256"], ref["frames"][1]["decoded_sha256"])
        np.testing.assert_allclose(ref["ring2_floor_center_px"], [998.5, 540.4], atol=.5)
        self.assertFalse(result["metric_valid"])
        self.assertFalse(result["motion_allowed"])

    def test_repower_archive_replayed_with_explicit_manifest(self):
        result = self.cli("ring2", "--empty-capture", str(REPOWER_CAPTURE))
        ref = result["reference"]
        self.assertEqual(ref["manifest_selection"], "explicit")
        self.assertEqual(Path(ref["capture_manifest"]["file"]), REPOWER_CAPTURE)
        np.testing.assert_allclose(ref["ring2_floor_center_px"], [956.98, 542.65], atol=.5)
        manifest = json.loads(REPOWER_CAPTURE.read_text(encoding="utf-8"))
        for row, name in zip(ref["frames"], ("before", "after")):
            self.assertEqual(row["size"], [1920, 1080])
            self.assertEqual(row["sha256"], manifest["images"][name]["sha256"])
        self.assertFalse(result["metric_valid"])

    def test_physical_label_cannot_use_implicit_or_known_old_pair(self):
        # No real loaded image in this test: exercise the CLI gate before image read.
        missing = self.root / "not-read.png"
        args = ("ring2", "--loaded", str(missing), "--provenance", "physical",
                "--same-view-confirmed", "--top-point", "1030,560")
        self.rejects("require --empty-capture", *args)
        self.rejects("replay-only", *args, "--empty-capture", str(review.EMPTY_CAPTURE))
        # A valid new reference passes the gate, but a missing loaded image still cannot yield a delta.
        self.rejects(missing.name, *args, "--empty-capture", str(REPOWER_CAPTURE))

    def test_moved_manifest_resolves_relative_and_colocated_absolute_paths(self):
        manifest = json.loads(REPOWER_CAPTURE.read_text(encoding="utf-8"))
        for name, image in manifest["images"].items():
            source = Path(image["local"])
            shutil.copyfile(source, self.root / source.name)
            if name == "before":
                image["local"] = source.name
        path = self.root / REPOWER_CAPTURE.name
        path.write_text(json.dumps(manifest), encoding="utf-8")
        result = self.cli("ring2", "--empty-capture", str(path))
        self.assertTrue(all(Path(row["file"]).parent == self.root
                            for row in result["reference"]["frames"]))
        manifest["images"]["after"]["sha256"] = "0" * 64
        path.write_text(json.dumps(manifest), encoding="utf-8")
        self.rejects("disagrees with its capture manifest", "ring2", "--empty-capture", str(path))

    def test_manifest_relative_path_does_not_escape(self):
        with self.assertRaisesRegex(ValueError, "inside its manifest directory"):
            review.capture_image_path(self.root / "capture.json", "../other.png")
        with self.assertRaisesRegex(ValueError, "network or device"):
            review.capture_image_path(self.root / "capture.json", r"\\example\share\other.png")

    def test_red_synthetic_two_frames_signed_px_and_artifacts_only(self):
        paths = []
        for i in (0, 1):
            frame = self.empty.copy()
            cv2.circle(frame, (1038 + i, 575), 35, (0, 0, 235), -1)
            paths.append(self.save(f"synthetic-{i}.png", frame))
        output = self.root / "output"
        result = self.cli("ring2", "--loaded", paths[0], "--loaded", paths[1],
                          "--provenance", "synthetic", "--top-roi", "975,510,145,135",
                          "--top-roi", "975,510,145,135", "--same-view-confirmed",
                          "--output-dir", str(output))
        self.assertEqual(result["status"], "REVIEW_PIXEL_OBSERVATIONS_ONLY")
        self.assertEqual(result["provenance"], "synthetic")
        self.assertFalse(result["acceptance_claim"])
        center = result["frames"][0]["red_top_observation_px"]
        np.testing.assert_allclose(center, [1038., 575.], atol=.2)
        delta = result["frames"][0]["top_minus_archived_floor_reference_px"]
        self.assertTrue(all(v > 0 for v in delta))
        self.assertEqual(json.loads((output / "report.json").read_text(encoding="utf-8")), result)
        self.assertEqual(len(list(output.glob("*.png"))), 6)
        self.assertTrue((output / "loaded-01-red-mask.png").is_file())

    def test_manual_mark_no_view_confirmation_no_delta(self):
        path = self.save("synthetic.png", np.zeros_like(self.empty))
        result = self.cli("ring2", "--loaded", path, "--provenance", "synthetic",
                          "--top-point", "1030,560")
        row = result["frames"][0]
        self.assertEqual(row["red_top_observation_px"], [1030., 560.])
        self.assertIsNone(row["top_minus_archived_floor_reference_px"])
        self.assertFalse(result["camera_pose_independently_verified"])

    def test_duplicate_loaded_frame_rejected(self):
        path1 = self.save("duplicate1.png", np.zeros_like(self.empty))
        path2 = self.save("duplicate2.png", np.zeros_like(self.empty))
        self.rejects("Duplicate decoded image", "ring2", "--loaded", path1, "--loaded",
                     path2, "--provenance", "synthetic")

    def test_archived_phone_crop_is_not_same_view(self):
        self.rejects("different dimensions", "ring2", "--loaded",
                     str(review.EVIDENCE / "ring2-first-placement-user.png"),
                     "--provenance", "archived", "--same-view-confirmed",
                     "--top-point", "240,168")

    def test_red_unresolved_if_missing_ambiguous_or_roi_clipped(self):
        frame = np.zeros((100, 100, 3), np.uint8)
        self.assertIsNone(review.red_top(frame, (0, 0, 100, 100), self.colors, frame.copy())[0])
        cv2.circle(frame, (0, 45), 20, (0, 0, 235), -1)
        self.assertIsNone(review.red_top(frame, (0, 0, 100, 100), self.colors, frame.copy())[0])
        cv2.circle(frame, (70, 55), 15, (0, 0, 235), -1)
        self.assertIsNone(review.red_top(frame, (0, 0, 100, 100), self.colors, frame.copy())[0])

    def stack_fixture(self):
        frame = np.full_like(self.empty, 200)
        for x, bgr in [(10, (0, 0, 235)), (40, (0, 235, 235)), (70, (10, 10, 10))]:
            frame[10:30, x:x + 20] = bgr
        return self.save("synthetic-stack.png", frame)

    def stack(self, path, profile=review.RING_PROFILE, task="215+213+512+231"):
        return self.cli("stack", "--loaded", path, "--task-code", task,
                        "--provenance", "synthetic", "--profile", str(profile),
                        "--top-roi", "10,10,20,20", "--top-roi", "40,10,20,20",
                        "--top-roi", "70,10,20,20")

    def test_task_physical_map_and_true_s_negative_no_empty_fallback(self):
        result = self.stack(self.stack_fixture())
        self.assertEqual(result["status"], "S_OR_SLOT_COLOR_UNCONFIRMED")
        self.assertEqual(result["expected_first_batch_color_by_slot"], {"1": 1, "2": 2, "3": 5})
        row = result["frames"][0]
        self.assertTrue(all(x["roi_color_matches"] for x in row["operator_selected_slot_colors"].values()))
        self.assertFalse(row["s_mode"]["detected"])
        self.assertNotIn("middle_ring_center_px", row["s_mode"])
        self.assertFalse(result["motion_allowed"])

    def test_s_observation_alone_cannot_verify_stack(self):
        path = self.stack_fixture()
        fake_s = {"middle": (250., 300.), "circle_support": .9, "line_quality": .95,
                  "segments": [((20, 30), (500, 30))]}
        with patch.object(review.StackedRingDetector, "_raw_detection", return_value=fake_s):
            matching = self.stack(path)
            wrong = self.stack(path, task="215+123+512+231")
        self.assertIn("REVIEW_S", matching["status"])
        self.assertFalse(matching["geometric_stack_acceptance"])
        self.assertEqual(wrong["status"], "S_OR_SLOT_COLOR_UNCONFIRMED")

    def test_s_requires_search_record_and_valid_task(self):
        profile = json.loads(review.RING_PROFILE.read_text(encoding="utf-8-sig"))
        profile["search_calibration"] = None
        path = self.root / "invalid-profile.json"
        path.write_text(json.dumps(profile), encoding="utf-8")
        frame = self.stack_fixture()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.stack(frame, path)
        with self.assertRaises(ValueError):
            review.expected_colors("215+113+512+231")


if __name__ == "__main__":
    unittest.main()
