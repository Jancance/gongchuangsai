"""复算整合8已归档方纸图的跨区域长度诊断；无硬件接口。"""
from pathlib import Path
import copy
import hashlib
import io
import json
import subprocess
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from smart_carrier.source_markers import fit_paper, project

FOLDER = ROOT / "evidence/integration/2026-11-05/source-markers"
TRANSFER = FOLDER / "paper100-transfer"
OUT = TRANSFER / "continuation"


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_image(path):
    # Windows中文路径经标准文件读取，避免OpenCV路径编码差异。
    image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法解码图像: {path}")
    return image


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    suite = unittest.TestSuite()
    for pattern in ("test_source_markers.py", "test_source_paper.py",
                    "test_source_paper_transfer.py", "test_calibration.py"):
        suite.addTests(unittest.defaultTestLoader.discover(str(ROOT / "tests"), pattern=pattern))
    log = io.StringIO()
    tested = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    (OUT / "unittest.txt").write_text(log.getvalue(), encoding="utf-8")
    measured_path = ROOT / "calibration/source_paper_100mm_20261105.json"
    measured = read(measured_path)
    calibration = fit_paper(measured)
    original_dir = FOLDER / "paper100-8fa7a4e70382"
    original_image = original_dir / "source_markers_20261105_185038.png"
    assert sha(original_image) == measured["pixel_extraction"]["image_sha256"]
    old_calibration = read(original_dir / "paper-fit-candidate.json")
    np.testing.assert_allclose(calibration["homography_image_to_plane"],
                               old_calibration["homography_image_to_plane"], rtol=0, atol=1e-10)
    calibration.update(field_calibration_accepted=False, pose_feedback_recorded=False,
                       physical_dimension_reference_confirmed=True)
    write(OUT / "reference-calibration.json", calibration)
    extracted = read(TRANSFER / "extracted-full-paper-views.json")
    assert {view["label"] for view in extracted} == {"left", "right"}
    original_gray = cv2.cvtColor(read_image(original_image), cv2.COLOR_BGR2GRAY)
    references = []
    images = {original_image.relative_to(ROOT).as_posix(): sha(original_image)}
    for view in extracted:
        image_path = ROOT / view["image"]
        assert sha(image_path) == view["sha256"]
        image = read_image(image_path)
        assert image is not None and image.shape[:2] == (1080, 1920)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        matches = []
        for x, y in [(970, 380), (915, 433), (935, 508), (1008, 528), (1060, 475)]:
            scores = cv2.matchTemplate(gray[y-18:y+19, x-18:x+19],
                                       original_gray[y-10:y+11, x-10:x+11], cv2.TM_CCOEFF_NORMED)
            _, score, _, loc = cv2.minMaxLoc(scores)
            matches.append({"reference_px": [x, y], "shift_px": [loc[0]-8, loc[1]-8],
                            "correlation": score})
        references.append({"label": view["label"], "matches": matches})
        images[image_path.relative_to(ROOT).as_posix()] = sha(image_path)
        # 核对已有提取数据与本轮重新求得的原H，而不是把旧报告当作当前计算。
        mm = project(calibration["homography_image_to_plane"], view["corners_ABCD_px"])
        np.testing.assert_allclose(mm, view["diagnostic_projected_corners_paper_mm"], atol=1e-8)
    dataset = {"calibration_id": calibration["calibration_id"], "context": calibration["context"],
               "length_error_limit_mm": measured["limits"]["validation_max_mm"],
               "views": [{"label": view["label"], "measured_paper_mm": [100.0, 100.0],
                          "corners_ABCD_px": view["corners_ABCD_px"], "image_sha256": view["sha256"]}
                         for view in extracted]}
    write(OUT / "transfer-input.json", dataset)
    missing = copy.deepcopy(dataset)
    missing["views"][0]["corners_ABCD_px"].pop()
    write(OUT / "missing-corner-input.json", missing)
    moved = copy.deepcopy(dataset)
    moved["context"]["arm_pose_id"] = "changed-pose-must-reject"
    write(OUT / "changed-pose-input.json", moved)
    wrong_size = copy.deepcopy(dataset)
    wrong_size["views"][0]["measured_paper_mm"] = [110.0, 110.0]
    write(OUT / "wrong-size-input.json", wrong_size)
    cli_checks = []
    for input_name, output_name, expected in (
        ("transfer-input.json", "transfer-report.json", 0),
        ("missing-corner-input.json", "missing-corner-rejected.json", 1),
        ("changed-pose-input.json", "changed-pose-rejected.json", 1),
        ("wrong-size-input.json", "wrong-size-rejected.json", 1),
    ):
        command = [sys.executable, "-X", "utf8", str(ROOT / "scripts/source_marker_calibration.py"),
                   "check-paper-transfer", str(OUT / input_name), "--calibration",
                   str(OUT / "reference-calibration.json"), "--output", str(OUT / output_name)]
        completed = subprocess.run(command, capture_output=True, text=True, encoding="utf-8")
        cli_checks.append({"input": input_name, "expected_exit": expected,
                           "actual_exit": completed.returncode, "passed": completed.returncode == expected})
    report = read(OUT / "transfer-report.json")
    sources = ["src/smart_carrier/source_markers.py", "scripts/source_marker_calibration.py",
               "scripts/check_source_paper_transfer_20261105.py", "tests/test_source_paper_transfer.py"]
    summary = {
        "unit_tests": tested.testsRun, "unit_failures": len(tested.failures), "unit_errors": len(tested.errors),
        "cli_checks": cli_checks, "length_consistency_passed": report["length_consistency_passed"],
        "views": report["views"], "static_references": references,
        "images_sha256": images, "measured_input_sha256": sha(measured_path),
        "corner_extraction_sha256": sha(TRANSFER / "extracted-full-paper-views.json"),
        "source_sha256": {p: sha(ROOT / p) for p in sources},
        "pose_state": "user reconfirmed all fixed and no reset in continuation chat 2026-11-05; no hardware readback",
        "field_calibration_accepted": False, "absolute_position_verified": False, "motion_allowed": False,
        "hardware_operated": False, "operational_extrapolation_enabled": False,
        "passed": tested.wasSuccessful() and all(c["passed"] for c in cli_checks)
                  and report["length_consistency_passed"],
    }
    write(OUT / "checks.json", summary)
    print(json.dumps({k: summary[k] for k in ("unit_tests", "unit_failures", "unit_errors", "cli_checks",
                                             "length_consistency_passed", "passed")}, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
