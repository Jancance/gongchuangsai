"""整合8静态图的圆边界→盘面→夹爪局部偏移，始终仅为离线诊断。"""
from pathlib import Path
import hashlib
import io
import json
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from smart_carrier.source_markers import fit_circle_in_plane, inside_hull, project, tcp_relative_offsets

BASE = ROOT / "evidence/integration/2026-11-05/source-markers"
OUT = BASE / "paper100-transfer/continuation"


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write(name, data):
    (OUT / name).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    calibration_path = OUT / "reference-calibration.json"
    calibration = read(calibration_path)
    reference_path = ROOT / "calibration/source_tcp_center_reference_20261105.json"
    reference = read(reference_path)
    assert reference["calibration_id"] == calibration["calibration_id"]
    assert reference["image_size"] == calibration["context"]["image_size"]
    assert read(OUT / "transfer-report.json")["length_consistency_passed"]
    matrix = calibration["homography_image_to_plane"]
    suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"), pattern="test_source_tcp_geometry.py")
    log = io.StringIO()
    tested = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    (OUT / "tcp-unittest.txt").write_text(log.getvalue(), encoding="utf-8")
    # 人工ROI/种子只选取已画出的盘面圆圈。不是当前在线M检测或物料中心。
    views = [
        ("reference", "paper100-8fa7a4e70382/source_markers_20261105_185038.png",
         [("upper-left", 631, 263), ("upper-right", 1340, 240)]),
        ("left-paper", "paper100-transfer/source_markers_20261105_194157.png",
         [("upper-right", 1340, 240), ("lower", 1005, 852)]),
        ("right-paper", "paper100-transfer/source_markers_20261105_194452.png",
         [("upper-left", 631, 263), ("lower", 1005, 852)]),
    ]
    edges = []
    fits = []
    for label, relative_path, seeds in views:
        path = BASE / relative_path
        image = cv2.imdecode(np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (1080, 1920):
            raise ValueError(f"输入不是原分辨率图像: {path}")
        yy, xx = np.where(cv2.Canny(image, 20, 60) > 0)
        for target_id, x, y in seeds:
            distances = np.hypot(xx-x, yy-y)
            pixels = np.column_stack((xx, yy))[(distances > 92) & (distances < 114)]
            fitted = fit_circle_in_plane(matrix, pixels)
            # 不按结果调门限，也不剔除残差以让实物数据通过。
            fitted.update(target_id=target_id, view=label,
                          image=path.relative_to(ROOT).as_posix(), image_sha256=sha(path),
                          fitted_center_image_px=project(np.linalg.inv(matrix), [fitted["center_plane_mm"]])[0].tolist(),
                          outside_original_fit_hull=not inside_hull(pixels, calibration["fit_hull_image_px"]))
            fits.append(fitted)
            edges.append({"target_id": target_id, "view": label, "image_sha256": sha(path),
                          "seed_px": [x, y], "boundary_image_px": pixels.tolist()})
    targets = []
    for target_id in ("upper-left", "upper-right", "lower"):
        matches = [f for f in fits if f["target_id"] == target_id]
        # 前两处取reference原图，下方取left-paper图；第二张只作提取重复性检查。
        primary = matches[0]
        delta = np.array(matches[1]["center_plane_mm"]) - primary["center_plane_mm"]
        targets.append({"id": target_id, "center_plane_mm": primary["center_plane_mm"],
                        "primary_image": primary["image"],
                        "same_scene_extraction_difference_mm": float(np.linalg.norm(delta)),
                        "extraction_difference_is_not_physical_accuracy": True})
    offsets = tcp_relative_offsets(matrix, reference["reference_image_px"], reference["extension_image_direction"],
                                   [t["center_plane_mm"] for t in targets])
    for target, offset in zip(targets, offsets["right_forward_mm"]):
        target["right_forward_mm"] = offset
    sources = ["src/smart_carrier/source_markers.py", "scripts/analyze_source_tcp_center_20261105.py",
               "tests/test_source_tcp_geometry.py"]
    report = {
        "schema_version": 1, "calibration_id": calibration["calibration_id"],
        "context": calibration["context"], "tcp_reference": reference, "basis": offsets,
        "method": "Canny 20/60; seed annulus (92,114)px; map every boundary point with original H then least-squares circle",
        "data_kind": "archived static drawn circles on source plane; not live materials",
        "circle_fits": fits, "targets": targets,
        "reference_inside_original_fit_hull": inside_hull([reference["reference_image_px"]], calibration["fit_hull_image_px"]),
        "unit_tests": tested.testsRun, "unit_failures": len(tested.failures), "unit_errors": len(tested.errors),
        "inputs_sha256": {"reference_calibration": sha(calibration_path), "tcp_reference": sha(reference_path)},
        "source_sha256": {name: sha(ROOT / name) for name in sources},
        "pending": ["current pivot-to-TCP horizontal length", "extension full usable stroke and rotation sign",
                    "actual TCP/height and independent absolute position verification",
                    "same-plane coverage approval; original preview still rejects extrapolation",
                    "joint/obstacle/power/fault-exit simulation before execution"],
        "operational_extrapolation_enabled": False, "field_calibration_accepted": False,
        "hardware_operated": False, "motion_allowed": False,
        "passed": tested.wasSuccessful(),
    }
    write("circle-boundary-inputs.json", edges)
    write("tcp-offset-diagnostic.json", report)
    print(json.dumps({"unit_tests": tested.testsRun, "passed": report["passed"],
                      "reference_plane_mm": offsets["reference_plane_mm"], "targets": targets,
                      "motion_allowed": False}, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
