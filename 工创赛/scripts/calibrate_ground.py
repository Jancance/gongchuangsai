#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from smart_carrier.calibration import estimate_ground_homography


def main() -> int:
    parser = argparse.ArgumentParser(description="Estimate image-to-ground homography from measured point pairs")
    parser.add_argument("points", help="YAML file containing image and ground_mm point pairs")
    parser.add_argument("--output", default="diagnostics/homography-result.yaml")
    parser.add_argument("--max-rms-mm", type=float, default=3.0)
    args = parser.parse_args()
    data = yaml.safe_load(Path(args.points).read_text(encoding="utf-8"))
    pairs = data.get("points", [])
    image = np.asarray([pair["image"] for pair in pairs], dtype=np.float64)
    ground = np.asarray([pair["ground_mm"] for pair in pairs], dtype=np.float64)
    matrix, rms, inliers = estimate_ground_homography(image, ground)
    report = {
        "calibration_id": data.get("calibration_id", "REPLACE_ME"),
        "homography_image_to_ground": matrix.tolist(),
        "rms_mm": rms,
        "inliers": inliers.tolist(),
        "accepted": rms <= args.max_rms_mm and bool(inliers.all()),
        "note": "Copy into vision.yaml only after independent validation points pass.",
    }
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

