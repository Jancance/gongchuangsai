#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from smart_carrier.camera import LatestFrameCamera
from smart_carrier.config import load_yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="Run both UVC cameras while dropping stale frames")
    parser.add_argument("--config", default="config/cameras.yaml")
    parser.add_argument("--duration-hours", type=float, default=8.0)
    parser.add_argument("--output", default="diagnostics/camera-stress.json")
    args = parser.parse_args()
    cameras_cfg = load_yaml(args.config)["cameras"]
    cameras = {name: LatestFrameCamera(name, cfg) for name, cfg in cameras_cfg.items()}
    for camera in cameras.values():
        camera.start()
    start = time.monotonic()
    last_sequences = {name: 0 for name in cameras}
    stagnant_samples = {name: 0 for name in cameras}
    samples = 0
    try:
        while time.monotonic() - start < args.duration_hours * 3600:
            time.sleep(1.0)
            samples += 1
            for name, camera in cameras.items():
                frame = camera.read_latest()
                sequence = 0 if frame is None else frame.sequence
                if sequence <= last_sequences[name]:
                    stagnant_samples[name] += 1
                last_sequences[name] = sequence
    finally:
        for camera in cameras.values():
            camera.stop()
    report = {
        "duration_seconds": time.monotonic() - start,
        "samples": samples,
        "cameras": {
            name: {
                "frames": last_sequences[name],
                "capture_failures": camera.failures,
                "stagnant_samples": stagnant_samples[name],
            }
            for name, camera in cameras.items()
        },
    }
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if all(item["frames"] > 0 and item["stagnant_samples"] == 0 for item in report["cameras"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())

