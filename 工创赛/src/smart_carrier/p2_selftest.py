from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import cv2
import yaml


def _load(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream) or {}
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def audit(config_dir: Path) -> tuple[list[str], list[str]]:
    cameras = _load(config_dir / "cameras.yaml")
    task_code = _load(config_dir / "task_code.yaml")
    errors: list[str] = []
    blockers: list[str] = []

    if not cv2.__version__:
        errors.append("OpenCV import failed")
    parser_config = task_code.get("parser", {})
    if not bool(parser_config.get("execution_locked_until_confirmed", True)):
        blockers.append("QR-04 mapping is unresolved but execution_lock is disabled")
    camera_devices = cameras.get("cameras", {})
    for camera_id in ("main", "front"):
        device = camera_devices.get(camera_id, {})
        if not device:
            errors.append(f"missing camera configuration: {camera_id}")
            continue
        if not device.get("verified", False):
            blockers.append(f"{camera_id}: physical V4L2/exposure/frame-rate acceptance pending")
    scanner = cameras.get("scanner", {})
    if not scanner.get("verified", False):
        blockers.append("task_scanner: USB CDC and illumination acceptance pending")
    return errors, blockers


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit P2 configuration without touching hardware")
    parser.add_argument("--config-dir", type=Path, default=Path("config"))
    args = parser.parse_args()
    errors, blockers = audit(args.config_dir)
    print(f"OpenCV: {cv2.__version__}; OpenCL: {cv2.ocl.haveOpenCL()}")
    for item in errors:
        print(f"ERROR: {item}")
    for item in blockers:
        print(f"BLOCKED: {item}")
    print(f"P2_SELFTEST={'FAIL' if errors else 'PASS_WITH_OPEN_ITEMS' if blockers else 'PASS'}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
