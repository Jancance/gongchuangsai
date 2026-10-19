from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from typing import Any

from .config import load_config_dir, validate_deployment_config


def _temperature_c() -> float | None:
    values: list[float] = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            raw = float(path.read_text().strip())
            values.append(raw / 1000.0 if raw > 1000 else raw)
        except (OSError, ValueError):
            continue
    return max(values) if values else None


def health_report(config_dir: str | Path, simulate: bool = False) -> dict[str, Any]:
    config = load_config_dir(config_dir)
    deployment_issues = validate_deployment_config(config)
    errors = [] if simulate else list(deployment_issues)
    checks: dict[str, Any] = {}
    expected_arch = config["hardware"]["platform"]["expected_architecture"]
    architecture = os.uname().machine if hasattr(os, "uname") else "non-linux"
    checks["architecture"] = {"actual": architecture, "expected": expected_arch, "ok": simulate or architecture == expected_arch}
    if not checks["architecture"]["ok"]:
        errors.append(f"expected architecture {expected_arch}, got {architecture}")

    temperature = _temperature_c()
    max_temp = float(config["hardware"]["power"]["max_temperature_c"])
    checks["temperature"] = {"actual_c": temperature, "max_c": max_temp, "ok": simulate or (temperature is not None and temperature < max_temp)}
    if not checks["temperature"]["ok"]:
        errors.append("temperature unavailable or above limit")

    disk = shutil.disk_usage("/")
    free_fraction = disk.free / disk.total
    checks["disk"] = {"free_fraction": free_fraction, "ok": free_fraction >= 0.15}
    if not checks["disk"]["ok"]:
        errors.append("less than 15% disk space free")

    for name, camera in config["cameras"]["cameras"].items():
        exists = Path(str(camera["device_by_id"])).exists()
        checks[f"camera_{name}"] = {"path": camera["device_by_id"], "ok": simulate or exists}
        if not checks[f"camera_{name}"]["ok"]:
            errors.append(f"camera {name} is missing")

    uart_path = Path(str(config["hardware"]["uart"]["device"]))
    checks["uart"] = {"path": str(uart_path), "ok": simulate or uart_path.exists()}
    if not checks["uart"]["ok"]:
        errors.append("configured UART device is missing")
    return {
        "timestamp": time.time(),
        "ok": not errors,
        "errors": sorted(set(errors)),
        "warnings": sorted(set(deployment_issues)) if simulate else [],
        "checks": checks,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Fail-closed Orange Pi robot health check")
    parser.add_argument("--config", default="/opt/smart-carrier/config")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--watch", type=float, default=0.0, metavar="SECONDS")
    parser.add_argument("--output")
    args = parser.parse_args()
    while True:
        report = health_report(args.config, args.simulate)
        text = json.dumps(report, ensure_ascii=False, sort_keys=True)
        print(text, flush=True)
        if args.output:
            target = Path(args.output)
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_suffix(".tmp")
            temporary.write_text(text + "\n", encoding="utf-8")
            temporary.replace(target)
        if args.watch <= 0:
            raise SystemExit(0 if report["ok"] else 1)
        time.sleep(args.watch)


if __name__ == "__main__":
    main()
