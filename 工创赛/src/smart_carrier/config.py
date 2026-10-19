from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    pass


def load_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ConfigError(f"configuration file not found: {source}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"configuration root must be a mapping: {source}")
    if data.get("schema_version") != 1:
        raise ConfigError(f"unsupported schema_version in {source}")
    return data


def load_config_dir(directory: str | Path) -> dict[str, dict[str, Any]]:
    root = Path(directory)
    config = {
        "hardware": load_yaml(root / "hardware.yaml"),
        "cameras": load_yaml(root / "cameras.yaml"),
        "vision": load_yaml(root / "vision.yaml"),
    }
    for optional in ("p2", "task_code"):
        path = root / f"{optional}.yaml"
        if path.exists():
            config[optional] = load_yaml(path)
    return config


def validate_deployment_config(config: dict[str, dict[str, Any]]) -> list[str]:
    errors: list[str] = []
    cameras = config["cameras"].get("cameras", {})
    for name in ("main", "front"):
        camera = cameras.get(name)
        if not isinstance(camera, dict):
            errors.append(f"camera {name} is missing")
            continue
        path = str(camera.get("device_by_id", ""))
        if "REPLACE_WITH" in path or not path.startswith("/dev/v4l/by-id/"):
            errors.append(f"camera {name} must use a real /dev/v4l/by-id path")
        if "unconfigured" in str(camera.get("calibration_id", "")):
            errors.append(f"camera {name} calibration_id is not configured")
    vision = config["vision"].get("front_camera", {})
    if "unconfigured" in str(vision.get("calibration_id", "")):
        errors.append("front vision calibration is not configured")
    hardware = config["hardware"]
    if float(hardware["uart"]["logic_voltage"]) != 3.3:
        errors.append("Orange Pi UART logic voltage must be 3.3V")
    if hardware["tof"]["stop_distance_mm"] >= hardware["tof"]["slow_distance_mm"]:
        errors.append("ToF stop distance must be smaller than slow distance")
    return errors
