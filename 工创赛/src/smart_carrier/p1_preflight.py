from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .config import ConfigError, load_yaml


PLACEHOLDERS = {"", "UNKNOWN", "REPLACE_WITH_PATH"}


def _value(data: dict[str, Any], dotted_key: str) -> Any:
    current: Any = data
    for key in dotted_key.split("."):
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _require_true(data: dict[str, Any], key: str, errors: list[str]) -> None:
    if _value(data, key) is not True:
        errors.append(f"{key} must be true")


def _require_known(data: dict[str, Any], key: str, errors: list[str]) -> None:
    value = _value(data, key)
    if value is None or str(value).strip().upper() in PLACEHOLDERS:
        errors.append(f"{key} is not configured")


def validate_p1_preflight(data: dict[str, Any], project_root: str | Path) -> list[str]:
    """Return fail-closed P1-0 blockers without enabling any actuator."""
    errors: list[str] = []
    root = Path(project_root).resolve()

    if data.get("gate_id") != "P1-0":
        errors.append("gate_id must be P1-0")
    if data.get("status") != "READY_FOR_REVIEW":
        errors.append("status must be READY_FOR_REVIEW")
    if _value(data, "mcu.part") != "STM32F407VET6":
        errors.append("mcu.part must be STM32F407VET6")
    if _value(data, "mcu.package") != "LQFP100":
        errors.append("mcu.package must be LQFP100")

    for key in (
        "mcu.physical_pin_reviewed",
        "mcu.alternate_function_reviewed",
        "motor_bus.electrical_levels_verified",
        "motor_bus.can1_transceiver_3v3_compatible",
        "motor_bus.can1_termination_reviewed",
        "motor_bus.can2_transceiver_3v3_compatible",
        "motor_bus.can2_termination_reviewed",
        "serial_interfaces.orange_pi_usart6.verified",
        "serial_interfaces.hwt101_usart2.verified",
        "serial_interfaces.maixcam2_usart3.verified",
        "serial_interfaces.display_uart4.verified",
        "safety.actuators_physically_disconnected",
        "safety.hardware_estop_cuts_actuator_enable",
        "safety.global_enable_safe_during_reset",
        "safety.servo_power_safe_during_reset",
        "safety.boot0_and_nrst_reviewed",
        "safety.watchdog_and_brownout_plan_reviewed",
        "mechanical_inputs.axis35_origin.electrical_interface_verified",
        "mechanical_inputs.axis28_origin.electrical_interface_verified",
    ):
        _require_true(data, key, errors)

    for key in (
        "motor_bus.protocol_mode",
        "motor_bus.bitrate",
        "motor_bus.can1_transceiver_part",
        "motor_bus.can2_transceiver_part",
        "serial_interfaces.hwt101_usart2.electrical_standard",
        "serial_interfaces.maixcam2_usart3.electrical_standard",
        "serial_interfaces.display_uart4.electrical_standard",
    ):
        _require_known(data, key, errors)

    for key in ("motor_bus.exact_models", "motor_bus.manual_paths", "motor_bus.node_ids"):
        value = _value(data, key)
        if not isinstance(value, list) or not value:
            errors.append(f"{key} must be a non-empty list")

    safe_serial_standards = {"3V3_TTL", "LEVEL_SHIFTED_TO_3V3", "RS232_TRANSCEIVER", "RS485_TRANSCEIVER"}
    for name in ("orange_pi_usart6", "hwt101_usart2", "maixcam2_usart3", "display_uart4"):
        standard = _value(data, f"serial_interfaces.{name}.electrical_standard")
        if standard not in safe_serial_standards:
            errors.append(f"serial_interfaces.{name}.electrical_standard is not a reviewed MCU-safe interface")

    artifacts = data.get("artifacts", {})
    if not isinstance(artifacts, dict):
        errors.append("artifacts must be a mapping")
    else:
        for name, raw_path in artifacts.items():
            if str(raw_path).strip().upper() in PLACEHOLDERS:
                errors.append(f"artifacts.{name} is not configured")
                continue
            candidate = (root / str(raw_path)).resolve()
            try:
                candidate.relative_to(root)
            except ValueError:
                errors.append(f"artifacts.{name} must stay inside the project")
                continue
            if not candidate.is_file():
                errors.append(f"artifacts.{name} does not exist: {raw_path}")

    manual_paths = _value(data, "motor_bus.manual_paths")
    if isinstance(manual_paths, list):
        for index, raw_path in enumerate(manual_paths):
            candidate = (root / str(raw_path)).resolve()
            try:
                candidate.relative_to(root)
            except ValueError:
                errors.append(f"motor_bus.manual_paths[{index}] must stay inside the project")
                continue
            if not candidate.is_file():
                errors.append(f"motor_bus.manual_paths[{index}] does not exist: {raw_path}")

    if _value(data, "mechanical_inputs.axis35_origin.pin") != "PE8":
        errors.append("axis35 origin must remain on PE8")
    if _value(data, "mechanical_inputs.axis28_origin.pin") != "PE10":
        errors.append("axis28 origin must remain on PE10")
    if _value(data, "mechanical_inputs.dm4310_origin.fitted") is not False:
        errors.append("DM4310 origin input must remain DNP until its later gate")
    if _value(data, "tof.fitted") is not False or _value(data, "tof.runtime_status") != "NOT_PRESENT":
        errors.append("first-board ToF must be DNP and report NOT_PRESENT")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Fail-closed P1-0 electrical preflight")
    parser.add_argument("--config", default="config/p1_preflight.yaml")
    parser.add_argument("--project-root", default=".")
    args = parser.parse_args()

    try:
        data = load_yaml(args.config)
    except ConfigError as exc:
        print(f"P1_PREFLIGHT=BLOCKED\n- {exc}")
        return 2

    errors = validate_p1_preflight(data, args.project_root)
    if errors:
        print("P1_PREFLIGHT=BLOCKED")
        for error in errors:
            print(f"- {error}")
        return 2

    print("P1_PREFLIGHT=PASS")
    print("P1-0 evidence is complete; an independent schematic review is still required before P1-1 power-up.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
