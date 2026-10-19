from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from smart_carrier.p1_preflight import validate_p1_preflight


def valid_config() -> dict:
    return {
        "schema_version": 1,
        "gate_id": "P1-0",
        "status": "READY_FOR_REVIEW",
        "mcu": {
            "part": "STM32F407VET6",
            "package": "LQFP100",
            "physical_pin_reviewed": True,
            "alternate_function_reviewed": True,
        },
        "artifacts": {
            "cubemx_ioc": "evidence/board.ioc",
            "schematic_pdf": "evidence/schematic.pdf",
            "pin_mux_report": "evidence/pins.txt",
            "power_budget": "evidence/power.csv",
            "reset_state_review": "evidence/reset.md",
        },
        "motor_bus": {
            "exact_models": ["MODEL-A"],
            "manual_paths": ["evidence/motor-manual.pdf"],
            "protocol_mode": "CAN",
            "bitrate": 1000000,
            "node_ids": [1],
            "electrical_levels_verified": True,
            "can1_transceiver_part": "TCAN3413",
            "can1_transceiver_3v3_compatible": True,
            "can1_termination_reviewed": True,
            "can2_transceiver_part": "TCAN3413",
            "can2_transceiver_3v3_compatible": True,
            "can2_termination_reviewed": True,
        },
        "serial_interfaces": {
            "orange_pi_usart6": {"electrical_standard": "3V3_TTL", "verified": True},
            "hwt101_usart2": {"electrical_standard": "3V3_TTL", "verified": True},
            "maixcam2_usart3": {"electrical_standard": "3V3_TTL", "verified": True},
            "display_uart4": {"electrical_standard": "3V3_TTL", "verified": True},
        },
        "safety": {
            "actuators_physically_disconnected": True,
            "hardware_estop_cuts_actuator_enable": True,
            "global_enable_safe_during_reset": True,
            "servo_power_safe_during_reset": True,
            "boot0_and_nrst_reviewed": True,
            "watchdog_and_brownout_plan_reviewed": True,
        },
        "mechanical_inputs": {
            "axis35_origin": {"pin": "PE8", "electrical_interface_verified": True},
            "axis28_origin": {"pin": "PE10", "electrical_interface_verified": True},
            "dm4310_origin": {"fitted": False},
        },
        "tof": {"fitted": False, "runtime_status": "NOT_PRESENT"},
    }


class P1PreflightTests(unittest.TestCase):
    def test_complete_evidence_passes(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = valid_config()
            paths = list(config["artifacts"].values()) + config["motor_bus"]["manual_paths"]
            for relative in paths:
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text("evidence", encoding="utf-8")
            self.assertEqual(validate_p1_preflight(config, root), [])

    def test_reset_state_failure_blocks_gate(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = valid_config()
            config["safety"]["global_enable_safe_during_reset"] = False
            errors = validate_p1_preflight(config, root)
            self.assertIn("safety.global_enable_safe_during_reset must be true", errors)

    def test_wrong_origin_and_present_tof_block_gate(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = valid_config()
            config["mechanical_inputs"]["axis35_origin"]["pin"] = "PE9"
            config["tof"]["fitted"] = True
            errors = validate_p1_preflight(config, root)
            self.assertIn("axis35 origin must remain on PE8", errors)
            self.assertIn("first-board ToF must be DNP and report NOT_PRESENT", errors)

    def test_unsafe_serial_level_and_missing_manual_block_gate(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = valid_config()
            config["serial_interfaces"]["hwt101_usart2"]["electrical_standard"] = "5V_TTL"
            errors = validate_p1_preflight(config, root)
            self.assertIn(
                "serial_interfaces.hwt101_usart2.electrical_standard is not a reviewed MCU-safe interface",
                errors,
            )
            self.assertIn(
                "motor_bus.manual_paths[0] does not exist: evidence/motor-manual.pdf",
                errors,
            )


if __name__ == "__main__":
    unittest.main()
