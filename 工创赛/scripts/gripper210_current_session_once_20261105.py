"""用户明确授权的单次210°PWM调整；仅写TIM2_CCR1，无复位/暂停/固件/其他轴写入。"""
from pathlib import Path
import argparse
import hashlib
import json
import time

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "evidence/integration/2026-11-05/gripper210-current-session"
CCR1 = 0x40000034
EXPECTED = {
    "g_vehicle_stage": 1, "g_vehicle_state": 0, "g_vehicle_cause": 0, "g_vehicle_run_id": 0,
    "g_vehicle_key": 0, "g_vehicle_alignment_state": 0, "g_vehicle_arm_state": 0,
    "g_vehicle_arm_cause": 0, "g_source_prepare_state": 0, "g_source_prepare_cause": 0,
    "g_source_prepare_done_mask": 0, "g_startup_axes_state": 0, "g_startup_axes_cause": 0,
    "g_vm_test_request": 0, "g_chassis_auto_active": 0, "g_rtos_fault": 0, "g_safety_fault": 0,
    "g_safety_ready": 1, "g_dm_jog_request": 0, "g_dm_jog_state": 0,
    "g_source_prepare_grip_deg": 145.0, "core_state": "RUNNING",
    "TIM2_CR1": 129, "TIM2_CCMR1": 104, "TIM2_CCER": 1, "TIM2_PSC": 83, "TIM2_ARR": 19999,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    plan = json.loads((OUT / "matlab-signal-plan.json").read_text(encoding="utf-8"))
    baseline = json.loads((OUT / "readonly-sync-report.json").read_text(encoding="utf-8"))
    assert plan["allowed_signal_plan"] and plan["failure_count"] == 0 and plan["engine"] == "local MATLAB"
    assert plan["goal_pulse_us"] == 2056 and plan["start_pulse_us"] == 1574
    assert plan["sole_target_write_address_hex"] == hex(CCR1)
    assert baseline["flash_matches"] and baseline["flash_bytes_compared"] == 148728
    pulses = list(map(int, plan["pulses_us"]))
    assert pulses[-1] == 2056 and all(1574 <= p <= 2056 for p in pulses)
    assert all(0 <= b-a <= 12 for a, b in zip([1574]+pulses, pulses))
    if not args.execute:
        print(json.dumps({"plan_valid": True, "execute_requested": False, "hardware_opened": False}))
        return 0
    symbols, segments = transport.live.load_image()
    image_sha = hashlib.sha256(transport.live.IMAGE.read_bytes()).hexdigest()
    assert image_sha == baseline["image_sha256"] == plan["baseline_image_sha256"]
    options = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
               "auto_unlock": False, "resume_on_disconnect": False, "dap_protocol": "swd", "no_config": True,
               "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    report = {"requested_nominal_angle_deg": 210, "temporary_debug_adjustment": True,
              "sole_write_address": hex(CCR1), "reset_requested": False, "halt_requested": False,
              "firmware_downloaded": False, "mission_triggered": False, "other_axis_commands": 0,
              "physical_angle_feedback_available": False, "image_sha256": image_sha, "writes": []}
    session = None
    try:
        settings = Session(None, options=options)
        probes = ConnectHelper.get_all_connected_probes(blocking=False)
        if len(probes) != 1 or probes[0].unique_id != baseline["interface_attributes"]["serial_number"]:
            raise RuntimeError("Probe identity/count changed")
        interface = probes[0]._link._interface
        if not getattr(interface, "is_bulk", False):
            raise RuntimeError("Expected previously inspected v2 bulk interface")
        transport.synchronous_interface(interface)
        session = Session(probes[0], options=options)
        session.open()
        target = session.target

        def guard(pulse):
            if target.get_state() != Target.State.RUNNING:
                raise RuntimeError("Core no longer running; do not resume/reset")
            state = transport.live.snapshot(target, symbols)
            for key, value in EXPECTED.items():
                if state.get(key) != value:
                    raise RuntimeError(f"Guard changed: {key}={state.get(key)!r}, expected {value!r}")
            if state["TIM2_CCR1"] != pulse or state["dbg_idcode"] != baseline["snapshot"]["dbg_idcode"]:
                raise RuntimeError("PWM/core identity changed")
            if state["GPIOA_MODER"] & 3 != 2 or state["GPIOA_AFRL"] & 15 != 1:
                raise RuntimeError("PA0 is no longer TIM2_CH1")
            for name in ("g_can_ctrl_fault_latched", "g_vehicle_arm_stop_pending"):
                if name in symbols and target.read32(symbols[name]) != 0:
                    raise RuntimeError(f"{name} changed")
            return state

        report["before"] = guard(1574)
        compared = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                expected = data[offset:offset+4096]
                if bytes(target.read_memory_block8(address+offset, len(expected))) != expected:
                    raise RuntimeError("Flash identity mismatch; no PWM write")
                compared += len(expected)
        report["flash_bytes_verified_before_write"] = compared
        report["matlab_signal_plan_sha256"] = hashlib.sha256((OUT / "matlab-signal-plan.json").read_bytes()).hexdigest()
        pulse = 1574
        start = time.monotonic()
        for index, next_pulse in enumerate(pulses, 1):
            if time.monotonic()-start > 30:
                raise RuntimeError("Bounded adjustment deadline reached; do not continue or reverse")
            guard(pulse)
            if next_pulse != pulse:
                target.write32(CCR1, next_pulse)  # 唯一执行器写地址；不写任务状态或历史命令字段。
                target.flush()
                report["writes"].append({"step": index, "pulse_us": next_pulse})
            pulse = next_pulse
            time.sleep(plan["minimum_interval_s"])
            guard(pulse)
        report["held_samples"] = []
        for _ in range(3):
            time.sleep(.3)
            report["held_samples"].append(guard(2056))
        report["pwm_command_completed"] = True
        report["other_state_unchanged"] = True
        report["elapsed_s"] = time.monotonic()-start
    except Exception as exc:
        report["error"] = repr(exc)
    finally:
        if session is not None:
            session.close()
        (OUT / "execute-result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("pwm_command_completed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
