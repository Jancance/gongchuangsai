"""Compile live motion C with offline CAN/RTOS stubs, then run in ARM emulator."""
import argparse
import importlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

from p1_motion_offline import ROOT, safe_output, save_json, sha256
from p1_simulation_report import TRACE_FIELDS, decode_sample, export_traces

CASES = ["equal_response", "independent_delay_0_20_40_60ms", "wheel3_20percent_slow",
         "wheel3_speed_reply_missing", "wheel1_position_reply_missing", "wheel2_stop_ack_missing",
         "wheel2_overspeed", "wheel4_reverse", "heading_command_stale", "CAN_stalled_during_stop"]
SOURCE_FILES = ["chassis_auto_test.c", "chassis_auto_test.h", "chassis_kinematics.c", "chassis_kinematics.h",
                "motion_profile.c", "motion_profile.h", "motion_distance_limits.h", "motion_trial20.h", "app_config.h", "app_entry_config.h",
                "app_entry.c", "app_entry.h", "motion_action_plan.h", "route_plan.h", "route_config.h", "motion_route_trial.h"]


def run_sim(out, arm_bin=Path(r"D:\Keil_v5\ARM\arm5_compiler\bin"), unicorn_path=None):
    out, arm_bin = Path(out), Path(arm_bin)
    if unicorn_path:
        sys.path.insert(0, str(Path(unicorn_path).resolve()))
    uc = importlib.import_module("unicorn")
    reg = importlib.import_module("unicorn.arm_const")
    src = out / "src"; src.mkdir()
    hashes = {}
    for name in SOURCE_FILES:
        original = ROOT / "3.stm32f407工程/USER" / name
        hashes[name] = sha256(original)
        shutil.copyfile(original, src / name)
    adapter = Path(__file__).parent / "p1_offline_tests"
    shutil.copytree(adapter / "stubs", out / "stubs")
    shutil.copyfile(adapter / "four_wheel.c", out / "unit.c")
    (out / "unit.sct").write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    commands = []
    for source, obj in [("unit.c", "unit.o"), ("src/chassis_kinematics.c", "kinematics.o"), ("src/motion_profile.c", "profile.o")]:
        commands.append([str(arm_bin / "armcc.exe"), "--cpu", "Cortex-M3", "--c99", "--split_sections", "-O1", "-I", "stubs", "-I", "src",
                         "-DAPP_DISTANCE_SCURVE_ENABLE=1", "-DAPP_SPEED20_TRIAL_ENABLE=1", "-DAPP_ENTRY=1", "-DAPP_DEBUG_MODE=3", "-c", source, "-o", obj])
    commands.append([str(arm_bin / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct", "unit.o", "kinematics.o", "profile.o", "--library_type=microlib", "-o", "unit.axf"])
    build_log = []
    for cmd in commands:
        p = subprocess.run(cmd, cwd=out, capture_output=True, text=True, errors="replace", timeout=60)
        build_log.append({"command": cmd, "returncode": p.returncode, "stdout": p.stdout, "stderr": p.stderr})
        save_json(out / "build.json", build_log)
        if p.returncode:
            raise RuntimeError(f"C test build failed: {p.stderr or p.stdout}")
    blob = (out / "unit.axf").read_bytes()
    h = struct.unpack_from("<16sHHIIIIIHHHHHH", blob)
    machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
    machine.mem_map(0x08000000, 0x100000); machine.mem_map(0x20000000, 0x20000)
    for i in range(h[10]):
        kind, offset, address, _, size, _, _, _ = struct.unpack_from("<IIIIIIII", blob, h[5] + i*h[9])
        if kind == 1 and size:
            machine.mem_write(address, blob[offset:offset+size])
    stop = 0x080FF000
    machine.reg_write(reg.UC_ARM_REG_SP, 0x2001FFF0); machine.reg_write(reg.UC_ARM_REG_LR, stop | 1)
    symbols = subprocess.check_output([str(arm_bin / "fromelf.exe"), "--text", "-s", "unit.axf"], cwd=out, text=True, errors="replace", timeout=30)
    def address(label):
        match = re.search(r'\b' + re.escape(label) + r'\s+(0x[0-9a-fA-F]+)', symbols)
        if not match:
            raise RuntimeError(f"missing simulation symbol: {label}")
        return int(match[1], 16)
    trace_address = address("trace_values")
    samples = []
    def capture_trace(machine, pc, size, user_data):
        values = struct.unpack('<' + str(len(TRACE_FIELDS)) + 'f', machine.mem_read(trace_address, len(TRACE_FIELDS)*4))
        samples.append(decode_sample(values))
    hook_address = address("Trace_Emit") & ~1
    machine.hook_add(uc.UC_HOOK_CODE, capture_trace, begin=hook_address, end=hook_address)
    machine.emu_start(h[4] | 1, stop, timeout=45000000, count=2000000000)
    found = re.search(r"\bresults\s+(0x[0-9a-fA-F]+)", symbols)
    if not found:
        raise RuntimeError("missing simulation result symbol")
    values = struct.unpack("<190I", machine.mem_read(int(found[1], 16), 760))
    rows = []
    for i, name in enumerate(CASES):
        v = values[i*19:(i+1)*19]
        rows.append(dict(zip(["case_id", "result", "cause", "detail", "address", "distance_reason", "reached", "stop_mask", "cut_vm_request", "elapsed_ms"], v[:10]),
                         name=name, wheel_encoder_mm=[x/1000 for x in v[10:14]], peak_command_rpm=list(v[14:18]), cumulative_assertions=v[18]))
    result = machine.reg_read(reg.UC_ARM_REG_R0)
    completed = machine.reg_read(reg.UC_ARM_REG_PC) == stop
    trace_report = export_traces(out, samples)
    report = {"ok": completed and not result & 0x80000000 and trace_report['ok'], "completed": completed,
              "assertions": result if completed and result < 0x80000000 else None,
              "first_failure_line": result & 0x7FFFFFFF if completed and result & 0x80000000 else None,
              "source_sha256": hashes, "cases": rows, "hardware_accessed": False,
              "trace_sample_count": len(samples), "trace_validation_ok": trace_report['ok'], "trace_index": "index.html",
              "limits": ["Actual chassis backend, kinematics and profile C; stubbed RTOS, CAN and VM.",
                         "10ms CAN task, instant stub transactions; independent wheel delay and first-order response are assumptions.",
                         "RUN entry is seeded; enable/VM preparation, frontend HWT PID, actual bus/RTOS timing are outside this harness.",
                         "No ground XY/yaw/slip model. Encoder estimates and cut-VM requests are not physical outcomes."]}
    save_json(out / "simulation.json", report)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--arm-bin", type=Path, default=Path(r"D:\Keil_v5\ARM\arm5_compiler\bin"))
    p.add_argument("--unicorn-path", type=Path)
    args = p.parse_args()
    report = run_sim(safe_output(args.out), args.arm_bin, args.unicorn_path)
    print(json.dumps({k: report[k] for k in ("ok", "assertions", "first_failure_line")}, ensure_ascii=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
