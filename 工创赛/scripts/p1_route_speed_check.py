"""Build and emulate live route-speed firmware with offline adapters; no hardware IO."""
import argparse
import importlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

from p1_route_c_check import extract
from p1_route_preview import USER, candidate_profile
from p1_four_wheel_sim import SOURCE_FILES


def run(out, arm_bin, unicorn_path=None, only_variant=None, left_trim=False):
    if unicorn_path:
        sys.path.insert(0, str(unicorn_path))
    uc = importlib.import_module("unicorn")
    reg = importlib.import_module("unicorn.arm_const")
    out.mkdir(parents=True, exist_ok=False)
    main = (USER / "main.c").read_bytes().decode("gbk")
    end = main.index("} ForwardHoldDebug;") + len("} ForwardHoldDebug;")
    forward_type = main[main.rfind("typedef struct", 0, end):end]
    frontend = "\n".join(extract(main, name) for name in ["App_MotionPlanValid", "App_MoveSubmit", "TurnStart", "App_CombinedLoadAction"])
    template = (Path(__file__).parent / "p1_offline_tests/route_speed.c").read_text(encoding="utf-8")
    template = template.replace("/* FRONT_TYPE */", forward_type).replace("/* FRONT_FUNCTIONS */", frontend)
    reports = []
    for name, custom, trial, entry, mode, altered in [
        ("candidate", 1, 1, 1, 3, 0), ("independent_caps", 1, 1, 1, 3, 1),
        ("disabled", 1, 0, 1, 3, 0), ("custom_off", 0, 1, 1, 3, 0),
        ("competition", 1, 1, 2, 3, 0), ("mode2", 1, 1, 1, 2, 0),
        ("full20_table", 1, 1, 1, 3, 0)]:
        if only_variant is not None and name != only_variant:
            continue
        directory = out / name; directory.mkdir()
        src = directory / "src"; src.mkdir()
        for filename in set(SOURCE_FILES + ["motion_control.h"]):
            shutil.copyfile(USER / filename, src / filename)
        shutil.copytree(Path(__file__).parent / "p1_offline_tests/stubs", directory / "stubs")
        config = src / "route_config.h"
        blob = config.read_bytes().replace(b"APP_ROUTE_USE_CUSTOM 0", f"APP_ROUTE_USE_CUSTOM {custom}".encode())
        blob = blob.replace(b"APP_ROUTE_SPEED_TRIAL_ENABLE 0", f"APP_ROUTE_SPEED_TRIAL_ENABLE {trial}".encode())
        if altered:
            blob = blob.replace(b"APP_ROUTE_LEFT_MAX_CM_S    20.0f", b"APP_ROUTE_LEFT_MAX_CM_S    15.0f")
            blob = blob.replace(b"APP_ROUTE_RIGHT_MAX_CM_S   20.0f", b"APP_ROUTE_RIGHT_MAX_CM_S   10.0f")
        if name == "full20_table":
            blob = blob.replace(b"X(FORWARD, 30.0f, 5.0f)", b"X(FORWARD, 210.0f, 20.0f)")
            blob = blob.replace(b"X(LEFT,    20.0f, 3.0f)", b"X(LEFT,    210.0f, 20.0f)")
            blob = blob.replace(b"X(TURN,    90.0f, 5.0f)", b"X(RIGHT,   210.0f, 20.0f)")
        config.write_bytes(blob)
        matrix = []
        for direction, kind in enumerate(["FORWARD", "LEFT", "RIGHT"]):
            for mm in [99, 100, 300, 2100, 2101]:
                for speed in [0, 12, 50, 100, 150, 200, 201]:
                    cap = [200, 150, 100][direction] if altered else 200
                    profile = (candidate_profile(kind, mm / 10, speed / 10, (77, 212, 206), left_trim=left_trim)
                               if 100 <= mm <= 2100 and 12.1 <= speed <= cap else None)
                    matrix.append([direction, mm, speed, int(profile is not None),
                                   profile["rpm"] if profile else 0, profile["run_ms"] if profile else 0,
                                   round(profile["peak_cm_s"] * 10000) if profile else 0])
        source = template.replace("/* PARITY_CASES */", "static const uint32_t parity[][7]={\n" +
                                  ",\n".join("{" + ",".join(str(v) + "U" for v in row) + "}" for row in matrix) + "\n};")
        # Real MoveStart/MoveDistance wrappers call the extracted admission function.
        wrappers = """
uint32_t MoveStart(uint32_t d,uint32_t r,uint32_t ms){return App_MoveSubmit(d,r,ms,0,0);}
uint32_t MoveDistanceStart(uint32_t d,uint32_t r,float cm){float mm=cm*10;return App_MoveSubmit(d,r,MotionDistanceRunMsFor(d,r,mm),mm,0);}
"""
        source += wrappers
        (directory / "unit.c").write_bytes(source.encode("gbk"))
        (directory / "unit.sct").write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
        commands = []
        for source_name, obj in [("unit.c", "unit.o"), ("src/chassis_kinematics.c", "kinematics.o"), ("src/motion_profile.c", "profile.o")]:
            commands.append([str(arm_bin / "armcc.exe"), "--cpu", "Cortex-M3", "--c99", "--split_sections", "-O1",
                             "-I", "src", "-I", "stubs", f"-DAPP_ENTRY={entry}", f"-DAPP_DEBUG_MODE={mode}",
                             f"-DAPP_LEFT_TRIM_ENABLE={int(left_trim)}", f"-DALTERED_CAPS={altered}", f"-DFULL20_TABLE={int(name == 'full20_table')}", "-c", source_name, "-o", obj])
        commands.append([str(arm_bin / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct", "unit.o", "kinematics.o", "profile.o", "--library_type=microlib", "-o", "unit.axf"])
        logs = []
        for command in commands:
            result = subprocess.run(command, cwd=directory, capture_output=True, text=True, errors="replace", timeout=60)
            logs.append(dict(command=command, returncode=result.returncode, stdout=result.stdout, stderr=result.stderr))
            (directory / "build.json").write_text(json.dumps(logs, indent=2))
            if result.returncode:
                raise RuntimeError(result.stdout + result.stderr)
        blob = (directory / "unit.axf").read_bytes(); header = struct.unpack_from("<16sHHIIIIIHHHHHH", blob)
        machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
        machine.mem_map(0x08000000, 0x100000); machine.mem_map(0x20000000, 0x20000)
        for index in range(header[10]):
            kind, offset, address, _, size, _, _, _ = struct.unpack_from("<IIIIIIII", blob, header[5] + index * header[9])
            if kind == 1 and size:
                machine.mem_write(address, blob[offset:offset + size])
        stop = 0x080FF000
        machine.reg_write(reg.UC_ARM_REG_SP, 0x2001FFF0); machine.reg_write(reg.UC_ARM_REG_LR, stop | 1)
        machine.emu_start(header[4] | 1, stop, count=4000000000, timeout=180000000)
        value = machine.reg_read(reg.UC_ARM_REG_R0)
        completed = machine.reg_read(reg.UC_ARM_REG_PC) == stop
        symbols = subprocess.check_output([str(arm_bin / "fromelf.exe"), "--text", "-s", "unit.axf"], cwd=directory, text=True, errors="replace")
        def data(label, size):
            found = re.search(r"\b" + label + r"\s+(0x[0-9a-fA-F]+)", symbols)
            return bytes(machine.mem_read(int(found[1], 16), size)) if found else None
        context = data("failure_context", 24)
        raw_models = data("model_results", 60 * 9 * 4)
        models = []
        if raw_models:
            values = struct.unpack("<540I", raw_models)
            for offset in range(0, len(values), 9):
                row = values[offset:offset + 9]
                if row[1]:
                    models.append(dict(zip(["direction", "distance_mm", "reference_mm_s", "delayed_minus3percent", "fault", "result", "cause", "distance_reason", "elapsed_ms"], row)))
        trimmed = data("trim_model_positions", 60*3*4)
        trim_positions = [list(row) for row in struct.iter_unpack("<3f", trimmed)] if trimmed else []
        report = dict(left_trim=left_trim, trim_model_positions=trim_positions, variant=name, ok=completed and not value & 0x80000000, completed=completed,
                      assertions=value if completed and value < 0x80000000 else None,
                      failure_line=value & 0x7fffffff if completed and value & 0x80000000 else None,
                      failure_context=list(struct.unpack("<6I", context)) if context else None,
                      profile_parity_cases=len(matrix), model_cases=models, hardware_accessed=False)
        reports.append(report); (out / "checks.json").write_text(json.dumps(reports, indent=2))
        print(json.dumps({k: report[k] for k in ["variant", "ok", "assertions", "failure_line", "failure_context"]}), flush=True)
        if not report["ok"]:
            raise RuntimeError(f"actual C check failed: {name}")
    return reports


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--arm-bin", type=Path, default=Path(r"D:\Keil_v5\ARM\arm5_compiler\bin"))
    parser.add_argument("--unicorn-path", type=Path)
    parser.add_argument("--variant", choices=["candidate", "independent_caps", "disabled", "custom_off", "competition", "mode2", "full20_table"])
    parser.add_argument("--left-trim", action="store_true", help="test the independent left compensation candidate")
    args = parser.parse_args(); run(args.out, args.arm_bin, args.unicorn_path, args.variant, args.left_trim)
