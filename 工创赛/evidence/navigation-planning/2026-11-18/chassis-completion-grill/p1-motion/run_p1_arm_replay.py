"""Compile the P1 kernel and chassis encoder for Cortex-M4F, then replay MATLAB data."""
import ctypes
import hashlib
import json
import re
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent
ROOT = next(p for p in EVIDENCE.parents if (p / "3.stm32f407工程").is_dir())
PROJECT = ROOT / "3.stm32f407工程"
SOURCE = PROJECT / "CHASSIS/USER"
ARM = Path("D:/Keil_v5/ARM/arm5_compiler/Bin")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def short_path(path):
    buffer = ctypes.create_unicode_buffer(32768)
    if not ctypes.windll.kernel32.GetShortPathNameW(str(path), buffer, len(buffer)):
        raise OSError(f"Cannot resolve short path {path}")
    return buffer.value


def c(value):
    if isinstance(value, list):
        return "{" + ", ".join(map(c, value)) + "}"
    if isinstance(value, bool):
        return str(int(value))
    if isinstance(value, int):
        return str(value)
    literal = format(value, ".12g")
    if "." not in literal and "e" not in literal:
        literal += ".0"
    return literal + "f"


def trace_row(row):
    return [int(row[0]), row[1:10]]


def generate(rows, summary, trajectories):
    initializers = []
    for row in rows:
        fields = [row["q0"], row["a0"], row["q1"], row["duration_ms"] or 0]
        if row["accepted"]:
            duration = row["duration_ms"]
            samples = [[i * duration // 4, sample] for i, sample in enumerate(row["qva_samples"])]
            fields.extend([row["distance_mm"], row["angular_rad"], row["control"], samples,
                           row["raw_at_samples"], row["direction_at_samples"]])
        initializers.append(c(fields))
    lines = ["static const VectorRow vectors[] = {", ",\n".join(initializers), "};"]
    for key, name in [("straight", "straight"), ("diagonal", "diagonal"), ("short", "short_stop")]:
        lines.extend([f"static const TraceRow {name}[] = {{",
                      ",\n".join(c(trace_row(row)) for row in trajectories[key]), "};"])
    keys = ["duration_ms", "held_ms", "distance_mm", "back_mm", "lateral_mm", "rotation_rad"]
    proposal = summary["proposal"]
    for name, src in [("brake", summary["brake"]), ("proposal_before", proposal["before"]),
                      ("proposal_after", proposal["after"])]:
        lines.append(f"static const float {name}[6] = {c([src[key] for key in keys])};")
    lines.append(f"static const unsigned proposal_duration_ms = {proposal['duration_ms']};")
    lines.append(f"static const float proposal_prefix_mm = {c(proposal['prefix_mm'])};")
    for key in ["prefix_q", "prefix_a"]:
        lines.append(f"static const FusionMotionVector proposal_{key} = {c(proposal[key])};")
    lines.append(f"static const FusionMotionVector proposal_target = {c([220, 0, 4 / summary['limits']['W'][3][2]])};")
    if "prefix_errors" in summary:
        probes = summary["prefix_errors"]
        wide = probes["wide"]
        lines.append("#define REPLAY_PREFIX_ERRORS")
        lines.append(f"static const float probe_prefix_mm = {c(wide['prefix_mm'])};")
        lines.append(f"static const float probe_forward = {c(wide['prefix_mm'] + wide['after']['distance_mm'] - 1)};")
        lines.append(f"static const float probe_back = {c(wide['after']['back_mm'] + 1)};")
        lines.append(f"static const float probe_lateral = {c(wide['after']['lateral_mm'] + 1)};")
        yaw = probes["yaw"]
        dt = probes["input"]["dt_ms"] / 1000
        progress = dt * 1000 / yaw["duration_ms"]
        # De Casteljau left controls for the MATLAB zero-yaw start and 0.06 rad/s target.
        ideal_yaw = dt / 4 * (progress**2 * 0.06 + yaw["prefix_q"][2])
        lines.append(f"static const float probe_angle = {c(yaw['after']['rotation_rad'] + ideal_yaw + 0.02)};")
        lateral = yaw["after"]["lateral_mm"] + (yaw["prefix_mm"] + yaw["after"]["distance_mm"]) * (ideal_yaw + 0.02)
        lines.append(f"static const float probe_yaw_tail = {c(lateral)};")
    if summary["scenarios"] >= 77:
        lines.append("#define REPLAY_EXACT_CAPS")
    if "initial_a" in rows[0]:
        lines.append("#define REPLAY_STABLE_DERIVATIVES")
    return "\n".join(lines) + "\n"


def run(command, directory, log):
    proc = subprocess.run(command, cwd=directory, capture_output=True, timeout=120, check=False)
    (directory / log).write_bytes(proc.stdout + proc.stderr)
    if proc.returncode:
        raise RuntimeError((proc.stdout + proc.stderr).decode("gbk", errors="replace"))
    return proc.stdout


def execute(directory):
    sys.path.insert(0, str(ROOT / "工创赛/tmp/position_alignment_deps"))
    import unicorn as uc
    from unicorn import arm_const as reg

    data = (directory / "unit.axf").read_bytes()
    elf = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
    machine.ctl_set_cpu_model(reg.UC_CPU_ARM_CORTEX_M4)
    for base, length in [(0x08000000, 0x100000), (0x20000000, 0x20000)]:
        machine.mem_map(base, length)
    for i in range(elf[12]):
        _, kind, flags, addr, offset, size, *_ = struct.unpack_from("<IIIIIIIIII", data, elf[6] + i*elf[11])
        if flags & 2 and size:
            machine.mem_write(addr, bytes(size) if kind == 8 else data[offset:offset+size])
    machine.reg_write(reg.UC_ARM_REG_C1_C0_2, 0x00F00000)
    machine.reg_write(reg.UC_ARM_REG_FPEXC, 0x40000000)
    stop = 0x080ff000
    machine.reg_write(reg.UC_ARM_REG_SP, 0x2001fff0)
    machine.reg_write(reg.UC_ARM_REG_LR, stop | 1)
    machine.emu_start(elf[4] | 1, stop, timeout=60_000_000, count=200_000_000)
    if machine.reg_read(reg.UC_ARM_REG_PC) != stop:
        raise RuntimeError("Emulation budget exhausted before returning")

    symbols = run([str(ARM / "fromelf.exe"), "--text", "-s", "unit.axf"], directory, "symbols.txt").decode("gbk", errors="replace")

    def read(name, fmt):
        address = int(re.search(r"\b" + name + r"\s+(0x[0-9a-fA-F]+)", symbols)[1], 16)
        return struct.unpack(fmt, machine.mem_read(address, 4))[0]

    return dict(failed_line=machine.reg_read(reg.UC_ARM_REG_R0),
                assertions=read("replay_assertions", "<I"),
                case=read("replay_case", "<I"), detail=read("replay_detail", "<I"),
                actual=read("replay_actual", "<f"), expected=read("replay_expected", "<f"),
                emulator=f"Unicorn {uc.__version__} with Cortex-M4 CPU and FPU enabled")


def main():
    model = EVIDENCE / sys.argv[1] if len(sys.argv) == 2 else EVIDENCE
    paths = [model / name for name in ["p1_kernel_vectors.json", "p1_kernel_summary.json",
                                      "p1_retained_trajectories.json"]]
    rows, summary, trajectories = [json.loads(p.read_text(encoding="utf-8")) for p in paths]
    unit = EVIDENCE / "p1_arm_replay.c"
    sources = [SOURCE / "fusion_motion.c", SOURCE / "chassis_kinematics.c"]
    inputs = [unit, Path(__file__), *paths, *sources, SOURCE / "fusion_motion.h", SOURCE / "chassis_kinematics.h",
              SOURCE / "app_config.h"]
    report = dict(hardware_accessed=False, cpu="Cortex-M4.fp.sp",
                  matlab_input_dir=model.relative_to(EVIDENCE).as_posix(), matlab_vectors=len(rows),
                  trajectory_samples={name: len(samples) for name, samples in trajectories.items()},
                  input_sha256={p.name: sha(p) for p in inputs})
    tmp = ROOT / "工创赛/tmp"
    tmp.mkdir(exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="p1_arm_replay_", dir=tmp))
    report["working_directory"] = directory.relative_to(ROOT).as_posix()
    try:
        (directory / "p1_arm_frozen.inc").write_text(generate(rows, summary, trajectories), encoding="ascii")
        (directory / "unit.c").write_bytes(unit.read_bytes())
        (directory / "unit.sct").write_text(
            "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
            "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n", encoding="ascii")
        control = ET.parse(PROJECT / "gongxun-f407.uvprojx").find(
            "./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls")
        opts = ["--cpu", report["cpu"], "--c99", "--gnu", "--split_sections", "-O1"]
        opts.extend("-D" + arg for arg in control.findtext("Define").split(","))
        opts.extend("-I" + short_path((PROJECT / inc).resolve()) for inc in control.findtext("IncludePath").split(";"))
        objects = []
        for source in [directory / "unit.c", *sources]:
            obj = source.stem + ".o"
            run([str(ARM / "armcc.exe"), *opts, "-c", short_path(source), "-o", obj],
                directory, source.stem + "_compile.log")
            objects.append(obj)
        run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct", *objects,
             "--library_type=microlib", "-o", "unit.axf"], directory, "link.log")
        report.update(execute(directory))
        report["axf_sha256"] = sha(directory / "unit.axf")
        if report["failed_line"]:
            report["failed_expression"] = unit.read_text().splitlines()[report["failed_line"]-1].strip()
        report["status"] = "FAIL" if report["failed_line"] else "PASS"
    except Exception as exc:
        report.update(status="ERROR", error=str(exc))
    if report["input_sha256"] != {p.name: sha(p) for p in inputs}:
        report.update(status="ERROR", error="Replay inputs changed during execution")
    (directory / "receipt.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
