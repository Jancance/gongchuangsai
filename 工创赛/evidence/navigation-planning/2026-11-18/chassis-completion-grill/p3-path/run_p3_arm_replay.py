"""Run P3 layout, path conversion and reference sweeps against frozen MATLAB vectors."""
import json
import runpy
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


STAGE = Path(__file__).resolve().parent
COMMON = runpy.run_path(str(STAGE.parent / "p1-motion/run_p1_arm_replay.py"))
ROOT, PROJECT, SOURCE, ARM = (COMMON[name] for name in ["ROOT", "PROJECT", "SOURCE", "ARM"])
run, execute, short, sha, c = (COMMON[name] for name in ["run", "execute", "short_path", "sha", "c"])


def turn(headings):
    return [headings["hold"], headings["first"], headings["second"]]


def leg(row):
    path = row["waypoints"]
    if isinstance(path, int):
        path = [path]
    start_heading = row["headings"]["start"]
    goal_heading = row["headings"]["terminal"]
    alternate = row["opposite_turn"]
    return [[row["from"], row["to"], start_heading, goal_heading], row["request_seq"],
            row["cache_hit"], len(path), path, sum(1 << i for i in row["cells"]),
            len(row["points_mm"]), row["points_mm"], turn(row["headings"]),
            turn(alternate) if alternate else [0, 0, 0], bool(alternate)]


def generate(model):
    legs = model["legs"]
    detour = model["detour"]
    bypass_fixture = {**legs[2], **detour, "request_seq": 10}
    rows = [c([*errors, all(margin >= 0 for margin in margins)])
            for errors, margins in zip(model["return_error_cases"], model["return_margin_mm"])]
    return ("static const LegVector legs[] = {\n" + ",\n".join(map(lambda row: c(leg(row)), legs)) +
            "\n};\nstatic const LegVector detour = " + c(leg(bypass_fixture)) +
            ";\nstatic const ReturnVector return_vectors[] = {\n" + ",\n".join(rows) + "\n};\n")


def main():
    vectors = STAGE / "p3_path_vectors.json"
    fixture = json.loads(vectors.read_text(encoding="utf-8"))
    unit = STAGE / "p3_arm_replay.c"
    sources = [SOURCE / name for name in [
        "zone2_nav_path.c", "field_map.c", "fusion_reference.c", "chassis_kinematics.c"]]
    headers = [SOURCE / name for name in [
        "zone2_nav_path.h", "field_map.h", "fusion_reference.h", "vehicle_tuning.h", "chassis_kinematics.h"]]
    headers.append(PROJECT / "CHASSIS/HARDWARE/RADAR_V6/radar_client.h")
    inputs = [Path(__file__), unit, vectors, *sources, *headers]
    report = dict(hardware_accessed=False, cpu="Cortex-M4.fp.sp",
                  planned_legs=len(fixture["legs"]), directed_requests=len(fixture["requests"]),
                  input_sha256={p.name: sha(p) for p in inputs})
    temp = Path(tempfile.mkdtemp(prefix="p3_arm_replay_", dir=ROOT / "工创赛/tmp"))
    report["working_directory"] = temp.relative_to(ROOT).as_posix()
    try:
        (temp / "unit.c").write_bytes(unit.read_bytes())
        (temp / "p3_frozen.inc").write_text(generate(fixture), encoding="ascii")
        (temp / "unit.sct").write_text(
            "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
            "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n", encoding="ascii")
        cfg = ET.parse(PROJECT / "gongxun-f407.uvprojx").find(
            "./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls")
        opts = ["--cpu", report["cpu"], "--c99", "--gnu", "--split_sections", "-O1"]
        opts.extend("-D" + arg for arg in cfg.findtext("Define").split(","))
        opts.extend("-I" + short((PROJECT / inc).resolve()) for inc in cfg.findtext("IncludePath").split(";"))
        objects = []
        for source in [temp / "unit.c", *sources]:
            obj = source.stem + ".o"
            run([str(ARM / "armcc.exe"), *opts, "-c", short(source), "-o", obj], temp, source.stem + "_compile.log")
            objects.append(obj)
        run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct", *objects,
             "--library_type=microlib", "-o", "unit.axf"], temp, "link.log")
        report.update(execute(temp))
        report["axf_sha256"] = sha(temp / "unit.axf")
        report["status"] = "FAIL" if report["failed_line"] else "PASS"
        if report["failed_line"]:
            report["failed_expression"] = unit.read_text().splitlines()[report["failed_line"] - 1].strip()
    except Exception as exc:
        report.update(status="ERROR", error=str(exc))
    if report["input_sha256"] != {p.name: sha(p) for p in inputs}:
        report.update(status="ERROR", error="Replay inputs changed during execution")
    (temp / "receipt.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
