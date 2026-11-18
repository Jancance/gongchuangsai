"""Compile the schedule and geometry kernels for isolated independent Cortex-M4F review."""
import json
from pathlib import Path
import runpy
import sys
import tempfile
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
COMMON = runpy.run_path(str(BASE / "p1-motion/run_p1_arm_replay.py"))
PROJECT, SOURCE, ARM = (COMMON[name] for name in ("PROJECT", "SOURCE", "ARM"))
run, execute, short, sha = (COMMON[name] for name in ("run", "execute", "short_path", "sha"))


def main():
    out = Path(tempfile.mkdtemp(prefix="zone2_high_review_", dir=BASE.parents[3] / "tmp"))
    harness = HERE / "zone2_review_fixture.c"
    sources = [SOURCE / f"{name}.c" for name in
               ("zone2_nav_schedule", "zone2_nav_path", "field_map", "fusion_reference", "chassis_kinematics")]
    project = PROJECT / "gongxun-f407.uvprojx"
    inputs = [Path(__file__), harness, project, *sources, *(source.with_suffix(".h") for source in sources)]
    hashes = {p.name: sha(p) for p in inputs}
    report = {"hardware_accessed": False, "input_sha256": hashes}
    try:
        cfg = ET.parse(project).find("./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls")
        opts = ["--cpu", "Cortex-M4.fp.sp", "--c99", "--gnu", "--split_sections", "-O1"]
        opts.extend("-D" + item for item in cfg.findtext("Define").split(","))
        opts.extend("-I" + short((PROJECT / inc).resolve()) for inc in cfg.findtext("IncludePath").split(";"))
        (out / "unit.sct").write_text(
            "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
            "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n", encoding="ascii")
        objects = []
        for source in [harness, *sources]:
            obj = ("unit" if source == harness else source.stem) + ".o"
            run([str(ARM / "armcc.exe"), *opts, "-c", short(source), "-o", obj], out, source.stem + ".log")
            objects.append(obj)
        run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct",
             *objects, "--library_type=microlib", "-o", "unit.axf"], out, "link.log")
        report.update(execute(out))
        report["axf_sha256"] = sha(out / "unit.axf")
        report["status"] = "FAIL" if report["failed_line"] else "PASS"
        if report["failed_line"]:
            report["failed_expression"] = harness.read_text().splitlines()[report["failed_line"] - 1].strip()
    except Exception as exc:
        report.update(status="ERROR", error=str(exc))
    if hashes != {p.name: sha(p) for p in inputs}:
        report.update(status="ERROR", error="Replay inputs changed during execution")
    (out / "receipt.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(out)
    print(json.dumps(report))
    return int(report["status"] != "PASS")


if __name__ == "__main__":
    sys.exit(main())
