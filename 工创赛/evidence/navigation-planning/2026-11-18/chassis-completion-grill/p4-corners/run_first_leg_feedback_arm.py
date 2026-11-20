"""Replay the first-leg feedback vectors using the P1 ARM toolchain."""
import json
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "p1-motion"))
from run_p1_arm_replay import ARM, PROJECT, ROOT, SOURCE, execute, run, sha, short_path

unit = HERE / "first_leg_feedback_arm.c"
sources = [SOURCE / "fusion_motion.c", SOURCE / "chassis_kinematics.c"]
directory = Path(tempfile.mkdtemp(prefix="first_leg_feedback_arm_", dir=ROOT / "工创赛/tmp"))
control = ET.parse(PROJECT / "gongxun-f407.uvprojx").find(
    "./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls")
opts = ["--cpu", "Cortex-M4.fp.sp", "--c99", "--gnu", "--split_sections", "-O1"]
opts += ["-D" + value for value in control.findtext("Define").split(",")]
opts += ["-I" + short_path((PROJECT / path).resolve()) for path in control.findtext("IncludePath").split(";")]
inputs = [Path(__file__), unit, *sources, SOURCE / "fusion_motion.h", SOURCE / "chassis_kinematics.h"]
input_hashes = {p.name: sha(p) for p in inputs}
for source, name in zip([unit, *sources], ["unit", "motion", "kinematics"]):
    run([str(ARM / "armcc.exe"), *opts, "-c", short_path(source), "-o", name + ".o"],
        directory, name + "_compile.log")
(directory / "unit.sct").write_text(
    "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
    "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n", encoding="ascii")
run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct",
     "unit.o", "motion.o", "kinematics.o", "--library_type=microlib", "-o", "unit.axf"],
    directory, "link.log")
receipt = execute(directory)
receipt.update(directory=directory.relative_to(ROOT).as_posix(), input_sha256=input_hashes,
               axf_sha256=sha(directory / "unit.axf"), hardware_accessed=False)
if receipt["failed_line"]:
    receipt["failed_expression"] = unit.read_text().splitlines()[receipt["failed_line"] - 1].strip()
passed = not receipt["failed_line"] and input_hashes == {p.name: sha(p) for p in inputs}
receipt["status"] = "PASS" if passed else "FAIL"
(directory / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
print(json.dumps(receipt, ensure_ascii=True))
sys.exit(0 if passed else 1)
