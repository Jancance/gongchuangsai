import json
import runpy
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
OBSERVER = HERE.parent / "p3-observer"
COMMON = runpy.run_path(str(HERE.parent / "p1-motion/run_p1_arm_replay.py"))
ROOT, PROJECT, SOURCE, ARM = (COMMON[k] for k in ("ROOT", "PROJECT", "SOURCE", "ARM"))
run, execute, short, sha = (COMMON[k] for k in ("run", "execute", "short_path", "sha"))


def main():
    radar = PROJECT / "CHASSIS/HARDWARE/RADAR_V6"
    servo = radar.parent / "RADAR_SERVO"
    out = Path(tempfile.mkdtemp(prefix="zone2_schedule_", dir=ROOT / "工创赛/tmp"))
    project = PROJECT / "gongxun-f407.uvprojx"
    fixture = HERE / "schedule_replay.c"
    sources = [fixture, radar / "radar_client.c", radar / "radar_observer.c",
               next(PROJECT.rglob("link_protocol.c"))]
    sources += [SOURCE / (stem + ".c") for stem in
                ("zone2_nav_schedule", "zone2_nav_path", "fusion_reference", "field_map", "chassis_kinematics")]
    inputs = [Path(__file__), fixture, project, *sources[1:],
              OBSERVER / "independent_fixture.c", OBSERVER / "independent_stm32_stub.h"]
    shutil.copyfile(fixture, out / "unit.c")
    for src, dest in (("independent_fixture.c", "observer_fixture.c"),
                      ("independent_stm32_stub.h", "stm32f4xx.h")):
        shutil.copyfile(OBSERVER / src, out / dest)
    (out / "FreeRTOS.h").write_text("#define portTICK_PERIOD_MS 1U\n#define pdMS_TO_TICKS(x) (x)\n", "ascii")
    (out / "task.h").write_text("#include <stdint.h>\nuint32_t xTaskGetTickCount(void);\nvoid vTaskDelay(uint32_t);\n", "ascii")
    (out / "unit.sct").write_text(
        "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
        "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n", "ascii")
    control = ET.parse(project).find("./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls")
    opts = ["--cpu", "Cortex-M4.fp.sp", "--c99", "--gnu", "--split_sections", "-O1", "-I."]
    opts.extend("-D" + arg for arg in control.findtext("Define").split(","))
    opts.extend("-I" + short((PROJECT / inc).resolve()) for inc in control.findtext("IncludePath").split(";"))
    opts.extend("-I" + short(p) for p in (radar, servo, sources[3].parent))
    hashes = {p.name: sha(p) for p in inputs}
    objects = []
    print(out, flush=True)
    for source in [out / "unit.c", *sources[1:]]:
        obj = source.stem + ".o"
        run([str(ARM / "armcc.exe"), *opts, "-c", short(source), "-o", obj], out, source.stem + ".log")
        objects.append(obj)
    run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct",
         *objects, "--library_type=microlib", "-o", "unit.axf"], out, "link.log")
    result = execute(out)
    result.update(hardware_accessed=False, input_sha256=hashes, axf_sha256=sha(out / "unit.axf"))
    if result["failed_line"]:
        result["failed_expression"] = fixture.read_text().splitlines()[result["failed_line"]-1].strip()
    if hashes != {p.name: sha(p) for p in inputs}:
        raise RuntimeError("Replay inputs changed")
    (out / "receipt.json").write_text(json.dumps(result, indent=2) + "\n", "utf-8")
    print(json.dumps(result))
    return int(bool(result["failed_line"]))


if __name__ == "__main__":
    raise SystemExit(main())
