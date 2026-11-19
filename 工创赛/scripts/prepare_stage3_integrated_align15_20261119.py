"""Prepare the Stage3 integrated forward-15/right-5 alignment firmware."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from prepare_stage3_turn_queue_20261119 import ROOT, TARGETS, digest, edit, extract_function, verify_sources

BASE = ROOT / "tmp/stage3_turn_queue_20261119/stage3-turn-queue-compatible.json"
OUT = ROOT / "tmp/stage3_integrated_align15_yaw365_20261119"
MATLAB = ROOT / "evidence/integration/2026-11-18/stage3-resume/stage3-integrated-align15-matlab.log"
SOURCE = "CHASSIS/USER/stage3_cycle.inc"
ONLINE_SHA256 = "fe6c0decaac3652fd1ad75ffff2e74e1cce6e277c63da7112aca388279d2666a"
TARGET_PIXEL = [997.944213867, 553.645187378]


def patch(project):
    edit(project, "stage3_cycle.inc", (
        "volatile uint32_t g_stage3_replay_enable,g_stage3_replay_state,g_stage3_replay_step;",
        """volatile uint32_t g_stage3_replay_enable,g_stage3_replay_state,g_stage3_replay_step;
volatile uint32_t g_stage3_integrated_compensation=1U;"""), (
        "volatile float g_stage3_carrier_yaw=37.0f,g_stage3_pick_depth=-1.0f;",
        "volatile float g_stage3_carrier_yaw=36.5f,g_stage3_pick_depth=-1.0f;"), (
        "    S3_Pose();g_stage3_state=3U;g_stage3_replay_state=2U;return;",
        """    /* Host visual error already includes the forward-15/right-5 target. */
    g_stage3_integrated_compensation=1U;
    S3_Pose();g_stage3_state=4U;g_stage3_replay_state=3U;
    /* Close an existing correction chain before requesting fresh wheel hold. */
    if(g_source_align_chain_active)g_source_body_step_request=7U;
    return;"""))


def fixture(project):
    firmware = (project / SOURCE).read_bytes().decode("gbk", errors="surrogateescape").replace("\r\n", "\n")
    harness = Path(__file__).with_name("stage3_integrated_align15_fixture_20261119.c").read_text(encoding="ascii")
    setup, cases = harness.split("// INSERT_EXTRACTED_FIRMWARE\n")
    car = next(line for line in firmware.splitlines() if line.startswith("volatile float g_stage3_carrier_yaw="))
    abi = next(line for line in firmware.splitlines() if line.startswith("volatile uint32_t g_stage3_integrated_compensation="))
    lock = extract_function(firmware, "static void S3_LockConfig(void)")
    replay = extract_function(firmware, "static void S3_ReplayService(void)")
    service = extract_function(firmware, "static void Stage3_Service(uint32_t now)")
    accept = service[service.index("  if(!S3_Fresh(now))return;"):
                     service.index("  /* Unsupported sub-step motion")]
    cancel = next(line for line in service.splitlines() if line.startswith(" if(request==9U && g_stage3_state"))
    dispatch = service[service.index(" if(g_stage3_replay_state>=2U && g_stage3_replay_state<=4U){"):
                       service.index(" if(g_stage3_replay_state==5U && request")]
    ifs = "static void visual_accept(void) { uint32_t now=0;\n" + accept + "}\n"
    finish = "static void final_service(uint32_t request) {\n" + cancel + "\n" + dispatch + "}\n"
    lock = lock.replace("s3_first=g_stage3_test_first_item;", "++config_count; s3_first=g_stage3_test_first_item;")
    source = "\n".join((setup, car, abi, lock, replay, ifs, finish, cases))
    cfile, binary = OUT / "actual-integrated-align15.c", OUT / "actual-integrated-align15.exe"
    cfile.write_text(source, encoding="ascii", errors="replace")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Werror", "-Wno-misleading-indentation",
                    str(cfile), "-o", str(binary)], check=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=30)
    log = OUT / "actual-integrated-align15.log"
    log.write_text(result.stdout, encoding="ascii")
    print(result.stdout, end="", flush=True)
    return {"log": str(log), "sha256": digest(log)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    if "PASS integrated visual compensation: 133 cases; forward15/right5; target 997.944213867 553.645187378" not in MATLAB.read_text(encoding="utf-8"):
        raise RuntimeError("Expected matching MATLAB pre-validation")
    manifest = json.loads(BASE.read_text(encoding="utf-8"))
    baseline = Path(manifest["project"]).resolve()
    if manifest["images"][TARGETS[0]]["sha256"] != ONLINE_SHA256:
        raise ValueError("Unexpected turn queue baseline")
    verify_sources(manifest, baseline)
    for image in manifest["images"].values():
        if digest(Path(image["axf"])) != image["sha256"]:
            raise ValueError(f"Baseline image hash mismatch: {image['axf']}")
    project = OUT / "project"
    result_path = OUT / "stage3-integrated-align15-compatible.json"
    if not project.exists():
        shutil.copytree(baseline, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
        patch(project)
        manifest["changes"][SOURCE] = digest(project / SOURCE)
        manifest["capabilities"]["stage3_integrated_visual_compensation"] = True
        manifest.pop("c_fixture", None)
        manifest.update(project=str(project), images={}, build_passed=False, flashed=False,
                        baseline_manifest=str(BASE), baseline_manifest_sha256=digest(BASE),
                        matlab_sha256=digest(MATLAB),
                        integrated_visual_compensation={"target_pixel": TARGET_PIXEL,
                            "forward_mm": 15, "right_mm": 5, "carrier_yaw_deg": 36.5,
                            "abi_symbol": "g_stage3_integrated_compensation", "abi_value": 1})
        result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    else:
        manifest = json.loads(result_path.read_text(encoding="utf-8"))
        if manifest["baseline_manifest_sha256"] != digest(BASE):
            raise ValueError("Baseline manifest changed")
    verify_sources(manifest, project)
    manifest["c_fixture"] = fixture(project)
    if args.build:
        manifest.update(images={}, build_passed=False)
        result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        for target in TARGETS:
            for directory in ("Objects", "Listings"):
                (project / directory / target).mkdir(parents=True, exist_ok=True)
            log = OUT / f"{target}.log"
            result = subprocess.run(["C:/Keil_v5/UV4/UV4.exe", "-r", str(project / "gongxun-f407.uvprojx"),
                                     "-t", target, "-j0", "-o", str(log)], timeout=180)
            if result.returncode or b"0 Error(s), 0 Warning(s)" not in log.read_bytes():
                raise RuntimeError(f"Build failed: {log}")
            image = project / "Objects" / target / f"{target}.axf"
            manifest["images"][target] = {"axf": str(image), "sha256": digest(image)}
            print(f"{target}: 0 errors, 0 warnings", flush=True)
        manifest["build_passed"] = True
    verify_sources(manifest, project)
    result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(result_path)


if __name__ == "__main__":
    main()
