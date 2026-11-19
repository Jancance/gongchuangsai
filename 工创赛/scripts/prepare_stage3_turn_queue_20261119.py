"""Build the isolated Stage3 queued-turn candidate."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from prepare_stage3_speed_offline_20261119 import ROOT, TARGETS, digest, edit, extract_function

BASE = ROOT / "tmp/stage3_turn_ready_20261119/stage3-three-place-compatible.json"
OUT = ROOT / "tmp/stage3_turn_queue_20261119"
MATLAB = ROOT / "evidence/integration/2026-11-18/stage3-resume/stage3-turn-queue-matlab.log"
MAIN = "CHASSIS/USER/main.c"


def patch(project):
    anchor = "    if ((g_heading_exec_phase != HR_WAIT) && (g_heading_exec_phase != HR_RUN))"
    edit(project, "main.c", (
        "static uint32_t s_combined_keep_vm, s_combined_powered_segment;",
        "static uint32_t s_combined_keep_vm, s_combined_powered_segment, s_combined_turn;"), (
        "static uint32_t s_combined_turn, s_combined_mode;",
        "static uint32_t s_combined_mode;"), (anchor, """#if APP_TEST_MODE == 3U
    /* The accepted turn owns the existing VM request until CAN consumes it. */
    if ((g_heading_exec_phase == HR_IDLE) && (g_heading_exec_request == 1U) &&
        s_motion_session_open && s_motion_has_action && s_combined_turn &&
        (s_combined_powered_segment == 1U) && (g_vm_test_request == 1U) &&
        !g_heading_exec_trip && !g_can_ctrl_fault_latched &&
        !status.active && !status.result && !status.cut_vm &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive())
    {
        taskEXIT_CRITICAL();
        return;
    }
#endif
""" + anchor))


def fixture(baseline, project):
    old, new = ((p / MAIN).read_bytes().decode("gbk", errors="surrogateescape").replace("\r\n", "\n")
                for p in (baseline, project))
    signature = "static void App_HeadingRuntimeSafety(void)"
    step = extract_function(new, "static void App_HeadingRuntimeStep(void)")
    start = step[step.index("    request = g_heading_exec_request;"):
                 step.index("    if ((g_heading_exec_phase == HR_WAIT) && (g_heading_exec_trip == 0U))")]
    start = start.replace("        HWT101_TimingEnd(0U,critical_cycle);\n", "")
    start = start.replace("        taskEXIT_CRITICAL();\n", "")
    harness = Path(__file__).with_name("stage3_turn_queue_fixture_20261119.c").read_text(encoding="ascii")
    setup, cases = harness.split("// INSERT_EXTRACTED_FIRMWARE\n")
    source = setup
    source += extract_function(old, signature).replace("App_HeadingRuntimeSafety", "old_safety") + "\n"
    source += extract_function(new, signature).replace("App_HeadingRuntimeSafety", "new_safety") + "\n"
    for signature in ("static void App_CombinedTestStop(uint32_t cause)\n{", "void MotionStop(void)\n{"):
        source += extract_function(new, signature) + "\n"
    # Stop the CAN extraction before its power-settle and backend logic.
    source += "static void can_admit(void) {\n ChassisAutoStatus status; uint32_t request, now=tick;\n" + start + "}\n"
    cfile, binary = OUT / "actual-turn-queue.c", OUT / "actual-turn-queue.exe"
    cfile.write_text(source + cases, encoding="ascii", errors="replace")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Werror", str(cfile), "-o", str(binary)],
                   check=True, timeout=30)
    output = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=30)
    log = OUT / "actual-turn-queue.log"
    log.write_text(output.stdout, encoding="ascii")
    print(output.stdout, end="", flush=True)
    return {"log": str(log), "sha256": digest(log)}


def verify_sources(manifest, project):
    for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
        if digest(project / name) != expected:
            raise ValueError(f"Source hash mismatch: {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    if "PASS turn queue VM ownership interleavings: 35 cases" not in MATLAB.read_text(encoding="utf-8"):
        raise RuntimeError("MATLAB prerequisite missing")
    manifest = json.loads(BASE.read_text(encoding="utf-8"))
    baseline = Path(manifest["project"]).resolve()
    verify_sources(manifest, baseline)
    for image in manifest["images"].values():
        if digest(Path(image["axf"])) != image["sha256"]:
            raise ValueError(f"Baseline image hash mismatch: {image['axf']}")
    project = OUT / "project"
    result_path = OUT / "stage3-turn-queue-compatible.json"
    if not project.exists():
        shutil.copytree(baseline, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
        patch(project)
        manifest["changes"][MAIN] = digest(project / MAIN)
        manifest["capabilities"]["stage3_turn_queue_vm_ownership"] = True
        manifest.update(project=str(project), images={}, build_passed=False, flashed=False,
                        baseline_manifest=str(BASE), baseline_manifest_sha256=digest(BASE),
                        matlab_sha256=digest(MATLAB))
        result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    else:
        manifest = json.loads(result_path.read_text(encoding="utf-8"))
        if manifest["baseline_manifest_sha256"] != digest(BASE):
            raise ValueError("Baseline manifest changed")
    verify_sources(manifest, project)
    manifest["c_fixture"] = fixture(baseline, project)
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
