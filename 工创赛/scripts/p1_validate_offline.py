"""Allowlisted software checks only. No serial, reset, download or vehicle action."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from p1_motion_offline import ROOT, check_config, compare, export_analysis, read_json, safe_output, save_json, sha256


def protected_hashes(package=None):
    """Include all live project files (including binaries) and frozen package files."""
    paths = list((ROOT / "3.stm32f407工程").rglob("*"))
    paths.extend([ROOT / "工创赛/scripts/p1_serial_motion.py", ROOT / "工创赛/scripts/test_p1_serial_motion.py",
                  ROOT / "工创赛/docs/P1_SERIAL_MOTION_SCHEMA_2026-10-24.json"])
    if package:
        paths.extend(Path(package).rglob("*"))
    return {str(p.resolve()): sha256(p) for p in paths if p.is_file()}


def validate(out, package=None, manifest=None, simulate=False, arm_bin=None, unicorn_path=None):
    before = protected_hashes(package)
    started = time.monotonic()
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "hardware_accessed": False,
              "checks": [], "limits": "Software verification only; no physical speed, stopping distance, drift or smoothness acceptance."}
    def stage(name, action):
        print(f"Checking {name} ...", flush=True)
        try:
            result = action()
            ok = result.get("ok", True)
            report["checks"].append({"name": name, "status": "passed" if ok else "failed", "detail": result})
        except Exception as exc:
            report["checks"].append({"name": name, "status": "failed", "error": f"{type(exc).__name__}: {exc}"})
        save_json(out / "validation.json", report)
    def unit_test(pattern, filename):
        # Fixed command, shell disabled. No user-supplied executable or script list.
        cmd = [sys.executable, "-X", "utf8", "-B", "-m", "unittest", "discover", "-s", str(ROOT / "工创赛/scripts"), "-p", pattern]
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120,
                           env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
        (out / filename).write_text(p.stdout + p.stderr, encoding="utf-8")
        return {"ok": p.returncode == 0, "returncode": p.returncode, "log": filename}
    stage("offline_tool_tests", lambda: unit_test("test_p1_motion_offline.py", "offline_tests.log"))
    stage("route_preview_tests", lambda: unit_test("test_p1_route_preview.py", "route_tests.log"))
    stage("existing_host_tests", lambda: unit_test("test_p1_serial_motion.py", "host_tests.log"))
    def config():
        r = check_config(package=package); save_json(out / "config.json", r)
        return {"ok": r["ok"], "count": len(r["checks"]), "failed": [c for c in r["checks"] if not c["ok"]]}
    stage("configuration_and_package", config)
    if manifest:
        def historical():
            directory = out / "comparison"; directory.mkdir()
            rows = compare(manifest, directory)
            items = read_json(manifest)["runs"]
            for i, item in enumerate(items):
                directory = out / f"run_{i+1:02}"; directory.mkdir()
                export_analysis(Path(manifest).resolve().parent / item["log"], directory, item.get("block_index"))
            return {"ok": True, "runs": len(rows), "limits": "Existing failures remain failures in the reports; analysis success is not motion success."}
        stage("historical_log_reports", historical)
    else:
        report["checks"].append({"name": "historical_log_reports", "status": "skipped", "reason": "No --manifest supplied"})
    if simulate:
        def simulation():
            from p1_four_wheel_sim import run_sim
            directory = out / "four_wheel"; directory.mkdir()
            kwargs = {"unicorn_path": unicorn_path}
            if arm_bin:
                kwargs["arm_bin"] = arm_bin
            r = run_sim(directory, **kwargs)
            return {"ok": r["ok"], "assertions": r["assertions"], "cases": len(r["cases"]), "first_failure_line": r["first_failure_line"],
                    "trace_samples": r["trace_sample_count"], "trace_validation_ok": r["trace_validation_ok"], "charts": "four_wheel/index.html"}
        stage("actual_C_four_wheel_simulation", simulation)
    else:
        report["checks"].append({"name": "actual_C_four_wheel_simulation", "status": "skipped", "reason": "Use --simulate with ARMCC5 and Unicorn; no implicit tool installation"})
    after = protected_hashes(package)
    changed = [p for p in sorted(before.keys() | after.keys()) if before.get(p) != after.get(p)]
    report["checks"].append({"name": "protected_files_unchanged", "status": "passed" if not changed else "failed", "file_count": len(before), "changed": changed})
    report["elapsed_s"] = round(time.monotonic() - started, 3)
    states = [c["status"] for c in report["checks"]]
    report["status"] = "failed" if "failed" in states else "passed_with_skips" if "skipped" in states else "passed"
    save_json(out / "validation.json", report)
    lines = ["# P1 offline validation", "", f"Status: {report['status']}", "", "No serial/reset/download/drive operation. Software evidence only.", ""]
    lines.extend(f"- {c['name']}: {c['status']}" for c in report["checks"])
    (out / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Offline validation: {report['status']} ({report['elapsed_s']}s)", flush=True)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--package", type=Path)
    p.add_argument("--manifest", type=Path)
    p.add_argument("--simulate", action="store_true")
    p.add_argument("--arm-bin", type=Path)
    p.add_argument("--unicorn-path", type=Path)
    args = p.parse_args()
    report = validate(safe_output(args.out), args.package, args.manifest, args.simulate, args.arm_bin, args.unicorn_path)
    return 1 if report["status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
