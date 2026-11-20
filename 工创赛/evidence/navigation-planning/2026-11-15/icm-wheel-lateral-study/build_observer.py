from pathlib import Path
import hashlib
import json
import re
import subprocess

ROOT = Path.cwd()
PROJECT = ROOT / "3.stm32f407工程" / "gongxun-f407.uvprojx"
EVIDENCE = ROOT / "工创赛/evidence/navigation-planning/2026-11-15/icm-wheel-lateral-study"
BUILD_ROOT = ROOT / "工创赛/tmp/icm-observer-build-20261115"
UV4 = Path("D:/Keil_v5/UV4/UV4.exe")
SOURCES = [
    "app_odometry.c", "app_odometry.h", "chassis_kinematics.c",
    "chassis_kinematics.h", "chassis_auto_test.c", "app_motion_runtime.c",
    "app_serial_runtime.c", "main.c", "app_icm_observer.c", "app_icm_observer.h",
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    project_sha = sha(PROJECT)
    source_sha = {name: sha(PROJECT.parent / "CHASSIS/USER" / name) for name in SOURCES}
    report = {"project_sha256": project_sha, "source_sha256": source_sha, "builds": {}}
    configs = [
        ("baseline40_integrated", "Vehicle_Integrated", "APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0"),
        ("baseline40_offline", "Vehicle_Offline", "APP_BASELINE40_ENABLE=1,VEHICLE_INTEGRATED_ENABLE=0,VEHICLE_STAGE1_ENABLE=0"),
        ("baseline_off_integrated", "Vehicle_Integrated", ""),
    ]
    for name, target, defines in configs:
        output = BUILD_ROOT / name
        output.mkdir(exist_ok=True)
        text = PROJECT.read_text(encoding="utf-8").replace(".\\", str(PROJECT.parent) + "\\")
        for tag in ("OutputDirectory", "ListingPath"):
            text = re.sub(f"<{tag}>.*?</{tag}>", f"<{tag}>{name}\\</{tag}>", text)
        if defines:
            text = text.replace("<Define>", f"<Define>{defines},")
        project_copy = BUILD_ROOT / f"{name}.uvprojx"
        project_copy.write_text(text, encoding="utf-8")
        log = EVIDENCE / f"{name}.log"
        command = [str(UV4), "-r", str(project_copy), "-t", target, "-o", str(log)]
        process = subprocess.run(command, timeout=300, creationflags=subprocess.CREATE_NO_WINDOW)
        log_text = log.read_bytes().decode("cp936", errors="replace")
        entry = {
            "returncode": process.returncode,
            "target": target,
            "defines": defines,
            "project_copy_sha256": sha(project_copy),
            "log_sha256": sha(log),
            "command": command,
        }
        report["builds"][name] = entry
        (EVIDENCE / "build_summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        if process.returncode != 0 or "0 Error(s), 0 Warning(s)" not in log_text:
            raise RuntimeError(f"{name} build failed; inspect {log}")
        axf = output / f"{target}.axf"
        entry["program_size"] = re.search(r"Program Size:.*", log_text)[0]
        entry["axf"] = str(axf)
        entry["axf_sha256"] = sha(axf)
        (EVIDENCE / "build_summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(name, entry["program_size"], entry["axf_sha256"], flush=True)
    if sha(PROJECT) != project_sha:
        raise RuntimeError("daily uvprojx changed during isolated build")
    if any(sha(PROJECT.parent / "CHASSIS/USER" / name) != digest for name, digest in source_sha.items()):
        raise RuntimeError("source changed during isolated build")


if __name__ == "__main__":
    main()
