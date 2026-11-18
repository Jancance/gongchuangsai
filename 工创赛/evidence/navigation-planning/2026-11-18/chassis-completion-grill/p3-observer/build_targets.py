"""Rebuild the working-tree Keil targets in an isolated directory."""
import argparse
import hashlib
import io
import json
import re
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("output", type=Path)
    args.add_argument("--define", action="append", default=[])
    args.add_argument("--require-compiled", action="append", default=[])
    options = args.parse_args()
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[6]
    project_folder = "3.stm32f407工程"
    project_name = "gongxun-f407.uvprojx"
    keil = Path("D:/Keil_v5/UV4/UV4.exe")
    base = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    changes = subprocess.check_output(
        ["git", "diff", "--name-only", "-z", base, "--", project_folder], cwd=root
    ).decode("utf-8").strip("\0").split("\0")
    added = subprocess.check_output(
        ["git", "ls-files", "--others", "--exclude-standard", "-z", "--", project_folder],
        cwd=root,
    ).decode("utf-8").strip("\0").split("\0")
    inputs = sorted({name for name in changes + added if Path(name).suffix in {
        ".c", ".h", ".s", ".inc", ".lib", ".sct", ".uvprojx",
    }} | {f"{project_folder}/{project_name}"})
    hashes = {}
    receipts = {}
    with tempfile.TemporaryDirectory(prefix="gongxun-zone2-") as temp:
        archive = subprocess.check_output(
            ["git", "archive", "--format=zip", base, "--", project_folder], cwd=root
        )
        with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
            zipped.extractall(temp)
        for name in inputs:
            source = root / name
            target = Path(temp) / name
            if not source.is_file():
                target.unlink(missing_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            hashes[name] = hashlib.sha256(target.read_bytes()).hexdigest()
        snapshot = Path(temp) / project_folder
        project = (snapshot / project_name).read_text(encoding="utf-8")
        if options.define:
            project = project.replace(
                "<Define>STM32F40_41xxx,",
                "<Define>" + ",".join(options.define) + ",STM32F40_41xxx,",
            )
        project = project.replace(".\\", str(snapshot) + "\\")
        for name in ("Vehicle_Integrated", "Vehicle_Offline"):
            directory = output / name
            directory.mkdir()
            project_copy = directory / project_name
            project_copy.write_text(re.sub(
                r"<(OutputDirectory|ListingPath)>.*?</\1>",
                r"<\1>build\\</\1>",
                project,
            ), encoding="utf-8")
            log = directory / "rebuild.log"
            result = subprocess.run(
                [str(keil), "-r", str(project_copy), "-t", name, "-o", str(log)],
                timeout=300, creationflags=subprocess.CREATE_NO_WINDOW,
            )
            contents = log.read_text(encoding="gbk", errors="replace")
            passed = result.returncode == 0 and "0 Error(s), 0 Warning(s)" in contents
            compiled = {source: f"compiling {source}" in contents
                        for source in options.require_compiled}
            receipts[name] = {
                "passed": passed and all(compiled.values()),
                "exit_code": result.returncode,
                "compiled": compiled,
            }
            print(name, receipts[name], flush=True)
            if not receipts[name]["passed"]:
                print(contents[-3000:], flush=True)
    assert all(hashlib.sha256((root / name).read_bytes()).hexdigest() == digest
               for name, digest in hashes.items()), "Working-tree build inputs changed"
    (output / "receipts.json").write_text(json.dumps(
        {"base": base, "defines": options.define, "inputs_sha256": hashes, "targets": receipts},
        ensure_ascii=False, indent=2,
    ) + "\n", encoding="utf-8")
    return 0 if all(receipt["passed"] for receipt in receipts.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
