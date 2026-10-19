from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess


def run(command: list[str]) -> dict[str, object]:
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    return {"command": command, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect UVC/V4L2 facts without guessing camera capabilities")
    parser.add_argument("devices", nargs="*", help="/dev/v4l/by-id paths")
    parser.add_argument("--output")
    args = parser.parse_args()
    devices = args.devices or [str(path) for path in Path("/dev/v4l/by-id").glob("*")]
    report = {"list_devices": run(["v4l2-ctl", "--list-devices"]), "usb_tree": run(["lsusb", "-t"]), "devices": {}}
    for device in devices:
        report["devices"][device] = {
            "all": run(["v4l2-ctl", "-d", device, "--all"]),
            "formats": run(["v4l2-ctl", "-d", device, "--list-formats-ext"]),
            "controls": run(["v4l2-ctl", "-d", device, "--list-ctrls-menus"]),
        }
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        target = Path(args.output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
