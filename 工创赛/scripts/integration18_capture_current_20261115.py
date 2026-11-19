"""Capture 15 current frames on the Pi with serial disabled and no motion."""
import argparse
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    args = parser.parse_args()
    code = Path("/home/orangepi/p2-source-observer-20261106/vision_app/code")
    sys.path.insert(0, str(code))
    import main as app
    from project_camera import DisabledSerialLink
    app.TurntableSerialLink = app.DisabledSerialLink = DisabledSerialLink
    import source_observe
    sys.argv = [str(code / "source_observe.py"), "--mode", "A", "--frames", "15",
                "--duration", "20", "--session-dir", str(args.session_dir)]
    return source_observe.main()


if __name__ == "__main__":
    raise SystemExit(main())
