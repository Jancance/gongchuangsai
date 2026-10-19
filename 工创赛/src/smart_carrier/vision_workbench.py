"""Project entry to the locally adapted multi-function vision application."""
from pathlib import Path
import runpy
import sys

def main() -> int:
    root=Path(__file__).resolve().parents[2]
    entry=root/'vision_app/run.py'
    if not entry.is_file():
        print('Integrated vision_app is missing; run from the source project checkout.')
        return 2
    sys.argv=[str(entry),*sys.argv[1:]]
    try:
        runpy.run_path(str(entry),run_name='__main__')
    except SystemExit as exc:
        return int(exc.code or 0) if isinstance(exc.code,(int,type(None))) else 1
    return 0

if __name__=='__main__':raise SystemExit(main())
