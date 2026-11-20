"""One run followed immediately by its authorized read, with no retries."""
from pathlib import Path
import json
import subprocess
import sys

E = Path(__file__).resolve().parent
from run_binding import validate
validate()
with (E / 'run_and_read_started.json').open('x', encoding='utf8') as stream:
    json.dump(dict(attempts=1, automatic_retry=False, read_before_operator_reset=True), stream)
process = subprocess.run([sys.executable, str(E / 'run_once.py')], timeout=90)
if process.returncode:
    raise SystemExit(process.returncode)
process = subprocess.run([sys.executable, str(E / 'read_stop_distance_once.py')], timeout=30)
raise SystemExit(process.returncode)
