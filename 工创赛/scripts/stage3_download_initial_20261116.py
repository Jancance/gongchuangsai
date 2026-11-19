"""Read one already captured PNG over the existing authorized SSH helper."""
import argparse
import base64
import hashlib
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
REMOTE = '/home/orangepi/vision_tools/stage3_ring_pi_20261116/records/initial/1794776769343850042-raw.png'
EXPECTED = '8bd6fc08d1655e9391dc5ab38e7ac8dc2569751e7bc31a21b9de7ec56fcabd37'
OUTPUT = ROOT/'evidence/integration/2026-11-16/stage3/field/initial.png'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--remote', default=REMOTE)
parser.add_argument('--sha256', default=EXPECTED)
parser.add_argument('--output', type=Path, default=OUTPUT)
args = parser.parse_args()
REMOTE, EXPECTED, OUTPUT = args.remote, args.sha256, args.output.resolve()
BEGIN = 'STAGE3_IMAGE_BASE64_BEGIN'
END = 'STAGE3_IMAGE_BASE64_END'
source = ('from pathlib import Path;import base64;print(' + repr(BEGIN)
          + ');print(base64.b64encode(Path(' + repr(REMOTE)
          + ').read_bytes()).decode("ascii"));print(' + repr(END) + ')')
result = subprocess.run([sys.executable, str(ROOT/'tmp/integration22_ssh.py'),
                         'command', 'python3 -c '+shlex.quote(source)],
                        cwd=ROOT, capture_output=True)
if result.returncode:
    raise RuntimeError('Read-only download helper failed, exit='+str(result.returncode))
lines = result.stdout.decode('utf-8', errors='replace').splitlines()
start = next(i for i,line in enumerate(lines) if line.strip() == BEGIN)
finish = next(i for i,line in enumerate(lines[start+1:], start+1) if line.strip() == END)
data = base64.b64decode(''.join(lines[start+1:finish]), validate=True)
assert data.startswith(b'\x89PNG\r\n\x1a\n'), 'Not PNG'
assert hashlib.sha256(data).hexdigest() == EXPECTED, 'Image SHA256 mismatch'
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_bytes(data)
print(str(OUTPUT))
print('SHA256 '+EXPECTED+' bytes='+str(len(data)))
