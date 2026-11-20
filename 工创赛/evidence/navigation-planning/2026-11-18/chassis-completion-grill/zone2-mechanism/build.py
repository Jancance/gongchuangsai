"""Rebuild the two existing targets without connecting to the vehicle."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

stage = Path(__file__).resolve().parent
root = stage.parents[5]
source = root / '3.stm32f407工程/gongxun-f407.uvprojx'
build = stage / 'build'
build.mkdir(exist_ok=True)
settings = 'ICM45686_SAMPLE_ONLY=0,APP_CONTINUOUS_MAP_ENABLE=1,APP_MAP_NAV_RESIDUAL_ENABLE=1'
project = source.read_text('utf-8').replace('.\\', str(source.parent) + '\\')
project = project.replace('<Define>', '<Define>' + settings + ',')
files = ['CHASSIS/USER/main.c', 'CHASSIS/HARDWARE/RADAR_V6/radar_observer.c']
hashes = {name: hashlib.sha256((source.parent / name).read_bytes()).hexdigest() for name in files}
receipts = {}
for target in ('Vehicle_Integrated', 'Vehicle_Offline'):
    output = build / target
    output.mkdir(exist_ok=True)
    candidate = build / (target + '.uvprojx')
    candidate.write_text(re.sub(
        r'<(OutputDirectory|ListingPath)>.*?</\1>',
        lambda match: f'<{match[1]}>{target}\\</{match[1]}>', project), 'utf-8')
    log = build / (target + '.log')
    assert not log.exists(), 'Preserve original build logs before retrying'
    process = subprocess.run([
        'D:/Keil_v5/UV4/UV4.exe', '-r', str(candidate), '-t', target, '-o', str(log)
    ], timeout=300, creationflags=subprocess.CREATE_NO_WINDOW)
    contents = log.read_text('gbk', errors='replace')
    passed = (process.returncode == 0 and '0 Error(s), 0 Warning(s)' in contents
              and all('compiling ' + Path(name).name in contents for name in files))
    receipts[target] = dict(passed=passed, exit_code=process.returncode, settings=settings,
                            source_sha256=hashes)
    (build / 'receipts.json').write_text(json.dumps(receipts, indent=2), 'utf-8')
    print(target, 'PASS' if passed else contents[-2000:], flush=True)
    assert passed
assert all(hashlib.sha256((source.parent / name).read_bytes()).hexdigest() == digest
           for name, digest in hashes.items())
