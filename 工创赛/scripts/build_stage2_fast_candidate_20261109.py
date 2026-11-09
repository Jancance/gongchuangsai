"""Build/check manifest for the isolated candidate, with no hardware calls."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tmp/stage2_sub20_20261109'
P = OUT / 'project'
E = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
manifest_path = E / 'stage2-fast-candidate-20261109.json'
m = json.loads(manifest_path.read_text(encoding='utf-8'))
for name, digest in m['changed'].items():
    assert hashlib.sha256((P / 'CHASSIS/USER' / name).read_bytes()).hexdigest() == digest, name
for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
    for folder in ('Objects', 'Listings'):
        (P / folder / target).mkdir(parents=True, exist_ok=True)
    log = OUT / (target + '-build.log')
    result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(P / 'gongxun-f407.uvprojx'),
                             '-t', target, '-j0', '-o', str(log)], timeout=180)
    data = log.read_bytes()
    print(data.decode('gb18030', errors='replace')[-3500:], flush=True)
    assert result.returncode == 0 and b'0 Error(s), 0 Warning(s)' in data, str(log)
    image = P / 'Objects' / target / (target+'.axf')
    m['images'][target] = {'axf': image.relative_to(ROOT).as_posix(),
        'sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
        'build_log': log.relative_to(ROOT).as_posix()}
manifest_path.write_text(json.dumps(m, indent=2)+'\n')
