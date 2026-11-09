"""Install the reviewed one-file scan configuration and rebuild actual targets."""
from pathlib import Path
import hashlib
import json
import subprocess

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-03/scan-display'
m = json.loads((E / 'manifest.json').read_text(encoding='utf-8'))
A, S = Path(m['actual']), Path(m['stage'])
rel = 'USER/vehicle_parameters.h'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
for target in ['Vehicle_Integrated', 'Vehicle_Offline']:
    assert '0 Error(s), 0 Warning(s)' in (E / f'stage-{target}.log').read_text(encoding='utf-8', errors='replace')
assert sha(A / rel) == m['before_sha256'], 'Actual parameters changed: re-review required'
assert sha(S / rel) == m['after_sha256']
# No unrelated source drift between preparation and installation.
for p in S.rglob('*'):
    if p.is_file() and p.suffix.lower() in {'.c', '.h', '.s', '.inc', '.uvprojx'}:
        name = p.relative_to(S)
        if str(name).replace('\\', '/') != rel:
            assert sha(A / name) == sha(p), str(name)
(A / rel).write_bytes((S / rel).read_bytes())
m['installed'] = True
(E / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
for target in ['Vehicle_Integrated', 'Vehicle_Offline']:
    log = E / f'actual-{target}.log'
    p = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(A / 'gongxun-f407.uvprojx'),
                        '-t', target, '-j0', '-o', str(log)], timeout=180)
    content = log.read_text(encoding='utf-8', errors='replace')
    assert p.returncode == 0 and '0 Error(s), 0 Warning(s)' in content, content[-1500:]
    print(target, '0 errors, 0 warnings', flush=True)
m['actual_builds_passed'] = True
(E / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
