"""Match user's 40-second driver setting; no flash/driver write/motor commands."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/lift_home_40s_20261101/project'
EV = ROOT / 'evidence/p2-arm/2026-11-01/lift-home-40s'
NAME = 'HARDWARE/LIFT/lift_home.h'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def save(m):
    (EV / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')

def build(folder, name):
    log = EV / name
    r = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder / 'gongxun-f407.uvprojx'), '-j0', '-o', str(log)], capture_output=True, timeout=180)
    output = log.read_text(encoding='utf-8', errors='replace')
    assert r.returncode == 0 and '0 Error(s), 0 Warning(s)' in output, output[-3000:]
    print(output[-500:], flush=True)

if '--install' in sys.argv:
    m = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    assert m['host_pass'] and m['stage_build_pass']
    assert sha(ACTUAL / NAME) == m['before'] and sha(STAGE / NAME) == m['after']
    for name, digest in m['protected'].items():
        assert sha(ACTUAL / name) == digest, name
    shutil.copy2(STAGE / NAME, ACTUAL / NAME)
    m['installed'] = True
    save(m)
    build(ACTUAL, 'actual-build.log')
    assert sha(ACTUAL / NAME) == m['after']
    for name, digest in m['protected'].items():
        assert sha(ACTUAL / name) == digest, name
    m['actual_build_pass'] = True
    save(m)
    print('Installed one header only. No flash, driver write or hardware motion.', flush=True)
    sys.exit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '*.uvguix.*', '.git'))
m = {'before': sha(ACTUAL / NAME), 'host_pass': False, 'stage_build_pass': False,
     'installed': False, 'actual_build_pass': False, 'flash_performed': False,
     'driver_write_performed': False, 'hardware_motion_performed': False,
     'physical_acceptance': False, 'driver_readback_40000_confirmed': False,
     'expected_rpm': 60, 'expected_driver_timeout_ms': 40000,
     'host_started_wait': 'unlimited',
     'protected': {str(p.relative_to(ACTUAL)).replace('\\', '/'): sha(p) for p in ACTUAL.rglob('*')
                   if p.is_file() and p.suffix.lower() in ['.c', '.h', '.uvprojx']
                   and str(p.relative_to(ACTUAL)).replace('\\', '/') != NAME}}
for group, folder in [('original', ACTUAL), ('final', STAGE)]:
    dest = EV / group / NAME
    dest.parent.mkdir(parents=True, exist_ok=True)
    if group == 'original':
        shutil.copy2(folder / NAME, dest)
p = STAGE / NAME
t = p.read_text(encoding='utf-8')
assert t.count('#define LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS 180000U') == 1
t = t.replace('#define LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS 180000U', '#define LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS 40000U')
t = t.replace('Manual menu maximum, finite; zero-as-infinite is not documented.',
              'User-selected 40-second driver limit; finite, separate from unlimited host wait.')
p.write_bytes(t.replace('\n', '\r\n').encode('utf-8'))
shutil.copy2(p, EV / 'final' / NAME)
m['after'] = sha(p)
save(m)
r = subprocess.run([sys.executable, str(ROOT / 'scripts/validate_lift_home_wait_speed_20261101.py'),
                    '--project', str(STAGE), '--host', str(STAGE.parent / 'host')], capture_output=True, text=True)
(EV / 'host-check.log').write_text(r.stdout + r.stderr, encoding='utf-8')
assert r.returncode == 0, r.stdout + r.stderr
print(r.stdout, flush=True)
m['host_pass'] = True
save(m)
build(STAGE, 'stage-build.log')
m['stage_build_pass'] = True
save(m)
print('Prepared 40-second matching; actual source not installed yet.', flush=True)
