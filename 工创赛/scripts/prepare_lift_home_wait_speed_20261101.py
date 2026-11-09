"""Prepare/validate lift-only home wait/speed change. Never flash or move hardware."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/lift_home_wait_speed_20261101/project'
EV = ROOT / 'evidence/p2-arm/2026-11-01/lift-home-wait-speed'
FILES = ['HARDWARE/LIFT/lift_home.c', 'HARDWARE/LIFT/lift_home.h']

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def save(m):
    (EV / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')

def build(folder, name):
    log = EV / name
    result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder / 'gongxun-f407.uvprojx'), '-j0', '-o', str(log)], capture_output=True, timeout=180)
    output = log.read_text(encoding='utf-8', errors='replace')
    assert result.returncode == 0 and '0 Error(s), 0 Warning(s)' in output, output[-3000:]
    print(output[-650:], flush=True)

if '--install' in sys.argv:
    m = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    assert m['host_pass'] and m['stage_build_pass']
    for name in FILES:
        assert sha(ACTUAL / name) == m['before'][name], name
        assert sha(STAGE / name) == m['after'][name], name
    for name, digest in m['protected'].items():
        assert sha(ACTUAL / name) == digest, name
    for name in FILES:
        shutil.copy2(STAGE / name, ACTUAL / name)
    m['installed'] = True
    save(m)
    build(ACTUAL, 'actual-build.log')
    for name, digest in m['protected'].items():
        assert sha(ACTUAL / name) == digest, name
    for name in FILES:
        assert sha(ACTUAL / name) == m['after'][name], name
    m['actual_build_pass'] = True
    save(m)
    print('Installed two source files; no flash, driver write or hardware motion.', flush=True)
    sys.exit(0)

assert not STAGE.exists(), 'Use existing evidence rather than overwriting a prepared result.'
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '*.uvguix.*', '.git'))
m = {'before': {f: sha(ACTUAL / f) for f in FILES},
     'protected': {str(p.relative_to(ACTUAL)).replace('\\', '/'): sha(p) for p in ACTUAL.rglob('*')
                   if p.is_file() and p.suffix.lower() in ['.c', '.h', '.uvprojx']
                   and str(p.relative_to(ACTUAL)).replace('\\', '/') not in FILES},
     'host_pass': False, 'stage_build_pass': False, 'installed': False,
     'actual_build_pass': False, 'flash_performed': False, 'driver_write_performed': False,
     'physical_acceptance': False, 'host_active_wait': 'unlimited',
     'driver_expected_rpm': 60, 'driver_expected_timeout_ms': 180000,
     'driver_infinite_semantics': 'not documented; 180000 ms is finite',
     'latest_failure_evidence': 'User reports no top contact, gap 8 mm; screenshot home 8/1/1, not host timeout.'}
for name in FILES:
    dest = EV / 'original' / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ACTUAL / name, dest)

p = STAGE / FILES[0]
t = p.read_text(encoding='utf-8')
old = 'if ((TickType_t)(now - s_started) >= pdMS_TO_TICKS(s_sent ? 11000U : 2500U))'
assert t.count(old) == 1
t = t.replace(old, '/* Active homing has no host duration limit; preparation still has its original limit. */\n    if (!s_sent && (TickType_t)(now - s_started) >= pdMS_TO_TICKS(2500U))')
t = t.replace('忙时正向请求不排队，不重置超时；', '忙时正向请求不排队；')
assert t.count('g_lift_home_params.rpm != 30U') == 1
assert t.count('g_lift_home_params.timeout_ms != 10000U') == 1
t = t.replace('g_lift_home_params.rpm != 30U', 'g_lift_home_params.rpm != LIFT_HOME_EXPECTED_RPM')
t = t.replace('g_lift_home_params.timeout_ms != 10000U', 'g_lift_home_params.timeout_ms != LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS')
p.write_bytes(t.replace('\n', '\r\n').encode('utf-8'))
p = STAGE / FILES[1]
t = p.read_text(encoding='utf-8')
marker = '#include "stm32f4xx.h"'
assert t.count(marker) == 1
t = t.replace(marker, marker + '\n/* Driver menu must match; host does not write parameters or auto-start. */\n#define LIFT_HOME_EXPECTED_RPM 60U\n/* Manual menu maximum, finite; zero-as-infinite is not documented. */\n#define LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS 180000U\n/* After start: no host total timeout. Stop, driver faults and CAN errors remain active. */')
t = t.replace('3超时，', '3启动准备超时，')
p.write_bytes(t.replace('\n', '\r\n').encode('utf-8'))
m['after'] = {f: sha(STAGE / f) for f in FILES}
for name in FILES:
    dest = EV / 'final' / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(STAGE / name, dest)
save(m)
result = subprocess.run([sys.executable, str(ROOT / 'scripts/validate_lift_home_wait_speed_20261101.py')], capture_output=True, text=True)
(EV / 'host-check.log').write_text(result.stdout + result.stderr, encoding='utf-8')
assert result.returncode == 0, result.stdout + result.stderr
print(result.stdout, flush=True)
m['host_pass'] = True
save(m)
build(STAGE, 'stage-build.log')
m['stage_build_pass'] = True
save(m)
print('Prepared and validated. No actual source installation yet.', flush=True)
