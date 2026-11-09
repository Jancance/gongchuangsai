"""仅将28试走步长16改160脉冲；不改速度、超时或其他模块，不烧录。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/extension_jog160_20261030/project'
EV = ROOT/'evidence/p2-arm/2026-10-30/extension-jog160'
CONFIG = 'HARDWARE/EXTENSION/extension_config.h'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def save(name, data):
    (EV/name).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def verify(folder, m):
    for name, digest in m['protected'].items():
        assert sha(folder/name) == digest, name


def build(folder, log):
    p = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'),
                        '-j0', '-o', str(log)], capture_output=True, timeout=120)
    output = log.read_text(encoding='utf-8', errors='replace')
    assert p.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', output), output[-5000:]
    print(output[-550:], flush=True)


if '--build' in sys.argv or '--install' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert sha(STAGE/CONFIG) == m['files'][0]['after']
    verify(STAGE, m)
    if '--build' in sys.argv:
        r = json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
        assert r['passed'] and r['expected_pulses'] == 160
        assert r['source_sha256'][CONFIG] == m['files'][0]['after']
        m['host_pass'] = True
        build(STAGE, EV/'stage-build.log')
        verify(STAGE, m)
        m['stage_build_pass'] = True
        save('manifest.json', m)
    else:
        assert m['host_pass'] and m['stage_build_pass']
        verify(ACTUAL, m)
        assert sha(ACTUAL/CONFIG) == m['files'][0]['before']
        shutil.copy2(STAGE/CONFIG, ACTUAL/CONFIG)
        assert sha(ACTUAL/CONFIG) == m['files'][0]['after']
        m.update(installed=True, actual_build_pass=False)
        save('installed.json', m)
        build(ACTUAL, EV/'actual-build.log')
        verify(ACTUAL, m)
        assert sha(ACTUAL/CONFIG) == m['files'][0]['after']
        m['actual_build_pass'] = True
        save('installed.json', m)
        print('160-pulse candidate installed; no flash or hardware commands.', flush=True)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
backup = EV/'original'/CONFIG
backup.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(ACTUAL/CONFIG, backup)
data = (STAGE/CONFIG).read_bytes()
old, new = b'#define EXTENSION_JOG_PULSES        16U', b'#define EXTENSION_JOG_PULSES        160U'
assert data.count(old) == 1
(STAGE/CONFIG).write_bytes(data.replace(old, new, 1))
protected = {p.relative_to(ACTUAL).as_posix(): sha(p) for p in ACTUAL.rglob('*')
             if p.is_file() and p.suffix.lower() in ('.c', '.h', '.uvprojx')
             and p.relative_to(ACTUAL).as_posix() != CONFIG}
m = dict(files=[dict(file=CONFIG, before=sha(ACTUAL/CONFIG), after=sha(STAGE/CONFIG))],
         protected=protected, user_text='好像是缩回，步长太小了给我拉大点',
         direction_report='tentative retract; not physically confirmed',
         candidate=dict(node=6, pulses=160, rpm=5, acceleration=20, timeout_ms=3000),
         host_pass=False, stage_build_pass=False, installed=False,
         flash_performed=False, hardware_commands_sent=False)
save('manifest.json', m)
print('Prepared one-line 16-to-160 pulse change; all other current source/project files protected.', flush=True)
