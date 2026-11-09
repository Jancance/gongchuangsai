"""Stage a bounded adjustable upward jog; install only after native/build checks."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/lift_up_step_20261028/project'
EV = ROOT / 'evidence/p2-arm/2026-10-28/lift-up-step'
FILES = ['HARDWARE/LIFT/lift_config.h', 'HARDWARE/LIFT/lift.h',
         'HARDWARE/LIFT/lift.c', 'HARDWARE/LIFT/lift_protocol.c', 'SYSTEM/CAN/can.c']

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def replace_once(path, old, new):
    raw = path.read_bytes()
    # Preserve existing source encodings and newline styles.
    old, new = old.encode('utf-8'), new.encode('utf-8')
    if b'\r\n' in raw:
        old, new = old.replace(b'\n', b'\r\n'), new.replace(b'\n', b'\r\n')
    if raw.count(old) != 1:
        raise RuntimeError(f'Expected one match: {path}: {old!r}')
    path.write_bytes(raw.replace(old, new, 1))

def prepare():
    if STAGE.exists():
        raise RuntimeError('Stage already exists; inspect it instead of overwriting')
    EV.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ACTUAL, STAGE)
    before = {}
    for name in FILES:
        before[name] = sha(ACTUAL / name)
        dest = EV / 'original' / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ACTUAL / name, dest)
    replace_once(STAGE / FILES[0], '#define LIFT_JOG_PULSES       5000U',
                 '#define LIFT_JOG_PULSES       5000U\n'
                 '/* 向上单次步长上限；默认仍为5000，需Watch显式调整。 */\n'
                 '#define LIFT_UP_MAX_PULSES    50000U')
    replace_once(STAGE / FILES[1], 'extern volatile uint32_t g_lift_motion_request;',
                 'extern volatile uint32_t g_lift_motion_request;\n'
                 '/* 仅向上请求2使用：1~50000，接单时锁存，默认5000。 */\n'
                 'extern volatile uint32_t g_lift_up_pulses;')
    replace_once(STAGE / FILES[2], 'volatile uint32_t g_lift_motion_request = 0U;',
                 'volatile uint32_t g_lift_motion_request = 0U;\n'
                 'volatile uint32_t g_lift_up_pulses = LIFT_JOG_PULSES;\n'
                 'static uint32_t s_motion_pulses = LIFT_JOG_PULSES;')
    replace_once(STAGE / FILES[2], '        s_direction = (request == 1U) ? 0U : 1U;',
                 '        /* 只在新请求接单时读取步长，运动中改Watch不改变本次目标。 */\n'
                 '        s_motion_pulses = (request == 2U) ? g_lift_up_pulses : LIFT_JOG_PULSES;\n'
                 '        if (s_motion_pulses == 0U ||\n'
                 '            s_motion_pulses > ((request == 2U) ? LIFT_UP_MAX_PULSES : LIFT_JOG_PULSES))\n'
                 '        { g_lift_motion_result = 6U; g_lift_motion_state = 9U; return; }\n'
                 '        s_direction = (request == 1U) ? 0U : 1U;')
    replace_once(STAGE / FILES[2], 'LiftProtocol_Move(s_direction,LIFT_JOG_PULSES,command)',
                 'LiftProtocol_Move(s_direction,s_motion_pulses,command)')
    replace_once(STAGE / FILES[3], 'pulses > LIFT_JOG_PULSES ||',
                 'pulses > (direction == 1U ? LIFT_UP_MAX_PULSES : LIFT_JOG_PULSES) ||')
    replace_once(STAGE / FILES[4], 'if (pulses == 0U || pulses > 5000U) return 7U;',
                 'if (pulses == 0U || pulses > (request[1] == 1U ? 50000U : 5000U)) return 7U;')
    manifest = {'actual': str(ACTUAL), 'stage': str(STAGE), 'files': [
        {'file': name, 'before_sha256': before[name], 'after_sha256': sha(STAGE/name)}
        for name in FILES], 'flash_performed': False, 'hardware_motion_executed': False}
    (EV/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Prepared five files; default upward 5000, limit 50000, downward 5000.')

def install():
    manifest = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    checks = json.loads((EV/'checks.json').read_text(encoding='utf-8'))
    assert checks['native_pass'] and checks['stage_build_pass']
    for entry in manifest['files']:
        assert sha(ACTUAL/entry['file']) == entry['before_sha256'], entry['file']
        assert sha(STAGE/entry['file']) == entry['after_sha256'], entry['file']
    for entry in manifest['files']:
        shutil.copy2(STAGE/entry['file'], ACTUAL/entry['file'])
    for entry in manifest['files']:
        assert sha(ACTUAL/entry['file']) == entry['after_sha256']
    log = EV/'actual-build.log'
    result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(ACTUAL/'gongxun-f407.uvprojx'),
                             '-j0', '-o', str(log)], capture_output=True)
    message = log.read_text(encoding='utf-8', errors='replace')
    assert result.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', message), message
    manifest.update(installed=True, actual_build_pass=True)
    (EV/'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(message[-650:])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    install() if args.install else prepare()
