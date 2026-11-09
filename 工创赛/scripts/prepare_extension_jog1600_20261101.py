"""28伸缩毫米标定试走候选：只改步长和超时，不烧录或发送硬件命令。"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/extension_jog1600_20261101/project'
EV = ROOT / 'evidence/p2-arm/2026-11-01/extension-jog1600'
ASSETS = ROOT / 'output/extension_jog1600_20261101'
CONFIG = 'HARDWARE/EXTENSION/extension_config.h'
KEIL = 'C:/Keil_v5/UV4/UV4.exe'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, data):
    (EV / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def verify(folder, manifest):
    for name, expected in manifest['protected'].items():
        if sha(folder / name) != expected:
            raise RuntimeError('Protected source changed: ' + name)


def build(folder, log):
    proc = subprocess.run([KEIL, '-b', str(folder / 'gongxun-f407.uvprojx'),
                           '-j0', '-o', str(log)], capture_output=True, timeout=180)
    output = log.read_text(encoding='utf-8', errors='replace')
    if proc.returncode != 0 or not re.search(r'0 Error\(s\), 0 Warning\(s\)', output):
        raise RuntimeError(output[-5000:])
    print(output[-650:], flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'build-stage', 'install'])
    args = parser.parse_args()
    EV.mkdir(parents=True, exist_ok=True)
    ASSETS.mkdir(parents=True, exist_ok=True)
    if args.action == 'prepare':
        if STAGE.exists() or (EV / 'manifest.json').exists():
            raise RuntimeError('Candidate exists; refusing to overwrite evidence')
        shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns(
            'Objects', 'Listings', '.git', '.vscode', '*.uvguix.*'))
        backup = EV / 'original' / CONFIG
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ACTUAL / CONFIG, backup)
        data = (STAGE / CONFIG).read_bytes()
        for old, new in [
            (b'#define EXTENSION_JOG_PULSES        160U', b'#define EXTENSION_JOG_PULSES        1600U'),
            (b'#define EXTENSION_MOVE_TIMEOUT_MS   3000U', b'#define EXTENSION_MOVE_TIMEOUT_MS   10000U'),
        ]:
            if data.count(old) != 1:
                raise RuntimeError('Unexpected current configuration: ' + repr(old))
            data = data.replace(old, new, 1)
        (STAGE / CONFIG).write_bytes(data)
        shutil.copy2(STAGE / CONFIG, ASSETS / 'extension_config.h')
        protected = {p.relative_to(ACTUAL).as_posix(): sha(p)
                     for p in ACTUAL.rglob('*')
                     if p.is_file() and p.suffix.lower() in ('.c', '.h', '.uvprojx')
                     and p.relative_to(ACTUAL).as_posix() != CONFIG}
        manifest = {
            'date': '2026-11-01',
            'actual_project': str(ACTUAL / 'gongxun-f407.uvprojx'),
            'files': [{'file': CONFIG, 'before': sha(ACTUAL / CONFIG), 'after': sha(STAGE / CONFIG)}],
            'protected': protected,
            'user_authorization': ['160脉冲太小了压根就不行，给我改大点，之前是怎么记录的？',
                                   '现在给我标定毫米行程，之前为什么不让我标定毫米行程'],
            'candidate': {'node': 6, 'pulses': 1600, 'rpm': 5, 'acceleration': 20,
                          'timeout_ms': 10000, 'direction1': 'retract', 'direction2': 'extend'},
            'previous': {'pulses': 160, 'timeout_ms': 3000},
            'physical_input': {'reported_retract_clearance_mm': 30, 'total_travel_mm': None,
                               'pulses_per_mm': None, 'home_valid': False},
            'scope': 'Supervised finite relative-step calibration only; no firmware metric mapping or formal motion permission.',
            'host_pass': False, 'stage_build_pass': False, 'installed': False,
            'flash_performed': False, 'hardware_commands_sent': False,
        }
        verify(STAGE, manifest)
        save('manifest.json', manifest)
        print(json.dumps({'prepared': True, 'protected_files': len(protected),
                          'changed_file': CONFIG, 'candidate': manifest['candidate']}, ensure_ascii=False))
        return

    manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE, manifest)
    if sha(STAGE / CONFIG) != manifest['files'][0]['after']:
        raise RuntimeError('Candidate hash mismatch')
    review = json.loads((EV / 'host-review.json').read_text(encoding='utf-8'))
    if not review['passed'] or review['config_sha256'] != sha(STAGE / CONFIG):
        raise RuntimeError('Matching host validation is required')
    if args.action == 'build-stage':
        build(STAGE, EV / 'stage-build.log')
        verify(STAGE, manifest)
        manifest.update(host_pass=True, stage_build_pass=True)
        save('manifest.json', manifest)
        return

    if not manifest['host_pass'] or not manifest['stage_build_pass']:
        raise RuntimeError('Stage checks must pass before install')
    verify(ACTUAL, manifest)
    if sha(ACTUAL / CONFIG) != manifest['files'][0]['before']:
        raise RuntimeError('Actual configuration changed since preparation')
    shutil.copy2(STAGE / CONFIG, ACTUAL / CONFIG)
    manifest.update(installed=True, actual_build_pass=False)
    save('installed.json', manifest)
    build(ACTUAL, EV / 'actual-build.log')
    verify(ACTUAL, manifest)
    if sha(ACTUAL / CONFIG) != manifest['files'][0]['after']:
        raise RuntimeError('Installed configuration hash mismatch')
    manifest['actual_build_pass'] = True
    save('installed.json', manifest)
    print(json.dumps({'installed': True, 'actual_build_pass': True,
                      'protected_files': len(manifest['protected']),
                      'flashed': False, 'hardware_commands_sent': False}))


if __name__ == '__main__':
    main()
