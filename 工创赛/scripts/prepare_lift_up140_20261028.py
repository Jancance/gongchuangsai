"""User-selected coarse conversion: 5000 pulses / 3 mm, upward 140 mm."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/lift_up140_20261028/project'
EV = ROOT/'evidence/p2-arm/2026-10-28/lift-up140'
FILES = ['HARDWARE/LIFT/lift_config.h', 'HARDWARE/LIFT/lift.h', 'SYSTEM/CAN/can.c']
UV4 = 'C:/Keil_v5/UV4/UV4.exe'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def build(project, logfile):
    proc = subprocess.run([UV4, '-b', str(project/'gongxun-f407.uvprojx'), '-j0', '-o', str(logfile)],
                          capture_output=True)
    log = logfile.read_text(encoding='utf-8', errors='replace')
    if proc.returncode != 0 or not re.search(r'0 Error\(s\), 0 Warning\(s\)', log):
        raise RuntimeError(log)
    print(log[-500:])

def prepare():
    assert not STAGE.exists(), 'Inspect existing stage instead of overwriting'
    EV.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ACTUAL, STAGE)
    replacements = [
        (FILES[0], b'#define LIFT_UP_MAX_PULSES    50000U', b'#define LIFT_UP_MAX_PULSES    233333U'),
        (FILES[1], '1~50000'.encode(), '1~233333'.encode()),
        (FILES[2], b'(request[1] == 1U ? 50000U : 5000U)', b'(request[1] == 1U ? 233333U : 5000U)'),
    ]
    entries = []
    for name, old, new in replacements:
        target = STAGE/name
        content = target.read_bytes()
        assert content.count(old) == 1, name
        backup = EV/'original'/name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ACTUAL/name, backup)
        target.write_bytes(content.replace(old, new, 1))
        entries.append({'file': name, 'before_sha256': sha(ACTUAL/name), 'after_sha256': sha(target)})
    manifest = {'actual': str(ACTUAL), 'stage': str(STAGE), 'files': entries,
                'conversion_source': 'User explicitly chose prior coarse 5000 pulses approximately 3 mm',
                'target_estimated_mm': 140, 'target_pulses': 233333,
                'default_up_pulses': 5000, 'down_pulses': 5000,
                'physical_distance_verified': False, 'flash_performed': False,
                'hardware_motion_executed': False}
    (EV/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Prepared upward limit 233333; defaults unchanged; no hardware action.')

def check():
    host = ROOT/'tmp/lift_up140_20261028/host-check'
    host.mkdir(parents=True, exist_ok=True)
    module = STAGE/'HARDWARE/LIFT'
    can = (STAGE/'SYSTEM/CAN/can.c').read_bytes().decode('utf-8', errors='replace')
    can = can[can.index('uint32_t CAN1_LiftExchange('):]
    guard = can[can.index('    speed_rpm ='):can.index('    memset(reply,')]
    code = '''#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include "lift_protocol.h"
static uint32_t validate(const uint8_t *request,uint8_t length) {
uint32_t pulses,speed_rpm;
'''+guard+'''return 1; }
int main(void) {
 uint8_t q[12];
 assert(LiftProtocol_Move(1,233333,q));
 assert(q[1]==1 && q[2]==11 && q[3]==184 && q[4]==250 && q[9]==2 && q[10]==0);
 assert((((uint32_t)q[5]<<24)|((uint32_t)q[6]<<16)|((uint32_t)q[7]<<8)|q[8])==233333);
 assert(validate(q,12)==1);q[8]++;assert(validate(q,12)==7);
 assert(!LiftProtocol_Move(1,233334,q));
 assert(!LiftProtocol_Move(0,233333,q));
 assert(LiftProtocol_Move(0,5000,q) && validate(q,12)==1);
 q[8]++;assert(validate(q,12)==7);
 assert(!LiftProtocol_Move(0,5001,q));
 assert(!LiftProtocol_Move(1,0,q));
 assert(LiftProtocol_Move(1,233333,q));q[9]=0;assert(validate(q,12)==7);
 puts("PASS: upward 233333, excess/zero rejection, downward 5000 boundary, relative mode and existing RPM/acceleration.");
 return 0;
}
'''
    (host/'check.c').write_text(code, encoding='utf-8')
    for args in [
        ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
         '-I'+str(module), str(module/'lift_protocol.c'), str(host/'check.c'), '-o', str(host/'check.exe')],
        [str(host/'check.exe')],
    ]:
        result = subprocess.run(args, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout+result.stderr
        if result.stdout:
            (EV/'boundary-check.log').write_text(result.stdout, encoding='utf-8')
            print(result.stdout)
    build(STAGE, EV/'stage-build.log')
    (EV/'checks.json').write_text(json.dumps({'boundary_pass': True, 'stage_build_pass': True}), encoding='utf-8')

def install():
    manifest = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    checks = json.loads((EV/'checks.json').read_text(encoding='utf-8'))
    assert checks['boundary_pass'] and checks['stage_build_pass']
    for entry in manifest['files']:
        assert sha(ACTUAL/entry['file']) == entry['before_sha256'], entry['file']
        assert sha(STAGE/entry['file']) == entry['after_sha256'], entry['file']
    for entry in manifest['files']:
        shutil.copy2(STAGE/entry['file'], ACTUAL/entry['file'])
        assert sha(ACTUAL/entry['file']) == entry['after_sha256']
    build(ACTUAL, EV/'actual-build.log')
    manifest.update(installed=True, actual_build_pass=True)
    (EV/'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    if args.install:
        install()
    else:
        prepare()
        check()
