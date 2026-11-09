"""修正相对工作角显示的整圈差；不改运动逻辑，不下载，不发硬件命令。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/dm_can2_reference_wrap_20261029/project'
EV = ROOT / 'evidence/p2-arm/2026-10-29/dm-can2-reference-wrap'
FILES = ['HARDWARE/DM_CAN2/dm_can2.h', 'HARDWARE/DM_CAN2/dm_can2_motion.c']
PROTECTED = ['HARDWARE/DM_CAN2/dm_can2.c', 'HARDWARE/DM_CAN2/dm_can2_protocol.c',
             'USER/main.c', 'gongxun-f407.uvprojx', 'SYSTEM/CAN/can.c', 'USER/app_config.h',
             'HARDWARE/LIFT/lift.c', 'HARDWARE/LIFT/lift_home.c',
             'HARDWARE/LIFT/lift_config.h', 'HARDWARE/EXTENSION/extension.c',
             'HARDWARE/EXTENSION/extension.h']


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def save(name, value):
    (EV / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def build(folder, log):
    proc = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder / 'gongxun-f407.uvprojx'),
                           '-j0', '-o', str(log)], capture_output=True, timeout=120)
    message = log.read_text(encoding='utf-8', errors='replace')
    assert proc.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', message), message[-5000:]
    print(message[-600:], flush=True)


if '--install' in sys.argv:
    manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['host_pass'] and manifest['stage_build_pass']
    for entry in manifest['files']:
        assert sha(ACTUAL / entry['file']) == entry['before'], entry['file']
        assert sha(STAGE / entry['file']) == entry['after'], entry['file']
    for name, expected in manifest['protected'].items():
        assert sha(ACTUAL / name) == expected, name
    for entry in manifest['files']:
        shutil.copy2(STAGE / entry['file'], ACTUAL / entry['file'])
        assert sha(ACTUAL / entry['file']) == entry['after']
    manifest.update(installed=True, actual_build_pass=False)
    save('installed.json', manifest)
    build(ACTUAL, EV / 'actual-build.log')
    for name, expected in manifest['protected'].items():
        assert sha(ACTUAL / name) == expected, name
    manifest['actual_build_pass'] = True
    save('installed.json', manifest)
    print('Display correction installed; no flash or hardware commands.', flush=True)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.vscode', '*.uvguix.*'))
manifest = {'files': [], 'protected': {n: sha(ACTUAL / n) for n in PROTECTED},
            'host_pass': False, 'stage_build_pass': False, 'installed': False,
            'purpose': 'single-turn display branch [-90,270); not motor multi-turn recovery',
            'left_reference_raw_deg': -296.544037, 'motor_zero_written': False,
            'flash_performed': False, 'motion_performed': False}
for name in FILES:
    backup = EV / 'original' / name
    backup.parent.mkdir(parents=True, exist_ok=True)
    assert not backup.exists()
    shutil.copy2(ACTUAL / name, backup)

p = STAGE / FILES[0]
data = p.read_bytes()
old = ' * 逻辑角=原始反馈角-左侧基准，逆时针为正；不取模、不裁剪角度。'.encode('utf-8')
new = ''' * 逻辑显示为原始角减左侧基准后的同圈分支，逆时针为正。
 * 显示范围[-90,270)度，覆盖预计0到180度工作区并保留零点负误差。
 * 这是单圈姿态显示，不恢复多圈数，不作为运动目标、软限位或安全放行。'''.replace('\n', '\r\n').encode('utf-8')
assert data.count(old) == 1
p.write_bytes(data.replace(old, new, 1))

p = STAGE / FILES[1]
data = p.read_bytes()
assert data.count(b'#include "task.h"') == 1
data = data.replace(b'#include "task.h"', b'#include "task.h"\n#include <math.h>', 1)
needle = b'void DM_CAN2_ModuleService(uint32_t ready, uint32_t busy)'
assert data.count(needle) == 1
helper = '''/* 仅用于显示：取覆盖0到180度工作区的同圈分支；不修改原始反馈或动作参考。 */
static float DM_CAN2_DisplayRelativeDeg(float raw_deg)
{
    float relative = fmodf(raw_deg - DM_CAN2_LEFT_ZERO_DEG + 90.0F, 360.0F);
    if (relative < 0.0F) relative += 360.0F;
    return relative - 90.0F;
}

'''.encode('utf-8')
data = data.replace(needle, helper + needle, 1)
old = b'g_dm_can2_logical_position_deg = g_dm_can2_position_deg - DM_CAN2_LEFT_ZERO_DEG;'
new = b'g_dm_can2_logical_position_deg = DM_CAN2_DisplayRelativeDeg(g_dm_can2_position_deg);'
assert data.count(old) == 1
p.write_bytes(data.replace(old, new, 1))

for name in FILES:
    manifest['files'].append({'file': name, 'before': sha(ACTUAL / name), 'after': sha(STAGE / name)})

host = ROOT / 'tmp/dm_can2_reference_wrap_20261029/host-check'
shutil.copytree(ROOT / 'tmp/dm_can2_left_reference_20261029/host-check', host,
                ignore=shutil.ignore_patterns('*.exe'))
p = host / 'motion.c'
model = p.read_text(encoding='utf-8')
old = 'const float raw[] = {-296.544037f, -129.447708f, -306.544037f, -106.544037f};'
new = '''const float raw[] = {-296.544037f, 63.4615974f, -656.544037f, 423.455963f,
      -129.447708f, 230.552292f, -306.544037f, -106.544037f,
      -116.544037f, 63.455963f, -386.544037f, -26.544037f};'''
assert model.count(old) == 1
model = model.replace(old, new, 1)
old = 'const float expected_logical[] = {0.0f, 167.096329f, -10.0f, 190.0f};'
new = '''const float expected_logical[] = {0.0f, 0.0056344f, 0.0f, 0.0f,
      167.096329f, 167.096329f, -10.0f, 190.0f, 180.0f, 0.0f, -90.0f, -90.0f};'''
assert model.count(old) == 1
model = model.replace(old, new, 1)
assert model.count('k<4') == 1
model = model.replace('k<4', 'k<12', 1)
p.write_text(model, encoding='utf-8')
module = STAGE / 'HARDWARE/DM_CAN2'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
       '-I' + str(host), '-I' + str(module), str(module / 'dm_can2_protocol.c'),
       str(module / 'dm_can2_motion.c'), str(host / 'motion.c'), '-lm', '-o', str(host / 'motion.exe')]
proc = subprocess.run(cmd, capture_output=True, text=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
logs = ['12 display regression vectors: observed +360 delta, +/- turns, signed zero neighborhood,',
        '180 endpoint preserved, [-90,270) display branch; invalid feedback display; no startup TX.']
for case in ['positive', 'negative', 'stale', 'txfail', 'noreply', 'fault', 'disabled',
             'tracking', 'limit', 'busy', 'abort', 'repeat', 'gap', 'badprofile', 'stoplost']:
    proc = subprocess.run([str(host / 'motion.exe'), case], capture_output=True, text=True)
    logs.append(proc.stdout + proc.stderr)
    assert proc.returncode == 0, (case, proc.stdout, proc.stderr)
(EV / 'host-check.log').write_text('\n'.join(logs), encoding='utf-8')
manifest['host_pass'] = True
save('manifest.json', manifest)
build(STAGE, EV / 'stage-build.log')
manifest['stage_build_pass'] = True
save('manifest.json', manifest)
print('Display correction staged; backups and checks ready; actual project untouched.', flush=True)
