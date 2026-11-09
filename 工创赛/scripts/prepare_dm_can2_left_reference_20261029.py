"""准备/安装左侧软件角度显示；不烧录、不设置电机零点、不发硬件命令。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/dm_can2_left_reference_20261029/project'
EV = ROOT / 'evidence/p2-arm/2026-10-29/dm-can2-left-reference'
FILES = ['HARDWARE/DM_CAN2/' + n for n in
         ['dm_can2.c', 'dm_can2.h', 'dm_can2_motion.c']]
PROTECTED = ['HARDWARE/DM_CAN2/dm_can2_protocol.c', 'USER/main.c',
             'gongxun-f407.uvprojx', 'SYSTEM/CAN/can.c', 'USER/app_config.h',
             'HARDWARE/LIFT/lift.c', 'HARDWARE/LIFT/lift_home.c',
             'HARDWARE/LIFT/lift_config.h', 'HARDWARE/EXTENSION/extension.c',
             'HARDWARE/EXTENSION/extension.h']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, value):
    (EV / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n',
                           encoding='utf-8')


def build(folder, log):
    proc = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b',
                           str(folder / 'gongxun-f407.uvprojx'), '-j0', '-o', str(log)],
                          capture_output=True, timeout=120)
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
    manifest['installed'] = True
    manifest['actual_build_pass'] = False
    save('installed.json', manifest)
    build(ACTUAL, EV / 'actual-build.log')
    for name, expected in manifest['protected'].items():
        assert sha(ACTUAL / name) == expected, name
    manifest['actual_build_pass'] = True
    save('installed.json', manifest)
    print('Installed display-only reference; no flash or hardware commands.', flush=True)
    raise SystemExit(0)

resuming = '--resume' in sys.argv
assert not STAGE.exists() or resuming, 'Stage already exists; inspect rather than overwrite.'
EV.mkdir(parents=True, exist_ok=True)
if not resuming:
    shutil.copytree(ACTUAL, STAGE,
                    ignore=shutil.ignore_patterns('Objects', 'Listings', '.vscode', '*.uvguix.*'))
manifest = {'files': [], 'protected': {n: sha(ACTUAL / n) for n in PROTECTED},
            'host_pass': False, 'stage_build_pass': False, 'installed': False,
            'left_reference_raw_deg': -296.544037,
            'purpose': 'software coordinate display only; not a homing or motion release',
            'motor_zero_written': False, 'flash_performed': False, 'motion_performed': False}
for name in FILES:
    backup = EV / 'original' / name
    backup.parent.mkdir(parents=True, exist_ok=True)
    if backup.exists():
        assert resuming and sha(backup) == sha(ACTUAL / name), name
    else:
        shutil.copy2(ACTUAL / name, backup)
    shutil.copy2(ACTUAL / name, STAGE / name)
for name, expected in manifest['protected'].items():
    assert sha(STAGE / name) == expected, name

p = STAGE / FILES[1]
data = p.read_bytes()
needle = b'extern volatile float g_dm_can2_position_deg;'
assert data.count(needle) == 1
extra = '''
/* 左侧人工基准候选：2026-10-29只读反馈；掉电恢复尚待实物验证。
 * 仅换算显示，不修改电机零点，不作为自动回零或行程放行依据。
 * 逻辑角=原始反馈角-左侧基准，逆时针为正；不取模、不裁剪角度。
 * 仅在feedback_valid为1且本次反馈新鲜时解释逻辑角；0值本身不代表已回零。 */
#define DM_CAN2_LEFT_ZERO_DEG (-296.544037F)
extern volatile float g_dm_can2_logical_position_deg;
'''.replace('\n', '\r\n').encode('utf-8')
p.write_bytes(data.replace(needle, needle + b'\r\n' + extra, 1))

p = STAGE / FILES[0]
data = p.read_bytes()
needle = b'volatile float g_dm_can2_position_deg = 0.0F;'
assert data.count(needle) == 1
p.write_bytes(data.replace(needle, needle + b'\r\nvolatile float g_dm_can2_logical_position_deg = 0.0F;', 1))

p = STAGE / FILES[2]
data = p.read_bytes()
line_end = b'\r\n' if b'\r\n' in data else b'\n'
needle = b'    JogService(ready, busy);' + line_end + b'}'
assert data.count(needle) == 1
replacement = '''    JogService(ready, busy);
    /* 与只读/试转的最新反馈统一更新；仅显示，不进入运动目标或限幅。 */
    if (g_dm_can2_feedback_valid == 1U)
        g_dm_can2_logical_position_deg = g_dm_can2_position_deg - DM_CAN2_LEFT_ZERO_DEG;
    else
        g_dm_can2_logical_position_deg = 0.0F;
}'''.encode('utf-8').replace(b'\n', line_end)
p.write_bytes(data.replace(needle, replacement, 1))

for name in FILES:
    manifest['files'].append({'file': name, 'before': sha(ACTUAL / name),
                              'after': sha(STAGE / name)})

host = ROOT / 'tmp/dm_can2_left_reference_20261029/host-check'
shutil.copytree(ROOT / 'tmp/dm_can2_jog45_20261029/host-check', host,
                ignore=shutil.ignore_patterns('*.exe'))
model = (ROOT / 'scripts/dm_can2_jog45_host_model.c').read_text(encoding='utf-8')
model = model.replace('CanRxMsg g_dm_can2_last_rx;',
                      'volatile float g_dm_can2_logical_position_deg;\nCanRxMsg g_dm_can2_last_rx;', 1)
needle = ' assert(argc==2);scenario=argv[1];'
assert model.count(needle) == 1
vectors = '''
 {
  const float raw[] = {-296.544037f, -129.447708f, -306.544037f, -106.544037f};
  const float expected_logical[] = {0.0f, 167.096329f, -10.0f, 190.0f};
  unsigned k;
  for(k=0;k<4;k++){
   g_dm_can2_position_deg=raw[k];g_dm_can2_feedback_valid=1;
   DM_CAN2_ModuleService(1,0);
   assert(ab(g_dm_can2_logical_position_deg-expected_logical[k])<0.001f);
   assert(txcount==0);
  }
  g_dm_can2_feedback_valid=0;DM_CAN2_ModuleService(1,0);
  assert(g_dm_can2_logical_position_deg==0.0f && txcount==0);
  g_dm_can2_feedback_valid=1;
 }
'''
model = model.replace(needle, needle + '\n' + vectors, 1)
(host / 'motion.c').write_text(model, encoding='utf-8')
module = STAGE / 'HARDWARE/DM_CAN2'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
       '-I' + str(host), '-I' + str(module), str(module / 'dm_can2_protocol.c'),
       str(module / 'dm_can2_motion.c'), str(host / 'motion.c'), '-o', str(host / 'motion.exe')]
proc = subprocess.run(cmd, capture_output=True, text=True)
assert proc.returncode == 0, proc.stdout + proc.stderr
logs = ['Software reference: 4 signed/no-wrap vectors and invalid-feedback display; no startup transmit.']
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
print('Reference candidate ready; original files backed up; actual project untouched.', flush=True)
