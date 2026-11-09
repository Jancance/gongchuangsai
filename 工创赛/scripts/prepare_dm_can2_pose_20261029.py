"""在已验收的CAN2只读模块上加入量程/状态读取，不烧录、不运动。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/dm_can2_pose_20261029/project'
EV = ROOT / 'evidence/p2-arm/2026-10-29/dm-can2-pose'
FILES = [f'HARDWARE/DM_CAN2/{name}' for name in ('dm_can2.c', 'dm_can2.h', 'dm_can2_protocol.c')]
PROTECTED = ['USER/main.c', 'gongxun-f407.uvprojx', 'SYSTEM/CAN/can.c', 'USER/app_config.h',
             'HARDWARE/LIFT/lift.c', 'HARDWARE/LIFT/lift_home.c', 'HARDWARE/LIFT/lift_config.h',
             'HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h']


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def build(folder, log):
    result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'),
                             '-j0', '-o', str(log)], capture_output=True, timeout=120)
    text = log.read_text(encoding='utf-8', errors='replace')
    assert result.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', text), text[-9000:]
    print(text[-650:], flush=True)


if '--install' in sys.argv:
    manifest = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert manifest['host_pass'] and manifest['build_pass']
    for item in manifest['files']:
        assert sha(ACTUAL/item['file']) == item['before'], 'Actual source drift: '+item['file']
        assert sha(STAGE/item['file']) == item['after']
    for name, digest in manifest['protected'].items():
        assert sha(ACTUAL/name) == digest
    for item in manifest['files']:
        shutil.copy2(STAGE/item['file'], ACTUAL/item['file'])
        assert sha(ACTUAL/item['file']) == item['after']
    manifest['installed'] = True
    manifest['actual_build_pass'] = False
    (EV/'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    build(ACTUAL, EV/'actual-build.log')
    for name, digest in manifest['protected'].items():
        assert sha(ACTUAL/name) == digest
    manifest['actual_build_pass'] = True
    (EV/'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Pose read-only module installed and built; no hardware commands.', flush=True)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.vscode', '*.uvguix.*'))
for name in FILES:
    shutil.copy2(ROOT/'output/dm_can2_pose_20261029'/Path(name).name, STAGE/name)
    backup = EV/'original'/name
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ACTUAL/name, backup)

host = ROOT/'tmp/dm_can2_pose_20261029/host-check'
host.mkdir()
old = ROOT/'tmp/dm_can2_readonly_20261029/host-check'
shutil.copy2(old/'stm32f4xx_can.h', host/'stm32f4xx_can.h')
text = (old/'test.c').read_text()
extra = '''
 {
  float f=0; DM_CAN2_Feedback feedback;
  assert(DM_CAN2_DecodeRange(0x41480000UL,&f) && f==12.5f);
  assert(!DM_CAN2_DecodeRange(0,&f));
  assert(!DM_CAN2_DecodeRange(0xBF800000UL,&f));
  assert(!DM_CAN2_DecodeRange(0x7F800000UL,&f));
  assert(!DM_CAN2_DecodeRange(0x7FC00000UL,&f));
  assert(!DM_CAN2_DecodeRange(0x41480000UL,0));
  for(i=0x15;i<=0x17;i++){
   assert(DM_CAN2_BuildRead((uint8_t)i,&tx));
   assert(tx.StdId==0x7FF && tx.DLC==8 && tx.Data[2]==0x33 && tx.Data[3]==i);
  }
  assert(DM_CAN2_BuildStatus(&tx));
  assert(tx.StdId==0x7FF && tx.DLC==8 && tx.Data[0]==1 && tx.Data[1]==0 && tx.Data[2]==0xCC);
  assert(tx.Data[3]==0 && tx.Data[4]==0 && tx.Data[5]==0 && tx.Data[6]==0 && tx.Data[7]==0);
  memset(&rx,0,sizeof(rx));rx.DLC=8;rx.Data[0]=1;
  rx.Data[1]=0x80;rx.Data[3]=0x80;rx.Data[4]=0x08;
  assert(DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));
  assert(feedback.position_raw==32768 && feedback.velocity_raw==2048 && feedback.torque_raw==2048);
  assert(feedback.position_rad>0 && feedback.position_rad<0.001f);
  assert(feedback.velocity_rad_s>0 && feedback.velocity_rad_s<0.01f);
  assert(feedback.torque_nm>0 && feedback.torque_nm<0.01f);
  rx.Data[0]=0x91;assert(DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));
  assert(feedback.motor_state==9); /* 故障码只保留，不伪装为正常状态 */
  assert(!DM_CAN2_ParseStatus(&rx,0,30.0f,10.0f,&feedback));
  rx.Data[0]=2;assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));rx.Data[0]=1;
  rx.StdId=1;assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));rx.StdId=0;
  rx.IDE=4;assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));rx.IDE=0;
  rx.RTR=2;assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));rx.RTR=0;
  rx.DLC=7;assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));rx.DLC=8;
  memset(rx.Data,0,8);rx.Data[0]=1;rx.Data[2]=0x33;rx.Data[3]=0x15;
  assert(!DM_CAN2_ParseStatus(&rx,12.5f,30.0f,10.0f,&feedback));
  assert(DM_CAN2_ParseRead(&rx,0x15,&value));
 }
'''
assert text.count(' puts(') == 1
text = text.replace(' puts(', extra+' puts(', 1)
text = text.replace('PASS: three read-only frames;', 'PASS: identity/range/status read-only frames; finite positive scales; status packing and late-parameter rejection;')
(host/'test.c').write_text(text, encoding='utf-8')
module = STAGE/'HARDWARE/DM_CAN2'
result = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                         '-I'+str(host), '-I'+str(module), str(module/'dm_can2_protocol.c'),
                         str(host/'test.c'), '-o', str(host/'test.exe')], capture_output=True, text=True)
assert result.returncode == 0, result.stdout+result.stderr
result = subprocess.run([str(host/'test.exe')], capture_output=True, text=True)
(EV/'host-protocol.log').write_text(result.stdout+result.stderr, encoding='utf-8')
assert result.returncode == 0, result.stdout+result.stderr
print(result.stdout, flush=True)
manifest = {'files': [{'file': name, 'before': sha(ACTUAL/name), 'after': sha(STAGE/name)} for name in FILES],
            'protected': {name: sha(ACTUAL/name) for name in PROTECTED},
            'host_pass': True, 'build_pass': False, 'read_only': True, 'flash_performed': False}
(EV/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
build(STAGE, EV/'stage-build.log')
manifest['build_pass'] = True
(EV/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
print('Stage compiled; actual source not changed.', flush=True)
