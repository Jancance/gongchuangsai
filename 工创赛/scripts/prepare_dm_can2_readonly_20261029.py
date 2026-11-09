"""暂存、校验及安装CAN2只读模块；不下载固件，不发电机命令。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/dm_can2_readonly_20261029/project'
EV = ROOT / 'evidence/p2-arm/2026-10-29/dm-can2-readonly'
MODULE = ROOT / 'output/dm_can2_readonly_20261029'
FILES = ['HARDWARE/DM_CAN2/dm_can2.c', 'HARDWARE/DM_CAN2/dm_can2.h',
         'HARDWARE/DM_CAN2/dm_can2_protocol.c', 'USER/main.c', 'gongxun-f407.uvprojx']
PROTECTED = ['SYSTEM/CAN/can.c', 'USER/app_config.h', 'HARDWARE/LIFT/lift.c',
             'HARDWARE/LIFT/lift_home.c', 'HARDWARE/LIFT/lift_config.h',
             'HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h']


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None


def build(folder, log):
    result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder / 'gongxun-f407.uvprojx'),
                             '-j0', '-o', str(log)], capture_output=True, timeout=120)
    text = log.read_text(encoding='utf-8', errors='replace')
    if result.returncode != 0 or not re.search(r'0 Error\(s\), 0 Warning\(s\)', text):
        raise RuntimeError(text[-10000:] + '\n' + repr(result.stdout + result.stderr))
    print(text[-700:], flush=True)


if '--install' in sys.argv:
    manifest = json.loads((EV / 'manifest.json').read_text(encoding='utf-8'))
    assert manifest['host_protocol_pass'] and manifest['stage_build_pass']
    for item in manifest['files']:
        assert digest(ACTUAL / item['file']) == item['before'], 'Actual source changed: ' + item['file']
        assert digest(STAGE / item['file']) == item['after'], 'Staged source changed: ' + item['file']
    for name, sha in manifest['protected'].items():
        assert digest(ACTUAL / name) == sha, 'Protected source changed: ' + name
    for item in manifest['files']:
        target = ACTUAL / item['file']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(STAGE / item['file'], target)
        assert digest(target) == item['after']
    # 先记录同步事实，即使后续构建中断也不丢失实际文件状态。
    manifest['installed'] = True
    manifest['actual_build_pass'] = False
    (EV / 'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    build(ACTUAL, EV / 'actual-build.log')
    for name, sha in manifest['protected'].items():
        assert digest(ACTUAL / name) == sha
    manifest['actual_build_pass'] = True
    (EV / 'installed.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Actual project synchronized and built. No flash or CAN commands.', flush=True)
    raise SystemExit(0)

assert not STAGE.exists(), 'Stage already exists; inspect before changing it'
EV.mkdir(parents=True, exist_ok=True)
shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.vscode', '*.uvguix.*'))
target = STAGE / 'HARDWARE/DM_CAN2'
target.mkdir()
for name in ('dm_can2.c', 'dm_can2.h', 'dm_can2_protocol.c'):
    shutil.copy2(MODULE / name, target / name)

p = STAGE / 'USER/main.c'
data = p.read_bytes()
assert data.count(b'#include "extension.h"') == 1
data = data.replace(b'#include "extension.h"', b'#include "extension.h"\r\n#include "dm_can2.h"', 1)
start = data.index(b'        if (ArmPower_IsIsolatedMode() != 0U)')
position = data.index(b'            g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);', start)
call = b'''            DM_CAN2_Service(((g_can_test_result == 1U) &&
                             (g_safety_ready == 1U) &&
                             (g_vm_test_request == 1U) &&
                             (g_arm_menu_power_state == 1U)) ? 1U : 0U,
                            (LiftHome_IsBusy() || g_lift_motion_request != 0U ||
                             g_lift_home_request != 0U ||
                             (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U) ||
                             g_extension_read_request != 0U ||
                             (g_extension_state >= 1U && g_extension_state <= 3U)) ? 1U : 0U);
'''.replace(b'\n', b'\r\n')
p.write_bytes(data[:position] + call + data[position:])

p = STAGE / 'gongxun-f407.uvprojx'
text = p.read_text(encoding='utf-8-sig')
paths = ['.\\HARDWARE\\EXTENSION', '.\\HARDWARE\\DM_CAN2']
def add_include(match):
    items = match.group(1).split(';')
    for path in paths:
        if path not in items:
            items.append(path)
    return '<IncludePath>' + ';'.join(items) + '</IncludePath>'
text, count = re.subn(r'<IncludePath>([^<]+)</IncludePath>', add_include, text)
assert count == 1
groups = ''
# 只补回已存在但被当前工程文件漏掉的伸缩模块引用，不改其源码。
for group, names in [('EXTENSION', ['extension.c', 'extension.h']),
                     ('DM_CAN2', ['dm_can2.c', 'dm_can2.h', 'dm_can2_protocol.c'])]:
    assert '<GroupName>' + group + '</GroupName>' not in text
    groups += '<Group><GroupName>' + group + '</GroupName><Files>'
    for name in names:
        kind = 1 if name.endswith('.c') else 5
        groups += f'<File><FileName>{name}</FileName><FileType>{kind}</FileType><FilePath>.\\HARDWARE\\{group}\\{name}</FilePath></File>'
    groups += '</Files></Group>\n'
assert text.count('</Groups>') == 1
p.write_text(text.replace('</Groups>', groups + '</Groups>'), encoding='utf-8')

host = ROOT / 'tmp/dm_can2_readonly_20261029/host-check'
host.mkdir()
(host / 'stm32f4xx_can.h').write_text('''#ifndef MOCK_CAN_H
#define MOCK_CAN_H
#include <stdint.h>
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC; uint8_t Data[8]; } CanTxMsg;
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC; uint8_t Data[8]; uint8_t FMI; } CanRxMsg;
#define CAN_Id_Standard 0
#define CAN_Id_Extended 4
#define CAN_RTR_Data 0
#define CAN_RTR_Remote 2
#endif
''')
(host / 'test.c').write_text('''#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "dm_can2.h"
int main(void){
 CanTxMsg tx; CanRxMsg rx; uint32_t value=99; unsigned i;
 const uint8_t rids[]={8,7,10};
 for(i=0;i<3;i++){
  assert(DM_CAN2_BuildRead(rids[i],&tx)==1);
  assert(tx.StdId==0x7FF && tx.IDE==0 && tx.RTR==0 && tx.DLC==8);
  assert(tx.Data[0]==1 && tx.Data[1]==0 && tx.Data[2]==0x33 && tx.Data[3]==rids[i]);
  assert(tx.Data[4]==0 && tx.Data[5]==0 && tx.Data[6]==0 && tx.Data[7]==0);
 }
 assert(!DM_CAN2_BuildRead(0x55,&tx)); assert(!DM_CAN2_BuildRead(8,0));
 memset(&rx,0,sizeof(rx)); rx.DLC=8; rx.Data[0]=1;rx.Data[2]=0x33;rx.Data[3]=7;
 assert(DM_CAN2_ParseRead(&rx,7,&value) && value==0); /* Master ID0必须成功 */
 rx.Data[3]=8;rx.Data[4]=1;assert(DM_CAN2_ParseRead(&rx,8,&value)&&value==1);
 rx.Data[4]=0x78;rx.Data[5]=0x56;rx.Data[6]=0x34;rx.Data[7]=0x12;
 assert(DM_CAN2_ParseRead(&rx,8,&value)&&value==0x12345678);
 rx.StdId=1;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.StdId=0;
 rx.IDE=4;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.IDE=0;
 rx.RTR=2;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.RTR=0;
 rx.DLC=7;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.DLC=8;
 rx.Data[0]=2;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.Data[0]=1;
 rx.Data[1]=1;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.Data[1]=0;
 rx.Data[2]=0x55;assert(!DM_CAN2_ParseRead(&rx,8,&value));rx.Data[2]=0x33;
 assert(!DM_CAN2_ParseRead(&rx,7,&value));assert(!DM_CAN2_ParseRead(&rx,8,0));
 puts("PASS: three read-only frames; standard ID0 reply; little-endian; wrong ID/node/opcode/register/IDE/RTR/DLC rejected.");return 0;
}
''')
result = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
                         '-I' + str(host), '-I' + str(target), str(target / 'dm_can2_protocol.c'),
                         str(host / 'test.c'), '-o', str(host / 'test.exe')], capture_output=True, text=True)
assert result.returncode == 0, result.stdout + result.stderr
result = subprocess.run([str(host / 'test.exe')], capture_output=True, text=True)
(EV / 'host-protocol.log').write_text(result.stdout + result.stderr, encoding='utf-8')
assert result.returncode == 0, result.stdout + result.stderr
print(result.stdout, flush=True)

manifest = {'files': [], 'protected': {name: digest(ACTUAL / name) for name in PROTECTED},
            'host_protocol_pass': True, 'stage_build_pass': False,
            'read_only': True, 'flash_performed': False, 'motor_commands_sent': False,
            'existing_project_issue': '主源码及伸缩模块仍为上次安装版本，uvprojx已变化且漏掉EXTENSION引用；本次只恢复引用。'}
for name in FILES:
    source = ACTUAL / name
    if source.exists():
        backup = EV / 'original' / name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, backup)
    manifest['files'].append({'file': name, 'before': digest(source), 'after': digest(STAGE / name)})
(EV / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
build(STAGE, EV / 'stage-build.log')
manifest['stage_build_pass'] = True
for name in PROTECTED:
    assert digest(STAGE / name) == manifest['protected'][name]
(EV / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
print('Stage and protocol checks passed; actual source untouched.', flush=True)
