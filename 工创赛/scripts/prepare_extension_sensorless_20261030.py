"""节点6手动有界碰撞回零候选；只构建/安装，不烧录或发送硬件命令。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/extension_sensorless_20261030/project'
EV = ROOT/'evidence/p2-arm/2026-10-30/extension-sensorless'
ASSETS = ROOT/'output/extension_sensorless_20261030'
FILES = ['HARDWARE/EXTENSION/extension.c','HARDWARE/EXTENSION/extension.h',
         'HARDWARE/EXTENSION/extension_home.c','HARDWARE/EXTENSION/extension_home.h',
         'SYSTEM/CAN/can.c','SYSTEM/CAN/can.h','USER/main.c','gongxun-f407.uvprojx']


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None


def save(name, data):
    (EV/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def one(data, old, new):
    assert data.count(old)==1, old[:100]
    return data.replace(old,new,1)


def verify(folder,m):
    for n,v in m['protected'].items():assert sha(folder/n)==v,n


def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),
                      '-j0','-o',str(log)],capture_output=True,timeout=120)
    s=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',s),s[-5000:]
    print(s[-600:],flush=True)


if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE,m)
    for e in m['files']:assert sha(STAGE/e['file'])==e['after'],e['file']
    if '--build' in sys.argv:
        for name in ['host-review.json','home-read-review.json','home-motion-review.json']:
            r=json.loads((EV/name).read_text(encoding='utf-8'));assert r['passed']
            for e in m['files']:assert r['source_sha256'][e['file']]==e['after'],e['file']
        m['host_pass']=True
        build(STAGE,EV/'stage-build.log');verify(STAGE,m)
        m['stage_build_pass']=True;save('manifest.json',m)
    else:
        assert m['host_pass'] and m['stage_build_pass'];verify(ACTUAL,m)
        for e in m['files']:assert sha(ACTUAL/e['file'])==e['before'],e['file']
        for e in m['files']:shutil.copy2(STAGE/e['file'],ACTUAL/e['file'])
        m.update(installed=True,actual_build_pass=False);save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log');verify(ACTUAL,m)
        for e in m['files']:assert sha(ACTUAL/e['file'])==e['after'],e['file']
        m['actual_build_pass']=True;save('installed.json',m)
        print('Manual node6 homing candidate installed; not flashed; no hardware commands.',flush=True)
    raise SystemExit(0)

resuming = '--resume' in sys.argv
assert not STAGE.exists() or resuming
EV.mkdir(parents=True,exist_ok=True)
if not resuming:
    shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    if (ACTUAL/n).exists():
        p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True)
        if p.exists(): assert resuming and sha(p)==sha(ACTUAL/n),n
        else: shutil.copy2(ACTUAL/n,p)
        shutil.copy2(ACTUAL/n,STAGE/n)
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
           if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
module=STAGE/'HARDWARE/EXTENSION'
for n in ['extension_home.c','extension_home.h']:shutil.copy2(ASSETS/n,module/n)
p=module/'extension.h';t=p.read_bytes()
t=one(t,b'#include "stm32f4xx_can.h"',b'#include "stm32f4xx_can.h"\n#include "extension_home.h"');p.write_bytes(t)
p=module/'extension.c';t=p.read_bytes()
t=one(t,b'if (Extension_MotionIsBusy() || g_extension_motion_request != 0U) return;',
      b'if (ExtensionHome_IsBusy() || Extension_MotionIsBusy() || g_extension_motion_request != 0U) return;')
anchor=b'    g_extension_motion_request = 0U;\n\n    if (request == 3U)'
guard=b'''    g_extension_motion_request = 0U;
    if (ExtensionHome_IsBusy())
    {
        if (request == 3U) { g_extension_home_request = 2U; return; }
        other_busy = 1U;
    }

    if (request == 3U)'''
if anchor not in t:
    anchor=anchor.replace(b'\n',b'\r\n')
    guard=guard.replace(b'\n',b'\r\n')
t=one(t,anchor,guard);p.write_bytes(t)

p=STAGE/'SYSTEM/CAN/can.c';t=p.read_bytes()
t=one(t,b'static void CAN1_CaptureLiftHomeNotice',b'volatile uint32_t g_can_extension_home_done_notice = 0U;\nstatic uint32_t s_extension_home_capture = 0U;\nstatic void CAN1_CaptureLiftHomeNotice')
old=b'        g_can_lift_home_done_notice = 1U;\n}'
if old not in t:old=old.replace(b'\n',b'\r\n')
newline=b'\r\n' if b'\r\n' in old else b'\n'
new=b'''        g_can_lift_home_done_notice = 1U;
    if (s_extension_home_capture && rx->IDE == CAN_Id_Extended && rx->RTR == CAN_RTR_Data &&
        rx->ExtId == 0x0600U && rx->DLC == 3U && rx->Data[0] == 0x9AU &&
        rx->Data[1] == 0x9FU && rx->Data[2] == 0x6BU)
        g_can_extension_home_done_notice = 1U;
}'''.replace(b'\n',newline)
t=one(t,old,new);t+=(ASSETS/'can_home.inc').read_bytes();p.write_bytes(t)
p=STAGE/'SYSTEM/CAN/can.h';t=p.read_bytes()
decl=b'''extern volatile uint32_t g_can_extension_home_done_notice;
uint32_t CAN1_StartExtensionHome(CanRxMsg *reply);
uint32_t CAN1_AbortExtensionHome(CanRxMsg *reply);
uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg *reply);
void CAN1_ExtensionHomeSessionEnd(void);

#endif'''
t=one(t,b'#endif',decl);p.write_bytes(t)

p=STAGE/'USER/main.c';t=p.read_bytes()
a=t.index(b'        if (ArmPower_IsIsolatedMode() != 0U)')
z=t.index(b'            g_can_task_stack_words =',a)
branch=t[a:z];newline=b'\r\n' if b'\r\n' in branch else b'\n';branch=branch.replace(b'\r\n',b'\n')
branch=one(branch,b'            if (Extension_MotionIsBusy())',b'            if (Extension_MotionIsBusy() || ExtensionHome_IsBusy())')
branch=one(branch,b'            Extension_Service(',b'''            if (ExtensionHome_IsBusy() || g_extension_home_request != 0U)
                g_extension_read_request = 0U;
            Extension_Service(''')
branch=one(branch,b'            Extension_HomeReadService(',b'''            ExtensionHome_Service(((g_can_test_result == 1U) &&
                                    (g_safety_ready == 1U) &&
                                    (g_vm_test_request == 1U) &&
                                    (g_arm_menu_power_state == 1U) &&
                                    (g_lift_state == 4U)) ? 1U : 0U,
                                   (DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||
                                    g_lift_motion_request != 0U || g_lift_home_request != 0U ||
                                    (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)) ? 1U : 0U);
            if (ExtensionHome_IsBusy()) g_extension_home_read_request = 0U;
            Extension_HomeReadService(''')
branch=one(branch,b'\n                                    (DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||',
            b'\n                                    (ExtensionHome_IsBusy() || DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||')
branch=one(branch,b'                            (Extension_MotionIsBusy() || g_extension_motion_request != 0U ||',
            b'                            (ExtensionHome_IsBusy() || g_extension_home_request != 0U ||\n                             Extension_MotionIsBusy() || g_extension_motion_request != 0U ||')
t=t[:a]+branch.replace(b'\n',newline)+t[z:];p.write_bytes(t)
p=STAGE/'gongxun-f407.uvprojx';t=p.read_text(encoding='utf-8')
pattern=r'(<Group>\s*<GroupName>EXTENSION</GroupName>.*?<Files>)(.*?)(</Files>)'
def add(m):
    files=m[2]
    for name,kind in [('extension_home.c',1),('extension_home.h',5)]:
        files+=f'<File><FileName>{name}</FileName><FileType>{kind}</FileType><FilePath>.\\HARDWARE\\EXTENSION\\{name}</FilePath></File>\n'
    return m[1]+files+m[3]
t,count=re.subn(pattern,add,t,flags=re.S);assert count==1;p.write_text(t,encoding='utf-8');ET.parse(p)
save('manifest.json',dict(files=[dict(file=n,before=sha(ACTUAL/n),after=sha(STAGE/n)) for n in FILES],
                         protected=protected,host_pass=False,stage_build_pass=False,installed=False,
                         user_origin_candidate='没有限位开关，但是可以限位，可以作为原点',
                         user_distance_text='有3.5cm',reported_retract_stop_distance_mm=35,
                         measured_params=dict(mode=0,direction=0,rpm=30,timeout_ms=10000,detect_rpm=300,detect_current_ma=800,detect_time_ms=60,auto_home=0),
                         default_home_request=0,flash_performed=False,hardware_commands_sent=False))
print('Prepared separate manual node6 homing candidate; current 35mm clearance is user-reported, not a calibration.',flush=True)
