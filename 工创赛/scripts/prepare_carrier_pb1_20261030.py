"""PB1转盘显式有界测试：副本构建后安装；不烧录，不发送硬件命令。"""
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
STAGE = ROOT/'tmp/carrier_pb1_20261030/project'
EV = ROOT/'evidence/p2-arm/2026-10-30/carrier-pb1'
ASSETS = ROOT/'output/carrier_pb1_20261030'
NEW = ['HARDWARE/CARRIER/'+n for n in
       ['carrier.c','carrier.h','carrier_config.h','carrier_pwm.c','carrier_pwm.h']]
CHANGED = ['USER/main.c','gongxun-f407.uvprojx']

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None

def save(name, data):
    (EV/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def verify(folder, manifest, after):
    for n, h in manifest['protected'].items():
        assert sha(folder/n) == h, n
    for entry in manifest['files']:
        assert sha(folder/entry['file']) == entry['after' if after else 'before'], entry['file']

def build(folder, logfile):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),
                      '-j0','-o',str(logfile)],capture_output=True,timeout=120)
    text=logfile.read_text(encoding='utf-8',errors='replace')
    assert r.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text), text[-4000:]
    print(text[-650:],flush=True)

if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    verify(STAGE,m,True)
    review=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
    assert review['passed']
    assert review['source_sha256'] == {e['file']:e['after'] for e in m['files']}
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log')
        verify(STAGE,m,True)
        m['stage_build_pass']=True
        save('manifest.json',m)
    else:
        assert m['stage_build_pass']
        verify(ACTUAL,m,False)
        for entry in m['files']:
            target=ACTUAL/entry['file']
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(STAGE/entry['file'],target)
        m['installed']=True
        save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log')
        verify(ACTUAL,m,True)
        m['actual_build_pass']=True
        save('installed.json',m)
        print('Installed source only; no flash or hardware commands.',flush=True)
    raise SystemExit(0)

assert not STAGE.exists(), 'Do not overwrite an existing stage.'
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns(
    'Objects','Listings','.git','.vscode','*.uvguix.*'))
for name in CHANGED:
    p=EV/'original'/name
    p.parent.mkdir(parents=True,exist_ok=True)
    assert not p.exists()
    shutil.copyfile(ACTUAL/name,p)
for name in NEW:
    assert not (ACTUAL/name).exists(), name
    p=STAGE/name
    p.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(ASSETS/Path(name).name,p)

p=STAGE/'USER/main.c'
data=p.read_bytes()
def replace(old,new):
    global data
    assert data.count(old)==1,old
    data=data.replace(old,new,1)
replace(b'#include "gripper.h"',b'#include "gripper.h"\n#include "carrier.h"')
wrapper=b'''/* PB1 carrier uses the same serialized arm scheduler; no boot motion. */
static void App_CarrierStep(void)
{
    uint32_t ready = (ArmPower_IsIsolatedMode() && g_can_test_result == 1U &&
                      g_safety_ready == 1U && g_vm_test_request == 1U &&
                      g_arm_menu_power_state == 1U && g_lift_state == 4U) ? 1U : 0U;
    uint32_t other_busy = (DM_CAN2_JogIsBusy() || g_dm_jog_request != 0U ||
                          LiftHome_IsBusy() || g_lift_home_request != 0U ||
                          g_lift_motion_request != 0U ||
                          (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U) ||
                          ExtensionHome_IsBusy() || g_extension_home_request != 0U ||
                          Extension_MotionIsBusy() || g_extension_motion_request != 0U ||
                          g_lift_motion_state == 9U || g_lift_home_state == 9U ||
                          g_extension_motion_state == 9U || g_extension_home_state == 9U) ? 1U : 0U;
    Carrier_Service(ready, other_busy,
                    (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    if (Carrier_IsBusy())
    {
        if (g_lift_motion_request != 3U) g_lift_motion_request = 0U;
        if (g_lift_home_request != 2U) g_lift_home_request = 0U;
        if (g_extension_motion_request != 3U) g_extension_motion_request = 0U;
        if (g_extension_home_request != 2U) g_extension_home_request = 0U;
        if (g_dm_jog_request != 3U) g_dm_jog_request = 0U;
    }
}

'''
replace(b'static void App_CanTask(void *argument)',wrapper+b'static void App_CanTask(void *argument)')
replace(b'        g_can_task_count++;',b'        g_can_task_count++;\n        App_CarrierStep();')
replace(b'    Gripper_Init();',b'    Carrier_Init();\n    Gripper_Init();')
p.write_bytes(data)

p=STAGE/'gongxun-f407.uvprojx'
t=p.read_text(encoding='utf-8-sig')
marker=r'.\HARDWARE\DM_CAN2</IncludePath>'
assert t.count(marker)==1
t=t.replace(marker,r'.\HARDWARE\DM_CAN2;.\HARDWARE\CARRIER</IncludePath>',1)
entries=''
for name in NEW:
    entries+=('            <File>\n'
              f'              <FileName>{Path(name).name}</FileName>\n'
              f'              <FileType>{1 if name.endswith(".c") else 5}</FileType>\n'
              '              <FilePath>.\\'+name.replace('/','\\')+'</FilePath>\n'
              '            </File>\n')
pattern=r'(<Group>\s*<GroupName>HARDWARE</GroupName>\s*<Files>)(.*?)(</Files>)'
t,count=re.subn(pattern,lambda m:m[1]+m[2]+entries+'          '+m[3],t,count=1,flags=re.S)
assert count==1
ET.fromstring(t)
p.write_text(t,encoding='utf-8',newline='\r\n')
files=[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in NEW+CHANGED]
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
           if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx')
           and p.relative_to(ACTUAL).as_posix() not in NEW+CHANGED}
m={'files':files,'protected':protected,'stage_build_pass':False,
   'user_wiring':'PB1, 5V','user_servo_identity':'same type as original 1.270-degree servo project',
   'reference_project':str(ACTUAL.parent/'1.270°舵机旋转'),
   'profile':{'period_us':20000,'pulse_min_us':500,'pulse_max_us':2500,'angle_max_deg':270},
   'reference_sha256':{n:sha(ACTUAL.parent/'1.270°舵机旋转'/n) for n in
                       ['HARDWARE/SERVO/servo.c','HARDWARE/SERVO/servo.h','USER/main.c']},
   'flash_performed':False,'hardware_commands_sent':False}
save('manifest.json',m)
verify(ACTUAL,m,False)
verify(STAGE,m,True)
print('Prepared seven files; protected source count:',len(protected))
