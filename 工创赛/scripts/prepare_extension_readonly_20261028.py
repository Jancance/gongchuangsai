"""Stage/install node6 read-only identification; no flashing or motor commands."""
from pathlib import Path
import hashlib, json, re, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT/'tmp/extension_readonly_20261028/project'
EV = ROOT/'evidence/p2-arm/2026-10-28/extension-readonly'
FILES = ['HARDWARE/EXTENSION/extension.c', 'HARDWARE/EXTENSION/extension.h', 'USER/main.c', 'gongxun-f407.uvprojx']

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None

def build(folder, log):
    r = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-b', str(folder/'gongxun-f407.uvprojx'), '-j0', '-o', str(log)], capture_output=True)
    t = log.read_text(encoding='utf-8', errors='replace')
    assert r.returncode == 0 and re.search(r'0 Error\(s\), 0 Warning\(s\)', t), t
    print(t[-500:])

if '--install' in sys.argv:
    m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    assert m['host_pass'] and m['build_pass']
    for f in m['files']:
        assert sha(ACTUAL/f['file']) == f['before']
        assert sha(STAGE/f['file']) == f['after']
    for f in m['files']:
        target = ACTUAL/f['file']
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(STAGE/f['file'], target)
        assert sha(target) == f['after']
    build(ACTUAL, EV/'actual-build.log')
    m['installed'] = True
    (EV/'installed.json').write_text(json.dumps(m, ensure_ascii=False, indent=2), encoding='utf-8')
else:
    assert not STAGE.exists()
    EV.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ACTUAL, STAGE)
    module = STAGE/'HARDWARE/EXTENSION'
    module.mkdir()
    for name in ['extension.c', 'extension.h']:
        shutil.copy2(ROOT/'output/extension_readonly_20261028'/name, module/name)
    p = STAGE/'USER/main.c'
    t = p.read_bytes()
    assert t.count(b'#include "lift.h"') == 1
    t = t.replace(b'#include "lift.h"', b'#include "lift.h"\r\n#include "lift_home.h"\r\n#include "extension.h"')
    old = b'            g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);'
    # Insert only inside the isolated arm branch immediately after Lift_Service.
    a = t.index(b'        if (ArmPower_IsIsolatedMode() != 0U)')
    z = t.index(old, a)
    call = b'''            Extension_Service(((g_can_test_result == 1U) &&
                               (g_safety_ready == 1U) &&
                               (g_vm_test_request == 1U) &&
                               (g_arm_menu_power_state == 1U) &&
                               (g_lift_state == 4U)) ? 1U : 0U,
                              (LiftHome_IsBusy() || g_lift_motion_request != 0U ||
                               g_lift_home_request != 0U ||
                               (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U)) ? 1U : 0U);
'''.replace(b'\n', b'\r\n')
    t = t[:z] + call + t[z:]
    p.write_bytes(t)
    p = STAGE/'gongxun-f407.uvprojx'
    t = p.read_text(encoding='utf-8')
    t = t.replace('.\\HARDWARE\\LIFT</IncludePath>', '.\\HARDWARE\\LIFT;.\\HARDWARE\\EXTENSION</IncludePath>')
    assert '.\\HARDWARE\\EXTENSION</IncludePath>' in t
    xml = '<Group><GroupName>EXTENSION</GroupName><Files>'
    for name, kind in [('extension.c',1), ('extension.h',5)]:
        xml += f'<File><FileName>{name}</FileName><FileType>{kind}</FileType><FilePath>.\\HARDWARE\\EXTENSION\\{name}</FilePath></File>'
    xml += '</Files></Group>\n'
    assert t.count('</Groups>') == 1
    p.write_text(t.replace('</Groups>', xml+'</Groups>'), encoding='utf-8')
    host = ROOT/'tmp/extension_readonly_20261028/host-check'
    host.mkdir()
    oldhost = ROOT/'tmp/lift_sensorless_20261025/host-check'
    for name in ['stm32f4xx.h','stm32f4xx_can.h']:
        shutil.copy2(oldhost/name, host/name)
    (host/'can.h').write_text('#include "stm32f4xx_can.h"\nuint32_t CAN1_ReadVersion(uint8_t,CanRxMsg*);\nuint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);\nuint32_t CAN1_ReadMotorOptions(uint8_t,CanRxMsg*);\n')
    (host/'test.c').write_text('''#include <assert.h>
#include <stdio.h>
#include "extension.h"
static unsigned calls=0,fail=0;
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==6);calls++;r->Data[0]=0x1F;return fail?5:1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){assert(a==6);calls++;r->Data[0]=0x3A;return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){assert(a==6);calls++;r->Data[0]=0x1A;return 1;}
int main(void){
Extension_Service(1,0);assert(calls==0);
g_extension_read_request=1;Extension_Service(0,0);assert(calls==0 && g_extension_error==7);
g_extension_read_request=1;Extension_Service(1,1);assert(calls==0);
g_extension_read_request=1;Extension_Service(1,0);assert(calls==1 && g_extension_state==2);
g_extension_read_request=1;Extension_Service(1,0);assert(calls==2 && g_extension_state==3);
Extension_Service(1,0);assert(calls==3 && g_extension_state==4);
Extension_Service(1,0);assert(calls==3);
Extension_Service(0,0);Extension_Service(1,0);assert(calls==3 && g_extension_state==0);
fail=1;g_extension_read_request=1;Extension_Service(1,0);assert(calls==4 && g_extension_state==5);
Extension_Service(1,0);assert(calls==4);
fail=0;g_extension_read_request=1;Extension_Service(1,0);
Extension_Service(1,1);assert(g_extension_state==5 && calls==5);
puts("PASS: node6 only; startup idle; power/busy guards; three bounded queries; no queue/retry; power invalidation; interruption.");return 0;}
''')
    r = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-I'+str(host),'-I'+str(module),str(module/'extension.c'),str(host/'test.c'),'-o',str(host/'test.exe')],capture_output=True,text=True)
    assert r.returncode == 0,r.stdout+r.stderr
    r = subprocess.run([str(host/'test.exe')],capture_output=True,text=True)
    (EV/'host-check.log').write_text(r.stdout+r.stderr,encoding='utf-8')
    assert r.returncode == 0,r.stdout+r.stderr
    print(r.stdout)
    build(STAGE, EV/'stage-build.log')
    m = {'files':[], 'host_pass':True,'build_pass':True,'node':6,'read_only':True,'flash_performed':False,'hardware_motion_executed':False}
    for name in FILES:
        src=ACTUAL/name
        if src.exists():
            backup=EV/'original'/name
            backup.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(src,backup)
        m['files'].append({'file':name,'before':sha(src),'after':sha(STAGE/name)})
    (EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
