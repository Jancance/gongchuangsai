"""一天快速开发首包：共享互锁和原位单件夹取/抬起/放回调试组，默认关闭。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_sequence_fast_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-sequence-fast'
ASSET=ROOT/'output/arm_sequence_fast_20261031'
FILES=['USER/gripper_task.c','USER/gripper_task.h','USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c','gongxun-f407.uvprojx']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
def save(n,d):(EV/n).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def verify(folder,m,after):
    for n,h in m['protected'].items():assert sha(folder/n)==h,n
    for e in m['files']:assert sha(folder/e['file'])==e['after' if after else 'before'],e['file']
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(folder/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],capture_output=True,timeout=120)
    s=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',s),s[-4000:]
    print(s[-650:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
    assert h['passed'] and h['source_sha256']=={e['file']:e['after'] for e in m['files']}
    verify(STAGE,m,True)
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');verify(STAGE,m,True)
        m['stage_build_pass']=True;save('manifest.json',m)
    else:
        assert m['stage_build_pass'];verify(ACTUAL,m,False)
        for e in m['files']:shutil.copyfile(STAGE/e['file'],ACTUAL/e['file'])
        m['installed']=True;save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log');verify(ACTUAL,m,True)
        m['actual_build_pass']=True;save('installed.json',m)
        print('Installed disabled-by-default debug sequence and shared command interlocks. Not flashed.',flush=True)
    raise SystemExit(0)
refresh='--refresh' in sys.argv
assert not STAGE.exists() or refresh;EV.mkdir(parents=True,exist_ok=True)
if not refresh:shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    if (ACTUAL/n).exists():
        p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True)
        if p.exists():assert sha(p)==sha(ACTUAL/n),n
        else:shutil.copyfile(ACTUAL/n,p)
        shutil.copyfile(ACTUAL/n,STAGE/n)
for n in ['gripper_task.c','gripper_task.h','arm_sequence.c','arm_sequence.h']:
    shutil.copyfile(ASSET/n,STAGE/'USER'/n)
p=STAGE/'USER/main.c';t=p.read_bytes()
assert t.count(b'#include "gripper_task.h"')==1
t=t.replace(b'#include "gripper_task.h"',b'#include "gripper_task.h"\n#include "arm_sequence.h"',1)
wrapper=b'''/* Shared command arbitration for the isolated commissioning sequence. */
static uint32_t App_ArmSequenceReady(void)
{
    return (ArmPower_IsIsolatedMode() && g_can_test_result==1U && g_safety_ready==1U &&
            g_vm_test_request==1U && g_arm_menu_power_state==1U && g_lift_state==4U)?1U:0U;
}
static void App_ArmSequenceGuard(void)
{
    uint32_t exclusive=(ArmSequence_IsBusy() || g_arm_sequence_request==1U);
    if(ArmSequence_IsBusy() && g_arm_sequence_state>=1U && g_arm_sequence_state<=6U &&
      (g_lift_motion_request==3U || g_lift_home_request==2U ||
       g_extension_motion_request==3U || g_extension_home_request==2U ||
       g_dm_jog_request==3U || g_carrier_request==2U || g_jiazhua_debug_request==2U))g_arm_sequence_request=2U;
    if(ArmSequence_IsBusy() && g_arm_sequence_request==2U)
    {
        Gripper_CancelControlled();
        if((g_lift_motion_state>=1U && g_lift_motion_state<=6U) || g_lift_motion_request==1U || g_lift_motion_request==2U)g_lift_motion_request=3U;
    }
    if(exclusive || Gripper_IsCommandBusy())
    {
        if(g_lift_motion_request!=3U && (!ArmSequence_IsBusy() || !ArmSequence_AllowLiftRequest(g_lift_motion_request)))g_lift_motion_request=0U;
        if(g_lift_home_request!=2U)g_lift_home_request=0U;
        if(g_extension_motion_request!=3U)g_extension_motion_request=0U;
        if(g_extension_home_request!=2U)g_extension_home_request=0U;
        if(g_dm_jog_request!=3U)g_dm_jog_request=0U;
        if(g_carrier_request!=2U)g_carrier_request=0U;
        if(exclusive){g_lift_read_request=0U;g_lift_home_read_request=0U;g_extension_read_request=0U;g_extension_home_read_request=0U;}
    }
    Gripper_SetAccess(App_ArmSequenceReady(),
        (Carrier_IsBusy() || g_carrier_request!=0U || DM_CAN2_JogIsBusy() || g_dm_jog_request!=0U ||
         LiftHome_IsBusy() || g_lift_home_request!=0U || g_lift_motion_request!=0U ||
         (g_lift_motion_state>=1U && g_lift_motion_state<=6U) || ExtensionHome_IsBusy() ||
         g_extension_home_request!=0U || Extension_MotionIsBusy() || g_extension_motion_request!=0U)?1U:0U,
         exclusive);
}

'''
assert t.count(b'static void App_CarrierStep(void)')==1
t=t.replace(b'static void App_CarrierStep(void)',wrapper+b'static void App_CarrierStep(void)',1)
assert t.count(b'        App_CarrierStep();')==1
t=t.replace(b'        App_CarrierStep();',b'        App_ArmSequenceGuard();\n        App_CarrierStep();',1)
start=t.index(b'            DM_CAN2_ModuleService(')
pos=t.index(b'            g_can_task_stack_words =',start)
service=b'''            ArmSequence_Service(App_ArmSequenceReady(),
                (Carrier_IsBusy() || DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||
                 ExtensionHome_IsBusy() || Extension_MotionIsBusy() ||
                 g_extension_motion_state==9U || g_extension_home_state==9U || g_lift_home_state==9U)?1U:0U,
                (uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));
'''
t=t[:pos]+service+t[pos:];p.write_bytes(t)
p=STAGE/'gongxun-f407.uvprojx';s=p.read_text(encoding='utf-8-sig')
entries=''
for n in ['arm_sequence.c','arm_sequence.h']:
    assert n not in s
    entries+=f'<File><FileName>{n}</FileName><FileType>{1 if n.endswith(".c") else 5}</FileType><FilePath>.\\USER\\{n}</FilePath></File>\n'
s,c=re.subn(r'(<Group>\s*<GroupName>USER</GroupName>\s*<Files>)(.*?)(</Files>)',lambda m:m[1]+m[2]+entries+m[3],s,count=1,flags=re.S)
assert c==1;ET.fromstring(s);p.write_text(s,encoding='utf-8',newline='\r\n')
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
m={'date':'2026-10-31','files':[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in FILES],
   'protected':protected,'stage_build_pass':False,'default_enable':0,'default_height_ready':0,
   'default_lift_pulses':5000,'max_lift_pulses':10000,'requires_fresh_grip_confirmation':True,
   'formal_autonomous_accepted':False,'flash_performed':False,'hardware_commands_sent':False}
save('manifest.json',m);verify(STAGE,m,True);verify(ACTUAL,m,False)
print('Prepared sequence+gripper arbitration;6changed/new files,protected:',len(protected))
