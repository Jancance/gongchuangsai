"""顶部夹取往返调试：先在副本准备/检查，再显式安装并编译；不烧录。"""
from pathlib import Path
import hashlib,json,shutil,subprocess,re,sys
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_top_cycle_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-top-cycle'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v): (EV/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    text=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text),text[-4000:]
    print(text[-750:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for e in m['files']: assert sha(STAGE/e['file'])==e['after'],e['file']
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');m['stage_build_pass']=True;save('manifest.json',m)
    else:
        review=json.loads((EV/'host-review.json').read_text())
        assert m['stage_build_pass'] and review['passed']
        assert review['source_sha256']=={e['file']:e['after'] for e in m['files']}
        for n,h in m['protected'].items(): assert sha(ACTUAL/n)==h,n
        for e in m['files']: assert sha(ACTUAL/e['file'])==e['before'],e['file']
        for e in m['files']: shutil.copyfile(STAGE/e['file'],ACTUAL/e['file'])
        save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log')
        for e in m['files']: assert sha(ACTUAL/e['file'])==e['after'],e['file']
        for n,h in m['protected'].items(): assert sha(ACTUAL/n)==h,n
        m['actual_build_pass']=True;save('installed.json',m)
    raise SystemExit(0)
assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ACTUAL/n,p)
p=STAGE/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=s.replace('extern volatile uint32_t g_arm_test_lift_pulses;', '''extern volatile uint32_t g_arm_test_lift_pulses;
/* 模式0保留小幅往返；模式1夹紧后顶部回零，再按显式记录的脉冲下降放回。 */
extern volatile uint32_t g_arm_sequence_mode;
/* 顶部到夹取处的已记录脉冲，默认0拒绝；不是毫米或自动测量值。 */
extern volatile uint32_t g_arm_return_down_pulses;
uint32_t ArmSequence_AllowHomeRequest(uint32_t request);
uint32_t ArmSequence_OwnsHome(void);''')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=s.replace('#include "lift_config.h"','#include "lift_config.h"\n#include "lift_home.h"')
s=s.replace('volatile uint32_t g_arm_test_lift_pulses=5000U;', 'volatile uint32_t g_arm_test_lift_pulses=5000U;\nvolatile uint32_t g_arm_sequence_mode=0U,g_arm_return_down_pulses=0U;')
s=s.replace('static uint32_t s_busy,','static uint32_t s_mode,s_home_requested,s_home_expected;\nstatic uint32_t s_busy,')
s=s.replace('static uint32_t LiftBusy(void)', '''uint32_t ArmSequence_OwnsHome(void)
{return s_busy && s_mode==1U && g_arm_sequence_state==3U && s_home_requested;}
uint32_t ArmSequence_AllowHomeRequest(uint32_t request)
{return (!s_busy || request==2U || (s_home_expected==1U && request==1U))?1U:0U;}
static uint32_t HomeStopUnknown(void)
{return g_lift_home_state==9U &&
    (g_lift_home_abort_result!=1U || g_lift_home_stop_result!=1U);}
static uint32_t LiftBusy(void)''')
s=s.replace('s_expected=0U;}','s_expected=0U;s_home_expected=0U;s_home_requested=0U;}')
s=s.replace('    Gripper_CancelControlled();\n    if(LiftBusy()', '''    Gripper_CancelControlled();
    if(LiftHome_IsBusy() || g_lift_home_request==1U || HomeStopUnknown())
        g_lift_home_request=2U;
    if(LiftBusy()''',1)
s=s.replace('    s_expected=0U;g_arm_grip_confirm', '    s_expected=0U;s_home_expected=0U;g_arm_grip_confirm')
s=s.replace('if(LiftBusy() || (g_lift_motion_state==9U && g_lift_stop_result!=1U))return;', 'if(LiftBusy() || LiftHome_IsBusy() || g_lift_home_request!=0U || HomeStopUnknown() ||\n           (g_lift_motion_state==9U && g_lift_stop_result!=1U))return;')
s=s.replace('g_lift_motion_request!=0U || g_arm_sequence_enable', 'g_lift_motion_request!=0U || g_lift_home_request!=0U || LiftHome_IsBusy() ||\n           g_lift_home_state==9U || g_arm_sequence_enable')
s=s.replace('        s_pulses=g_arm_test_lift_pulses;\n        if(s_pulses==0U || s_pulses>10000U || s_pulses>LIFT_DOWN_MAX_PULSES)', '''        s_mode=g_arm_sequence_mode;
        s_pulses=s_mode==1U?g_arm_return_down_pulses:g_arm_test_lift_pulses;
        if(s_mode>1U || s_pulses==0U || (s_mode==0U && s_pulses>10000U) || s_pulses>LIFT_DOWN_MAX_PULSES)''')
s=s.replace('s_busy=1U;s_started=now;s_last=now;s_expected=0U;', 's_busy=1U;s_started=now;s_last=now;s_expected=0U;s_home_requested=0U;s_home_expected=0U;')
s=s.replace('{g_arm_grip_confirm=0U;LiftStart(2U,now);}', '''{
            g_arm_grip_confirm=0U;
            if(s_mode==1U)
            {s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(3U,now);}
            else LiftStart(2U,now);
        }''')
s=s.replace('    case 3U: case 5U:\n', '''    case 3U: case 5U:
        if(s_mode==1U && g_arm_sequence_state==3U)
        {
            if(LiftHome_IsBusy()){s_seen=1U;s_home_expected=0U;}
            if(g_lift_home_state==9U || g_lift_home_state==10U){Fail(6U,9U);break;}
            if(s_seen && g_lift_home_state==8U && g_lift_home_result==1U && g_lift_home_reported==1U)
            {s_home_expected=0U;s_home_requested=0U;Phase(4U,now);}
            else if(now-s_phase>15000U)Fail(7U,9U);
            break;
        }
''')
s=s.replace('{Restore();s_busy=0U;g_arm_sequence_enable=0U;', '{Restore();s_busy=0U;g_arm_sequence_enable=0U;if(s_mode==1U)g_arm_grip_height_ready=0U;')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/main.c';b=p.read_bytes()
old=b'        if(g_lift_home_request!=2U)g_lift_home_request=0U;'
assert b.count(old)==1
b=b.replace(old,b'        if(!ArmSequence_AllowHomeRequest(g_lift_home_request))g_lift_home_request=0U;')
needle=b'        Gripper_CancelControlled();\n        if((g_lift_motion_state'
assert b.count(needle)==1
b=b.replace(needle,b'        Gripper_CancelControlled();\n        if(LiftHome_IsBusy() || g_lift_home_request==1U)g_lift_home_request=2U;\n        if((g_lift_motion_state')
needle=b'                (Carrier_IsBusy() || DM_CAN2_JogIsBusy() || LiftHome_IsBusy() ||\n'
assert b.count(needle)==1
b=b.replace(needle,b'                (Carrier_IsBusy() || DM_CAN2_JogIsBusy() || (LiftHome_IsBusy() && !ArmSequence_OwnsHome()) ||\n')
b=b.replace(b'g_extension_home_state==9U || g_lift_home_state==9U)?1U:0U,\n',b'g_extension_home_state==9U || (g_lift_home_state==9U && !ArmSequence_OwnsHome()))?1U:0U,\n')
p.write_bytes(b)
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
save('manifest.json',{'date':'2026-10-31','files':[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in FILES],'protected':protected,'stage_build_pass':False,'assistant_flash':False,'assistant_hardware_commands':False})
print('Prepared 3-file change, protected',len(protected),'other sources/project files.')
