"""顶部起步单件受控调试组；先副本验证，显式安装；不烧录或发硬件命令。"""
from pathlib import Path
import hashlib,json,shutil,subprocess,re,sys
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_top_start_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-top-start'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v): (EV/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    s=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',s),s[-4000:]
    print(s[-800:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for e in m['files']: assert sha(STAGE/e['file'])==e['after']
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');m['stage_build_pass']=True;save('manifest.json',m)
    else:
        h=json.loads((EV/'host-review.json').read_text())
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']=={e['file']:e['after'] for e in m['files']}
        for n,h in m['protected'].items(): assert sha(ACTUAL/n)==h,n
        for e in m['files']: assert sha(ACTUAL/e['file'])==e['before'],e['file']
        for e in m['files']: shutil.copyfile(STAGE/e['file'],ACTUAL/e['file'])
        save('installed.json',m);build(ACTUAL,EV/'actual-build.log')
        for e in m['files']: assert sha(ACTUAL/e['file'])==e['after'],e['file']
        for n,h in m['protected'].items(): assert sha(ACTUAL/n)==h,n
        m['actual_build_pass']=True;save('installed.json',m)
    raise SystemExit(0)
assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ACTUAL/n,p)
def replace_once(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b,1)
p=STAGE/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=replace_once(s,'extern volatile uint32_t g_arm_return_down_pulses;', '''extern volatile uint32_t g_arm_return_down_pulses;
/* 模式2：从人工确认的顶部开始，下降夹取、有限抬升、低速回顶、下放。 */
/* 顶部准入接单即消耗；高度/配置改变后必须重新人工确认。默认0。 */
extern volatile uint32_t g_arm_top_start_ready;
extern volatile uint32_t g_arm_pick_down_pulses;
extern volatile uint32_t g_arm_top_approach_pulses;
/* 到state12且实际已落稳才写1；进入该状态会清除旧值。 */
extern volatile uint32_t g_arm_place_confirm;''')
s=s.replace('/* 0空闲，1闭合，2等待夹住确认，3抬升，4短暂停留，5放回，6张开，7软件完成，8中止，9失败。 */', '/* 0空闲，1闭合，2夹住确认，3抬升，4顶部短停，5放回，6张开，7结束，8中止，9失败；模式2另有10首次下降、11低速回顶、12落稳确认、13准备张开。 */')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=replace_once(s,'static uint32_t s_mode,s_home_requested,s_home_expected;', '''volatile uint32_t g_arm_top_start_ready=0U,g_arm_pick_down_pulses=0U;
volatile uint32_t g_arm_top_approach_pulses=0U,g_arm_place_confirm=0U;
static uint32_t s_pick,s_approach,s_return;
static uint32_t s_mode,s_home_requested,s_home_expected;''')
s=replace_once(s,'return s_busy && s_mode==1U && g_arm_sequence_state==3U && s_home_requested;', 'return s_busy && s_home_requested && ((s_mode==1U && g_arm_sequence_state==3U) || (s_mode==2U && g_arm_sequence_state==11U));')
s=replace_once(s,'    g_arm_sequence_state=state;g_arm_sequence_result=result;', '    g_arm_top_start_ready=0U;g_arm_place_confirm=0U;\n    g_arm_sequence_state=state;g_arm_sequence_result=result;')
s=replace_once(s,'g_arm_grip_confirm=0U;g_arm_grip_height_ready=0U;g_arm_sequence_state=0U;', 'g_arm_grip_confirm=0U;g_arm_grip_height_ready=0U;g_arm_top_start_ready=0U;g_arm_place_confirm=0U;g_arm_sequence_state=0U;')
s=replace_once(s,'g_lift_home_state==9U || g_arm_sequence_enable!=1U || g_arm_grip_height_ready!=1U)', '''g_lift_home_state==9U || g_arm_sequence_enable!=1U ||
           (g_arm_sequence_mode==2U?g_arm_top_start_ready!=1U:g_arm_grip_height_ready!=1U))''')
s=replace_once(s,'''        s_pulses=s_mode==1U?g_arm_return_down_pulses:g_arm_test_lift_pulses;
        if(s_mode>1U || s_pulses==0U || (s_mode==0U && s_pulses>10000U) || s_pulses>LIFT_DOWN_MAX_PULSES)''', '''        s_pick=g_arm_pick_down_pulses;s_approach=g_arm_top_approach_pulses;s_return=g_arm_return_down_pulses;
        s_pulses=s_mode==0U?g_arm_test_lift_pulses:s_return;
        if(s_mode>2U || s_pulses==0U || (s_mode==0U && s_pulses>10000U) || s_pulses>LIFT_DOWN_MAX_PULSES ||
           (s_mode==2U && (s_pick==0U || s_pick>LIFT_DOWN_MAX_PULSES || s_approach==0U ||
                           s_approach>=s_pick || s_approach>LIFT_UP_MAX_PULSES)))''')
s=replace_once(s,'        Phase(1U,now);s_ticket=Gripper_ControlledCommand(140.0f);', '''        g_arm_place_confirm=0U;
        if(s_mode==2U)
        {g_arm_top_start_ready=0U;g_arm_grip_height_ready=0U;Phase(13U,now);s_ticket=Gripper_ControlledCommand(180.0f);}
        else {Phase(1U,now);s_ticket=Gripper_ControlledCommand(140.0f);}''')
s=replace_once(s,'g_arm_sequence_enable!=1U || g_arm_grip_height_ready!=1U || g_gripper_signal_stop_count', 'g_arm_sequence_enable!=1U || (s_mode!=2U && g_arm_grip_height_ready!=1U) || g_gripper_signal_stop_count')
s=replace_once(s,'            else LiftStart(2U,now);', '            else {if(s_mode==2U)s_pulses=s_approach;LiftStart(2U,now);}')
s=replace_once(s,'    case 3U: case 5U:\n        if(s_mode==1U && g_arm_sequence_state==3U)', '    case 3U: case 5U: case 10U: case 11U:\n        if((s_mode==1U && g_arm_sequence_state==3U) || (s_mode==2U && g_arm_sequence_state==11U))')
s=replace_once(s,'{s_home_expected=0U;s_home_requested=0U;Phase(4U,now);}', '{s_home_expected=0U;s_home_requested=0U;if(s_mode==2U)s_pulses=s_return;Phase(4U,now);}')
s=replace_once(s,'''            if(g_arm_sequence_state==3U)Phase(4U,now);
            else {Phase(6U,now);s_ticket=0U;}''', '''            if(s_mode==2U && g_arm_sequence_state==10U)
            {Phase(1U,now);s_ticket=Gripper_ControlledCommand(140.0f);}
            else if(s_mode==2U && g_arm_sequence_state==3U)
            {s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(11U,now);}
            else if(g_arm_sequence_state==3U)Phase(4U,now);
            else if(s_mode==2U){g_arm_place_confirm=0U;Phase(12U,now);}
            else {Phase(6U,now);s_ticket=0U;}''')
s=replace_once(s,'    case 6U:\n', '''    case 12U:
        if(g_arm_place_confirm==1U)
        {g_arm_place_confirm=0U;Phase(6U,now);s_ticket=0U;}
        else if(now-s_phase>15000U)Fail(7U,9U);
        break;
    case 13U:
        if(!s_ticket)
        {s_ticket=Gripper_ControlledCommand(180.0f);if(!s_ticket){if(now-s_phase>1500U)Fail(5U,9U);break;}}
        result=Gripper_ControlledResult(s_ticket);
        if(result==2U){Fail(5U,9U);break;}
        if(result==1U && !s_settling){s_settling=1U;s_phase=now;}
        if(s_settling && now-s_phase>=500U)
        {s_pulses=s_pick;LiftStart(1U,now);Phase(10U,now);}
        else if(!s_settling && now-s_phase>1500U)Fail(7U,9U);
        break;
    case 6U:
''')
s=replace_once(s,'if(s_mode==1U)g_arm_grip_height_ready=0U;', 'if(s_mode!=0U)g_arm_grip_height_ready=0U;g_arm_top_start_ready=0U;g_arm_place_confirm=0U;')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/main.c';b=p.read_bytes()
b=replace_once(b,b'ArmSequence_IsBusy() && g_arm_sequence_state>=1U && g_arm_sequence_state<=6U &&', b'ArmSequence_IsBusy() && g_arm_sequence_state!=8U && g_arm_sequence_state!=9U &&')
p.write_bytes(b)
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
save('manifest.json',{'date':'2026-10-31','files':[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in FILES],'protected':protected,'stage_build_pass':False,'flash':False,'hardware_commands':False})
print('Prepared3-file top-start mode2; other source/project files protected:',len(protected))
