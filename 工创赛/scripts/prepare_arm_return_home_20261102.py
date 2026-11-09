"""用户指定102000预升再回零；仅改动作协调层，不改底层参数，不烧录。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/arm_return_home_20261102/project'
E=R/'evidence/p2-arm/2026-11-02/source-return-home'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):(E/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def once(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b,1)
def build(p,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(p/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    t=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',t),t[-4000:]
    print(t[-750:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((E/'manifest.json').read_text(encoding='utf-8'))
    for n,v in m['after'].items():assert sha(S/n)==v,n
    if '--build' in sys.argv:
        build(S,E/'stage-build.log');m['stage_build_pass']=True
    else:
        h=json.loads((E/'host-review.json').read_text(encoding='utf-8'))
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']==m['after']
        for n,v in m['before'].items():assert sha(A/n)==v,n
        for n,v in m['protected'].items():assert sha(A/n)==v,n
        for n in FILES:shutil.copyfile(S/n,A/n)
        m['installed']=True;save('manifest.json',m)
        build(A,E/'actual-build.log')
        for n,v in m['after'].items():assert sha(A/n)==v,n
        for n,v in m['protected'].items():assert sha(A/n)==v,n
        m['actual_build_pass']=True
    save('manifest.json',m);raise SystemExit(0)
assert not S.exists()
E.mkdir(parents=True,exist_ok=True)
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=E/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,p)
p=S/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=once(s,'(s_mode==2U && g_arm_sequence_state==11U));',
       '(s_mode==2U && g_arm_sequence_state==11U) || (s_mode==3U && g_arm_sequence_state==29U) || (s_mode==5U && g_arm_sequence_state==33U));')
helpers='''static void SourceTopStart(uint32_t state,uint32_t now)
{s_seen=0U;s_home_requested=1U;s_home_expected=1U;g_lift_home_request=1U;Phase(state,now);}
static uint32_t SourceTopDone(void)
{
    if(LiftHome_IsBusy()){s_seen=1U;s_home_expected=0U;}
    if(g_lift_home_state==9U || g_lift_home_state==10U){Fail(6U,9U);return 0U;}
    if(s_seen && g_lift_home_state==8U && g_lift_home_result==1U && g_lift_home_reported==1U)
    {s_home_requested=0U;s_home_expected=0U;return 1U;}
    return 0U;
}
'''
s=once(s,'static void ReplayService(uint32_t ready,uint32_t other_busy,uint32_t now)',
       helpers+'\nstatic void ReplayService(uint32_t ready,uint32_t other_busy,uint32_t now)')
a=s.index('static void ReplayService(');b=s.index('/* 模式4',a)
part=s[a:b]
part=once(part,'if(s_pick==0U || s_pick>LIFT_DOWN_MAX_PULSES',
          'if(s_pick<=ARM_SOURCE_TOP_MARGIN_PULSES || s_pick>LIFT_DOWN_MAX_PULSES')
part=once(part,'if(ReplayGripperDone(140.0f,now))ReplayLift(2U,s_pick,24U,now);',
          'if(ReplayGripperDone(140.0f,now))ReplayLift(2U,s_pick-ARM_SOURCE_TOP_MARGIN_PULSES,24U,now);')
part=once(part,'''    case 24U:
        if(ReplayLiftDone())ReplayRotate(5U,25U,now);
        break;''', '''    case 24U:
        if(ReplayLiftDone())SourceTopStart(29U,now);
        break;
    case 29U:
        if(SourceTopDone())ReplayRotate(5U,25U,now);
        break;''')
s=s[:a]+part+s[b:]
fragment=(R/'handoff/arm_return_home_20261102/return.c.fragment').read_text(encoding='utf-8')
s=once(s,'void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)',
       fragment+'\nvoid ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)')
s=once(s,'    uint32_t request=g_arm_sequence_request,result;',
       '    uint32_t request=g_arm_sequence_request,result;\n    if((s_busy && s_mode==5U) || (!s_busy && g_arm_sequence_mode==5U))\n    {SourceReturnService(ready,other_busy,now);return;}')
p.write_text(s,encoding='utf-8')
p=S/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'#endif','''/* 源105000夹取点返回：先有限上升102000，再走原60RPM/40秒回零。
 * mode3源上升保留3000等效脉冲余量，state29回零成功后才转车载。
 * mode5只执行返回，不动夹爪/DM；从源夹取点置grip_height_ready1才可启动。
 * mode5 state32上升、33回零，7/1仅软件完成；不得在已经顶部时启动。 */
#define ARM_SOURCE_RETURN_UP_PULSES 102000U
#define ARM_SOURCE_TOP_MARGIN_PULSES 3000U
#endif''')
p.write_text(s,encoding='utf-8')
protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file()
    and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in FILES}
save('manifest.json',dict(before={n:sha(A/n) for n in FILES},after={n:sha(S/n) for n in FILES},
     protected=protected,stage_build_pass=False,actual_build_pass=False,installed=False,
     source_pick_down_pulses=105000,source_return_prehome_up_pulses=102000,
     home_rpm_unchanged=60,home_driver_timeout_ms_unchanged=40000,
     host_active_home_wait='unchanged/unlimited',flash=False,hardware_commands=False))
print('Prepared mode3 prehome margin + mode5 return; protected:',len(protected))
