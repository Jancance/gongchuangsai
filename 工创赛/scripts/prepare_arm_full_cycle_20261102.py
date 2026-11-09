"""完整固定循环，仅安装源码和构建，不自动下载或发运动。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/arm_full_cycle_20261102/project'
E=R/'evidence/p2-arm/2026-11-02/full-cycle'
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
s=once(s,'(s_mode==3U && g_arm_sequence_state==29U)',
       '(s_mode==3U && (g_arm_sequence_state==29U || g_arm_sequence_state==34U))')
s=once(s,'static uint32_t s_replay_index,s_dm_expected,s_dm_seen;',
       'static uint32_t s_dm_expected,s_dm_seen;')
s=once(s,'static const uint32_t s_replay_down[3]={16000U,8000U,4000U};\nstatic const uint32_t s_replay_up[2]={16000U,12000U};',
       'static const uint32_t s_carrier_down_pulses=28000U;\nstatic const uint32_t s_carrier_return_up_pulses=25000U;')
s=once(s,'else {s_replay_index=0U;ReplayLift(1U,s_replay_down[0],26U,now);}',
       'else ReplayLift(1U,s_carrier_down_pulses,26U,now);')
s=once(s,'''        if(ReplayLiftDone())
        {
            s_replay_index++;
            if(s_replay_index<3U)ReplayLift(1U,s_replay_down[s_replay_index],26U,now);
            else {s_ticket=0U;Phase(27U,now);}
        }''', '''        if(ReplayLiftDone()){s_ticket=0U;Phase(27U,now);}''')
s=once(s,'{s_replay_index=0U;ReplayLift(2U,s_replay_up[0],28U,now);}',
       'ReplayLift(2U,s_carrier_return_up_pulses,28U,now);')
s=once(s,'''    case 28U:
        if(ReplayLiftDone())
        {
            s_replay_index++;
            if(s_replay_index<2U)ReplayLift(2U,s_replay_up[s_replay_index],28U,now);
            else {Restore();s_busy=0U;s_dm_expected=0U;g_arm_sequence_enable=0U;
                  g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        }
        break;''', '''    case 28U:
        if(ReplayLiftDone())SourceTopStart(34U,now);
        break;
    case 34U:
        if(SourceTopDone())
        {Restore();s_busy=0U;s_dm_expected=0U;g_arm_sequence_enable=0U;
         g_arm_sequence_state=7U;g_arm_sequence_result=1U;}
        break;''')
p.write_text(s,encoding='utf-8')
p=S/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'#endif','''/* 完整模式3：源下降105000/夹紧/预升102000/回零/转车载；
 * 车载单次下降28000/松爪/预升25000/回零，state34为最终回零。
 * 最终7/1回到车载侧顶部，保持张开及DM模式2，不自动重跑。 */
#endif''')
p.write_text(s,encoding='utf-8')
protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file()
    and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in FILES}
save('manifest.json',dict(before={n:sha(A/n) for n in FILES},after={n:sha(S/n) for n in FILES},
     protected=protected,stage_build_pass=False,actual_build_pass=False,installed=False,
     source_pick_down_pulses=105000,source_return_prehome_up_pulses=102000,
     carrier_down_pulses=28000,carrier_return_prehome_up_pulses=25000,
     carrier_single_position_move=True,final_home_required=True,flash=False,hardware_commands=False))
print('Prepared complete fixed cycle; protected:',len(protected))
