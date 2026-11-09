"""用户要求单次105000源下降；仅准备/构建/安装，不下载或发动作。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=ROOT/'tmp/arm_source_single_20261102/project'
EV=ROOT/'evidence/p2-arm/2026-11-02/source-pick-single'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h','HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):(EV/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def once(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b,1)
def build(project,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(project/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    t=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',t),t[-4000:]
    print(t[-750:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for n,v in m['after'].items():assert sha(S/n)==v,n
    if '--build' in sys.argv:
        build(S,EV/'stage-build.log');m['stage_build_pass']=True
    else:
        h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
        low=json.loads((EV/'lift-review.json').read_text(encoding='utf-8'))
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']==m['after'] and low['passed']
        assert low['source_sha256']==m['after']
        for n,v in m['before'].items():assert sha(A/n)==v,n
        for n,v in m['protected'].items():assert sha(A/n)==v,n
        for n in FILES:shutil.copyfile(S/n,A/n)
        m['installed']=True;save('manifest.json',m)
        build(A,EV/'actual-build.log')
        for n,v in m['after'].items():assert sha(A/n)==v,n
        for n,v in m['protected'].items():assert sha(A/n)==v,n
        m['actual_build_pass']=True
    save('manifest.json',m);raise SystemExit(0)
assert not S.exists()
profile=json.loads((ROOT/'config/p2_source_pick_height_20261102.json').read_text(encoding='utf-8'))
assert profile['height_user_accepted'] and profile['source_pick_down_total_virtual_pulses']==105000
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,p)
p=S/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=once(s,'static const uint32_t s_source_down_segments[6]={5000U,40000U,30000U,10000U,10000U,10000U};\nstatic uint32_t s_source_index;',
       'static const uint32_t s_source_down_pulses=105000U;')
s=once(s,'s_stop_count=g_gripper_signal_stop_count;s_source_index=0U;',
       's_stop_count=g_gripper_signal_stop_count;')
s=once(s,'ReplayLift(1U,s_source_down_segments[0],30U,now);return;',
       'ReplayLift(1U,s_source_down_pulses,30U,now);return;')
s=once(s,'''            g_arm_source_down_done_pulses+=s_source_down_segments[s_source_index];
            s_source_index++;
            if(s_source_index<6U)ReplayLift(1U,s_source_down_segments[s_source_index],30U,now);
            else {s_ticket=0U;Phase(31U,now);}''',
       '''            g_arm_source_down_done_pulses=s_source_down_pulses;
            s_ticket=0U;Phase(31U,now);''')
p.write_text(s,encoding='utf-8')
p=S/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'只按本次示教105000脉冲自动分段下降后闭合140度',
       '只按本次示教105000脉冲单条X位置命令下降后闭合140度')
p.write_text(s,encoding='utf-8')
p=S/'HARDWARE/LIFT/lift_config.h';s=p.read_text(encoding='utf-8')
s=once(s,'/* 夹取前下降：可调脉冲，默认仍5000；50000只是粗估30mm上限。 */\n#define LIFT_DOWN_MAX_PULSES  50000U',
       '/* 用户2026-11-02要求：放行已示教105000等效脉冲的单条X下降；默认步长仍5000。 */\n#define LIFT_DOWN_MAX_PULSES  105000U')
p.write_text(s,encoding='utf-8')
p=S/'HARDWARE/LIFT/lift.h';s=p.read_text(encoding='utf-8')
s=once(s,'请求1向下使用：1~50000','请求1向下使用：1~105000')
p.write_text(s,encoding='utf-8')
protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file()
           and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in FILES}
save('manifest.json',dict(before={n:sha(A/n) for n in FILES},after={n:sha(S/n) for n in FILES},
                         protected=protected,stage_build_pass=False,actual_build_pass=False,installed=False,
                         default_mode=0,default_enable=0,default_top_start_ready=0,
                         source_total_pulses=105000,position_commands_per_source_descent=1,
                         down_cap_before=50000,down_cap_after=105000,
                         flash=False,hardware_commands=False))
print('Prepared single X source move in four files; protected other source/project files:',len(protected))
