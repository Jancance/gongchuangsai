"""源侧一键下降夹取模式4；仅准备/构建/安装源码，不下载或发硬件命令。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=ROOT/'tmp/arm_source_pick_20261102/project'
EV=ROOT/'evidence/p2-arm/2026-11-02/source-pick-mode4'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h']
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
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']==m['after']
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
assert profile['source_pick_down_segments_virtual_pulses']==[5000,40000,30000,10000,10000,10000]
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,p)
p=S/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
fragment=(ROOT/'handoff/arm_source_pick_20261102/source-pick.c.fragment').read_text(encoding='utf-8')
s=once(s,'void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)',
       fragment+'\nvoid ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)')
s=once(s,'    uint32_t request=g_arm_sequence_request,result;',
       '    uint32_t request=g_arm_sequence_request,result;\n    if((s_busy && s_mode==4U) || (!s_busy && g_arm_sequence_mode==4U))\n    {SourcePickService(ready,other_busy,now);return;}')
p.write_text(s,encoding='utf-8')
p=S/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'#endif','''/* 模式4：只按本次示教105000脉冲自动分段下降后闭合140度。
 * 不转DM/转盘、不伸缩、不回零、不抬升；默认关闭，顶部且已对中才置top_start_ready。
 * mode4/enable1/top_start_ready1后request1一次；request2中止，停稳后request3清锁存。
 * state30下降，31闭合；7/1只是软件完成，真实夹住需现场确认。 */
extern volatile uint32_t g_arm_source_down_done_pulses;
#endif''')
p.write_text(s,encoding='utf-8')
protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file()
           and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in FILES}
save('manifest.json',dict(before={n:sha(A/n) for n in FILES},after={n:sha(S/n) for n in FILES},
                         protected=protected,stage_build_pass=False,actual_build_pass=False,installed=False,
                         default_mode=0,default_enable=0,default_top_start_ready=0,
                         source_total_pulses=105000,segments=[5000,40000,30000,10000,10000,10000],
                         low_level_down_cap_unchanged=True,flash=False,hardware_commands=False))
print('Prepared mode4 in two files; protected other source/project files:',len(protected))
