from pathlib import Path
import hashlib,json,shutil,subprocess,sys
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/arm_parameters_20261102/project'
E=R/'evidence/p2-arm/2026-11-02/parameters-145-104000-27000'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(m):(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
def build(p,name):
    log=E/name
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(p/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    t=log.read_text(encoding='utf-8',errors='replace')
    print(t[-900:])
    assert r.returncode==0 and '0 Error(s), 0 Warning(s)' in t
if '--install' in sys.argv:
    m=json.loads((E/'manifest.json').read_text(encoding='utf-8'))
    FILES=list(m['after'])
    assert m['stage_build_pass'] and json.loads((E/'host-review.json').read_text())['passed']
    for n,v in m['before'].items():assert sha(A/n)==v,n
    for n,v in m['protected'].items():assert sha(A/n)==v,n
    for n,v in m['after'].items():assert sha(S/n)==v,n
    for n in FILES:shutil.copyfile(S/n,A/n)
    m['installed']=True;save(m)
    build(A,'actual-build.log')
    for n,v in m['after'].items():assert sha(A/n)==v,n
    for n,v in m['protected'].items():assert sha(A/n)==v,n
    m['actual_build_pass']=True;save(m)
    print('Installed and built; protected files:',len(m['protected']));sys.exit()
assert not S.exists() and not E.exists()
E.mkdir(parents=True)
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=E/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,p)
for n in FILES:
    p=S/n;s=p.read_text(encoding='utf-8')
    if n.endswith('.c'):
        assert s.count('140.0f')==5
        s=s.replace('140.0f','145.0f').replace('s_carrier_return_up_pulses=25000U','s_carrier_return_up_pulses=27000U').replace('102000有限上升','104000有限上升')
    else:
        s=s.replace('140度','145度').replace('102000','104000').replace('3000','1000').replace('25000','27000')
    p.write_text(s,encoding='utf-8')
m=dict(before={n:sha(A/n) for n in FILES},after={n:sha(S/n) for n in FILES},
       protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in FILES},
       installed=False,stage_build_pass=False,actual_build_pass=False,flash=False,hardware_commands=False,
       close_deg=145,source_down=105000,source_up_before_home=104000,carrier_down=28000,carrier_up_before_home=27000)
save(m)
t=(R/'scripts/check_arm_full_cycle_20261102.py').read_text(encoding='utf-8')
t=t.replace('arm_full_cycle_20261102','arm_parameters_20261102').replace("2026-11-02/full-cycle'","2026-11-02/parameters-145-104000-27000'")
t=t.replace('102000','104000').replace('25000','27000').replace('==140','==145')
t=t.replace('until(28);assert(moves==3', 'until(28);assert(angles[1]==145);assert(moves==3')
check=R/'scripts/check_arm_parameters_20261102.py';check.write_text(t,encoding='utf-8')
subprocess.run([sys.executable,str(check)],check=True)
build(S,'stage-build.log');m['stage_build_pass']=True;save(m)
print('Prepared and verified; no hardware commands')
