from pathlib import Path
import hashlib,json,shutil,subprocess
R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
E=R/'evidence/p2-arm/2026-11-02/close-135'
files=['USER/arm_sequence.c','USER/arm_sequence.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not E.exists()
E.mkdir(parents=True)
before={n:sha(A/n) for n in files}
protected={p.relative_to(A).as_posix():sha(p) for p in A.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(A).as_posix() not in files}
for n in files:
    p=E/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(A/n,p)
c=(A/files[0]).read_text(encoding='utf-8');h=(A/files[1]).read_text(encoding='utf-8')
assert c.count('145.0f')==5 and h.count('闭合145度')==1
c=c.replace('145.0f','135.0f');h=h.replace('闭合145度','闭合135度')
(A/files[0]).write_text(c,encoding='utf-8');(A/files[1]).write_text(h,encoding='utf-8')
log=E/'actual-build.log'
r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(A/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
t=log.read_text(encoding='utf-8',errors='replace');print(t[-800:])
passed=r.returncode==0 and '0 Error(s), 0 Warning(s)' in t
for n,v in protected.items():assert sha(A/n)==v,n
(E/'manifest.json').write_text(json.dumps(dict(before=before,after={n:sha(A/n) for n in files},protected=protected,build_pass=passed,close_angle_deg=135,replacements=5,flash=False,hardware_commands=False),indent=2),encoding='utf-8')
assert passed
print('Closed angle updated to 135; protected files:',len(protected))
