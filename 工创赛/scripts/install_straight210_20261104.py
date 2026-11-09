"""Install checked isolated straight-test files, with restorable integration sources."""
from pathlib import Path
import json,hashlib,subprocess
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-04/straight210'
m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));A=Path(m['actual']);S=Path(m['stage'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
t=json.loads((E/'after/result.json').read_text(encoding='utf-8'));assert t['passed'] and t['cases']==10
for n,h in t['source_sha256'].items():assert sha(S/n)==h,n
for name in ['Vehicle_Integrated','Vehicle_Offline']:assert '0 Error(s), 0 Warning(s)' in (E/f'stage-{name}.log').read_text(encoding='utf-8',errors='replace')
def sources(root):
 return {p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file() and p.suffix.lower() in {'.c','.h','.inc','.s','.uvprojx'} and not {'Objects','ObjectsOffline','Listings','.git'}.intersection(p.relative_to(root).parts)}
before=sources(A)
for n,h in m['original'].items():
 if h is None:assert n not in before,n
 else:assert before[n]==h,n
for n,h in m['changed'].items():assert sha(S/n)==h,n
for n in m['changed']:(A/n).write_bytes((S/n).read_bytes())
after=sources(A);protected={n:h for n,h in before.items() if n not in m['changed']}
assert all(after[n]==h for n,h in protected.items())
assert after.keys()==before.keys()|{n for n,h in m['original'].items() if h is None}
m.update(installed=True,protected_source_files_verified=len(protected));print('Installed',len(m['changed']),'files; preserved',len(protected),'source/project files',flush=True)
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
for name in ['Vehicle_Integrated','Vehicle_Offline']:
 log=E/f'actual-{name}.log';p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(A/'gongxun-f407.uvprojx'),'-t',name,'-j0','-o',str(log)],timeout=180)
 assert p.returncode==0 and '0 Error(s), 0 Warning(s)' in log.read_text(encoding='utf-8',errors='replace');print(name,'0 errors, 0 warnings',flush=True)
m['actual_builds_passed']=True;(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
