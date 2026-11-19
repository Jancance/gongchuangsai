"""Build speed trial candidate, never flash/open a debugger."""
from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parents[1];P=R/'tmp/pickup_second_diag_20261110/project'
E=R/'evidence/integration/2026-11-10/second-diag';m=json.loads((E/'candidate.json').read_text(encoding='utf-8'));m['images']={}
for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
 for folder in ('Objects','Listings'):(P/folder/target).mkdir(parents=True,exist_ok=True)
 log=E/(target+'-build.log');r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
 data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-3200:],flush=True)
 assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data,str(log)
 f=P/'Objects'/target/(target+'.axf');m['images'][target]={'axf':f.relative_to(R).as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'build_log':log.relative_to(R).as_posix()}
base=R/'handoff/pzq/project';m['changed']={}
for f in P.rglob('*'):
 if not f.is_file() or any(x in f.relative_to(P).parts for x in ('Objects','Listings')):continue
 old=base/f.relative_to(P)
 if not old.exists() or old.read_bytes()!=f.read_bytes():m['changed'][f.relative_to(P).as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
(E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
