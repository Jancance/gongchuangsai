"""Build current isolated candidate, never flash or open devices."""
from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parents[1];P=R/'tmp/source_station_offline_20261109/project'
E=R/'evidence/integration/2026-11-09/source-station-offline';m=json.loads((E/'candidate.json').read_text())
m['images']={}
for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
 for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
 log=E/(target+'-build.log');r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
 data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-2700:],flush=True)
 assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data,str(log)
 f=P/'Objects'/target/(target+'.axf');m['images'][target]={'path':f.relative_to(R).as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()}
base=R/'handoff/pzq/project';m['changed']={}
for p in P.rglob('*'):
 if not p.is_file() or 'Objects' in p.relative_to(P).parts or 'Listings' in p.relative_to(P).parts:continue
 old=base/p.relative_to(P)
 if not old.exists() or old.read_bytes()!=p.read_bytes():m['changed'][p.relative_to(P).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
