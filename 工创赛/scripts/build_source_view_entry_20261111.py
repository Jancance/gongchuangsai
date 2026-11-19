"""Build the isolated view candidate; no flash."""
from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parents[1];P=R/'tmp/source_view_entry_20261111/project'
E=R/'evidence/integration/2026-11-11/source-view-entry'
m=json.loads((E/'candidate.json').read_text());assert json.loads((E/'checks.json').read_text())['passed']
m['images']={}
for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
    for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
    log=E/(target+'-build.log')
    p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
    data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-900:],flush=True)
    assert p.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
    f=P/'Objects'/target/(target+'.axf')
    m['images'][target]=dict(axf=f.relative_to(R).as_posix(),sha256=hashlib.sha256(f.read_bytes()).hexdigest())
B=R/'tmp/pzq_release50_20261111/project'
m['changes']={}
for f in P.rglob('*'):
    if not f.is_file() or any(n in ('Objects','Listings') for n in f.relative_to(P).parts):continue
    rel=f.relative_to(P);before=B/rel
    if not before.exists() or before.read_bytes()!=f.read_bytes():
        m['changes'][rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
assert set(m['changes'])=={'CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/source_view_entry.inc'}
(E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
