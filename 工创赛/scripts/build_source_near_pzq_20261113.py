"""Rebuild isolated one-step candidate; no flash or connected devices."""
from pathlib import Path
import argparse,difflib,hashlib,json,subprocess
R=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--base');parser.add_argument('--project');parser.add_argument('--evidence')
args=parser.parse_args()
B=R/(args.base or 'tmp/pzq_release50_20261111/project')
P=R/(args.project or 'tmp/source_body_step_20261111/project');E=R/(args.evidence or 'evidence/integration/2026-11-11/source-body-step')
assert json.loads((E/'checks.json').read_text())['passed']
m=json.loads((E/'candidate.json').read_text());m['images']={}
for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
    for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
    log=E/(target+'-build.log')
    p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
    data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-1100:],flush=True)
    assert p.returncode==0 and b'0 Error(s), 0 Warning(s)' in data,str(log)
    f=P/'Objects'/target/(target+'.axf')
    m['images'][target]=dict(axf=f.relative_to(R).as_posix(),sha256=hashlib.sha256(f.read_bytes()).hexdigest())
changes={};patch=[]
for f in P.rglob('*'):
    rel=f.relative_to(P)
    if not f.is_file() or any(n in ('Objects','Listings') for n in rel.parts):continue
    before=B/rel
    if not before.exists() or before.read_bytes()!=f.read_bytes():
        changes[rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
        def decode(data):
            try:return data.decode('utf-8')
            except UnicodeDecodeError:return data.decode('gb18030')
        patch.extend(difflib.unified_diff(decode(before.read_bytes()).splitlines(True) if before.exists() else [],
            decode(f.read_bytes()).splitlines(True),fromfile=B.parent.name+'/'+rel.as_posix(),tofile=P.parent.name+'/'+rel.as_posix()))
assert set(changes)==set(m['changes'])
# The owner delta is intentional; low-level motion backend must remain identical.
for rel in ['CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/lift_ruler_calibration.c','CHASSIS/SYSTEM/SAFETY/safety.c']:
    assert (P/rel).read_bytes()==(B/rel).read_bytes()
m['changes']=changes;m['build_passed']=True
(E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
(E/'source-diff.patch').write_text(''.join(patch),encoding='utf-8')
print('Two builds passed; near owner and guard delta only; low-level motion backend unchanged; not flashed.')
