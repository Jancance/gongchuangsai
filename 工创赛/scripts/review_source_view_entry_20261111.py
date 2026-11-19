"""Audit the completed source/build scope without rebuilding or opening devices."""
from pathlib import Path
import difflib,hashlib,json
R=Path(__file__).resolve().parents[1];B=R/'tmp/pzq_release50_20261111/project'
P=R/'tmp/source_view_entry_20261111/project';E=R/'evidence/integration/2026-11-11/source-view-entry'
m=json.loads((E/'candidate.json').read_text());changes={};patch=[]
for f in P.rglob('*'):
    if not f.is_file() or any(n in ('Objects','Listings') for n in f.relative_to(P).parts):continue
    rel=f.relative_to(P);before=B/rel
    if not before.exists() or before.read_bytes()!=f.read_bytes():
        changes[rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
        patch.extend(difflib.unified_diff(before.read_text(encoding='utf-8').splitlines(True) if before.exists() else [],
                     f.read_text(encoding='utf-8').splitlines(True),fromfile='pzq50/'+rel.as_posix(),tofile='view-entry/'+rel.as_posix()))
assert set(changes)=={'CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/source_view_entry.inc'}
assert (P/'CHASSIS/USER/integration13_immediate.inc').read_bytes()==(B/'CHASSIS/USER/integration13_immediate.inc').read_bytes()
assert (P/'CHASSIS/USER/source_view_entry.inc').read_bytes()==(R/'software_interfaces/source_station/source_view_entry.inc').read_bytes()
for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
    assert b'0 Error(s), 0 Warning(s)' in (E/(target+'-build.log')).read_bytes()
    image=m['images'][target];assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
m['changes']=changes;(E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n')
(E/'source-diff.patch').write_text(''.join(patch),encoding='utf-8')
print('PASS: exact two-file integration scope; pzq50 scheduler identical; both successful builds and AXF hashes verified. No devices opened.')
