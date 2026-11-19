"""Prepare an isolated supervised view entry. Never connect or flash."""
from pathlib import Path
import hashlib, json, shutil
R=Path(__file__).resolve().parents[1]
B=R/'tmp/pzq_release50_20261111/project'
P=R/'tmp/source_view_entry_20261111/project'
E=R/'evidence/integration/2026-11-11/source-view-entry'
def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert not P.exists(), 'Preserve existing candidate'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    u=P/'CHASSIS/USER'
    shutil.copy2(R/'software_interfaces/source_station/source_view_entry.inc',u/'source_view_entry.inc')
    p=u/'integration10_complete.inc';data=p.read_bytes()
    old=b'#include "integration13_immediate.inc"'
    assert data.count(old)==1
    data=data.replace(old,old+b'\r\n#include "source_view_entry.inc"',1)
    old=b' uint32_t request=g_it_request,now=(uint32_t)xTaskGetTickCount();'
    assert data.count(old)==1
    data=data.replace(old,b' uint32_t request,now=(uint32_t)xTaskGetTickCount();\r\n SourceView_Service(now);request=g_it_request;',1)
    p.write_bytes(data)
    # Existing mechanical scheduler and all its timing remain byte-identical.
    assert (u/'integration13_immediate.inc').read_bytes()==(B/'CHASSIS/USER/integration13_immediate.inc').read_bytes()
    changed={}
    for f in P.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(P);before=B/rel
        if not before.exists() or before.read_bytes()!=f.read_bytes():
            changed[rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
    assert set(changed)=={'CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/source_view_entry.inc'}
    (E/'candidate.json').write_text(json.dumps(dict(base=str(B),changes=changed,hardware_access=False,
        deployed=False,body_motion_enabled=False,metric_alignment_verified=False),indent=2)+'\n')
    print('Isolated view-entry candidate prepared; original pzq50 unchanged.')
if __name__=='__main__':main()
