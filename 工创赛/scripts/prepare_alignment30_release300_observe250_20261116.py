"""Two observed-source opening targets230->250 only; no hardware or deployment."""
from pathlib import Path
import hashlib
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_alignment30_release300_observe230_20261116/project'
P=R/'tmp/source_alignment30_release300_observe250_20261116/project'
E=R/'evidence/integration/2026-11-16/alignment30-release300-observe250'


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['local_matlab'] and proof['all_source_observation_visits_250']
    assert not P.exists(),'Preserve previous candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    for name,before,after in [
        ('integration10_complete.inc',b'jiazhua(request==5U?140.0f:230.0f)',b'jiazhua(request==5U?140.0f:250.0f)'),
        ('pzq_commit_overlap.inc',b'if(low_slot<3U && jiazhua(230.0f)',b'if(low_slot<3U && jiazhua(250.0f)')]:
        path=P/'CHASSIS/USER'/name;data=path.read_bytes();assert data.count(before)==1
        path.write_bytes(data.replace(before,after,1))
    changes={}
    for path in P.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(P);data=path.read_bytes();old=(BASE/rel).read_bytes()
        if data!=old:
            assert len(data)==len(old) and sum(a!=b for a,b in zip(data,old))==1
            changes[rel.as_posix()]=hashlib.sha256(data).hexdigest()
    assert len(changes)==2
    (E/'prepared.json').write_text(json.dumps(dict(changes=changes,observation_open_deg=250,
        observation_nominal_pulse_us=2352,preclose_deg=170,close_deg=140,carrier_open_deg=210,
        carrier_release_ms=300,source_step_max_mm=30,source_alignment_speed_mm_s=45,
        pi_changed=False,hardware_access=False,flashed=False),indent=2)+'\n',encoding='utf-8')
    print('Prepared observation250 two one-byte changes only; no deployment')


if __name__=='__main__':main()
