"""Observation220 at initial request6 and completed slot1/2 return; no hardware."""
from pathlib import Path
import hashlib
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_alignment30_release300_20261116/project'
P=R/'tmp/source_alignment30_release300_observe220_20261116/project'
E=R/'evidence/integration/2026-11-16/alignment30-release300-observe220'


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['local_matlab'] and proof['observation_open_deg']==220
    assert proof['all_source_observation_visits_220'] and proof['slot3_park_angle_unchanged']
    assert not P.exists(),'Preserve previous candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    file=P/'CHASSIS/USER/integration10_complete.inc';data=file.read_bytes()
    old=b'jiazhua(request==5U?140.0f:210.0f)';new=b'jiazhua(request==5U?140.0f:220.0f)'
    assert data.count(old)==1
    file.write_bytes(data.replace(old,new,1))
    file=P/'CHASSIS/USER/pzq_commit_overlap.inc';data=file.read_bytes()
    old=b' case 7U:\r\n  Low_PublishLift(1U);g_low_state=2U;g_low_phase=10U;'
    new=b' case 7U:\r\n  if(low_slot<3U && jiazhua(220.0f)!=1U){Low_Fail(7U);return;}\r\n  Low_PublishLift(1U);g_low_state=2U;g_low_phase=10U;'
    assert data.count(old)==1;file.write_bytes(data.replace(old,new,1))
    changes={}
    for file in P.rglob('*'):
        if not file.is_file():continue
        rel=file.relative_to(P);data=file.read_bytes();old=(BASE/rel).read_bytes()
        if data!=old:
            assert data.count(b'\n')-data.count(b'\r\n')==old.count(b'\n')-old.count(b'\r\n')
            changes[rel.as_posix()]=hashlib.sha256(data).hexdigest()
    assert set(changes)=={'CHASSIS/USER/integration10_complete.inc','CHASSIS/USER/pzq_commit_overlap.inc'}
    (E/'prepared.json').write_text(json.dumps(dict(changes=changes,observation_open_deg=220,
        observation_nominal_pulse_us=2130,preclose_deg=170,close_deg=140,
        carrier_open_deg=210,carrier_release_ms=300,source_step_max_mm=30,
        source_alignment_speed_mm_s=45,all_source_observation_visits_220=True,
        slot3_park_angle_unchanged=True,pi_changed=False,hardware_access=False,flashed=False),indent=2)+'\n',encoding='utf-8')
    print('Prepared observation220 initial request6 + completed slot1/2 returns only; no deployment')


if __name__=='__main__':main()
