"""Prepare isolated bounded alignment entry after local MATLAB validation."""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1];B=R/'tmp/source_body_hold_20261111/project'
P=R/'tmp/source_alignment_20261112/project';E=R/'evidence/integration/2026-11-12/source-align'

def main():
    assert json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))['passed']
    assert not P.exists(),'preserve existing candidate'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*'))
    U=P/'CHASSIS/USER'
    data=(R/'software_interfaces/source_station/source_body_alignment_20261112.inc').read_text(encoding='utf-8')
    (U/'source_body_step.inc').write_bytes(data.replace('\r\n','\n').replace('\n','\r\n').encode('ascii'))
    p=U/'source_body_step.h';data=p.read_bytes();assert data.count(b'#endif')==1
    p.write_bytes(data.replace(b'#endif',b'uint32_t SourceBodyStep_SmallDistanceWindow(void);\r\n#endif'))
    p=U/'motion_route_trial.h';data=p.read_bytes();old=b'mm>=VEHICLE_MIN_ROUTE_CM*10.0f'
    assert data.count(old)==1
    data=data.replace(b'#include "motion_profile.h"',b'#include "motion_profile.h"\r\n#include "source_body_step.h"')
    p.write_bytes(data.replace(old,b'mm>=(SourceBodyStep_SmallDistanceWindow()?1.0f:VEHICLE_MIN_ROUTE_CM*10.0f)'))
    changes={}
    for f in P.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(P);old=B/rel
        if not old.exists() or old.read_bytes()!=f.read_bytes():changes[rel.as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
    assert set(changes)=={'CHASSIS/USER/source_body_step.inc','CHASSIS/USER/source_body_step.h','CHASSIS/USER/motion_route_trial.h'}
    (E/'candidate.json').write_text(json.dumps(dict(base=B.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),changes=changes,
        hardware_access=False,flashed=False,matlab_prior_to_implementation=True,mechanical_scheduler_unchanged=True,
        scanner_and_screen_unchanged=True,normal_route_lower_limit_unchanged=True),indent=2),encoding='utf-8')
    print('Prepared isolated alignment; only3 owner/route files changed, original pzq/STOP/health predicates preserved.')
if __name__=='__main__':main()
