"""Prepare the two authorized local deltas after both local MATLAB proofs."""
from pathlib import Path
import hashlib
import json
import shutil

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_stopped5_fastalign_20261116/project'
P=R/'tmp/source_alignment30_release300_20261116/project'
E=R/'evidence/integration/2026-11-16/source-alignment30-release300'


def replace(path,before,after):
    data=path.read_bytes();assert data.count(before)==1,str(path)
    path.write_bytes(data.replace(before,after,1))


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    release=json.loads((R/'evidence/integration/2026-11-16/carrier-release300/matlab-result.json').read_text(encoding='utf-8'))
    assert proof['validation_passed'] and proof['local_matlab'] and release['passed'] and release['local_matlab']
    assert release['carrier_release_ms']==300 and not P.exists(),'Preserve older candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    replace(P/'CHASSIS/USER/source_mcu_cycle.inc',b'labs((long)amount)>=20000L?20U:',b'labs((long)amount)>=30000L?30U:')
    replace(P/'CHASSIS/USER/source_body_step.inc',b'axis1..20mm at15mm/s',b'axis1..30mm; source chain45mm/s, legacy15mm/s')
    replace(P/'CHASSIS/USER/source_body_step.inc',b'g_source_align_distance_mm<=20.0f',b'g_source_align_distance_mm<=30.0f')
    replace(P/'USER/vehicle_parameters.h',b'#define SOURCE_RELEASE_DWELL_MS 200U',b'#define SOURCE_RELEASE_DWELL_MS 300U')
    changed={}
    for f in P.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(P);old=(BASE/rel).read_bytes();data=f.read_bytes()
        if data!=old:
            assert data.count(b'\n')-data.count(b'\r\n')==old.count(b'\n')-old.count(b'\r\n')
            changed[rel.as_posix()]=hashlib.sha256(data).hexdigest()
    assert set(changed)=={'CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/source_body_step.inc','USER/vehicle_parameters.h'}
    (E/'prepared.json').write_text(json.dumps(dict(changes=changed,source_step_max_mm=30,
        source_alignment_speed_mm_s=45,carrier_release_ms=300,source_extra_wait_ms=0,
        matlab_alignment_checks=proof['checks'],matlab_release_checks=release['checks'],
        hardware_access=False,flashed=False,pi_changed=False),indent=2)+'\n',encoding='utf-8')
    print('Prepared alignment cap30mm and applied-release300ms only; existing45mm/s unchanged; no deployment')


if __name__=='__main__':main()
