"""Actual owner regressions and dual Keil build of the requested isolated change."""
import json,shutil
from pathlib import Path
import build_integration18_ext150_20261115 as build
import build_integration23_sync_20261115 as prior
from prepare_stopped3_grip170_release200_20261115 import R,BASE,P,E

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    previous=R/'evidence/integration/2026-11-15/integration23-sync'
    for f in previous.iterdir():
        if f.suffix in ('.c','.h','.inc','.txt') or f.name=='parameter-and-poll-checks.json':
            data=f.read_bytes()
            if f.suffix=='.c':
                s=data.decode('utf-8').replace('angle==160','angle==170')
                if f.name in ('all-regions-owner.c','base-commit-fixture.c'):
                    s=s.replace('tick+=99;','tick+=199;').replace('g_low_release_wait_ms==100','g_low_release_wait_ms==200')
                s=s.replace('pulse+100','pulse+200').replace('100ms release','200ms release')
                if f.name=='static-cycle-fixture.c':
                    s=s.replace('if(g_low_phase==6){tick=pulse_apply_at+50;Low_Service(tick);tick+=100;',
                                'if(g_low_phase==6){tick=pulse_apply_at+50;Low_Service(tick);tick+=200;')
                if f.name=='owner-faults.c':s=s.replace('tick+=100U;','tick+=200U;')
                data=s.encode('utf-8')
            (E/f.name).write_bytes(data)
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(
        b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
    assert (P/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes()==(BASE/'CHASSIS/USER/vehicle_source_parallel.inc').read_bytes()
    prior.P=P;prior.E=E
    for name in ('final-park-owner','owner-faults'):
        prior.run(name,(E/(name+'.c')).read_text(encoding='utf-8'))
    source=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')")
    source=source.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'stopped3_build','__file__':build.__file__}
    exec(compile(source,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    f=E/'checks.json';d=json.loads(f.read_text());d.update(
        preclose_deg=170,carrier_release_ms=200,required_stable_frames=3,
        physical_stopped_truth_validated=False,physical_sweep_validated=False,
        source_extra_wait_ms=0,synchronized_backend_byte_identical=True)
    f.write_text(json.dumps(d,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
