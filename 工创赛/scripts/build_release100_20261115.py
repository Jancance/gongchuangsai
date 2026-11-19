"""Only confirmed post-release100ms change; strict same-start remains unresolved."""
from pathlib import Path
import json,shutil,subprocess
import build_integration18_ext150_20261115 as build

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_extension_lead_pick_20261115/project'
P=R/'tmp/source_release100_20261115/project'
E=R/'evidence/integration/2026-11-15/release100-overlap'


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed'] and proof['release_dwell_ms']==100 and proof['local_matlab']
    assert not P.exists(),'Preserve existing candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    f=P/'USER/vehicle_parameters.h';data=f.read_bytes()
    old=b'#define SOURCE_RELEASE_DWELL_MS 50U'
    assert data.count(old)==1
    f.write_bytes(data.replace(old,b'#define SOURCE_RELEASE_DWELL_MS 100U',1))
    previous=R/'evidence/integration/2026-11-15/extension-lead-pick'
    for source in previous.iterdir():
        if source.suffix in ('.c','.h','.inc','.txt') or source.name=='parameter-and-poll-checks.json':
            shutil.copyfile(source,E/source.name)
    for name in ('all-regions-owner.c','base-commit-fixture.c'):
        f=E/name;s=f.read_text(encoding='utf-8')
        assert 'tick+=49;' in s and 'g_low_release_wait_ms==50' in s
        s=s.replace('tick+=49;','tick+=99;').replace('g_low_release_wait_ms==50','g_low_release_wait_ms==100').replace('pulse+50','pulse+100')
        f.write_text(s,encoding='utf-8')
    f=E/'static-cycle-fixture.c';s=f.read_text(encoding='utf-8');assert 'tick+=50;' in s
    f.write_text(s.replace('tick+=50;','tick+=100;'),encoding='utf-8')
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* diagnostic tested separately */'))
    for name,path in [('final-park-owner','tests/source_rimpark_owner_20261115.c'),('owner-faults','tests/continuous_arm_faults_20261115.c')]:
        source=(R/path).read_text(encoding='utf-8').replace('tick+=50U;','tick+=100U;').replace('50ms release','100ms release')
        f=E/(name+'.c');f.write_text(source,encoding='utf-8');exe=E/(name+'.exe')
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',
                        '-I'+str(E),'-I'+str(R/'tests'),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(f),'-lm','-o',str(exe)],check=True)
        result=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True)
        (E/(name+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        print(result.stdout+result.stderr,flush=True);assert result.returncode==0
    source=Path(build.__file__).read_text(encoding='utf-8')
    source=source.replace(".read_text()", ".read_text(encoding='utf-8')")
    source=source.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'release100_build','__file__':build.__file__}
    exec(compile(source,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    checks=json.loads((E/'checks.json').read_text())
    checks.update(scope='post-release100ms only',strict_simultaneous_start_implemented=False,
                  release_dwell_from_applied_pwm=True,post_release_ms=100)
    (E/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')


if __name__=='__main__':main()
