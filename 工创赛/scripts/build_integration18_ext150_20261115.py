"""Local regressions and Keil builds only; never flash or connect hardware."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/source_top_simplified_20261114/project'
PROJECT=ROOT/'tmp/source_top_extension150_20261115/project'
E=ROOT/'evidence/integration/2026-11-15/integration18/ext150-clog3000'


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert json.loads((E/'parameter-and-poll-checks.json').read_text())['passed']
    (E/'can.h').write_text('#ifndef FIXTURE_CAN_H\n#define FIXTURE_CAN_H\n#include <stdint.h>\ntypedef struct {uint8_t Data[8];} CanRxMsg;\n#endif\n',encoding='ascii')
    owner=(PROJECT/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    (E/'tested-commit-owner.inc').write_bytes(owner.replace(b'#include "low_readonly_diag.inc"',b'/* Parameter service tested separately; owner IO fixture. */'))
    for name,source in [('tested-wheel-hold.inc','output/wheel_hold_current_position_20261114.inc'),
                        ('tested-close-held.inc','output/close_held_completed_20261114.inc')]:
        (E/name).write_bytes((ROOT/source).read_bytes())
    results={}
    for name,source,extra in [('owner','overlap_commit_owner_20261114.c',[]),
                             ('wheel','wheel_hold_and_rearm_20261114.c',[]),
                             ('group','simplified_parallel_group_20261114.c',[str(PROJECT/'CHASSIS/USER/source_parallel_flow.c')])]:
        exe=E/(name+'.exe')
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function','-Wno-misleading-indentation',
            '-I'+str(E),'-I'+str(PROJECT/'CHASSIS/USER'),'-I'+str(PROJECT/'USER'),
            str(ROOT/'tests'/source),*extra,'-lm','-o',str(exe)],check=True)
        r=subprocess.run([str(exe)],capture_output=True,text=True,check=True)
        (E/(name+'.log')).write_text(r.stdout+r.stderr,encoding='utf-8');results[name]=r.stdout.strip();print(r.stdout.strip(),flush=True)
    changes={}
    for p in PROJECT.rglob('*'):
        rel=p.relative_to(PROJECT)
        if not p.is_file() or any(x in ('Objects','Listings') for x in rel.parts):continue
        before=BASE/rel;data=p.read_bytes()
        if not before.exists() or before.read_bytes()!=data:
            assert b'<<<<<<<' not in data and b'>>>>>>>' not in data
            if before.exists():
                old=before.read_bytes()
                assert (old.count(b'\r\n')==old.count(b'\n')) == (data.count(b'\r\n')==data.count(b'\n')),str(rel)
            changes[rel.as_posix()]=hashlib.sha256(data).hexdigest()
    candidate=dict(base=BASE.relative_to(ROOT).as_posix(),project=PROJECT.relative_to(ROOT).as_posix(),
        changes=changes,images={},build_passed=False,flashed=False,hardware_access=False,
        matlab_prior_to_implementation=True,read_only_diag_supported=True,
        explicit_driver_config_request=2,driver_config_automatically_applied=False)
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2),encoding='utf-8')
    for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
        for folder in ['Objects','Listings']:(PROJECT/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(PROJECT/'gongxun-f407.uvprojx'),
            '-t',target,'-j0','-o',str(log)],timeout=180)
        data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-950:],flush=True)
        assert result.returncode==0 and b'0 Error(s), 0 Warning(s)' in data,target
        image=PROJECT/'Objects'/target/(target+'.axf')
        candidate['images'][target]=dict(axf=image.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    candidate['build_passed']=True
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2),encoding='utf-8')
    checks=dict(passed=True,results=results,dual_build=True,matlab_checks=json.loads((E/'matlab-result.json').read_text())['checks'],
        hardware_access=False,flashed=False,driver_parameters_applied=False,
        retry_requires_new_user_approval=True,position_and_hard_fault_protection_preserved=True)
    (E/'checks.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
    print(json.dumps(candidate['images']),flush=True)


if __name__=='__main__':main()
