"""Local-only final validation/build/review bundle. Never attaches or deploys."""
from pathlib import Path
import hashlib,json,subprocess,sys,zipfile
import build_top_pick_lead8_20261114 as build

R=Path(__file__).resolve().parents[1]
P=R/'tmp/source_top_overlap200_20261114/project'
B=R/'tmp/source_top_pick_pb9_20261114/project'
E=R/'evidence/integration/2026-11-14/overlap200-commit'
ALLOW_READ_RETRY_CHANGE=False

def run():
    for name in ['matlab-result.json','body-registration-matlab.json','wheel-hold-rearm-matlab.json']:
        assert json.loads((E/name).read_text())['passed'],name
    (E/'can.h').write_text('#ifndef FIXTURE_CAN_H\n#define FIXTURE_CAN_H\n#include <stdint.h>\ntypedef struct {uint8_t Data[8];} CanRxMsg;\n#endif\n',encoding='ascii')
    owner=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes()
    (E/'tested-commit-owner.inc').write_bytes(owner.replace(b'#include "low_readonly_diag.inc"',b'/* unrelated read-only diagnostic excluded from IO fixture */'))
    for source,name,production in [
      ('output/wheel_hold_current_position_20261114.inc','tested-wheel-hold.inc','CHASSIS/SYSTEM/CAN/can.c'),
      ('output/close_held_completed_20261114.inc','tested-close-held.inc','CHASSIS/USER/chassis_auto_test.c')]:
        data=(R/source).read_bytes().replace(b'\r\n',b'\n')
        assert data in (P/production).read_bytes().replace(b'\r\n',b'\n')
        (E/name).write_bytes(data)
    results={}
    for name,source in [('commit-owner','overlap_commit_owner_20261114.c'),('wheel-hold','wheel_hold_and_rearm_20261114.c')]:
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Wno-unused-function',
          '-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(R/'tests'/source),
          '-lm','-o',str(E/(name+'.exe'))],check=True)
        result=subprocess.run([str(E/(name+'.exe'))],check=True,capture_output=True,text=True)
        (E/(name+'.log')).write_text(result.stdout,encoding='utf-8');print(result.stdout.strip(),flush=True)
        results[name]=result.stdout.strip()
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-p','test_committed_workflow_20261114.py'],cwd=R,check=True,capture_output=True,text=True)
    (E/'host-tests.log').write_text(result.stdout+result.stderr,encoding='utf-8');results['host']=result.stderr.strip();print(result.stderr[-500:],flush=True)
    if not ALLOW_READ_RETRY_CHANGE:
        assert (P/'CHASSIS/USER/lift_ruler_calibration.c').read_bytes()==(B/'CHASSIS/USER/lift_ruler_calibration.c').read_bytes()
    assert (P/'CHASSIS/HARDWARE/RADAR_SERVO/radar_servo.c').read_bytes()==(B/'CHASSIS/HARDWARE/RADAR_SERVO/radar_servo.c').read_bytes()
    for file in P.rglob('*'):
        rel=file.relative_to(P)
        if not file.is_file() or any(x in ('Objects','Listings') for x in rel.parts):continue
        before=B/rel
        if before.exists() and file.read_bytes()!=before.read_bytes():
            old=before.read_bytes();new=file.read_bytes()
            assert b'<<<<<<<' not in new and b'>>>>>>>' not in new,rel
            assert (old.count(b'\n')==old.count(b'\r\n'))==(new.count(b'\n')==new.count(b'\r\n')),rel
    build.B=B;build.P=P;build.E=E;build.main()
    candidate=json.loads((E/'candidate.json').read_text())
    result=dict(passed=True,actual_c_and_host=results,matlab_before_implementation=True,
      hardware_access=False,flashed=False,physical_trial_passed=False,
      execution_requires_user_approval=True,unchanged=['radar_servo.c']+([] if ALLOW_READ_RETRY_CHANGE else ['lift_ruler_calibration.c']))
    (E/'checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    files={}
    names=['scripts/source_commit_trial_20261114.py','scripts/build_overlap200_commit_20261114.py',
      'output/commit_material_stream_20261114.py','output/source_body_registration_20261114.json',
      'output/material_pickup_top65_20261114.json',
      'src/smart_carrier/vision_manipulation/committed_pick_board.py',
      'src/smart_carrier/vision_manipulation/source_body_registration.py']
    for name in names:files[name]=hashlib.sha256((R/name).read_bytes()).hexdigest()
    (E/'software-manifest.json').write_text(json.dumps(dict(files=files,firmware=candidate['images'],not_deployed=True),indent=2),encoding='utf-8')
    bundle=R/('output/source_simplified_20261114_review.zip' if ALLOW_READ_RETRY_CHANGE else 'output/source_overlap200_commit_20261114_review.zip')
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
        for file in P.rglob('*'):
            if file.is_file() and not any(x in ('Objects','Listings') for x in file.relative_to(P).parts):
                z.write(file,'firmware/'+file.relative_to(P).as_posix())
        for name in names:z.write(R/name,name)
        for name in ['matlab-result.json','body-registration-matlab.json','wheel-hold-rearm-matlab.json','checks.json','candidate.json','software-manifest.json']:
            z.write(E/name,'evidence/'+name)
        for target,image in candidate['images'].items():z.write(R/image['axf'],'images/'+target+'.axf')
    print('LOCAL REVIEW BUNDLE',bundle,flush=True)

if __name__=='__main__':run()
