"""Isolated host-read fence and each-observation120 package; no deployment."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import zipfile

R=Path(__file__).resolve().parents[1]
BASE=R/'output/source_stopped5_initial_branch_pi_20261116'
P=R/'output/source_observe_ready_each120_pi_20261116'
E=R/'evidence/integration/2026-11-16/observe-ready-each120'


def once(text,before,after):
    assert text.count(before)==1,before
    return text.replace(before,after,1)


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['validation_passed'] and proof['local_matlab'] and proof['checks']>=928
    boundary=json.loads((E/'boundary-rebind-matlab.json').read_text(encoding='utf-8'))
    assert boundary['passed'] and boundary['local_matlab']
    assert not P.exists(),'Preserve earlier candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('__pycache__','manifest.json'))
    for name in ('observation_frame_epoch.py','each_observation_rotation.py'):
        shutil.copyfile(R/'src/smart_carrier/vision_manipulation'/name,P/name)
    for name in ('start.sh','start-trial.sh'):
        path=P/name;path.write_text(path.read_text().replace(BASE.name,P.name),encoding='utf-8',newline='\n')
    path=P/'source_fast_cycle_view_20261115.py';s=path.read_text(encoding='utf-8')
    s=once(s,'from initial_observation_branch import InitialObservationRotation',
           'from each_observation_rotation import EachObservationRotation\n    from observation_frame_epoch import ObservationFrameEpoch,camera_metadata')
    s=once(s,'initial_rotation=InitialObservationRotation(orbit_config,pickup_config)',
           'initial_rotation=EachObservationRotation(orbit_config,pickup_config)\n    frame_epoch=ObservationFrameEpoch();epoch_revision=frame_epoch.revision')
    s=once(s,'packets,window_ready,current_run','packets,window_ready,current_run,epoch_revision')
    s=once(s,"                if status!=latest:print('MCU_STATUS '+json.dumps(latest),flush=True)\n                status=latest;received=time.monotonic()",
           "                received_at=time.monotonic()\n                if not frame_epoch.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60):continue\n                if status!=latest:print('MCU_STATUS '+json.dumps(latest),flush=True)\n                status=latest;received=received_at")
    s=once(s,"        active=status is not None and now-received<=.60",
           "        now=time.monotonic()\n        active=status is not None and now-received<=.60\n        if not active:frame_epoch.invalidate()\n        metadata=camera_metadata(app.camera)\n        ready_frame=active and status['state']==6 and frame_epoch.frame(metadata,now)\n        if epoch_revision!=frame_epoch.revision:\n            direct.reset();initial_rotation.pause();epoch_revision=frame_epoch.revision")
    s=once(s,"        stamp=getattr(app.camera,'last_read_completed_time',None)\n        observed=stamp if isinstance(stamp,(int,float)) and stamp<=now else now",
           "        stamp=getattr(app.camera,'last_read_completed_time',None)\n        observed=stamp if isinstance(stamp,(int,float)) and stamp<=now else None")
    s=once(s,"        if active and (status['state']==6 or study):",
           "        if active and (ready_frame or study):")
    s=once(s,"                rotation_ready=status['item']>0 or initial_rotation.update(value,observed,status['colors'][0],status['colors'])",
           "                target=status['colors'][status['item']]\n                remaining=status['colors'][status['item']:]\n                was_rotation_open=initial_rotation.rotation.opened\n                rotation_ready=initial_rotation.update(value,observed,target,remaining)\n                if initial_rotation.branch=='initial_stopped' and not was_rotation_open and initial_rotation.rotation.opened:direct.reset()")
    s=once(s,"        if send_phase and time.monotonic()-last_sent>=.10:",
           "        if send_phase and observed is not None and time.monotonic()-last_sent>=.10:")
    s=once(s,"            initial_rotation=initial_rotation.diagnostic(),\n            observation_host_timestamp=observed,exposure_timestamp_verified=False,",
           "            initial_rotation=initial_rotation.diagnostic(),\n            frame_epoch=frame_epoch.diagnostic(),prearrival_motion='UNKNOWN',premotion_exception_enabled=False,\n            observation_host_timestamp=observed,exposure_timestamp_verified=False,")
    # Refresh the phase geometry from the same fixed ready pose, once per epoch.
    anchor="            detected=app.material_pickup_detector.detect(frame,timestamp=now)"
    s=once(s,anchor,"            if ready_frame and initial_rotation.previous is None and initial_rotation.branch is None:\n                try:\n                    ready_body=rim.estimate(frame) if rim is not None else estimate(frame,profile)\n                    if rim_observation_valid(ready_body,3.0):initial_rotation.register_rim(ready_body['center_px'])\n                except (ValueError,ArithmeticError):pass\n"+anchor)
    path.write_text(s,encoding='utf-8',newline='\n')
    # Show the current item's gate too, instead of hiding the later120 wait.
    path=P/'source_cycle_dashboard_20261115.py';s=path.read_text(encoding='utf-8')
    s=s.replace(" and status['item']==0 and initial_rotation",' and initial_rotation')
    path.write_text(s,encoding='utf-8',newline='\n')
    files={p.relative_to(P).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in P.rglob('*') if p.is_file()}
    for name in files:
        if name.endswith('.py'):ast.parse((P/name).read_text(encoding='utf-8'))
    (P/'manifest.json').write_text(json.dumps(files,indent=2)+'\n',encoding='utf-8')
    with zipfile.ZipFile(P.with_suffix('.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for name in [*files,'manifest.json']:z.write(P/name,name)
    print('Prepared each-ready host frame fence + each-target120; premotion UNKNOWN, no deployment')


if __name__=='__main__':main()
