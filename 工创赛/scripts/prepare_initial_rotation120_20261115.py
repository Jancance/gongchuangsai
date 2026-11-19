"""Isolated first120 observation gate; preserve deployed successful package."""
from pathlib import Path
import ast,hashlib,json,shutil,zipfile

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/initial-rotation120'
P=R/'output/source_initial_rotation120_pi_20261115'


def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    assert not P.exists(),'Preserve existing candidate'
    shutil.copytree(R/'output/source_direct_frame_vehicle_20261115',P,ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copyfile(R/'src/smart_carrier/vision_manipulation/initial_rotation.py',P/'initial_rotation.py')
    f=P/'source_fast_cycle_view_20261115.py';s=f.read_text(encoding='utf-8')
    def change(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new,1)
    change('from direct_frame_trigger_20261115 import DirectFrameTrigger','from direct_frame_trigger_20261115 import DirectFrameTrigger\n    from initial_rotation import InitialRotation')
    change('direct=DirectFrameTrigger(pickup_config)','direct=DirectFrameTrigger(pickup_config)\n    initial_rotation=InitialRotation(orbit_config)')
    change("if predictor is not None and active and current_run!=status['run']:\n            predictor.reset();current_run=status['run']", "if active and current_run!=status['run']:\n            initial_rotation.reset();current_run=status['run']")
    change("if valid and predictor is not None:predictor.register_rim(body['center_px'])", "if valid:initial_rotation.register_rim(body['center_px'])")
    change("prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])", "rotation_ready=status['item']>0 or initial_rotation.update(value,observed,status['colors'][0])\n                if rotation_ready:\n                    prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])\n                else:\n                    direct.reset()")
    change("learning=False  # No whole-turn learning stage in this candidate.","if not active or status['state']!=6:initial_rotation.pause()\n        learning=False  # Observe120 degrees once; no full-turn learning.")
    change('learning=learning,preview_only=args.preview_only)','learning=learning,preview_only=args.preview_only,initial_rotation=initial_rotation.diagnostic())')
    change('direct_frame_trigger=prediction,prediction_used=False,continuous_pick=args.continuous_pick,','direct_frame_trigger=prediction,prediction_used=False,continuous_pick=args.continuous_pick,\n            initial_rotation=initial_rotation.diagnostic(),')
    ast.parse(s);f.write_text(s,encoding='utf-8')
    f=P/'source_cycle_dashboard_20261115.py';s=f.read_text(encoding='utf-8')
    s=s.replace('learning=False,preview_only=False):','learning=False,preview_only=False,initial_rotation=None):')
    anchor="    if preview_only:title='盘缘诊断预览：未发送纠偏量'"
    assert s.count(anchor)==1
    s=s.replace(anchor,"    if active and status['state']==6 and status['item']==0 and initial_rotation and not initial_rotation['ready']:\n        title=f\"等待转盘换位 {abs(initial_rotation['observed_net_deg']):.0f}/120°\"\n"+anchor)
    ast.parse(s);f.write_text(s,encoding='utf-8')
    for name in ('start.sh','start-trial.sh'):
        f=P/name;s=f.read_text().replace('/source_direct_frame_vehicle_20261115','/source_initial_rotation120_pi_20261115').replace('%Y%m%d-%H%M%S','%Y%m%d-%H%M%S-%N')
        f.write_text(s,encoding='utf-8',newline='\n')
    (P/'start-preview.sh').write_text('#!/bin/bash\nexec /bin/bash "$(dirname "$0")/start-trial.sh" --preview-only "$@"\n',encoding='utf-8',newline='\n')
    files={f.relative_to(P).as_posix():hashlib.sha256(f.read_bytes()).hexdigest() for f in P.rglob('*') if f.is_file() and f.name!='manifest.json' and '__pycache__' not in f.parts}
    (P/'manifest.json').write_text(json.dumps(files,indent=2)+'\n')
    with zipfile.ZipFile(P.with_suffix('.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for name in [*files,'manifest.json']:z.write(P/name,name)
    print('Prepared isolated120-degree package; deployed package untouched')


if __name__=='__main__':main()
