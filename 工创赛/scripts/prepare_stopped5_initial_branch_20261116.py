"""Prepare isolated local Pi candidate after the local MATLAB prerequisite."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import zipfile

R = Path(__file__).resolve().parents[1]
NAME = 'source_stopped5_initial_branch_pi_20261116'
PI = R / 'output' / NAME
OLD = 'source_stopped3_grip170_release200_pi_20261115'


def replace_once(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after, 1)


def main():
    proof = json.loads((R / 'evidence/integration/2026-11-16/stopped5-initial-branch/matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab'] and proof['required_stable_frames'] == 5
    assert not PI.exists() and not PI.with_suffix('.zip').exists(), 'Preserve earlier candidates'
    shutil.copytree(R / 'output' / OLD, PI, ignore=shutil.ignore_patterns('__pycache__', 'manifest.json'))
    for name in ('stopped_frame_trigger.py', 'initial_observation_branch.py'):
        shutil.copyfile(R / 'src/smart_carrier/vision_manipulation' / name, PI / name)
    for name in ('start.sh', 'start-trial.sh'):
        p = PI / name
        p.write_text(p.read_text().replace(OLD, NAME), encoding='utf-8', newline='\n')
    p = PI / 'source_fast_cycle_view_20261115.py'
    s = p.read_text(encoding='utf-8')
    s = replace_once(s, 'from initial_rotation import InitialRotation',
                     'from initial_observation_branch import InitialObservationRotation')
    s = replace_once(s, 'direct=DirectFrameTrigger(pickup_config)', 'direct=DirectFrameTrigger(pickup_config, required_frames=5)')
    s = replace_once(s, 'initial_rotation=InitialRotation(orbit_config)',
                     'initial_rotation=InitialObservationRotation(orbit_config,pickup_config)')
    s = replace_once(s, 'context=key;frame_count=0;sequence=0;gate.reset();direct.reset()',
                     'context=key;frame_count=0;sequence=0;gate.reset();direct.reset();initial_rotation.pause()')
    before = """                rotation_ready=status['item']>0 or initial_rotation.update(value,observed,status['colors'][0])
                if rotation_ready:
                    prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])
                else:
                    direct.reset()
"""
    after = """                # Both gates observe every frame; do not short-circuit phase/stop history.
                rotation_ready=status['item']>0 or initial_rotation.update(value,observed,status['colors'][0],status['colors'])
                stopped_prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])
                prediction=stopped_prediction if rotation_ready else None
"""
    s = replace_once(s, before, after)
    p.write_text(s, encoding='utf-8', newline='\n')
    p = PI / 'source_cycle_dashboard_20261115.py'
    s = p.read_text(encoding='utf-8')
    anchor = '    if preview_only:title='
    assert s.count(anchor) == 1
    s = s.replace(anchor, "    if active and status['state']==6 and status['item']==0 and initial_rotation and initial_rotation.get('initial_branch')=='classifying':\n        title='正在判断双鸣后首次运动状态'\n" + anchor)
    s = s.replace("else '转盘持续旋转'", "else '物料连续停稳5帧后抓取'")
    p.write_text(s, encoding='utf-8', newline='\n')
    files = {f.relative_to(PI).as_posix(): hashlib.sha256(f.read_bytes()).hexdigest()
             for f in PI.rglob('*') if f.is_file()}
    for name in files:
        if name.endswith('.py'):
            ast.parse((PI / name).read_text(encoding='utf-8'))
    (PI / 'manifest.json').write_text(json.dumps(files, indent=2) + '\n', encoding='utf-8')
    with zipfile.ZipFile(PI.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as z:
        for name in [*files, 'manifest.json']:
            z.write(PI / name, name)
    print(f'Prepared {NAME}: {len(files)} manifest files; no deployment')


if __name__ == '__main__':
    main()
