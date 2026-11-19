"""Local isolated firmware and Pi package; requires prior local MATLAB pass."""
from pathlib import Path
import ast,hashlib,json,shutil,zipfile

R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/source_integration23_sync_20261115/project'
P=R/'tmp/source_stopped3_grip170_release200_20261115/project'
E=R/'evidence/integration/2026-11-15/stopped3-grip170-release200'
PI=R/'output/source_stopped3_grip170_release200_pi_20261115'

def main():
    proof=json.loads((E/'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab'] and proof['required_stable_frames']==3
    assert not P.exists() and not PI.exists(),'Preserve earlier candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    for name,old,new in [('CHASSIS/USER/pzq_commit_overlap.inc',b'jiazhua(160.0f)',b'jiazhua(170.0f)'),
                         ('USER/vehicle_parameters.h',b'#define SOURCE_RELEASE_DWELL_MS 100U',b'#define SOURCE_RELEASE_DWELL_MS 200U')]:
        f=P/name;data=f.read_bytes();assert data.count(old)==1
        f.write_bytes(data.replace(old,new,1))
    shutil.copytree(R/'output/source_initial_rotation120_pi_20261115',PI,
                    ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copyfile(R/'src/smart_carrier/vision_manipulation/stopped_frame_trigger.py',PI/'stopped_frame_trigger.py')
    (PI/'direct_frame_trigger_20261115.py').write_text(
        '"""Compatibility import for the existing camera entry point."""\n'
        'from stopped_frame_trigger import StoppedFrameTrigger as DirectFrameTrigger\n',encoding='utf-8')
    for name in ('start.sh','start-trial.sh'):
        f=PI/name;s=f.read_text().replace('/source_initial_rotation120_pi_20261115',
                                        '/source_stopped3_grip170_release200_pi_20261115')
        f.write_text(s,encoding='utf-8',newline='\n')
    f=PI/'source_fast_cycle_view_20261115.py';s=f.read_text(encoding='utf-8')
    s=s.replace('direct_frame_trigger=prediction,prediction_used=False,',
                'stop_confirmation=direct.diagnostic(),direct_frame_trigger=prediction,prediction_used=False,')
    f.write_text(s,encoding='utf-8')
    files={f.relative_to(PI).as_posix():hashlib.sha256(f.read_bytes()).hexdigest()
           for f in PI.rglob('*') if f.is_file() and f.name!='manifest.json'}
    for name in files:
        if name.endswith('.py'):ast.parse((PI/name).read_text(encoding='utf-8'))
    (PI/'manifest.json').write_text(json.dumps(files,indent=2)+'\n',encoding='utf-8')
    with zipfile.ZipFile(PI.with_suffix('.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for name in [*files,'manifest.json']:z.write(PI/name,name)
    print('Prepared grip170, carrier-release200, stopped3 and preserved initial120; no deployment')

if __name__=='__main__':main()
