"""Stage the authorized one-batch trial; no device access or autostart edits."""
from pathlib import Path
import ast
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/extension-lead-pick'
P=R/'output/source_extension_lead_vehicle_20261115'


def main():
    firmware=json.loads((E/'candidate.json').read_text())
    assert firmware['build_passed']
    assert json.loads((E/'checks.json').read_text())['actual_timed_release_and_stop_test_passed']
    assert not P.exists(),'Preserve earlier delivery'
    shutil.copytree(R/'output/source_fast_observation_pi_20261115',P,
                    ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copyfile(E/'trial-orbit-config.json',P/'orbit-config.json')
    config=json.loads((P/'orbit-config.json').read_text())
    # Metadata supplied to the viewer belongs to THIS trial timing as well.
    prediction=json.loads((P/'prediction-config.json').read_text())
    prediction['lead_ms_by_region']=config['lead_ms_by_region']
    prediction['learn_one_revolution']=False
    prediction['lead_source']=config['source']
    (P/'prediction-config.json').write_text(json.dumps(prediction,indent=2)+'\n')
    launcher=(P/'start.sh').read_text()
    launcher=launcher.replace('/source_fast_observation_pi_20261115','/source_extension_lead_vehicle_20261115')
    # Keep the normal launcher observation-only for review. The explicit trial
    # script is the concrete executable authorized by the user's one-run request.
    (P/'start.sh').write_text(launcher,encoding='utf-8',newline='\n')
    trial=launcher.replace(' --preview-only "$@"',' "$@"')
    (P/'start-trial.sh').write_text(trial,encoding='utf-8',newline='\n')
    for source in P.rglob('*.py'):
        ast.parse(source.read_text(encoding='utf-8'))
    # Run the existing meaningful predictor scenarios with THIS config and
    # THIS packaged model; include UART encode/decode compatibility checks.
    code=r'''
import json,sys,unittest
from pathlib import Path
r=Path.cwd();p=r/'output/source_extension_lead_vehicle_20261115'
sys.path[:0]=[str(p),str(r/'tests')]
import test_fast_turntable_20261115 as tests
def setup(self):self.config=json.loads((p/'orbit-config.json').read_text())
tests.FastTests.setUp=setup
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(tests.FastTests))
assert result.wasSuccessful()
from mcu_cycle_protocol import continuous_observation,decode_status,frame
wire=continuous_observation(1,4,2,True,[2,5,1],10,2,[300,100,400])
assert wire.startswith(b'OC2,1,4,2,1,0,0,0,1,2,5,1,10,2,300,100,400,')
assert decode_status(frame('SC2',[1,7,5,6,0,2,1,5,0,0,0,1,100]))['scheduled']==1
print('PASS trial-config predictor3 and unchanged OC2/SC2 protocol')
'''
    result=subprocess.run([sys.executable,'-X','utf8','-c',code],cwd=R,capture_output=True,text=True)
    (E/'vehicle-package-tests.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    print(result.stdout+result.stderr);assert result.returncode==0
    manifest={f.relative_to(P).as_posix():hashlib.sha256(f.read_bytes()).hexdigest()
              for f in P.rglob('*') if f.is_file() and '__pycache__' not in f.parts and f.name!='manifest.json'}
    (P/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with zipfile.ZipFile(R/'output/source_extension_lead_vehicle_20261115.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in [*manifest,'manifest.json']:z.write(P/name,name)
    (E/'vehicle-package.json').write_text(json.dumps(dict(local_package_complete=True,
        firmware_sha256=firmware['images']['Vehicle_LiftRuler']['sha256'],
        trial_lead_ms_by_region=config['lead_ms_by_region'],pi_deployed=False,flashed=False,
        physical_timing_calibrated=False,geometry_p95_px=config['observed_radius_residual_p95_px'],
        trial_start_is_PC13_only=True,host_starts_cycle=False,tests_passed=True),indent=2)+'\n')


if __name__=='__main__':main()
