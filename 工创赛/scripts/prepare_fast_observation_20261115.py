"""Create a reviewable offline viewer with no full-orbit learning stage."""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import os

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/source-rim-reacquire/fast-observation'
P = R / 'output/source_fast_observation_pi_20261115'


def main():
    proof = json.loads((E / 'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab']
    env = os.environ.copy()
    env['PYTHONPATH'] = str(R / 'src')
    result = subprocess.run([sys.executable, '-X', 'utf8', '-m', 'unittest', 'discover',
        '-s', 'tests', '-p', 'test_fast_turntable_20261115.py', '-v'],
        cwd=R, env=env, capture_output=True, text=True)
    (E / 'python-tests.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    assert result.returncode == 0, result.stdout + result.stderr
    assert not P.exists(), 'Preserve previous candidates'
    shutil.copytree(R / 'output/source_rimpark_pi_20261115', P)
    # Keep the previous observer module intact and select a separate entry.
    viewer = (R / 'output/source_mcu_cycle_view_20261115.py').read_text(encoding='utf-8')
    edits = [
        ("parser.add_argument('--rim-detector-config',type=Path)",
         "parser.add_argument('--rim-detector-config',type=Path,required=True)\n    parser.add_argument('--orbit-config',type=Path,required=True)"),
        ('from continuous_turntable import ContinuousTurntable,rim_observation_valid',
         'from continuous_turntable import rim_observation_valid\n    from smart_carrier.vision_manipulation.fast_turntable import FastTurntable'),
        ("predictor=ContinuousTurntable(prediction_config['target_angles_rad'],prediction_config['lead_ms_by_region'],\n        pickup_config['motion']['maximum_frame_interval_s']) if prediction_config else None",
         "orbit_config=json.loads(args.orbit_config.read_text())\n    predictor=FastTurntable(orbit_config) if args.continuous_pick else None"),
        ("valid=frame_count>=2 and body['observation_valid']",
         "valid=frame_count>=2 and body['observation_valid']\n                if valid and predictor is not None:predictor.register_rim(body['center_px'])"),
        ("learning=bool(args.continuous_pick and active and status['state']==6 and not predictor.learned)",
         "learning=False  # No whole-turn learning stage in this candidate."),
    ]
    for old, new in edits:
        assert viewer.count(old) == 1, old
        viewer = viewer.replace(old, new, 1)
    ast.parse(viewer)
    (P / 'source_fast_cycle_view_20261115.py').write_text(viewer, encoding='utf-8')
    package = P / 'smart_carrier/vision_manipulation'
    package.mkdir(parents=True)
    (P / 'smart_carrier/__init__.py').write_text('')
    (package / '__init__.py').write_text('')
    for name in ('fast_turntable.py', 'continuous_turntable.py'):
        shutil.copyfile(R / 'src/smart_carrier/vision_manipulation' / name, package / name)
    shutil.copyfile(E / 'orbit-config.json', P / 'orbit-config.json')
    launcher = (P / 'start.sh').read_text()
    launcher = launcher.replace('/source_rimpark_20261115', '/source_fast_observation_pi_20261115')
    launcher = launcher.replace('source_mcu_cycle_view_20261115.py', 'source_fast_cycle_view_20261115.py')
    launcher = launcher.replace('--rim-detector-config rim-detector.json',
                                '--orbit-config orbit-config.json --rim-detector-config rim-detector.json')
    # Offline delivery defaults to preview only; deployment and execution
    # are explicitly deferred by the user, including changes to autostart.
    launcher = launcher.replace('"$@" >>', '--preview-only "$@" >>')
    (P / 'start.sh').write_text(launcher, encoding='utf-8', newline='\n')
    (P / 'source-rimpark.desktop').unlink()
    manifest = {f.relative_to(P).as_posix(): hashlib.sha256(f.read_bytes()).hexdigest()
                for f in P.rglob('*') if f.is_file() and f.name != 'manifest.json'}
    (P / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    checks = dict(local_complete=True, matlab_checks=proof['checks'],
        python_tests=3, whole_turn_learning_required=False, fresh_frame_count=2,
        uart_protocol_and_mcu_unchanged=True, current_rim_registration_in_viewer=True,
        preview_only_default=True, pi_deployed=False, firmware_flashed=False,
        current_red_threshold_unmodified=True, physical_grasp_passed=False,
        recorded_geometry_radius_residual_p95_px=proof['recorded_orbit_radius_residual_p95_px'],
        live_execution_deferred_by_user=True)
    (E / 'checks.json').write_text(json.dumps(checks, indent=2) + '\n')
    print(json.dumps(checks, indent=2))


if __name__ == '__main__':
    main()
