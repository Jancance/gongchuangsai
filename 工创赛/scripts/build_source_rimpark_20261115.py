"""Validate the isolated rim/final-park delivery; never access hardware."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import build_source_continuous_20261115 as continuous
import build_integration18_ext150_20261115 as firmware

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/source-rim-reacquire'
P = R / 'tmp/source_mcu_rimpark_20261115/project'
BASE = R / 'tmp/source_mcu_continuous_20261115/project'


def main():
    proof = json.loads((E / 'matlab-result.json').read_text())
    assert proof['passed'] and proof['local_matlab'] and proof['final_request'] == 5
    env = os.environ.copy()
    env['PYTHONPATH'] = os.pathsep.join(str(R / x) for x in ('output', 'vision_app/code', 'src'))
    result = subprocess.run([sys.executable, '-X', 'utf8', '-m', 'unittest', 'discover',
                             '-s', 'tests', '-p', 'test_adaptive_source_rim_20261115.py', '-v'],
                            cwd=R, env=env, capture_output=True, text=True)
    (E / 'adaptive-tests.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout + result.stderr, flush=True)
    assert result.returncode == 0
    # Reuse the actual protocol, poll, key and scheduler fixtures with THIS
    # project's source. The old owner main is renamed, not substituted.
    continuous.P, continuous.E = P, E
    continuous.main(build_firmware=False)
    previous = (R / 'tests/source_continuous_owner_20261115.c').read_text()
    assert previous.count('int main(void){') == 1
    (E / 'prior-continuous-test.c').write_text(
        previous.replace('int main(void){', 'static int prior_continuous_test_main(void){', 1),
        encoding='utf-8')
    exe = E / 'final-park-owner.exe'
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-Wno-misleading-indentation',
                    '-I' + str(E), '-I' + str(R / 'tests'),
                    '-I' + str(P / 'CHASSIS/USER'), '-I' + str(P / 'USER'),
                    str(R / 'tests/source_rimpark_owner_20261115.c'), '-lm', '-o', str(exe)], check=True)
    result = subprocess.run([str(exe)], cwd=E, capture_output=True, text=True)
    (E / 'final-park-owner.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    print(result.stdout + result.stderr, flush=True)
    assert result.returncode == 0
    # The generic owner regression covers all three regions x all three slots.
    # Update only its endpoint expectation for the new third-slot requirement.
    fixture = (R / 'tests/overlap_commit_owner_20261114.c').read_text()
    old = 'extension==65 && fabsf(depth-600)<.1f && g_dm_can2_logical_position_deg==180'
    new = 'extension==(slot==3?0:65) && fabsf(depth-600)<.1f && g_dm_can2_logical_position_deg==(slot==3?0:180)'
    assert fixture.count(old) == 1
    owner_test = E / 'all-regions-final-park.c'
    owner_test.write_text(fixture.replace(old, new), encoding='utf-8')
    # Run the existing build routine with this updated expectation, preserving
    # its wheel/group checks and both complete Keil builds.
    source = (R / 'scripts/build_integration18_ext150_20261115.py').read_text()
    old = "str(ROOT/'tests'/source)"
    assert source.count(old) == 1
    source = source.replace(old, "str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope = {'__name__': 'rimpark_firmware_build', '__file__': firmware.__file__}
    exec(compile(source, firmware.__file__, 'exec'), scope)
    scope.update(PROJECT=P, BASE=BASE, E=E, OWNER_TEST=owner_test)
    scope['main']()
    files = ['output/adaptive_source_rim_20261115.py', 'output/source_mcu_cycle_view_20261115.py',
             'output/source_cycle_dashboard_20261115.py', 'vision_app/code/turntable_detector.py',
             'evidence/integration/2026-11-15/source-rim-reacquire/actual-turntable-detector.json']
    manifest = {f: hashlib.sha256((R / f).read_bytes()).hexdigest() for f in files}
    (E / 'delivery-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (E / 'ready-status.json').write_text(json.dumps(dict(local_complete=True,
        offline_checks_passed=True, final_park_test=result.stdout.strip(),
        flashed=False, pi_deployed=False, hardware_access=False,
        physical_alignment_and_final_park_verified=False, github_pushed=False), indent=2) + '\n')


if __name__ == '__main__':
    main()
