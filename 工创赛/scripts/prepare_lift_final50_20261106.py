"""Prepare a single bounded approach to about50mm after local MATLAB proof."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import check_lift_ruler_candidate_20261105 as prior

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_final50_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-final50'


def adapt_unit(text):
    replacements = [
        ('total=3.625', 'total=8.5'),
        ('t<3.125', 't<8.0'),
        ('562.5', '1440.0'), ('565.5', '1443.0'), ('5000U*360U', '12800U*360U'),
        ('==300 &&', '==300 &&'), ('u=500;', 'u=500;'),
        ('i<220', 'i<340'), ('>=3625U', '>=8500U'),
    ]
    for old, new in replacements:
        assert old in text, old
        text = text.replace(old, new)
    return text


def main():
    proof = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['model_checks_passed'] and not proof['failure_count']
    assert not STAGE.exists(), 'Preserve existing candidate'
    base = ROOT / 'tmp/lift_ruler_20261105/project'
    shutil.copytree(base, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    relative = 'CHASSIS/USER/lift_ruler_calibration.c'
    p = STAGE / relative
    raw = p.read_bytes()
    t = raw.decode('utf-8').replace('\r\n', '\n')
    for old, new in [('#define LRC_STEP_PULSES 5000U', '#define LRC_STEP_PULSES 12800U'),
                     ('#define LRC_SPEED_DECI_RPM 300U', '#define LRC_SPEED_DECI_RPM 300U'),
                     ('>=8000U', '>=12000U'), ('value-36864U', 'value-0U'),
                     ('c->speed_raw>450U', 'c->speed_raw>450U')]:
        assert t.count(old) == 1, old
        t = t.replace(old, new)
    p.write_bytes(t.replace('\n', '\r\n').encode('utf-8'))
    changed = {relative: {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(p.read_bytes()).hexdigest()}}
    relative = 'CHASSIS/USER/vehicle_mission.inc'
    p = STAGE / relative
    raw = p.read_bytes()
    t = raw.decode('utf-8')
    assert t.count(',5000,300,%lu,') == 1
    p.write_bytes(t.replace(',5000,300,%lu,', ',12800,300,%lu,').encode('utf-8'))
    changed[relative] = {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(p.read_bytes()).hexdigest()}
    unit = adapt_unit(prior.UNIT).replace('#include <stdio.h>', '#include <stdio.h>\n#include <stdlib.h>')
    source, exe = OUT / 'final50-host.c', OUT / 'final50-host.exe'
    source.write_text(unit, encoding='utf-8')
    cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
           '-I' + str(STAGE / 'CHASSIS/USER'), str(source), str(STAGE / 'CHASSIS/USER/lift_ruler_calibration.c'), '-lm', '-o', str(exe)]
    built = subprocess.run(cmd, capture_output=True, text=True)
    (OUT / 'host-build.log').write_text(built.stdout + built.stderr, encoding='utf-8')
    assert built.returncode == 0, built.stderr
    ran = subprocess.run([str(exe)], capture_output=True, text=True)
    (OUT / 'host-check.log').write_text(ran.stdout + ran.stderr, encoding='utf-8')
    print(ran.stdout)
    assert ran.returncode == 0, ran.stdout + ran.stderr
    report = {'passed': True, 'stage': str(STAGE), 'changed_files': changed, 'host_result': ran.stdout.strip(),
              'step_virtual_pulses': 12800, 'rpm': 30, 'acceleration_rpm_s': 60, 'timeout_ms': 12000,
              'maximum_samples': 1, 'nominal_step_mm': 8.0, 'nominal_end_height_mm': 50.0,
              'previous_top_cumulative_pulses': 98000, 'nominal_new_top_cumulative_pulses': 110800,
              'ordinary_top_referenced_limit_pulses': 105000, 'ordinary_limit_changed': False, 'commissioning_test_target_pulses': 110800, 'test_target_basis': 'user-confirmed63mm remaining at113mm; measured descent55mm to58mm; final8mm',
              'mechanical_remaining_at_least63mm_user_confirmed': True,
              'installed_in_external_actual_source': False, 'firmware_downloaded': False, 'hardware_operated': False}
    (OUT / 'candidate-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (OUT / 'dispatch-matlab-result.json').write_text(json.dumps({'passed': proof['model_checks_passed'], 'check_count': proof['check_count'], 'source': 'matlab-result.json'}), encoding='utf-8')


if __name__ == '__main__':
    main()
