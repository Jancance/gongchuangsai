"""Prepare a separate bounded upward trim after the local MATLAB gate. No flash."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import check_lift_ruler_candidate_20261105 as prior

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/lift_ruler_20261105/project'
STAGE = ROOT / 'tmp/lift_trim_up_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-trim-up'


def adapt_unit(text):
    # The simulated travel magnitude is positive; position and encoder go up
    # in the already verified negative motor direction. Verify the wire bit.
    changes = [
        ('total=3.625', 'total=2.0'), ('t<3.125', 't<1.5'),
        ('562.5', '270.0'), ('565.5', '273.0'),
        ('5000U*360U', '2400U*360U'), ('>=3625U', '>=2000U'),
        ('angle=initial+delta;', 'angle=initial-delta;'),
        ('angle=initial-3;', 'angle=initial+3;'),
        ('angle=initial+273.0;', 'angle=initial-273.0;'),
        ('llround(delta*65536/360)', '-llround(delta*65536/360)'),
        ('initial+270.0+', 'initial-270.0+'),
        ('CK(q[1]==0 &&', 'CK(q[1]==1 &&'),
        ('c.position_after_raw-c.command_before_raw-(int64_t)c.move_units',
         'c.position_after_raw-c.command_before_raw+(int64_t)c.move_units'),
    ]
    for old, new in changes:
        assert old in text, old
        text = text.replace(old, new)
    return text


def main():
    proof = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['engine'] == 'local MATLAB' and proof['passed'] and not proof['failure_count']
    assert proof['trim_up_virtual_pulses'] == 2400 and proof['initial_height_user_mm'] == 48.5
    assert not STAGE.exists(), 'Preserve the existing trim candidate'
    shutil.copytree(BASE, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    changed = {}

    def edit(name, transform):
        path = STAGE / name
        raw = path.read_bytes()
        try:
            text, encoding = raw.decode('utf-8'), 'utf-8'
        except UnicodeDecodeError:
            text, encoding = raw.decode('gbk'), 'gbk'
        result = transform(text.replace('\r\n', '\n'))
        path.write_bytes(result.replace('\n', '\r\n').encode(encoding))
        changed[name] = {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(path.read_bytes()).hexdigest()}

    def controller(t):
        changes = [
            ('#define LRC_STEP_PULSES 5000U', '#define LRC_STEP_PULSES 2400U'),
            ('c->goal_raw=pos+(int64_t)c->move_units;', 'c->goal_raw=pos-(int64_t)c->move_units;'),
            ('c->move_frame[1]=0; /* 已验向下：正位置方向。 */', 'c->move_frame[1]=1; /* Explicit upward trim only. */'),
            ('pos<c->command_before_raw-2*(int64_t)c->scale || pos>c->goal_raw+2*(int64_t)c->scale',
             'pos>c->command_before_raw+2*(int64_t)c->scale || pos<c->goal_raw-2*(int64_t)c->scale'),
            ('value-36864U', 'value-16384U'),
        ]
        for old, new in changes:
            assert t.count(old) == 1, old
            t = t.replace(old, new)
        return t

    edit('CHASSIS/USER/lift_ruler_calibration.c', controller)
    edit('CHASSIS/USER/vehicle_mission.inc', lambda t: t.replace(',5000,300,%lu,', ',2400,300,%lu,'))
    # Retain the user-confirmed ordinary reference speed configuration and its
    # matching admission thresholds; these other-axis services stay disabled.
    compare = ROOT / 'tmp/lift_speed_compare_20261106/project'
    for name in ['USER/vehicle_parameters.h', 'CHASSIS/USER/source_prepare.inc']:
        edit(name, lambda _, name=name: (compare / name).read_text(encoding='utf-8'))

    unit = adapt_unit(prior.UNIT).replace('#include <stdio.h>', '#include <stdio.h>\n#include <stdlib.h>')
    source, exe = OUT / 'trim-up-host.c', OUT / 'trim-up-host.exe'
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
              'step_direction': 'up', 'step_virtual_pulses': 2400, 'rpm': 30, 'acceleration_rpm_s': 60,
              'maximum_steps': 1, 'nominal_start_height_mm': 48.5, 'nominal_end_height_mm': 50,
              'VM_held_during_move_stop_retry_and_DONE': True, 'automatic_repeat': False,
              'home_or_full_return_command_in_trim': False, 'firmware_downloaded': False,
              'requires_additional_VM_deployment_exception': True, 'hardware_operated': False}
    (OUT / 'candidate-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (OUT / 'dispatch-matlab-result.json').write_text(json.dumps({'passed': proof['passed'], 'check_count': proof['check_count']}), encoding='utf-8')


if __name__ == '__main__':
    main()
