"""Isolate one braking hypothesis; do not access the vehicle or alter pzq."""
from pathlib import Path
import json
import hashlib
import shutil

R = Path(__file__).resolve().parents[1]
OLD = R / 'tmp/pickup_speed_trials_20261110/project'
P = R / 'tmp/pickup_trial1_braking_20261110/project'
E = R / 'evidence/integration/2026-11-10/speed-trials-braking'
PREV = R / 'evidence/integration/2026-11-10/speed-trials'


def main():
    model = json.loads((E / 'matlab.json').read_text())
    assert model['passed'] and model['extension_decel_rpm_s'] == 400
    assert not P.exists(), 'Preserve the existing candidate'
    manifest = json.loads((PREV / 'candidate.json').read_text())
    for rel, digest in manifest['changed'].items():
        assert hashlib.sha256((OLD / rel).read_bytes()).hexdigest() == digest, rel
    shutil.copytree(OLD, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    u = P / 'CHASSIS/USER'
    header = u / 'pickup_trial_profile.h'
    data = header.read_bytes()
    nl = b'\r\n' if b'\r\n' in data else b'\n'
    addition = ('/* 第一档制动候选：200 RPM，起步600 RPM/s，减速400 RPM/s。\n'
                ' * 仅调整第一档；0档和第二、三档沿用原加减速。实物超调改善待验。 */\n'
                'static uint32_t PickupTrial_ExtDecel(void)\n'
                '{return g_pick_trial_profile==1U?400U:PickupTrial_ExtAccel();}\n')
    assert data.count(b'#endif') == 1
    header.write_bytes(data.replace(b'#endif', addition.encode('utf-8').replace(b'\n', nl) + b'#endif'))
    vsp = u / 'vehicle_source_parallel.inc'
    data = vsp.read_bytes()
    old = b'  a->move[4]=(uint8_t)(c->stepper_accel_rpm_s>>8);a->move[5]=(uint8_t)c->stepper_accel_rpm_s;'
    new = (b'  a->move[4]=(uint8_t)((address==6U?PickupTrial_ExtDecel():c->stepper_accel_rpm_s)>>8);'
           b'a->move[5]=(uint8_t)(address==6U?PickupTrial_ExtDecel():c->stepper_accel_rpm_s);')
    assert data.count(old) == 1
    vsp.write_bytes(data.replace(old, new))
    changed = [p.relative_to(P).as_posix() for p in P.rglob('*')
               if p.is_file() and p.read_bytes() != (OLD / p.relative_to(P)).read_bytes()]
    assert sorted(changed) == ['CHASSIS/USER/pickup_trial_profile.h', 'CHASSIS/USER/vehicle_source_parallel.inc']
    manifest['base'] = 'tmp/pickup_speed_trials_20261110/project'
    manifest['parent_deployed_sha256'] = manifest['images']['Vehicle_LiftRuler']['sha256']
    manifest['images'] = {}
    manifest['revision'] = 'trial1_deceleration_400'
    manifest['profiles'][0]['extension_decel_rpm_s'] = 400
    manifest['profiles'][1]['extension_decel_rpm_s'] = 1200
    manifest['profiles'][2]['extension_decel_rpm_s'] = 2000
    manifest['extension_decel_rpm_s'] = [400, 1200, 2000]
    manifest['field_verified'] = False
    manifest['hardware_access'] = False
    manifest['actual_tests_run'] = 0
    manifest['motion_change'] = 'Only trial1 node6 deceleration 600 -> 400 RPM/s; speed/acceleration/goals/windows/stops unchanged.'
    manifest['changed'] = {rel: hashlib.sha256((P / rel).read_bytes()).hexdigest()
                           for rel in manifest['changed']}
    (E / 'candidate.json').write_text(json.dumps(manifest, indent=2) + '\n')
    for name in ('timing-model.json', 'integration13-three-direct-matlab-20261109.json'):
        shutil.copy2(PREV / name, E / name)
    # Keep the diagnostics added after the first failed field run.
    for kind in ('build', 'check', 'live'):
        source = R / 'scripts' / (f'{kind}_pickup_speed_trials_20261110.py' if kind != 'live'
                                  else 'pickup_speed_trials_live_20261110.py')
        text = source.read_text(encoding='utf-8')
        text = text.replace('pickup_speed_trials_20261110', 'pickup_trial1_braking_20261110')
        text = text.replace('2026-11-10/speed-trials', '2026-11-10/speed-trials-braking')
        dest = R / 'scripts' / (f'{kind}_pickup_trial1_braking_20261110.py' if kind != 'live'
                                else 'pickup_trial1_braking_live_20261110.py')
        dest.write_text(text, encoding='utf-8')
    print('Prepared isolated candidate; two firmware files changed. No hardware access.')


if __name__ == '__main__':
    main()
