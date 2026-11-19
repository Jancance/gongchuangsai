"""Build both half-descent targets and fingerprint delivery; no hardware access."""
from pathlib import Path
import hashlib
import json
import subprocess

R = Path(__file__).resolve().parents[1]
P = R / 'tmp/pickup_midpoint_guard_20261111/project'
E = R / 'evidence/integration/2026-11-11/midpoint-guard'


def main():
    E.mkdir(parents=True, exist_ok=True)
    m = {
        'revision': 'pickup_midpoint_hold_until_extension_done',
        'base': 'tmp/pickup_driver_fault_read_20261111/project',
        'parent_deployed_sha256': '81801798f81df0a4e358a08730248e0754ba729c63862eb17ac6c81fb98285df',
        'hardware_access': False, 'deployed': False, 'actual_tests_run': 0,
        'field_verified': False, 'field_repeatability_verified': False,
        'high_speed_overrun_resolved': False,
        'active_trial_index': 2,
        'active_parameters': {'lift_rpm': 2500, 'lift_accel_rpm_s': 45055,
                              'lift_decel_rpm_s': 45055, 'extension_rpm': 200,
                              'extension_accel_rpm_s': 600, 'extension_decel_rpm_s': 600},
        'extension_staged': False, 'extension_and_descent_start_together': False,
        'extension_release_fraction_of_actual_descent': 0.5,
        'release_to_next_request_ms': 50,
        'place_yaw_deg': 37, 'left_extension_mm': 156, 'right_extension_mm': 160,
        'motion_scope': 'One supervised continuous three-item trial; no chassis or profile3',
        'images': {}, 'changed': {},
    }
    for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
        for folder in ('Objects', 'Listings'):
            (P / folder / target).mkdir(parents=True, exist_ok=True)
        log = E / (target + '-build.log')
        r = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(P / 'gongxun-f407.uvprojx'),
                            '-t', target, '-j0', '-o', str(log)], timeout=180)
        content = log.read_bytes()
        print(content.decode('gb18030', errors='replace')[-1200:], flush=True)
        assert r.returncode == 0 and b'0 Error(s), 0 Warning(s)' in content, str(log)
        f = P / 'Objects' / target / (target + '.axf')
        m['images'][target] = {'axf': f.relative_to(R).as_posix(),
                               'sha256': hashlib.sha256(f.read_bytes()).hexdigest(),
                               'build_log': log.relative_to(R).as_posix()}
    base = R / 'handoff/pzq/project'
    diff_parent = []
    parent = R / m['base']
    for f in P.rglob('*'):
        if not f.is_file() or any(x in f.relative_to(P).parts for x in ('Objects', 'Listings')):
            continue
        rel = f.relative_to(P)
        old = base / rel
        if not old.exists() or old.read_bytes() != f.read_bytes():
            m['changed'][rel.as_posix()] = hashlib.sha256(f.read_bytes()).hexdigest()
        old = parent / rel
        if not old.exists() or old.read_bytes() != f.read_bytes():
            diff_parent.append(rel.as_posix())
    m['changed_from_current_board'] = sorted(diff_parent)
    (E / 'candidate.json').write_text(json.dumps(m, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'images': m['images'], 'changed_from_current_board': diff_parent}, indent=2))


if __name__ == '__main__':
    main()
