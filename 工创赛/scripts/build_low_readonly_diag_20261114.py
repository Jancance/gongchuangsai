"""Build one-shot read-only diagnostic access; no actuator/profile changes."""
from pathlib import Path
import hashlib
import json
import subprocess

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/source_low_pick_observe_settle_20261114/project'
P = R / 'tmp/source_low_readonly_diag_20261114/project'
E = R / 'evidence/integration/2026-11-14/low-readonly-diag'


def main():
    result = subprocess.run([str(E / 'config-parser.exe')], check=True, capture_output=True, text=True)
    (E / 'config-parser.log').write_text(result.stdout, encoding='utf-8')
    changes = {}
    for file in P.rglob('*'):
        rel = file.relative_to(P)
        if not file.is_file() or any(n in ('Objects', 'Listings') for n in rel.parts):
            continue
        old = B / rel
        if not old.exists() or file.read_bytes() != old.read_bytes():
            changes[rel.as_posix()] = hashlib.sha256(file.read_bytes()).hexdigest()
    assert set(changes) == {'CHASSIS/SYSTEM/CAN/can.c', 'CHASSIS/SYSTEM/CAN/can.h',
        'CHASSIS/USER/pzq_low_pick.inc', 'CHASSIS/USER/driver_config_reply.h', 'CHASSIS/USER/low_readonly_diag.inc'}
    for name in ['vehicle_source_parallel.inc', 'integration10_complete.inc', 'lift_ruler_calibration.c']:
        assert (P / 'CHASSIS/USER' / name).read_bytes() == (B / 'CHASSIS/USER' / name).read_bytes()
    original = (B / 'CHASSIS/USER/pzq_low_pick.inc').read_bytes()
    updated = (P / 'CHASSIS/USER/pzq_low_pick.inc').read_bytes()
    assert updated.replace(b'#include "low_readonly_diag.inc"\r\n', b'').replace(b'\r\n Low_ReadonlyDiag();', b'') == original
    candidate = dict(base=B.relative_to(R).as_posix(), project=P.relative_to(R).as_posix(),
                     changes=changes, images={}, build_passed=False, flashed=False,
                     hardware_access=False, motion_and_fault_logic_unchanged=True)
    for target in ['Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline']:
        for folder in ['Objects', 'Listings']:
            (P / folder / target).mkdir(parents=True, exist_ok=True)
        log = E / (target + '-build.log')
        build = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(P / 'gongxun-f407.uvprojx'),
                                '-t', target, '-j0', '-o', str(log)], timeout=180)
        raw = log.read_bytes()
        print(raw.decode('gb18030', errors='replace')[-500:], flush=True)
        assert build.returncode == 0 and b'0 Error(s), 0 Warning(s)' in raw
        file = P / 'Objects' / target / (target + '.axf')
        candidate['images'][target] = dict(axf=file.relative_to(R).as_posix(), sha256=hashlib.sha256(file.read_bytes()).hexdigest())
    candidate['build_passed'] = True
    (E / 'candidate.json').write_text(json.dumps(candidate, indent=2), encoding='utf-8')
    (E / 'checks.json').write_text(json.dumps(dict(passed=True, parser=result.stdout.strip(),
        motion_and_fault_logic_unchanged=True, hardware_access=False), indent=2), encoding='utf-8')
    print(json.dumps(candidate['images'], indent=2))


if __name__ == '__main__':
    main()
