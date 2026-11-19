"""Build the fixed-chassis three-ring parameter candidate. Offline only."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/stage3_carrier37_20261118/project'
PROJECT = ROOT / 'tmp/stage3_three_ring_points_20261118/project'
EVIDENCE = ROOT / 'evidence/integration/2026-11-18/stage3-resume'
SOURCE = Path('CHASSIS/USER/stage3_cycle.inc')


def main():
    model = json.loads((EVIDENCE / 'manual-xyz-teach-matlab.json').read_text(encoding='utf-8'))
    assert model['passed']
    raw = (BASE / SOURCE).read_bytes()
    # All selected targets already pass the same bounded range checks below.
    old = (b'!(g_stage3_carrier_yaw>=0.0f && g_stage3_carrier_yaw<=270.0f) ||'
           b'\r\n    g_stage3_target_yaw[1]!=180.0f || g_stage3_target_x[1]!=0.0f)return 0U;')
    new = b'!(g_stage3_carrier_yaw>=0.0f && g_stage3_carrier_yaw<=270.0f))return 0U;'
    assert raw.count(old) == 1
    shutil.copytree(BASE, PROJECT, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    candidate = raw.replace(old, new)
    (PROJECT / SOURCE).write_bytes(candidate)
    assert candidate.replace(new, old, 1) == raw
    for path in PROJECT.rglob('*'):
        if path.is_file() and path.relative_to(PROJECT) != SOURCE:
            assert path.read_bytes() == (BASE / path.relative_to(PROJECT)).read_bytes(), path
    images = {}
    for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
        for folder in ('Objects', 'Listings'):
            (PROJECT / folder / target).mkdir(parents=True, exist_ok=True)
        log = EVIDENCE / ('three-ring-points-' + target + '.log')
        run = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(PROJECT/'gongxun-f407.uvprojx'),
                              '-t', target, '-j0', '-o', str(log)], timeout=180)
        assert run.returncode == 0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes()
        axf = PROJECT / 'Objects' / target / (target+'.axf')
        images[target] = {'axf': str(axf), 'sha256': hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target, '0 errors, 0 warnings', flush=True)
    report = {'project': str(PROJECT), 'change': str(SOURCE), 'images': images,
              'center_point_uses_common_range_validation': True, 'build_passed': True,
              'manual_disable_implemented': False, 'flashed': False}
    with (EVIDENCE/'three-ring-points-candidate.json').open('x',encoding='utf-8') as stream:
        json.dump(report,stream,indent=2)


if __name__ == '__main__':
    main()
