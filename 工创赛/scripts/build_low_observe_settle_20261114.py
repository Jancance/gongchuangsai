"""Build only the scoped observer-return settling extension; no devices."""
from pathlib import Path
import difflib
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/source_low_pick_homeexit_20261114/project'
PROJECT = ROOT / 'tmp/source_low_pick_observe_settle_20261114/project'
E = ROOT / 'evidence/integration/2026-11-14/low-pick-observe-settle'


def source_text(path):
    data = path.read_bytes()
    try:
        return data.decode('utf-8')
    except UnicodeDecodeError:
        return data.decode('gb18030')


def main():
    assert json.loads((E / 'matlab-result.json').read_text(encoding='utf-8'))['passed']
    checks = {}
    for name in ('position-window', 'low-owner'):
        result = subprocess.run([str(E / (name + '.exe'))], check=True, capture_output=True, text=True)
        (E / (name + '.log')).write_text(result.stdout, encoding='utf-8')
        checks[name] = result.stdout.strip()
    changes = {}
    patch = []
    for path in PROJECT.rglob('*'):
        relative = path.relative_to(PROJECT)
        if not path.is_file() or any(p in ('Objects', 'Listings') for p in relative.parts):
            continue
        original = BASE / relative
        if not original.exists() or path.read_bytes() != original.read_bytes():
            changes[relative.as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
            patch.extend(difflib.unified_diff(source_text(original).splitlines(True),
                                              source_text(path).splitlines(True),
                                              fromfile='3afb365c/' + relative.as_posix(),
                                              tofile='observe-settle/' + relative.as_posix()))
    assert set(changes) == {'CHASSIS/USER/vehicle_arm.c', 'CHASSIS/USER/vehicle_source_parallel.inc',
                            'CHASSIS/USER/pzq_low_pick.inc'}, changes
    for name in ('integration10_complete.inc', 'lift_ruler_calibration.c', 'source_parallel_flow.h'):
        assert (PROJECT / 'CHASSIS/USER' / name).read_bytes() == (BASE / 'CHASSIS/USER' / name).read_bytes()
    assert (PROJECT / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes() == (BASE / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes()
    candidate = dict(base=BASE.relative_to(ROOT).as_posix(), project=PROJECT.relative_to(ROOT).as_posix(),
                     changes=changes, images={}, build_passed=False, flashed=False, hardware_access=False,
                     matlab_prior_to_implementation=True)
    for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
        for folder in ('Objects', 'Listings'):
            (PROJECT / folder / target).mkdir(parents=True, exist_ok=True)
        log = E / (target + '-build.log')
        result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(PROJECT / 'gongxun-f407.uvprojx'),
                                 '-t', target, '-j0', '-o', str(log)], timeout=180)
        data = log.read_bytes()
        print(data.decode('gb18030', errors='replace')[-500:], flush=True)
        assert result.returncode == 0 and b'0 Error(s), 0 Warning(s)' in data
        image = PROJECT / 'Objects' / target / (target + '.axf')
        candidate['images'][target] = dict(axf=image.relative_to(ROOT).as_posix(),
                                           sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    candidate['build_passed'] = True
    (E / 'candidate.json').write_text(json.dumps(candidate, indent=2), encoding='utf-8')
    (E / 'checks.json').write_text(json.dumps(dict(passed=True, runs=checks, hardware_access=False), indent=2), encoding='utf-8')
    (E / 'source-diff.patch').write_text(''.join(patch), encoding='utf-8')
    print(json.dumps(candidate['images'], indent=2))


if __name__ == '__main__':
    main()
