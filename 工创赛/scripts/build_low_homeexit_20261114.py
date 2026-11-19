"""Build the single bounded homing-exit delta; never flash or connect devices."""
from pathlib import Path
import difflib
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/source_low_pick_20261114/project'
PROJECT = ROOT / 'tmp/source_low_pick_homeexit_20261114/project'
EVIDENCE = ROOT / 'evidence/integration/2026-11-14/low-pick-homeexit'


def main():
    proof = json.loads((EVIDENCE / 'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['passed']
    result = subprocess.run([str(EVIDENCE / 'home-exit.exe')], check=True, capture_output=True, text=True)
    (EVIDENCE / 'home-exit.log').write_text(result.stdout, encoding='utf-8')
    changed = {}
    for path in PROJECT.rglob('*'):
        relative = path.relative_to(PROJECT)
        if not path.is_file() or any(p in ('Objects', 'Listings') for p in relative.parts):
            continue
        original = BASE / relative
        if not original.exists() or path.read_bytes() != original.read_bytes():
            changed[relative.as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    assert set(changed) == {'CHASSIS/USER/lift_ruler_calibration.c'}, changed
    for filename in ('pzq_low_pick.inc', 'integration10_complete.inc', 'vehicle_source_parallel.inc'):
        assert (PROJECT / 'CHASSIS/USER' / filename).read_bytes() == (BASE / 'CHASSIS/USER' / filename).read_bytes()
    assert (PROJECT / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes() == (BASE / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes()
    candidate = dict(base=BASE.relative_to(ROOT).as_posix(), project=PROJECT.relative_to(ROOT).as_posix(),
                     changes=changed, images={}, build_passed=False, flashed=False, hardware_access=False,
                     matlab_prior_to_implementation=True, original_low_owner_unchanged=True)
    for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
        for folder in ('Objects', 'Listings'):
            (PROJECT / folder / target).mkdir(parents=True, exist_ok=True)
        log = EVIDENCE / (target + '-build.log')
        build = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(PROJECT / 'gongxun-f407.uvprojx'),
                                '-t', target, '-j0', '-o', str(log)], timeout=180)
        data = log.read_bytes()
        print(data.decode('gb18030', errors='replace')[-600:], flush=True)
        assert build.returncode == 0 and b'0 Error(s), 0 Warning(s)' in data
        image = PROJECT / 'Objects' / target / (target + '.axf')
        candidate['images'][target] = dict(axf=image.relative_to(ROOT).as_posix(),
                                           sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    candidate['build_passed'] = True
    source = 'CHASSIS/USER/lift_ruler_calibration.c'
    patch = difflib.unified_diff((BASE / source).read_text(encoding='gb18030').splitlines(True),
                                (PROJECT / source).read_text(encoding='gb18030').splitlines(True),
                                fromfile='404f1506/' + source, tofile='home-exit-fix/' + source)
    (EVIDENCE / 'source-diff.patch').write_text(''.join(patch), encoding='utf-8')
    (EVIDENCE / 'candidate.json').write_text(json.dumps(candidate, indent=2), encoding='utf-8')
    (EVIDENCE / 'checks.json').write_text(json.dumps(dict(passed=True, home_controller=result.stdout.strip(),
        inherited_low_owner_checks='396; source unchanged from prior candidate', hardware_access=False), indent=2), encoding='utf-8')
    print(json.dumps(candidate['images'], indent=2), flush=True)


if __name__ == '__main__':
    main()
