"""Isolate the evidenced IT hold/body ownership fix; preserve flashed e2d0b367."""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/source_body_stop_20261111/project'
P = R / 'tmp/source_body_hold_20261111/project'
E = R / 'evidence/integration/2026-11-11/source-body-hold'


def main():
    assert json.loads((E / 'matlab-result.json').read_text())['passed']
    previous = json.loads((R / 'evidence/integration/2026-11-11/source-body-stop/candidate.json').read_text())
    assert previous['build_passed'] and not P.exists()
    for rel, sha in previous['changes'].items():
        assert hashlib.sha256((B / rel).read_bytes()).hexdigest() == sha
    shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    rel = 'CHASSIS/USER/integration10_complete.inc'
    path = P / rel
    data = path.read_bytes()
    data.decode('utf-8')
    newline = b'\r\n' if b'\r\n' in data else b'\n'
    original = '''static uint32_t it_platform(void)
{return g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched && !g_chassis_auto_active;}'''.replace('\n', newline.decode()).encode('ascii')
    helper = '''
/* Holding keeps its real health checks during the explicit body-step owner.
 * Ordinary mechanical command admission still uses it_platform unchanged. */
static uint32_t it_hold_platform(void)
{return it_platform() || (SourceBodyStep_WheelWindow() && StartupAxes_Holding() &&
 g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched);}
'''.replace('\n', newline.decode()).encode('ascii')
    assert data.count(original) == 1
    data = data.replace(original, original + helper, 1)
    old = b'StartupAxes_Service(it_platform(),g_can_guard_ready);'
    assert data.count(old) == 1
    data = data.replace(old, b'StartupAxes_Service(it_hold_platform(),g_can_guard_ready);', 1)
    path.write_bytes(data)
    changes = {}
    for f in P.rglob('*'):
        if f.is_file() and f.read_bytes() != (B / f.relative_to(P)).read_bytes():
            changes[f.relative_to(P).as_posix()] = hashlib.sha256(f.read_bytes()).hexdigest()
    assert set(changes) == {rel}
    (E / 'candidate.json').write_text(json.dumps(dict(
        base=B.relative_to(R).as_posix(), project=P.relative_to(R).as_posix(),
        base_online_sha256=previous['images']['Vehicle_LiftRuler']['sha256'], changes=changes,
        hardware_access=False, flashed=False, reset=False, ordinary_it_platform_unchanged=True,
        stop_repair_unchanged=True, mechanical_scheduler_unchanged=True,
        matlab_prior_to_implementation=True), indent=2) + '\n')
    print('Prepared IT hold handoff fix; ordinary command admission, STOP and pzq50 scheduler unchanged.')


if __name__ == '__main__':
    main()
