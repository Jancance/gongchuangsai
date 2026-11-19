"""Clone the frozen pzq; change only release-to-next-yaw200ms to50ms."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'handoff/pzq'
P = R / 'tmp/pzq_release50_20261111/project'
E = R / 'evidence/integration/2026-11-11/pzq-release50'


def main():
    assert not P.exists(), 'Review existing candidate rather than overwrite it'
    E.mkdir(parents=True, exist_ok=True)
    frozen = json.loads((B/'source-manifest.json').read_text(encoding='utf-8'))
    for rel, digest in frozen['files'].items():
        assert hashlib.sha256((B/rel).read_bytes()).hexdigest() == digest, rel
    assert hashlib.sha256((B/'images/Vehicle_LiftRuler.axf').read_bytes()).hexdigest() == '8da8171cb852556eaa8669ee200c18979195d078d084a0a195c78655bf587808'
    shutil.copytree(B/'project', P, ignore=shutil.ignore_patterns('Objects','Listings'))
    relative = 'CHASSIS/USER/integration13_immediate.inc'
    p = P/relative
    old = p.read_bytes()
    assert old.count(b'if(g_it_auto_phase_ms<200U)return;') == 1
    new = old.replace(b'if(g_it_auto_phase_ms<200U)return;', b'if(g_it_auto_phase_ms<50U)return;', 1)
    new = new.replace(b'before200ms.', b'before50ms.', 1)
    p.write_bytes(new)
    changed = []
    for f in P.rglob('*'):
        if f.is_file() and f.read_bytes() != (B/'project'/f.relative_to(P)).read_bytes():
            changed.append(f.relative_to(P).as_posix())
    assert changed == [relative], changed
    m = dict(revision='pzq_original_release50_only', base='handoff/pzq/project',
             original_pzq_sha256=frozen['online_axf_sha256'], frozen_files_verified=len(frozen['files']),
             changed_from_pzq=changed, hardware_access=False, deployed=False, actual_tests_run=0,
             field_verified=False, extension_rpm=106, extension_accel_decel_rpm_s=600,
             lift_rpm=1846.4, lift_accel_decel_rpm_s=45055, place_yaw_deg=37,
             release_to_next_yaw_min_ms=50, extension_left_mm=156, extension_right_mm=160,
             original_parallel_descent_and_extension=True, changes={relative:hashlib.sha256(new).hexdigest()})
    (E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
    (E/'source-diff.patch').write_text(''.join(difflib.unified_diff(old.decode('utf-8').splitlines(True),new.decode('utf-8').splitlines(True),fromfile='frozen_pzq/'+relative,tofile='pzq_release50/'+relative)),encoding='utf-8')
    print('Verified',len(frozen['files']),'frozen pzq files. Only release interval source changed; no hardware access.')


if __name__=='__main__':
    main()
