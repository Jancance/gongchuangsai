"""Prepare one authorized field package; never access hardware on import/run."""
from pathlib import Path
import ast
from datetime import datetime, timezone, timedelta
import hashlib
import json

R = Path.cwd()
BASE = R / '工创赛/evidence/navigation-planning/2026-11-15'
E = BASE / 'baseline40-field/s25-run1'
OLD = BASE / 'baseline40-field/bounded-run4-retry1'
OLD_AXF = 'dacf226de2d6b77e3480c9bcb4ff8f20c9d8841a16189bc96268c065cf50d583'
NEW_AXF = '41dd9265f971f4d78a83bf93a2db6b52535d01701409aa83c528f4d2a758f7df'
OLD_SOURCE = '368fc6aa0cd2aada7168843c234dce7aa5c8f49512f83d9838ec1fe0226c22b9'
NEW_SOURCE = '5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not E.exists(), 'Field package already exists; inspect instead of preparing again.'
    review = BASE / 's25-field-candidate/independent_review.md'
    verify = BASE / 's25-field-candidate/final_verification.json'
    build = json.loads((BASE / 's25-field-candidate/build_summary.json').read_text('utf8'))
    check = json.loads(verify.read_text('utf8'))
    assert sha(review) == check['independent_review_sha256']
    image = Path(build['builds']['baseline40_integrated']['axf'])
    assert sha(image) == NEW_AXF
    sources = {f'3.stm32f407工程/CHASSIS/USER/{n}': h for n, h in build['source_sha256'].items()}
    for p, h in sources.items():
        assert sha(R / p) == h, p
    client = OLD / 'p1_serial_motion_powered_done.py'
    client_hash = 'c8d775e8d91904bb9767a16c44e7b6fae33e3abb6453184f753252d999fbfcb1'
    assert sha(client) == client_hash
    scripts = {}
    paths = {
        'download_once.py': BASE / 'baseline40-field/bounded-run1/download_once.py',
        'reset_once.py': OLD / 'reset_once.py',
        'run_once.py': OLD / 'run_once.py',
    }
    for name, path in paths.items():
        text = path.read_text('utf8')
        assert OLD_AXF in text and OLD_SOURCE in text
        text = text.replace(OLD_AXF, NEW_AXF).replace(OLD_SOURCE, NEW_SOURCE)
        text = text.replace('bounded-lateral-build-20261115', 'baseline40-s25-build-20261115')
        text = text.replace('2026-11-15/bounded-lateral-enhancement/build_summary.json', '2026-11-15/s25-field-candidate/build_summary.json')
        text = text.replace("(E.parent / 'bounded-run1/download_once_receipt.json')", "(E / 'download_once_receipt.json')")
        assert OLD_AXF not in text and OLD_SOURCE not in text
        assert 'bounded-run1/download_once_receipt.json' not in text
        ast.parse(text)
        scripts[name] = text
    E.mkdir()
    (E / 'p1_serial_motion_powered_done.py').write_bytes(client.read_bytes())
    for name, text in scripts.items():
        (E / name).write_text(text, encoding='utf8')
    auth = dict(
        scope='One S2.5 PLAN8 trial after user says 可以了开始; same setup/load/clear space as briefed; no retry.',
        site_ready=True, site_reply='可以了开始',
        site_confirmation_interpretation='Current user go-ahead after the stated same-origin/load/clear-space test plan; no independent visual verification of the scene.',
        local_review_passed=True, review_scope='Code preparation only; prior model selection gates not passed; experimental physical discrimination authorized.',
        review_report=str(review), review_report_sha256=sha(review), final_verification_sha256=sha(verify),
        downloads_authorized=1, resets_authorized=1, starts_authorized=1, automatic_retry=False,
        candidate_sha256=NEW_AXF, source_sha256=NEW_SOURCE,
        kinematics_sha256=build['source_sha256']['chassis_kinematics.c'], all_changed_source_sha256=sources,
        client_sha256=client_hash, normal_completion_keeps_VM=True,
        reference_policy='Read fresh current HWT for this ground-aligned run; no old cross-day angle.',
        reset_policy='One explicit assistant reset after download, regardless of prior operator reset; no additional automatic reset.',
        authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
    )
    (E / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    receipt = dict(hardware_accessed=False, client_unchanged=True,
                   reused_templates={n: sha(p) for n, p in paths.items()},
                   package_hashes={p.name: sha(p) for p in E.iterdir() if p.is_file()})
    (E / 'preparation_check.json').write_text(json.dumps(receipt, indent=2), encoding='utf8')
    print('Prepared one-run S2.5 package, no hardware accessed:', E)
    print('AXF SHA256:', NEW_AXF)


if __name__ == '__main__':
    main()
