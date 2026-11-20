"""Bind one explicitly authorized timing/tail observation run, without hardware."""
from pathlib import Path
import ast
from datetime import datetime, timezone, timedelta
import hashlib
import json

R = Path.cwd()
BASE = R / '工创赛/evidence/navigation-planning/2026-11-15'
E = BASE / 'baseline40-field/diagnostic-run1'
OLD = BASE / 'baseline40-field/s25-run1'
STUDY = BASE / 'stop-distance-study/snapshot-candidate'
AXF = '5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
AUTO = 'ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (E / 'authorization.json').exists(), 'Do not recreate a prepared field authorization.'
    build = json.loads((STUDY / 'build_summary.json').read_text('utf8'))
    verification = json.loads((STUDY / 'final_verification.json').read_text('utf8'))
    review = STUDY / 'independent_review.md'
    assert verification['all_passed'] and not verification['B1_enabled']
    assert sha(review) == '44facbae5a3ca92b0e818422976dce81c58ea1277dfda17984def625ead3806c'
    image = Path(build['builds']['baseline40_integrated']['axf'])
    assert sha(image) == AXF
    sources = {f'3.stm32f407工程/CHASSIS/USER/{n}': h for n, h in build['source_sha256'].items()}
    for name in ['chassis_auto_test.h', 'hwt101.c', 'hwt101.h']:
        path = f'3.stm32f407工程/CHASSIS/USER/{name}'
        sources[path] = sha(R / path)
    for path, digest in sources.items():
        assert sha(R / path) == digest, path
    assert sources['3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c'] == AUTO
    client = OLD / 'p1_serial_motion_powered_done.py'
    assert sha(client) == 'c8d775e8d91904bb9767a16c44e7b6fae33e3abb6453184f753252d999fbfcb1'
    texts = {}
    for name in ['download_once.py', 'reset_once.py', 'run_once.py']:
        text = (OLD / name).read_text('utf8')
        assert '41dd9265f971f4d78a83bf93a2db6b52535d01701409aa83c528f4d2a758f7df' in text
        text = text.replace('41dd9265f971f4d78a83bf93a2db6b52535d01701409aa83c528f4d2a758f7df', AXF)
        text = text.replace('5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b', AUTO)
        text = text.replace('baseline40-s25-build-20261115', 'tail-snapshot-build-20261115')
        text = text.replace('2026-11-15/s25-field-candidate/build_summary.json', '2026-11-15/stop-distance-study/snapshot-candidate/build_summary.json')
        ast.parse(text)
        texts[name] = text
    E.mkdir(exist_ok=True)
    for name, text in texts.items():
        assert not (E / name).exists()
        (E / name).write_text(text, encoding='utf8')
    (E / 'p1_serial_motion_powered_done.py').write_bytes(client.read_bytes())
    (E / 'analyze_once.py').write_bytes((OLD / 'analyze_once.py').read_bytes())
    auth = dict(
        scope='One new observation-only candidate trial; same S2.5 control, 40cm/s and210cm; read timing+tail RAM after powered tail before operator reset.',
        site_ready=True, site_reply='可以直接实测我车摆好了',
        local_review_passed=True, review_scope='Observation code only; B0 not identifiable and B1/control change not deployed.',
        review_report=str(review), review_report_sha256=sha(review), final_verification_sha256=sha(STUDY / 'final_verification.json'),
        downloads_authorized=1, resets_authorized=1, starts_authorized=1,
        diagnostic_reads_authorized=1, automatic_retry=False,
        candidate_sha256=AXF, source_sha256=AUTO,
        kinematics_sha256=build['source_sha256']['chassis_kinematics.c'],
        all_changed_source_sha256=sources, client_sha256=sha(client), normal_completion_keeps_VM=True,
        reference_policy='Read fresh current HWT for this ground-aligned run; no old cross-day angle.',
        reset_policy='One assistant reset before run; do not reset after completion until RAM acquisition ends.',
        authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
    )
    (E / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    print('Prepared one observation run; no hardware accessed. AXF SHA256:', AXF)


if __name__ == '__main__':
    main()
