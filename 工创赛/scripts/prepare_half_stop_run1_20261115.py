"""Prepare one explicitly authorized half-control stop-distance measurement run."""
from pathlib import Path
import ast
from datetime import datetime, timezone, timedelta
import hashlib
import json

R = Path.cwd()
BASE = R / '工创赛/evidence/navigation-planning/2026-11-15'
E = BASE / 'baseline40-field/half-stop-run1'
OLD = BASE / 'baseline40-field/diagnostic-run1'
STUDY = BASE / 'rollback-half-ff-20261115/stop-distance-observer'
AXF = '3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d'
AUTO = 'ef5e6540df59b976b6200f7601443e7acdbc565ea3c8779f9f15343b64a59a24'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert not (E / 'authorization.json').exists(), 'No repeat or overwrite.'
    build = json.loads((STUDY / 'build_summary.json').read_text('utf8'))
    download = json.loads((STUDY / 'deployment/download_once_receipt.json').read_text('utf8'))
    assert download['success'] and download['candidate_sha256'] == AXF
    review = STUDY / 'independent_review.md'
    assert sha(review) == '37b3aec64d035d3c925a65ad4dffab49541847017868e3c4e6dfeb1c6200319b'
    assert sha(Path(build['builds']['baseline40_integrated']['axf'])) == AXF
    sources = {f'3.stm32f407工程/CHASSIS/USER/{name}': h for name, h in build['source_sha256'].items()}
    for path, digest in sources.items():
        assert sha(R / path) == digest, path
    assert sources['3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c'] == AUTO
    client = OLD / 'p1_serial_motion_powered_done.py'
    assert sha(client) == 'c8d775e8d91904bb9767a16c44e7b6fae33e3abb6453184f753252d999fbfcb1'
    texts = {}
    for name in ['reset_once.py', 'run_once.py']:
        text = (OLD / name).read_text('utf8')
        text = text.replace('5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185', AXF)
        text = text.replace('ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b', AUTO)
        text = text.replace('tail-snapshot-build-20261115', 'half-stop-observer-build-20261115')
        before = "(E / 'download_once_receipt.json')"
        assert text.count(before) == 1
        text = text.replace(before, "(R / '工创赛/evidence/navigation-planning/2026-11-15/rollback-half-ff-20261115/stop-distance-observer/deployment/download_once_receipt.json')")
        ast.parse(text)
        texts[name] = text
    E.mkdir(exist_ok=True)
    for name, text in texts.items():
        assert not (E / name).exists()
        (E / name).write_text(text, encoding='utf8')
    (E / 'p1_serial_motion_powered_done.py').write_bytes(client.read_bytes())
    analysis = (OLD / 'analyze_once.py').read_text('utf8')
    before = "dict(M=1.5, Pscale_mm_s=2.5, Ky=0.75)"
    assert analysis.count(before) == 1
    analysis = analysis.replace(before, "dict(form='P+H', Ky=0.75, geometric_fraction=0.5, enhanced_scheduling=False)")
    (E / 'analyze_once.py').write_text(analysis, encoding='utf8')
    auth = dict(scope='New explicit authorization after rollback: one half-control PLAN8 run with tail-only RAM read; no reflash.',
        site_ready=True, site_reply='搞完就实车测试我已经摆好了', local_review_passed=True,
        review_report=str(review), review_report_sha256=sha(review),
        candidate_sha256=AXF, source_sha256=AUTO,
        kinematics_sha256=build['source_sha256']['chassis_kinematics.c'], all_changed_source_sha256=sources,
        downloads_authorized=0, resets_authorized=1, starts_authorized=1, tail_reads_authorized=1,
        automatic_retry=False, client_sha256=sha(client), normal_completion_keeps_VM=True,
        reference_policy='Fresh current HWT for this run; do not reuse reset-crossing absolute yaw.',
        reset_policy='Reset once before run; read176B tail after20s powered window before operator reset.',
        authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat())
    (E / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    print('Prepared half + tail-only single run; no hardware accessed.')


if __name__ == '__main__':
    main()
