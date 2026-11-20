"""Prepare one explicitly requested second half/tail run; no hardware access."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import ast
import hashlib
import json

R = Path.cwd()
FIELD = R / '工创赛/evidence/navigation-planning/2026-11-15/baseline40-field'
FIRST = FIELD / 'half-stop-run1'
OUT = FIELD / 'half-stop-run2'


def main():
    if OUT.exists():
        assert {p.name for p in OUT.iterdir()} == {'run_and_read_once.py'}, 'Inspect existing attempt; do not overwrite.'
    result = json.loads((FIRST / 'result_summary.json').read_text('utf8'))
    assert result['status'] == 'software_complete' and result['powered_tail_valid']
    measurement = FIRST / 'operator_measurement.json'
    if not measurement.exists():
        measurement.write_text(json.dumps(dict(
            source='User ruler report after physical stop', user_reply='右偏1.5cm，少走0.5cm，再测一次',
            right_mm=15.0, forward_mm=2095.0, forward_error_mm=-5.0,
            middle_ground_observation=None,
            tail_snapshot_status='not_acquired_after_operator_reset',
            operator_reset_confirmation='我已经复位了重新测把我们'), indent=2, ensure_ascii=False), encoding='utf8')
    auth = json.loads((FIRST / 'authorization.json').read_text('utf8'))
    for path, digest in auth['all_changed_source_sha256'].items():
        assert hashlib.sha256((R / path).read_bytes()).hexdigest() == digest
    auth.update(scope='One newly authorized second half-control run; reset once, start once, immediately read tail RAM after powered tail; no download.',
                site_reply='我已经复位了重新测把我们', downloads_authorized=0,
                resets_authorized=1, starts_authorized=1, tail_reads_authorized=1,
                previous_tail_loss_reason='Operator confirmed resetting before the first RAM acquisition',
                authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat())
    OUT.mkdir(exist_ok=True)
    for name in ['reset_once.py', 'run_once.py', 'p1_serial_motion_powered_done.py', 'read_stop_distance_once.py', 'analyze_once.py']:
        data = (FIRST / name).read_bytes()
        ast.parse(data)
        (OUT / name).write_bytes(data)
    assert hashlib.sha256((OUT / 'p1_serial_motion_powered_done.py').read_bytes()).hexdigest() == auth['client_sha256']
    (OUT / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    print('Prepared second authorized half/tail run; original logs retained, no hardware accessed.')


if __name__ == '__main__':
    main()
