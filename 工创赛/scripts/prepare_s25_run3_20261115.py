"""Prepare the user's third one-shot trial; never touch hardware."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import ast
import hashlib
import json

R = Path.cwd()
FIELD = R / '工创赛/evidence/navigation-planning/2026-11-15/baseline40-field'
FIRST, PREVIOUS, OUT = [FIELD / name for name in ['s25-run1', 's25-run2', 's25-run3']]


def main():
    assert not OUT.exists(), 'Do not overwrite or repeat an existing run.'
    result = json.loads((PREVIOUS / 'result_summary.json').read_text('utf8'))
    assert result['status'] == 'software_complete' and result['powered_tail_valid']
    measurement = json.loads((PREVIOUS / 'operator_measurement.json').read_text('utf8'))
    assert measurement['next_run_explicitly_authorized']
    auth = json.loads((PREVIOUS / 'authorization.json').read_text('utf8'))
    for name, digest in auth['all_changed_source_sha256'].items():
        assert hashlib.sha256((R / name).read_bytes()).hexdigest() == digest
    downloaded = json.loads((FIRST / 'download_once_receipt.json').read_text('utf8'))
    assert downloaded['success'] and downloaded['candidate_sha256'] == auth['candidate_sha256']
    client = (PREVIOUS / 'p1_serial_motion_powered_done.py').read_bytes()
    assert hashlib.sha256(client).hexdigest() == auth['client_sha256']
    scripts = {name: (PREVIOUS / name).read_bytes() for name in ['reset_once.py', 'run_once.py', 'analyze_once.py']}
    for name, data in scripts.items():
        ast.parse(data)
        if name != 'analyze_once.py':
            assert data.count(b"(E.parent / 's25-run1/download_once_receipt.json')") == 1
    auth.update(scope='Explicit third trial requested by operator; one reset and one start, same S2.5, no download.',
                site_reply=measurement['user_reply'], downloads_authorized=0,
                resets_authorized=1, starts_authorized=1,
                authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat())
    OUT.mkdir()
    (OUT / 'p1_serial_motion_powered_done.py').write_bytes(client)
    for name, data in scripts.items():
        (OUT / name).write_bytes(data)
    (OUT / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    print('Third run package ready; no reset/start/download by this preparation.')


if __name__ == '__main__':
    main()
