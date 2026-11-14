"""Prepare the explicitly authorized second run, with no flash or hardware access."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import ast
import hashlib
import json

R = Path.cwd()
FIELD = R / '工创赛/evidence/navigation-planning/2026-11-15/baseline40-field'
FIRST = FIELD / 's25-run1'
OUT = FIELD / 's25-run2'


def main():
    assert not OUT.exists(), 'Do not recreate or retry an existing run.'
    previous = json.loads((FIRST / 'result_summary.json').read_text('utf8'))
    assert previous['status'] == 'software_complete' and previous['powered_tail_valid']
    auth = json.loads((FIRST / 'authorization.json').read_text('utf8'))
    for name, digest in auth['all_changed_source_sha256'].items():
        assert hashlib.sha256((R / name).read_bytes()).hexdigest() == digest
    downloaded = json.loads((FIRST / 'download_once_receipt.json').read_text('utf8'))
    assert downloaded['success'] and downloaded['candidate_sha256'] == auth['candidate_sha256']
    client = (FIRST / 'p1_serial_motion_powered_done.py').read_bytes()
    assert hashlib.sha256(client).hexdigest() == auth['client_sha256']
    scripts = {}
    for name in ['reset_once.py', 'run_once.py']:
        text = (FIRST / name).read_text('utf8')
        reference = "(E / 'download_once_receipt.json')"
        assert text.count(reference) == 1
        text = text.replace(reference, "(E.parent / 's25-run1/download_once_receipt.json')")
        ast.parse(text)
        scripts[name] = text
    auth.update(
        scope='User explicitly requests one second trial, already placed, reset once only; same S2.5 firmware, no download.',
        site_reply='右偏0.8cm，多走0.5cm可以再测一次已经摆好你复位就行',
        downloads_authorized=0, resets_authorized=1, starts_authorized=1,
        authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat(),
    )
    OUT.mkdir()
    (OUT / 'p1_serial_motion_powered_done.py').write_bytes(client)
    for name, text in scripts.items():
        (OUT / name).write_text(text, encoding='utf8')
    (OUT / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    (OUT / 'analyze_once.py').write_bytes((FIRST / 'analyze_once.py').read_bytes())
    print('Second run package ready; no download, reset or start performed by preparation.')


if __name__ == '__main__':
    main()
