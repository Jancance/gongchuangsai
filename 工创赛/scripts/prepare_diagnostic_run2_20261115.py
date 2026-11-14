"""Prepare the explicit second diagnostic run; no hardware access."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import ast
import hashlib
import json

R = Path.cwd()
FIELD = R / '工创赛/evidence/navigation-planning/2026-11-15/baseline40-field'
FIRST = FIELD / 'diagnostic-run1'
OUT = FIELD / 'diagnostic-run2'


def main():
    assert not OUT.exists(), 'No implicit repeat or overwrite.'
    previous = json.loads((FIRST / 'result_summary.json').read_text('utf8'))
    assert previous['status'] == 'software_complete' and previous['powered_tail_valid']
    assert json.loads((FIRST / 'diagnostic_read_receipt.json').read_text('utf8'))['accepted']
    measure = json.loads((FIRST / 'operator_measurement.json').read_text('utf8'))
    assert measure['next_run_explicitly_authorized']
    auth = json.loads((FIRST / 'authorization.json').read_text('utf8'))
    for path, digest in auth['all_changed_source_sha256'].items():
        assert hashlib.sha256((R / path).read_bytes()).hexdigest() == digest
    download = json.loads((FIRST / 'download_once_receipt.json').read_text('utf8'))
    assert download['success'] and download['candidate_sha256'] == auth['candidate_sha256']
    scripts = {}
    for name in ['reset_once.py', 'run_once.py']:
        text = (FIRST / name).read_text('utf8')
        reference = "(E / 'download_once_receipt.json')"
        assert text.count(reference) == 1
        text = text.replace(reference, "(E.parent / 'diagnostic-run1/download_once_receipt.json')")
        ast.parse(text)
        scripts[name] = text
    auth.update(scope='One explicitly requested second diagnostic trial, same firmware, no download; one reset/start and one post-tail RAM double-read acquisition.',
                site_reply=measure['user_reply'], downloads_authorized=0,
                resets_authorized=1, starts_authorized=1, diagnostic_reads_authorized=1,
                authorized_at=datetime.now(timezone(timedelta(hours=8))).isoformat())
    OUT.mkdir()
    for name, text in scripts.items():
        (OUT / name).write_text(text, encoding='utf8')
    for name in ['p1_serial_motion_powered_done.py', 'analyze_once.py', 'read_diagnostics_once.py']:
        (OUT / name).write_bytes((FIRST / name).read_bytes())
    assert hashlib.sha256((OUT / 'p1_serial_motion_powered_done.py').read_bytes()).hexdigest() == auth['client_sha256']
    (OUT / 'authorization.json').write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
    print('Second diagnostic package ready. No hardware accessed.')


if __name__ == '__main__':
    main()
