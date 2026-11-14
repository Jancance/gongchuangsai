from pathlib import Path
import datetime
import hashlib
import json
import shutil
import subprocess

R = Path(__file__).resolve().parents[6]
E = Path(__file__).parent
F = E.parent
auth = json.loads((E / 'authorization.json').read_text('utf8'))

for source, expected_hash in auth['all_changed_source_sha256'].items():
    assert hashlib.sha256((R / source).read_bytes()).hexdigest() == expected_hash
assert auth['site_ready'] and auth['local_review_passed'], 'Prepared only: local final review and this-run site confirmation required.'
assert auth['candidate_sha256'] == '5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
assert hashlib.sha256((R / '3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c').read_bytes()).hexdigest() == 'ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b'
assert hashlib.sha256((R / '3.stm32f407工程/CHASSIS/USER/chassis_kinematics.c').read_bytes()).hexdigest() == 'b54a676083a557fd5a1266d34b5c174e48b569fae5dd98ab46e53e18059ee5e0'
assert auth['downloads_authorized'] == 1 and auth['starts_authorized'] == 1
B = R / '\u5de5\u521b\u8d5b/tmp/tail-snapshot-build-20261115'
P = B / 'baseline40_integrated.uvprojx'
A = B / 'baseline40_integrated/Vehicle_Integrated.axf'
O = B / 'baseline40_integrated.uvoptx'
expected = '5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(A) == expected
source = R / '3.stm32f407\u5de5\u7a0b/CHASSIS/USER/chassis_auto_test.c'
assert sha(source) == 'ada294ab54d230e1c791a701b99c9007dc3c15ee7711633d63fe9855b4de122b'
build = json.loads((R / '工创赛/evidence/navigation-planning/2026-11-15/stop-distance-study/snapshot-candidate/build_summary.json').read_text('utf8'))
assert sha(P) == build['builds']['baseline40_integrated']['project_copy_sha256']
template = R / '\u5de5\u521b\u8d5b/tmp/lateral-feedback-build-20261113/baseline40_integrated.uvoptx'
assert sha(template) == 'ef26b9526f5e67ece67bd4e7c268f1f57f628aedb1736be2ba357ebc07b420ba'
receipt = E / 'download_once_receipt.json'
assert not receipt.exists(), 'Download already attempted; do not retry.'
assert not O.exists(), 'Inspect existing options before proceeding.'
shutil.copyfile(template, O)
stamp = datetime.datetime.now().strftime('%H%M%S_%f')
log = E / ('flash_' + stamp + '.log')
r = dict(command=['D:/Keil_v5/UV4/UV4.exe', '-f', str(P), '-t', 'Vehicle_Integrated', '-o', str(log)],
         candidate_sha256=expected, project_sha256=sha(P), options_sha256=sha(O), attempts=1,
         automatic_retry=False, motion_request_written=False, log=str(log), started_at=datetime.datetime.now().astimezone().isoformat())
receipt.write_text(json.dumps(r, indent=2), 'utf8')
try:
    p = subprocess.run(r['command'], cwd=R, timeout=30, capture_output=True)
    r['returncode'] = p.returncode
except subprocess.TimeoutExpired:
    r['error'] = '30s timeout; no automatic retry'
finally:
    data = log.read_bytes().decode('cp936', errors='replace') if log.exists() else ''
    r['markers'] = {m: m in data for m in ['Erase Done.', 'Programming Done.', 'Verify OK.', 'Application running ...']}
    r['success'] = r.get('returncode') == 0 and all(r['markers'].values())
    r['candidate_unchanged'] = sha(A) == expected
    receipt.write_text(json.dumps(r, indent=2), 'utf8')
    print(json.dumps(r, ensure_ascii=False), flush=True)
if not r['success'] or not r['candidate_unchanged']:
    raise SystemExit(1)
