from pathlib import Path
import datetime
import hashlib
import importlib.util
import json
import subprocess
import sys

E = Path(__file__).parent
client = E / 'p1_serial_motion_powered_done.py'
R = Path(__file__).resolve().parents[6]
auth = json.loads((E / 'authorization.json').read_text('utf8'))

for source, expected_hash in auth['all_changed_source_sha256'].items():
    assert hashlib.sha256((R / source).read_bytes()).hexdigest() == expected_hash
assert auth['site_ready'] and auth['local_review_passed'], 'Prepared only: local final review and this-run site confirmation required.'
assert auth['candidate_sha256'] == '41dd9265f971f4d78a83bf93a2db6b52535d01701409aa83c528f4d2a758f7df'
assert hashlib.sha256((R / '3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c').read_bytes()).hexdigest() == '5742f22ba37b9fc211c5129d6a8dab607fdf8f376e5bc6fcc31070ec18208c1b'
assert hashlib.sha256((R / '3.stm32f407工程/CHASSIS/USER/chassis_kinematics.c').read_bytes()).hexdigest() == 'b54a676083a557fd5a1266d34b5c174e48b569fae5dd98ab46e53e18059ee5e0'
download = json.loads((E.parent / 's25-run1/download_once_receipt.json').read_text('utf8'))
assert download['success'] and download['candidate_unchanged']
assert download['candidate_sha256'] == auth['candidate_sha256']
assert hashlib.sha256(client.read_bytes()).hexdigest() == auth['client_sha256']
assert auth['site_ready'] and auth['starts_authorized'] == 1
reset = json.loads((E / 'reset_once_receipt.json').read_text('utf8'))
assert reset['reset']
marker = E / 'run_once_started.json'
assert not marker.exists(), 'This single run has already been attempted.'
pre = E / 'preflight'
assert not pre.exists(), 'Inspect the existing preflight; do not repeat implicitly.'
cmd = [sys.executable, str(client), '--port', 'COM17', '--mode', '3']
with (E / 'preflight_client.log').open('wb') as log:
    p = subprocess.run(cmd + ['--action', 'monitor', '--seconds', '3', '--output', str(pre)], stdout=log, stderr=subprocess.STDOUT, timeout=12)
summaries = list(pre.glob('*.summary.json'))
assert p.returncode == 0 and len(summaries) == 1
monitor = json.loads(summaries[0].read_text('utf8'))
spec = importlib.util.spec_from_file_location('field_client', client)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
last = monitor['last']
assert m.Controller('run-once', 3, 8)._healthy(last), 'Preflight is not healthy; no run.'
assert last['debug_plan_id'] == 8
last['fresh_start_reference_hwt_deg'] = last['yaw_mdeg']/1000
last['reference_policy'] = auth['reference_policy']
(E / 'preflight_verified.json').write_text(json.dumps(dict(passed=True, last=last), indent=2), 'utf8')
run_cmd = cmd + ['--action', 'run-once', '--plan', 'forward210_40', '--expect-powered-done', '--seconds', '30', '--output', str(E / 'serial')]
marker.write_text(json.dumps(dict(started_at=datetime.datetime.now().astimezone().isoformat(), command=run_cmd, attempts=1, automatic_retry=False), indent=2), 'utf8')
print('Preflight healthy. Fresh start HWT=' + str(last['yaw_mdeg']/1000) + ' deg. Launching the authorized single PLAN8 run.', flush=True)
with (E / 'run_client.log').open('wb') as log:
    p = subprocess.run(run_cmd, stdout=log, stderr=subprocess.STDOUT, timeout=65)
summaries = list((E / 'serial').glob('*.summary.json'))
assert len(summaries) == 1
result = json.loads(summaries[0].read_text('utf8'))
compact = {k: result.get(k) for k in ['status', 'error', 'valid_frames', 'invalid_frames', 'sent_commands', 'max_abs_rpm', 'max_abs_error_deg', 'last_fault', 'last', 'capture_prefix', 'powered_tail_valid', 'powered_tail_frames', 'powered_tail_last_elapsed_s', 'powered_tail_first_invalid', 'powered_tail_incomplete']}
compact['returncode'] = p.returncode
compact['physical_measurement'] = 'pending operator observation'
(E / 'result_summary.json').write_text(json.dumps(compact, indent=2), 'utf8')
print(json.dumps(compact, ensure_ascii=True), flush=True)
raise SystemExit(p.returncode)
