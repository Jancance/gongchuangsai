"""保存任务事务的软件验证证据，不访问硬件。"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys


root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root/'src'))
from smart_carrier.task_transaction_replay import replay


def main():
    directory = root/'evidence/p2-task-carrier/2026-11-01/task-transactions'
    directory.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [sys.executable, '-X', 'utf8', '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_task*.py', '-v'],
        cwd=root, capture_output=True, text=True, encoding='utf-8')
    (directory/'tests.log').write_text(result.stdout+result.stderr, encoding='utf-8')
    intake = json.loads((root/'config/p2_task_intake_20261101.json').read_text(encoding='utf-8'))
    for start_id in (1, 2):
        snapshot = replay(intake, root/'config/task_materials_preliminary_2027.json', start_id, [])
        (directory/f'intake-replay-start{start_id}.json').write_text(
            json.dumps(snapshot, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    files = ['src/smart_carrier/task_transactions.py', 'src/smart_carrier/task_transaction_replay.py',
             'tests/test_task_transactions.py', 'config/p2_task_intake_20261101.json',
             'docs/P2_TASK_TRANSACTIONS_2026-11-01.md', 'scripts/validate_task_transactions.py']
    manifest = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), scope='software_only',
                    test_exit_code=result.returncode, motion_allowed=False,
                    hardware_commands_sent=False, firmware_modified=False,
                    real_scanner_capture_verified=False, task3_field_acceptance=False,
                    test_log='tests.log', intake_replays=['intake-replay-start1.json', 'intake-replay-start2.json'],
                    sha256={f: hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files})
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(exit_code=result.returncode, evidence=str(directory)), ensure_ascii=False))
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
