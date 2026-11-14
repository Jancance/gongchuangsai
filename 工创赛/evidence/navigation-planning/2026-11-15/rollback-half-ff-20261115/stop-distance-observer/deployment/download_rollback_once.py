"""One explicitly requested rollback flash. Never sends ARM/START or runs a test."""
from pathlib import Path
import datetime
import hashlib
import json
import shutil
import subprocess

R = Path.cwd()
E = Path(__file__).resolve().parent
STUDY = E.parent
EXPECTED = '3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    build = json.loads((STUDY / 'build_summary.json').read_text('utf8'))
    review = STUDY / 'independent_review.md'
    assert sha(review) == '37b3aec64d035d3c925a65ad4dffab49541847017868e3c4e6dfeb1c6200319b'
    for name, digest in build['source_sha256'].items():
        assert sha(R / '3.stm32f407工程/CHASSIS/USER' / name) == digest, name
    config = build['builds']['baseline40_integrated']
    image = Path(config['axf'])
    project = Path(config['command'][2])
    assert sha(image) == EXPECTED and sha(project) == config['project_copy_sha256']
    template = R / '工创赛/tmp/lateral-feedback-build-20261113/baseline40_integrated.uvoptx'
    expected_options = 'ef26b9526f5e67ece67bd4e7c268f1f57f628aedb1736be2ba357ebc07b420ba'
    assert sha(template) == expected_options
    options = project.with_suffix('.uvoptx')
    if options.exists():
        assert sha(options) == expected_options, 'Do not overwrite unknown download options.'
    else:
        shutil.copyfile(template, options)
    log = E / 'download.log'
    record = dict(
        user_request='算了不测了…现在调回到数据好的时候；然后在原来代码基础上增加那个停止的测距',
        user_authorized_rollback=True, downloads_authorized=1, starts_authorized=0,
        automatic_retry=False, attempts=1, candidate_sha256=EXPECTED,
        review_sha256=sha(review), source_sha256=build['source_sha256'],
        command=['D:/Keil_v5/UV4/UV4.exe', '-f', str(project), '-t', 'Vehicle_Integrated', '-o', str(log)],
        download_initializes_MCU=True, separate_reset_requested=False,
        ARM_START_written=False, serial_opened=False, actuator_requests_written=0,
        started_at=datetime.datetime.now().astimezone().isoformat(),
    )
    receipt = E / 'download_once_receipt.json'
    with receipt.open('x', encoding='utf8') as stream:
        json.dump(record, stream, indent=2, ensure_ascii=False)
    try:
        process = subprocess.run(record['command'], cwd=R, timeout=30, capture_output=True)
        record['returncode'] = process.returncode
    except subprocess.TimeoutExpired:
        record['error'] = '30s download timeout; no automatic retry'
    finally:
        text = log.read_bytes().decode('cp936', errors='replace') if log.exists() else ''
        record['markers'] = {s: s in text for s in ['Erase Done.', 'Programming Done.', 'Verify OK.', 'Application running ...']}
        record['candidate_unchanged'] = sha(image) == EXPECTED
        record['success'] = record.get('returncode') == 0 and all(record['markers'].values()) and record['candidate_unchanged']
        receipt.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding='utf8')
        print(json.dumps(record, ensure_ascii=True))
    if not record['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
