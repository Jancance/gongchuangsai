"""Prepare only the operator's explicit retry; keep failed attempt untouched."""
from pathlib import Path
import json

R = Path.cwd()
original = R / '工创赛/scripts/prepare_diagnostic_run2_20261115.py'
source = original.read_text(encoding='utf8')
before = "OUT = FIELD / 'diagnostic-run2'"
assert source.count(before) == 1
source = source.replace(before, "OUT = FIELD / 'diagnostic-run2-retry1'")
namespace = {'__file__': str(original), '__name__': 'retry_package_template'}
exec(compile(source, str(original), 'exec'), namespace)
namespace['main']()
out = namespace['OUT']
path = out / 'authorization.json'
auth = json.loads(path.read_text('utf8'))
auth.update(
    scope='Explicitly authorized single retry after telemetry_session_changed; preserve failed attempt, one reset/start, no download, one post-tail RAM acquisition.',
    site_reply='重新来一次复位然后开始',
    retry_explicitly_authorized=True,
    previous_failure_reference='../diagnostic-run2/first_failure_summary.json',
    previous_probe_read_error='../diagnostic-run2/reset_observation.json',
    cause='Unknown: operator did not answer whether a manual reset occurred; do not attribute reset cause.',
)
path.write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
print('Explicit retry package ready; no hardware accessed.')
