"""Prepare the explicitly requested retry after probe-open failed before reset."""
from pathlib import Path
import json

R = Path.cwd()
original = R / '工创赛/scripts/prepare_diagnostic_run2_20261115.py'
text = original.read_text(encoding='utf8')
before = "OUT = FIELD / 'diagnostic-run2'"
assert text.count(before) == 1
text = text.replace(before, "OUT = FIELD / 'diagnostic-run3-retry1'")
namespace = {'__file__': str(original), '__name__': 'explicit_probe_failure_retry'}
exec(compile(text, str(original), 'exec'), namespace)
namespace['main']()
path = namespace['OUT'] / 'authorization.json'
auth = json.loads(path.read_text('utf8'))
auth.update(scope='One explicit retry following USB probe-open timeout before any reset or start; no download, one reset/start and one post-tail RAM acquisition.',
            site_reply='再来一次', retry_explicitly_authorized=True,
            previous_failure_reference='../diagnostic-run3/reset_once_receipt.json',
            operator_USB_replug_confirmation='not explicitly provided; do not claim a replug occurred')
path.write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
print('Explicitly authorized probe-failure retry prepared; no hardware action by preparation.')
