"""Prepare one explicit retry after operator reports no vehicle movement."""
from pathlib import Path
import json

R = Path.cwd()
original = R / '工创赛/scripts/prepare_diagnostic_run2_20261115.py'
text = original.read_text(encoding='utf8')
before = "OUT = FIELD / 'diagnostic-run2'"
assert text.count(before) == 1
text = text.replace(before, "OUT = FIELD / 'diagnostic-run3-retry2'")
namespace = {'__file__': str(original), '__name__': 'explicit_no_motion_retry'}
exec(compile(text, str(original), 'exec'), namespace)
namespace['main']()
path = namespace['OUT'] / 'authorization.json'
auth = json.loads(path.read_text('utf8'))
auth.update(scope='One explicitly requested retry after operator confirmed no movement; no download, one reset/start and after-tail RAM acquisition.',
            site_reply='没动过再来一次就行', retry_explicitly_authorized=True,
            previous_failure_reference='../diagnostic-run3-retry1/first_failure_summary.json',
            operator_motion_in_previous_attempt=False,
            previous_reset_cause='Unknown; operator did not explicitly describe reset/power/cable actions')
path.write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
print('Explicit retry2 package prepared; no hardware accessed.')
