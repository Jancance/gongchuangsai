"""Prepare one third diagnostic trial after explicit new operator authorization."""
from pathlib import Path
import json

R = Path.cwd()
original = R / '工创赛/scripts/prepare_diagnostic_run2_20261115.py'
text = original.read_text(encoding='utf8')
before = "OUT = FIELD / 'diagnostic-run2'"
assert text.count(before) == 1
text = text.replace(before, "OUT = FIELD / 'diagnostic-run3'")
namespace = {'__file__': str(original), '__name__': 'third_diagnostic_package'}
exec(compile(text, str(original), 'exec'), namespace)
namespace['main']()
out = namespace['OUT']
path = out / 'authorization.json'
auth = json.loads(path.read_text('utf8'))
auth.update(scope='Explicit third diagnostic trial, no download or parameter change; one reset/start and one after-tail read acquisition.',
            site_reply='右偏3cm，多走1cm，再来一次，直接复位',
            previous_success_reference='../diagnostic-run2-retry1/result_summary.json',
            separate_failure_reference='../diagnostic-run2/first_failure_summary.json')
path.write_text(json.dumps(auth, indent=2, ensure_ascii=False), encoding='utf8')
print('Third explicitly authorized diagnostic package prepared; no hardware accessed.')
