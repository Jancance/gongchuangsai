from pathlib import Path

root = Path.cwd()
original = root / "工创赛/evidence/navigation-planning/2026-11-15/bounded-lateral-enhancement/check_actual_entry.py"
source = original.read_text(encoding="utf-8")
old_evidence = "工创赛/evidence/navigation-planning/2026-11-15/bounded-lateral-enhancement"
new_evidence = "工创赛/evidence/navigation-planning/2026-11-15/s25-field-candidate"
old_temp = "工创赛/tmp/bounded-lateral-c-check-20261115"
new_temp = "工创赛/tmp/baseline40-s25-c-check-20261115"
assert source.count(old_evidence) >= 1 and source.count(old_temp) == 1
source = source.replace(old_evidence, new_evidence).replace(old_temp, new_temp)
unit_build = "unit=prefix+"
assert source.count(unit_build) == 1
source = source.replace(
    unit_build,
    "tests=tests.replace('static const int32_t ex[4]={0,5,10,20};','static const int32_t ex[4]={0,5,11,20};')\n" + unit_build,
)
namespace = {"__file__": __file__, "__name__": "__main__"}
exec(compile(source, str(original), "exec"), namespace)
import hashlib
import json

receipt = root / new_evidence / 'actual_c_check.json'
report = json.loads(receipt.read_text(encoding='utf-8'))
report['reused_fixture_source_sha256'] = hashlib.sha256(original.read_bytes()).hexdigest()
report['effective_runner_source_sha256'] = hashlib.sha256(source.encode('utf-8')).hexdigest()
report['expectation_change'] = 'S3 errors 0/1/2/4 mm -> deci 0/5/10/20; S2.5 -> 0/5/11/20; all other assertions unchanged'
receipt.write_text(json.dumps(report, indent=2), encoding='utf-8')
