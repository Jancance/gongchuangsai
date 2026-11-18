"""Replay the current STOP return boundary in the actual Cortex-M4 owner."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


stage = Path(__file__).resolve().parent
repo = next(path for path in stage.parents if (path / ".git").exists())
prior = repo / "工创赛/scripts/check_emm_migration.py"
output = stage / f"stop-arm-attempt{int(sys.argv[1])}"
assert not output.exists()
baseline = len(sys.argv) == 3 and sys.argv[2] == "before"
source = repo / "3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c"
if baseline:
    output.mkdir()
    reference = subprocess.check_output(
        ["git", "show", "8d7138f5:" + source.relative_to(repo).as_posix()], cwd=repo)
    source = output / source.name
    source.write_bytes(reference)

runner = prior.read_text(encoding="utf-8")
for before, after in (
    ("'Cortex-M3','--c99'", "'Cortex-M4','--c99'"),
    ("return test_tick++;", "return test_tick;"),
    ("[(0,0),(0,1),(1,0),(1,1)]", "[(0,1),(1,1)]"),
    ("inc=[stub,FW/'CORE'", "inc=[stub,OUT,FW/'CORE'"),
    ("        sources += [(FW/'StdPeriph_Driver/src'/f'stm32f4xx_{n}.c',n+'.o') for n in ['can','rcc','gpio']]",
     "        sources.append((FW/'USER/motion_profile.c','motion_profile.o'))"),
    ("report['checks']>2000 and not report['failures']",
     "report['checks']>=40 and bool(report['failures'])==BASELINE"),
):
    assert runner.count(before) == 1
    runner = runner.replace(before, after)

ns = {"__name__": "__p2_stop_fixture__", "__file__": str(prior)}
exec(compile(runner, str(prior), "exec"), ns)
fixture = stage / "stop_fixture.c"
ns["UNIT"] = fixture.read_text(encoding="utf-8")
ns["OUT"] = output
ns["FW"] = repo / "3.stm32f407工程/CHASSIS"
ns["BASELINE"] = baseline
sys.argv = [sys.argv[0]]
digest = lambda file: hashlib.sha256(file.read_bytes()).hexdigest()
paths = [source, fixture, prior, Path(__file__).resolve()]
before = {str(path.relative_to(repo)): digest(path) for path in paths}
try:
    ns["main"]()
finally:
    output.mkdir(exist_ok=True)
    after = {str(path.relative_to(repo)): digest(path) for path in paths}
    (output / "inputs.json").write_text(
        json.dumps({"baseline_expected_failures": baseline, "before": before, "after": after},
                   indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
assert before == after
