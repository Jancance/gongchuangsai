"""Run the existing Cortex-M4 harness against the real CAN monitor call site."""
from hashlib import sha256
import json
from pathlib import Path
import sys


stage = Path(__file__).resolve().parent
repo = next(path for path in stage.parents if (path / ".git").exists())
prior = repo / "工创赛/scripts/check_emm_migration.py"
fixture = stage / "monitor_odometry_fixture.c"
source = repo / "3.stm32f407工程/CHASSIS/USER/chassis_auto_test.c"
output = stage / f"monitor-arm-attempt{int(sys.argv[1])}"
assert not output.exists()
runner = prior.read_text(encoding="utf-8")
for before, after in (
    ("'Cortex-M3','--c99'", "'Cortex-M4','--c99'"),
    ("return test_tick++;", "return test_tick;"),
    ("[(0,0),(0,1),(1,0),(1,1)]", "[(0,1),(1,1)]"),
    ("        sources += [(FW/'StdPeriph_Driver/src'/f'stm32f4xx_{n}.c',n+'.o') for n in ['can','rcc','gpio']]",
     "        sources.append((FW/'USER/motion_profile.c','motion_profile.o'))"),
    ("report['checks']>2000", "report['checks']>=50"),
):
    assert runner.count(before) == 1
    runner = runner.replace(before, after)

ns = {"__name__": "__p2_monitor_fixture__", "__file__": str(prior)}
exec(compile(runner, str(prior), "exec"), ns)
ns["UNIT"] = fixture.read_text(encoding="utf-8")
ns["OUT"] = output
ns["FW"] = repo / "3.stm32f407工程/CHASSIS"
sys.argv = [sys.argv[0]]
tracked = (source, fixture, prior, Path(__file__).resolve())
before = {path.name: sha256(path.read_bytes()).hexdigest() for path in tracked}
ns["main"]()
after = {path.name: sha256(path.read_bytes()).hexdigest() for path in tracked}
assert before == after
(output / "inputs.json").write_text(json.dumps(before, indent=2) + "\n", encoding="utf-8")
