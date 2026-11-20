"""Compile the real owner for Cortex-M4, with controlled CAN and RTOS replies."""
import hashlib
import json
from pathlib import Path
import sys


stage = Path(__file__).resolve().parent
repo = next(path for path in stage.parents if (path / ".git").exists())
prior = repo / "工创赛/scripts/check_emm_migration.py"
output = Path(sys.argv[1]).resolve()
assert not output.exists()
runner = prior.read_text(encoding="utf-8")
for before, after in (
    ("'Cortex-M3','--c99'", "'Cortex-M4','--c99'"),
    ("return test_tick++;", "return test_tick;"),
    ("[(0,0),(0,1),(1,0),(1,1)]", "[(0,1)]"),
    ("        sources += [(FW/'StdPeriph_Driver/src'/f'stm32f4xx_{n}.c',n+'.o') for n in ['can','rcc','gpio']]",
     "        sources += [(FW/'USER'/f'{n}.c',f'{n}.o') for n in ('motion_profile','fusion_motion')]"),
    ("'-DUSE_STDPERIPH_DRIVER',", "'-DUSE_STDPERIPH_DRIVER','-DAPP_ZONE2_NAV_ENABLE=1','-DAPP_CONTINUOUS_MAP_ENABLE=1',"),
    ("report['checks']>2000", "report['checks']>=100"),
):
    assert runner.count(before) == 1
    runner = runner.replace(before, after)

ns = {"__name__": "__zone2_runtime_owner_fixture__", "__file__": str(prior)}
exec(compile(runner, str(prior), "exec"), ns)
fixture = stage / "runtime_owner_fixture.c"
fw = repo / "3.stm32f407工程/CHASSIS"
ns.update(UNIT=fixture.read_text(encoding="utf-8"), OUT=output, FW=fw)
sys.argv = [sys.argv[0]]
tracked = [fixture, prior, Path(__file__).resolve(), fw / "USER/chassis_auto_test.c",
           fw / "USER/chassis_auto_test.h", fw / "SYSTEM/CAN/can.h"]
before = {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
ns["main"]()
assert before == {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
(output / "inputs.json").write_text(json.dumps(before, indent=2) + "\n", encoding="utf-8")
