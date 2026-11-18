"""Replay the zone 2 front end and real CAN owner with simulated radar, CAN/RTOS and pose sources."""
import hashlib
import json
from pathlib import Path
import sys

here = Path(__file__).resolve().parent
repo = next(path for path in here.parents if (path / ".git").exists())
evidence = here.parents[1]
prior = repo / "工创赛/scripts/check_emm_migration.py"
output = Path(sys.argv[1]).resolve()
assert not output.exists()
runner = prior.read_text(encoding="utf-8")
units = ("motion_profile", "fusion_motion", "zone2_nav_schedule", "zone2_nav_path",
         "fusion_reference", "field_map")
for before, after in (
    ("'Cortex-M3','--c99'", "'Cortex-M4','--c99'"),
    ("return test_tick++;", "return test_tick;"),
    ("[(0,0),(0,1),(1,0),(1,1)]", "[(0,1)]"),
    ("        sources += [(FW/'StdPeriph_Driver/src'/f'stm32f4xx_{n}.c',n+'.o') for n in ['can','rcc','gpio']]",
     f"        sources += [(FW/'USER'/f'{{n}}.c',f'{{n}}.o') for n in {units!r}]"),
    ("FW/'HARDWARE/POWER']", "FW/'HARDWARE/POWER',FW/'HARDWARE/RADAR_V6',FW/'HARDWARE/RADAR_SERVO']"),
    ("'-DUSE_STDPERIPH_DRIVER',", "'-DUSE_STDPERIPH_DRIVER','-DAPP_ZONE2_NAV_ENABLE=1','-DAPP_CONTINUOUS_MAP_ENABLE=1',"),
    ("report['checks']>2000", "report['checks']>=100"),
    ("        reports.append(report);", '        report.update({key: value("front_report_"+key) for key in ("phase","fault","reference","path","sends","stops","backend")})\n        reports.append(report);'),
):
    assert runner.count(before) == 1
    runner = runner.replace(before, after)
ns = {"__name__": "__zone2_front_fixture__", "__file__": str(prior)}
exec(compile(runner, str(prior), "exec"), ns)
fixture = here / "zone2_front_fixture.c"
owner = evidence / "p2-owner/runtime_owner_fixture.c"
fw = repo / "3.stm32f407工程/CHASSIS"
owner_text = owner.read_text(encoding="utf-8").replace("void test_entry(void)", "void owner_test_entry(void)")
ns.update(UNIT=owner_text + "\n" + fixture.read_text(encoding="utf-8"), OUT=output, FW=fw)
sys.argv = [sys.argv[0]]
inputs = [Path(__file__), fixture, owner, prior, fw / "USER/app_zone2.c",
          fw / "USER/chassis_auto_test.c", *(fw / f"USER/{n}.c" for n in units)]
hashes = {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
ns["main"]()
assert hashes == {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
(output / "inputs.json").write_text(json.dumps(hashes, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
