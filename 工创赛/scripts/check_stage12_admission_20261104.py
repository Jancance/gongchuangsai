"""Run the whole-chain fixture with the actual main.c admission predicates."""
from pathlib import Path
import os
R=Path(__file__).resolve().parents[1]
os.environ.setdefault('STAGE12_SOURCE_ROOT',str(R/'tmp/stage12_admission_20261104'))
os.environ.setdefault('STAGE12_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/stage12-admission/after'))
source=(R/'scripts/check_stage12_chain_20261104.py').read_text(encoding='utf-8')
source=source.replace("unit=prefix+r'''", "from stage12_admission_fixture import inject,extend\nprefix=inject(prefix,S)\nunit=prefix+r'''")
source=source.replace("p=E/'whole-unit.c';", "unit=extend(unit)\np=E/'whole-unit.c';")
source=source.replace('cases=40,','cases=60,')
source=source.replace("(E/'result.json').write_text", "report['actual_main_admission_predicates']=True\nreport['source_sha256']['CHASSIS/USER/main.c']=hashlib.sha256((S/'CHASSIS/USER/main.c').read_bytes()).hexdigest()\n(E/'result.json').write_text")
exec(compile(source,str(R/'scripts/check_stage12_chain_20261104.py'),'exec'))
from vehicle_uart_capture import parse_mission
frames=[parse_mission(l.encode()) for l in f.stdout.splitlines() if l.startswith('VM2,')]
assert frames and all(v is not None for v in frames)
report['VM2_roundtrip_frames']=len(frames)
(E/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
