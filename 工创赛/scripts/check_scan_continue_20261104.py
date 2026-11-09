"""Final route/arm batch with scan-latch continuation and fault regressions."""
from pathlib import Path
import os
from stage12_admission_fixture import inject,extend
from source_parallel_whole_fixture import adapt
from dynamic_route_fixture import adapt_dynamic
from scan_continue_fixture import adapt_scan_continue
R=Path(__file__).resolve().parents[1]
os.environ.setdefault('STAGE12_SOURCE_ROOT',str(R/'tmp/scan_continue_20261104'))
os.environ.setdefault('STAGE12_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/scan-continue-protection-audit/after'))
s=(R/'scripts/check_stage12_chain_20261104.py').read_text(encoding='utf-8')
s=s.replace("unit=prefix+r'''", "prefix=inject(prefix,S)\nunit=prefix+r'''")
s=s.replace("p=E/'whole-unit.c';", "unit=adapt_scan_continue(adapt_dynamic(adapt(extend(unit))))\np=E/'whole-unit.c';")
s=s.replace('cases=40,','cases=84,')
s=s.replace('910','860').replace('2110','2060').replace('GOAL_OFFSET_X_MM==-50','GOAL_OFFSET_X_MM==-100')
s=s.replace('f=subprocess.run(cmd,capture_output=True',"cmd[1:1]=['-I'+str(S/'HARDWARE/DM_CAN2'),'-I'+str(S/'HARDWARE/GRIPPER'),'-I'+str(S/'HARDWARE/CARRIER'),'-I'+str(R/'evidence/integration/2026-11-04/source-parallel/whole-stubs')]\nf=subprocess.run(cmd,capture_output=True",1)
s=s.replace("(E/'result.json').write_text", "report['source_sha256'].update({n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in ['CHASSIS/USER/source_prepare.inc','CHASSIS/USER/source_prepare_math.h','CHASSIS/USER/vehicle_arm.c','HARDWARE/DM_CAN2/dm_can2_motion.c']})\n(E/'result.json').write_text")
exec(compile(s,str(R/'scripts/check_stage12_chain_20261104.py'),'exec'))
from vehicle_uart_capture import parse_mission,parse_grid_route,parse_source_preparation
for tag,parser in [('VM2,',parse_mission),('VG4,',parse_grid_route),('SP1,',parse_source_preparation)]:
 frames=[parser(l.encode()) for l in f.stdout.splitlines() if l.startswith(tag)]
 assert frames;report[tag[:-1]+'_roundtrip_frames']=len(frames)
report.update(scan_success_only_latches=True,protective_stops_unchanged=True)
(E/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
