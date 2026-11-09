"""Execute the final source/route batch with refreshed encoder-based stage2 legs."""
from pathlib import Path
import os
R=Path(__file__).resolve().parents[1]
os.environ.setdefault('STAGE12_SOURCE_ROOT',str(R/'tmp/dynamic_route_20261104'))
os.environ.setdefault('STAGE12_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/dynamic-route/after'))
s=(R/'scripts/check_source_parallel_whole_20261104.py').read_text(encoding='utf-8')
s=s.replace('unit=adapt(extend(unit))','unit=adapt_dynamic(adapt(extend(unit)))')
s=s.replace('from source_parallel_whole_fixture import adapt','from source_parallel_whole_fixture import adapt\\nfrom dynamic_route_fixture import adapt_dynamic')
s=s.replace("l.startswith('VG3,')","l.startswith(('VG3,','VG4,'))")
s=s.replace('cases=74,','cases=80,')
exec(compile(s,str(R/'scripts/check_source_parallel_whole_20261104.py'),'exec'))
from vehicle_uart_capture import parse_grid_route
frames=[parse_grid_route(l.encode()) for l in f.stdout.splitlines() if l.startswith('VG4,')]
assert frames
report.update(VG4_roundtrip_frames=len(frames),dynamic_per_leg_replanning=True,return_to_y240=False,stage1_forward_mm=150,goal_mm=[2060,1200])
(E/'result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
