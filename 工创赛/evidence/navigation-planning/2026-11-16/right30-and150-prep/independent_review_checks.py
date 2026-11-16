"""Independent in-memory client checks. No COM, SWD, build or motion access."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
spec = importlib.util.spec_from_file_location('review_client', HERE/'p1_serial_motion_powered_done.py')
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)
expected = {8:(100,4,2100),9:(102,5,2100),10:(102,6,2100),11:(100,7,2100),12:(102,6,1500),13:(102,6,300)}
checks = 0
assert client.BASELINE40_SPECS == expected
checks += 1
for plan, (rpm, kind, distance) in expected.items():
    ctl = client.Controller('run-once',3,plan,expected_complete_powered=True)
    ctl.plan_selected = True
    ctl.baseline_tick = 1000
    r = dict(debug_plan_id=plan,tick_ms=1000,base_rpm=rpm,feedback_limit_rpm=130,
             sequence_count=1,action_kind=kind,target_theoretical_milli_cm_s=40000)
    for echoed in expected:
        ctl.baseline_config = client.baseline40_config(echoed)
        assert bool(ctl._speed_profile_healthy(r)) == (echoed == plan)
        checks += 1
    ctl.baseline_config = client.baseline40_config(plan)
    for field, bad in [('action_kind',kind+1),('base_rpm',rpm-1),('feedback_limit_rpm',129),
                       ('sequence_count',2),('target_theoretical_milli_cm_s',16793),('tick_ms',2501)]:
        changed = dict(r,**{field:bad})
        assert not ctl._speed_profile_healthy(changed)
        checks += 1
    assert f'PLAN{plan},'.encode() in client.command(f'PLAN{plan}',(1,1000))
    checks += 1
for bad in ('PLAN013','PLAN13x','PLAN130','PLAN14','PLAN-13','PLAN 13','PLAN13 '):
    try:
        client.command(bad)
    except ValueError:
        checks += 1
    else:
        raise AssertionError(bad)
unchanged = {}
for name in ('motion_profile.c','chassis_kinematics.c','chassis_auto_test.c','app_config.h','heading_controller.c'):
    path = '3.stm32f407工程/CHASSIS/USER/' + name
    baseline = subprocess.check_output(['git','show','607368de:'+path],cwd=ROOT)
    current = (ROOT/path).read_bytes()
    # Git stores LF while this working tree uses CRLF. Compare exact content
    # after only Git's line-ending normalization, and preserve current hash.
    assert baseline.replace(b'\r\n',b'\n') == current.replace(b'\r\n',b'\n'), path
    unchanged[path] = hashlib.sha256(current).hexdigest()
    checks += 1
report = dict(status='PASS',checks=checks,hardware_accessed=False,serial_objects_constructed=0,
              commands_transmitted=0,unchanged_parameter_and_controller_sources=unchanged,
              client_sha256=hashlib.sha256((HERE/'p1_serial_motion_powered_done.py').read_bytes()).hexdigest())
print(json.dumps(report,indent=2))
