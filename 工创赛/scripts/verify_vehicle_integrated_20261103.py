"""Run the final source-level checks; all motor/camera I/O is mocked or absent."""
from pathlib import Path
from vehicle_check_paths import firmware_root
import hashlib,json,subprocess,sys

R=Path(__file__).resolve().parents[1]
O=R/'evidence/integration/2026-11-03/early-scan-startup'
tests=['check_early_scan_startup_20261103.py','check_startup_axes_20261103.py',
       'check_dm_startup_barrier_20261103.py','check_vehicle_arm_20261103.py',
       'check_vehicle_mission_20261103.py','test_vehicle_uart_capture.py']
reports=[]
for name in tests:
    p=subprocess.run([sys.executable,'-X','utf8',str(R/'scripts'/name)],cwd=R,
                     capture_output=True,text=True,errors='replace',timeout=120)
    log=O/('final-'+name.removesuffix('.py')+'.log')
    log.write_text(p.stdout+p.stderr,encoding='utf-8')
    report=dict(script=name,script_sha256=hashlib.sha256((R/'scripts'/name).read_bytes()).hexdigest(),
                returncode=p.returncode,passed=p.returncode==0,log=str(log))
    reports.append(report);print(name,p.returncode,flush=True)
    (O/'test-suite.json').write_text(json.dumps(dict(passed=all(x['passed'] for x in reports),
        tests=reports,hardware_accessed=False,scope='Actual C modules/state machines; fake peripherals; logger tests. Not RTOS or vehicle acceptance.'),indent=2),encoding='utf-8')
    assert p.returncode==0,(name,p.stdout[-2000:],p.stderr[-2000:])
# A debug bypass must never silently turn into a competition profile.
p=subprocess.run(['C:/mingw/mingw64/bin/gcc.exe','-E','-DAPP_ENTRY=2',
    '-I'+str(firmware_root()/'CHASSIS/USER'),
    str(firmware_root()/'CHASSIS/USER/app_config.h')],capture_output=True,text=True,errors='replace')
assert p.returncode!=0 and 'Integrated_field_debug_is_not_verified_competition_profile' in p.stderr
(O/'competition-profile-guard.json').write_text(json.dumps(dict(passed=True,
    competition_profile_refused=True,hardware_accessed=False),indent=2),encoding='utf-8')
print('competition profile guard passed',flush=True)
