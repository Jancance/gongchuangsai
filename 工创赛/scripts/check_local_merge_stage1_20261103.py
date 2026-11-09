"""Run upstream actual-C stage-1 assertions in this imported workspace layout; no hardware."""
from pathlib import Path
import importlib.util,subprocess,json
R=Path(__file__).resolve().parents[1];FW=R/'3.stm32f407工程'
EMM=(FW/'USER/zdt_emm_config.h').exists()
OUT=R/('evidence/integration/2026-11-03/local-emm-update' if EMM else 'evidence/integration/2026-11-03/local-merge')
OUT.mkdir(parents=True,exist_ok=True)
spec=importlib.util.spec_from_file_location('upstream_stage1',R/'scripts/check_map_stage1.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
unit=module.UNIT+'\nint main(void){test_entry();printf("checks=%u failures=%u\\n",(unsigned)checks,(unsigned)failures);return failures?1:0;}\n'
(OUT/'unit.c').write_text(unit,encoding='utf-8')
reports=[]
for offline in [0,1]:
    exe=OUT/('stage1-'+str(offline)+'.exe')
    define='APP_MOTOR_OFFLINE_ONLY' if EMM else 'APP_X_OFFLINE_ONLY'
    cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-D'+define+'='+str(offline),'-I'+str(FW/'USER'),str(OUT/'unit.c')]
    cmd += [str(FW/'USER'/name) for name in ['field_map.c','app_entry.c','motion_profile.c','chassis_kinematics.c']]
    cmd += ['-lm','-o',str(exe)]
    q=subprocess.run(cmd,capture_output=True,text=True,errors='replace');(OUT/('host-build-'+str(offline)+'.log')).write_text(q.stdout+q.stderr,encoding='utf-8');assert q.returncode==0,q.stderr[-2500:]
    q=subprocess.run([str(exe)],capture_output=True,text=True);print('offline',offline,q.stdout.strip(),flush=True)
    reports.append(dict(offline=offline,returncode=q.returncode,output=q.stdout,stderr=q.stderr));assert q.returncode==0,q.stdout+q.stderr
(OUT/'host-check.json').write_text(json.dumps(dict(passed=True,cases=reports,actual_c=True,io_stubbed=True,host_platform='Windows GCC; not ARM emulation or vehicle acceptance',hardware_accessed=False),indent=2),encoding='utf-8')
