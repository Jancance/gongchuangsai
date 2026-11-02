"""Build-gate and actual-C checks for the prepared pair; never hardware."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'工创赛/scripts'))
from p1_longitudinal_feedforward_sim import adapter

out=Path(__file__).resolve().parent
gate=out/'compile_gates';gate.mkdir(exist_ok=True)
(gate/'unit.c').write_text('#include <stdint.h>\n#include "longitudinal_feedforward.h"\nfloat probe(void){return LongitudinalFeedforward(0,50);}\n')
tests=[('default_off_vehicle',['APP_X_OFFLINE_ONLY=0'],True),
       ('ff_vehicle_without_optin',['APP_X_OFFLINE_ONLY=0','APP_FF_FORWARD_ENABLE=1'],False),
       ('ff_explicit_vehicle',['APP_X_OFFLINE_ONLY=0','APP_FF_FORWARD_ENABLE=1','APP_FF_FORWARD_PERMILLE=5','APP_FF_VEHICLE_OPT_IN=1'],True),
       ('ff_offline',['APP_X_OFFLINE_ONLY=1','APP_FF_FORWARD_ENABLE=1','APP_FF_FORWARD_PERMILLE=5'],True),
       ('invalid_optin',['APP_FF_VEHICLE_OPT_IN=2'],False),
       ('non_mode3',['APP_DEBUG_MODE=2','APP_FF_FORWARD_ENABLE=1'],False),
       ('competition',['APP_ENTRY=2','APP_FF_FORWARD_ENABLE=1'],False)]
rows=[]
for name,defines,expected in tests:
    command=['D:/Keil_v5/ARM/arm5_compiler/bin/armcc.exe','--cpu','Cortex-M3','--c99',
             '-I',str(out/'firmware/USER'),*['-D'+d for d in defines],'-c','unit.c','-o',name+'.o']
    p=subprocess.run(command,cwd=gate,capture_output=True,text=True,errors='replace',timeout=60)
    rows.append(dict(name=name,expected_compile=expected,returncode=p.returncode,passed=(p.returncode==0)==expected,
                     output=p.stdout+p.stderr,command=command))
(gate/'report.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
assert all(r['passed'] for r in rows)
print('7 build gates passed',flush=True)
checks=out/'eligible_wheel_check';checks.mkdir(exist_ok=False)
module=adapter(checks,5,-1,False,corrected_endpoint=True)
p=checks/'adapter_regression/fixture.c';source=p.read_text(encoding='utf8')
source=source.replace('static uint32_t micro_case,micro_injected;',
    'static uint32_t micro_case,micro_injected,s_test_endpoint_wheel;\nvolatile uint32_t endpoint_wheel[56];')
source=source.replace(' micro_injected=0;', ' micro_injected=0;s_test_endpoint_wheel=0;')
source=source.replace('a==1U && s_translation_mode==2U','a==s_test_endpoint_wheel && s_translation_mode==2U')
anchor=' if((micro_case==2U && micro_injected'
assert source.count(anchor)==1
source=source.replace(anchor,''' /* Select an actually eligible wheel; do not claim coverage if ID1 never qualifies. */
 if(micro_case && !micro_injected && s_translation_mode==2U &&
    (g_chassis_auto_state==CHASSIS_AUTO_RUN || g_chassis_auto_state==CHASSIS_AUTO_START) &&
    EndpointInjectionAllowed(a))s_test_endpoint_wheel=a;
'''+anchor)
source=source.replace('if(!micro_injected) {\n   endpoint_injections',
    'if(!micro_injected) {\n   endpoint_wheel[scenario]=a;\n   endpoint_injections')
p.write_text(source,encoding='utf8')
p=checks/'adapter_regression/run_models.py';runner=p.read_text(encoding='utf8')
marker="    (out/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')"
runner=runner.replace(marker,"    report['endpoint_wheel']=list(struct.unpack('<56I',m.mem_read(address('endpoint_wheel'),224)))[:count]\n"+marker)
p.write_text(runner,encoding='utf8')
spec=importlib.util.spec_from_file_location('small_ff_selected_endpoint',p)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
report=module.run(out/'firmware',checks/'models_ff005_forward_only')
assert report['passed']
assert len(report['endpoint_injections'])==4
assert all(report['endpoint_wheel'][r['scenario']] in (1,2,3,4) for r in report['endpoint_injections'])
print('Prepared forward-only 0.5% configuration passed actual-C checks',flush=True)
