"""Replay the unchanged pre-instrumentation source through the same ARM fixture."""
from pathlib import Path
import json
E=Path(__file__).parent
s=(E/'check_actual_entry.py').read_text('utf8')
s=s.replace("else read('chassis_auto_test.c');kin=","else (E/'chassis_auto_test.c.before').read_bytes().decode('gbk');kin=")
s=s.replace("globals_='#if APP_BASELINE40_ACTIVE\\n'+auto[auto.index('volatile ChassisAutoLateralTiming')", "globals_=auto[auto.index('volatile uint32_t g_chassis_auto_request')")
s='\n'.join(line for line in s.splitlines() if not line.startswith("tests=tests.replace(' uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;'"))
s=s.replace('工创赛/tmp/float-timing-c-check-20261115','工创赛/tmp/float-timing-before-c-check-20261115')
s=s.replace("'actual_c_check.json'","'baseline_control_check.json'").replace("'after_unit.c'","'baseline_unit.c'")
s=s.replace("U/'chassis_auto_test.c').read_bytes()).hexdigest()","E/'chassis_auto_test.c.before').read_bytes()).hexdigest()")
exec(compile(s,str(E/'check_actual_entry.py'),'exec'),{'__file__':str(__file__),'__name__':'__main__'})
a=json.loads((E/'actual_c_check.json').read_text('utf8'));b=json.loads((E/'baseline_control_check.json').read_text('utf8'))
assert a['entry_observation']==b['entry_observation'] and a['ff_observation']==b['ff_observation']
assert a['cases']==b['cases']+12
(E/'control_comparison.json').write_text(json.dumps({'all_passed':True,'entry_equal':True,'ff_equal':True,'before_cases':b['cases'],'after_cases':a['cases'],'before_assertions':b['assertions'],'after_assertions':a['assertions'],'chip_cycles_measured':False,'limits':'Identical fixed inputs and stubs; no proof of unchanged wall-time scheduling.'},indent=2),'utf8')
