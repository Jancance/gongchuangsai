"""Integrate immediate owner-service chaining in the isolated candidate."""
from pathlib import Path
import hashlib,json,shutil
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-immediate-matlab-20261109.json').read_text())['passed']
p=P/'integration10_complete.inc';d=p.read_bytes()
d=replace(d,'void Integration10_Service(void)\r\n',
 '#include "integration13_immediate.inc"\r\nvoid Integration10_Service(void)\r\n')
d=replace(d,' if(request==9U){Stage2Fast_Cancel(2U);',
 ' if(request==9U){if(g_it_auto_state==1U){g_it_auto_state=9U;g_it_auto_fault=2U;}Stage2Fast_Cancel(2U);')
d=replace(d,' if(request){\r\n  if(Stage2Fast_IsActive()',
 ' if(request==16U){\r\n'
 '  if(!it_auto_start(now))g_it_rejected=12U;request=0U;\r\n'
 ' }\r\n'
 ' if(request){\r\n  if(Stage2Fast_IsActive()')
d=replace(d,' if(g_it_state!=1U)return;\r\n',
 ' if(g_it_state!=1U){it_auto_service((uint32_t)xTaskGetTickCount());return;}\r\n')
assert d.endswith(b' }\r\n}\r\n')
d=d[:-len(b' }\r\n}\r\n')]+b' }\r\n it_auto_service((uint32_t)xTaskGetTickCount());\r\n}\r\n'
p.write_bytes(d);s=R/'software_interfaces/stage2_fast_cycle/integration13_immediate.inc';shutil.copy2(s,P/s.name)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
for p in (p,P/s.name):m['changed'][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
m['immediate_chain']={'it_request':16,'fixed_grip_delay_ms':0,'fixed_release_delay_ms':0,
 'source':'software_interfaces/stage2_fast_cycle/integration13_immediate.inc',
 'physical_grasp_or_placement_feedback':False,'route3_enabled':False}
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Prepared sole-owner immediate three-item chain. No device access.')
