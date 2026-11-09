"""Implement the user's last5mm grip/yaw triggers with actual position gates."""
from pathlib import Path
import hashlib,json
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-lead5-matlab-20261109.json').read_text())['passed']
p=P/'integration10_complete.inc';d=p.read_bytes()
d=replace(d,'volatile uint32_t g_it_grip_prepared;\r\n',
 'volatile uint32_t g_it_grip_prepared;\r\nvolatile uint32_t g_it_lead5_grip_done;\r\n')
d=replace(d,' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;\r\n',
 ' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;\r\n')
d=replace(d,' if((vsp_group.done&SPF_LIFT) &&\r\n',
 ' if(depth>=0.0f && depth<=15200.0f &&\r\n')
helper='''static void it_lead5_grip_service(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1];float depth;
 if(g_it_state!=1U || g_it_action!=14U || g_it_lead5_grip_done ||
  !(g_source_parallel_started_mask&SPF_LIFT) || !z->position_cycle ||
  ((uint32_t)(now-z->position_tick)>150U && !(vsp_group.done&SPF_LIFT)) ||
  ((it_command.axes_mask&SPF_EXT) && !(vsp_group.done&SPF_EXT)))return;
 depth=(z->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 if(depth<110800.0f || depth>119200.0f)return;
 if(jiazhua(140.0f)!=1U){it_fail(7U);return;}
 g_it_lead5_grip_done=1U;
#else
 (void)now;
#endif
}

'''
d=replace(d,'#include "integration13_immediate.inc"\r\n',helper.replace('\n','\r\n')+'#include "integration13_immediate.inc"\r\n')
d=replace(d,' Stage2Fast_Service((uint32_t)xTaskGetTickCount());\r\n',
 ' Stage2Fast_Service((uint32_t)xTaskGetTickCount());\r\n'
 ' it_lead5_grip_service((uint32_t)xTaskGetTickCount());\r\n')
p.write_bytes(d)
a=P/'integration13_immediate.inc';b=a.read_bytes()
b=replace(b,'  if(jiazhua(140.0f)!=1U){',
 '  if(!g_it_lead5_grip_done && jiazhua(140.0f)!=1U){')
a.write_bytes(b);(R/'software_interfaces/stage2_fast_cycle'/a.name).write_bytes(b)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
for p in (p,a):m['changed'][p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
m['lead5']={'grip_min_depth_pulses':110800,'yaw_max_depth_pulses':15200,
 'extension_ready_before_grip':True,'extension_zero_before_yaw':True,'field_verified':False}
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n');print('Prepared actual-position last5mm grip/yaw gates.')
