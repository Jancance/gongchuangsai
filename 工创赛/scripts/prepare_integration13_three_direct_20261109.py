"""Generalize the two verified move entries to one supervised three-piece run."""
import hashlib,json
from pathlib import Path
from prepare_stage2_fast_candidate_20261109 import replace
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'integration13-three-direct-matlab-20261109.json').read_text())['passed']
p=P/'integration10_complete.inc';d=p.read_bytes()
d=replace(d,'  fabsf(g_dm_can2_logical_position_deg-195.0f)>3.0f ||\r\n',
 '  !(g_it_dm_deg==180.0f || g_it_dm_deg==195.0f || g_it_dm_deg==165.0f) ||\r\n'
 '  fabsf(g_dm_can2_logical_position_deg-g_it_dm_deg)>1.0f ||\r\n')
d=replace(d,' it_command.axes_mask=SPF_EXT|SPF_LIFT;\r\n',
 ' it_command.axes_mask=SPF_LIFT|(g_it_extension_mm>0.0f?SPF_EXT:0U);\r\n')
d=replace(d,' it_command.goal.dm_deg=195.0f;it_command.goal.extension_mm=g_it_extension_mm;\r\n',
 ' it_command.goal.dm_deg=g_it_dm_deg;it_command.goal.extension_mm=g_it_extension_mm;\r\n')
d=replace(d,' vsp_fast_lift=1U;return 1U;\r\n#endif\r\n}\r\n\r\nstatic void it_empty_overlap_publish',
 ' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;\r\n'
 ' it_carrier_latched_deg=g_it_carrier_auto_deg;\r\n'
 ' if(jiazhua(160.0f)!=1U){VehicleSourceParallel_Cancel();return 0U;}\r\n'
 ' return 1U;\r\n#endif\r\n}\r\n\r\nstatic void it_empty_overlap_publish')
d=replace(d,'  fabsf(g_dm_can2_logical_position_deg-195.0f)>1.0f ||\r\n'
 '  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg+5537.025f)>45.0f ||\r\n',
 '  !(g_it_dm_deg==180.0f || g_it_dm_deg==195.0f || g_it_dm_deg==165.0f) ||\r\n'
 '  fabsf(g_dm_can2_logical_position_deg-g_it_dm_deg)>1.0f ||\r\n'
 '  !(g_it_extension_mm==0.0f || g_it_extension_mm==156.0f) ||\r\n'
 '  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg+\r\n'
 '   g_it_extension_mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)>45.0f ||\r\n')
d=replace(d,' it_command.axes_mask=7U;it_command.goal.dm_deg=38.380798f;\r\n',
 ' it_command.axes_mask=SPF_LIFT|SPF_DM|(g_it_extension_mm>0.0f?SPF_EXT:0U);\r\n'
 ' it_command.goal.dm_deg=38.380798f;\r\n')
d=replace(d,' vsp_group.delayed_mask=SPF_EXT|SPF_DM;vsp_group.release_mask=SPF_LIFT;\r\n',
 ' vsp_group.delayed_mask=it_command.axes_mask&~SPF_LIFT;vsp_group.release_mask=SPF_LIFT;\r\n')
d=replace(d,' if((vsp_group.done&(SPF_EXT|SPF_LIFT))==(SPF_EXT|SPF_LIFT) &&\r\n'
 '  vsp_axis[0].samples>=3U && fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=1.0f)\r\n',
 ' if((vsp_group.done&SPF_LIFT) &&\r\n'
 '  (!(vsp_group.command.axes_mask&SPF_EXT) || ((vsp_group.done&SPF_EXT) &&\r\n'
 '   vsp_axis[0].samples>=3U && fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=1.0f)))\r\n')
d=replace(d,'   g_source_parallel_done_mask==7U){it_loaded_overlap_publish();it_done();}break;\r\n',
 '   (g_source_parallel_done_mask&it_command.axes_mask)==it_command.axes_mask){it_loaded_overlap_publish();it_done();}break;\r\n')
d=replace(d,'   (g_source_parallel_done_mask&6U)==6U){it_empty_overlap_publish();it_done();}break;\r\n',
 '   (g_source_parallel_done_mask&it_command.axes_mask)==it_command.axes_mask &&\r\n'
 '   (it_carrier_latched_deg<0.0f || (g_it_carrier_auto_done &&\r\n'
 '    (uint32_t)(now-it_carrier_pwm_tick)>=700U))){it_empty_overlap_publish();it_done();}break;\r\n')
old=' it_loaded_overlap_gates((uint32_t)xTaskGetTickCount());\r\n'
new=''' if(g_it_state==1U && g_it_action==14U && !g_it_carrier_auto_done &&
  it_carrier_latched_deg>=0.0f && it_carrier_latched_deg<=270.0f &&
  (g_source_parallel_started_mask&SPF_LIFT)){
  if(!CarrierPwm_Write((uint32_t)(500.0f+it_carrier_latched_deg*2000.0f/270.0f+0.5f))){it_fail(9U);return;}
  it_carrier_pwm_tick=(uint32_t)xTaskGetTickCount();g_it_carrier_auto_done=1U;
 }
'''.replace('\n','\r\n')+old
d=replace(d,old,new);p.write_bytes(d)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text());m['changed'][p.name]=hashlib.sha256(d).hexdigest();m['images']={}
m['three_direct']={'angles':[180,195,165],'extension_mm':[0,156,156],'carry_retract_after_mm':5,
 'route3_enabled':False,'right_early_retract_field_verified':False,'after_release_stow_deg':0}
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Prepared supervised direct-three entries; no hardware call.')
