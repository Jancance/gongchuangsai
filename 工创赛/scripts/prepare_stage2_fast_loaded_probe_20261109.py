"""Add supervised left carry after observed 5mm clearance. No hardware access."""
import hashlib
import json
from pathlib import Path
from prepare_stage2_fast_candidate_20261109 import replace

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text())['gate_cases']==191
assert json.loads((E/'integration13-left-up5-once-20261109-201739.json').read_text())['complete']
p=P/'integration10_complete.inc'
d=p.read_bytes()
helper='''/* Left single-piece loaded commissioning: the operator confirmed actual
 * clearance after 5mm rise with the other source positions and tray empty.
 * Retract at that real height, keep rising, and yaw only at full safe height.
 * No general three-piece path permission is established by this entry. */
static uint32_t it_loaded_overlap_begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 if(!SingleTeach_ReferencesReady() || !it_dm_fresh() ||
  fabsf(g_dm_can2_logical_position_deg-195.0f)>1.0f ||
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg+5537.025f)>45.0f ||
  g_lift_ruler_controller.operation!=2U || g_lift_ruler_controller.samples==0U ||
  g_lift_compare_target_pulses!=118800U || g_it_lift_pulses!=7200.0f ||
  g_lift_ruler_request || g_lift_ruler_stop_pending || VehicleArm_Busy())return 0U;
 memset(&it_command,0,sizeof(it_command));
 it_command.sequence=++g_it_sequence;it_command.kind=SPF_CMD_TRANSFER;
 it_command.axes_mask=7U;it_command.goal.dm_deg=38.380798f;
 it_command.goal.extension_mm=0.0f;it_command.goal.lift_pulses=7200.0f;
 it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=SPF_EXT_RPM;
 it_command.lift_rpm=SPF_LIFT_RPM;it_command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;
 it_command.keep_vm_on=1U;
 if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U))return 0U;
 vsp_group.delayed_mask=SPF_EXT|SPF_DM;vsp_group.release_mask=SPF_LIFT;
 vsp_fast_lift=1U;return 1U;
#endif
}
static void it_loaded_overlap_gates(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1];float depth;
 if(g_it_state!=1U || g_it_action!=15U || !VehicleSourceParallel_Owns() ||
  !z->position_cycle || ((uint32_t)(now-z->position_tick)>150U && !(vsp_group.done&SPF_LIFT)))return;
 depth=(z->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 if(depth>=0.0f && depth<=110800.0f)vsp_group.release_mask|=SPF_EXT;
 if((vsp_group.done&(SPF_EXT|SPF_LIFT))==(SPF_EXT|SPF_LIFT) &&
  vsp_axis[0].samples>=3U && fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=1.0f)
  vsp_group.release_mask|=SPF_DM;
#else
 (void)now;
#endif
}
static void it_loaded_overlap_publish(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
 /* Publish only an actual fully completed group, never its target/ACK. */
 g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*
  (float)g_lift_ruler_controller.top_scale+0.5f);
 g_lift_ruler_controller.safe_position_raw=g_lift_ruler_controller.position_after_raw;
 g_lift_ruler_controller.backoff_valid=1U;g_lift_ruler_controller.operation=3U;
 g_lift_ruler_controller.samples++;
#endif
}

'''
d=replace(d,'void Integration10_Service(void)\r\n',helper.replace('\n','\r\n')+'void Integration10_Service(void)\r\n')
d=replace(d,'request>14U','request>15U')
d=replace(d,'   }else if(request==14U){\r\n',
          '   }else if(request==15U){\r\n'
          '    if(it_loaded_overlap_begin())g_it_state=1U;else g_it_rejected=11U;\r\n'
          '   }else if(request==14U){\r\n')
d=replace(d,' if(VehicleSourceParallel_Owns())VehicleArm_Service();\r\n',
          ' it_loaded_overlap_gates((uint32_t)xTaskGetTickCount());\r\n'
          ' if(VehicleSourceParallel_Owns())VehicleArm_Service();\r\n')
d=replace(d,' case 14U:\r\n',
          ' case 15U:\r\n'
          '  if(g_source_parallel_state==4U){it_fail(8U);return;}\r\n'
          '  if(g_source_parallel_state==3U && !VehicleSourceParallel_Owns() &&\r\n'
          '   g_source_parallel_done_mask==7U){it_loaded_overlap_publish();it_done();}break;\r\n'
          ' case 14U:\r\n')
p.write_bytes(d)
f=E/'stage2-fast-candidate-20261109.json';m=json.loads(f.read_text())
history=m.setdefault('previous_deployed_images',[])
history.append({'images':m['images'],'flash_log':m.get('flash_log'),'empty_field_run':m.get('empty_field_run')})
m['changed'][p.name]=hashlib.sha256(d).hexdigest();m['images']={}
m['deployed']=False;m['hardware_motion']=False
m['loaded_test_entry']={'it_request':15,'left_yaw_deg':195,'retract_depth_virtual_pulses':110800,
 'safe_depth_virtual_pulses':7200,'tray_other_source_positions_empty_required':True,'field_verified':False}
f.write_text(json.dumps(m,indent=2,ensure_ascii=False)+'\n')
print('Prepared IT15 loaded single-piece candidate; not flashed.')
