/* Included by vehicle_arm.c after Integration10. Uses its sole CAN task. */
#include "stage2_fast_cycle.h"
volatile uint32_t g_stage2_fast_state,g_stage2_fast_fault,g_stage2_fast_item;
volatile uint32_t g_stage2_fast_loaded_mask,g_stage2_fast_release_ms,g_stage2_fast_departure_ms;
#if !APP_MOTOR_OFFLINE_ONLY
static struct {
 S2Fast_Profile profile;S2Fast_Hooks hooks;SPF_Command command;
 uint32_t configured,issued,begin_ms,phase_ms,closed,close_ms,tray,tray_ms;
 uint32_t route_attempted,route_started,route_done,stop_pending;
} sf;
static uint32_t SF_Healthy(void)
{
 return g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
  !g_can_ctrl_fault_latched && !g_startup_axes_cause && !g_dm_jog_first_fault &&
  !g_lift_ruler_cause && !g_vehicle_arm_cause &&
  (!g_chassis_auto_active || g_stage2_fast_state==S2F_DEPART);
}
static void SF_State(uint32_t state,uint32_t now)
{g_stage2_fast_state=state;sf.issued=0U;sf.phase_ms=now;}
static uint32_t SF_ProfileValid(const S2Fast_Profile *p)
{
 uint32_t i;
 if(!p || !p->run_id || !p->pose_version || p->paths_verified!=1U ||
  p->timing_verified!=1U || p->driving_stow_verified!=1U ||
  !(p->slot_deg>=0.0f && p->slot_deg<=270.0f) ||
  p->pick_pulses<100000U || p->pick_pulses>SPF_MAX_LIFT_PULSES ||
  p->safe_pulses<2400U || p->safe_pulses>=p->pick_pulses ||
  p->close_lead_pulses>p->pick_pulses-p->safe_pulses ||
  !p->close_ms || p->close_ms>2000U || !p->release_ms || p->release_ms>2000U ||
  !p->carrier_ms || p->carrier_ms>2000U)return 0U;
 for(i=0U;i<3U;i++)if(!(p->pick_deg[i]>=0.0f && p->pick_deg[i]<=270.0f) ||
  !(p->extension_mm[i]>=0.0f && p->extension_mm[i]<=170.0f) ||
  p->retract_at_pulses[i]<p->safe_pulses || p->retract_at_pulses[i]>p->pick_pulses ||
  p->turn_at_pulses[i]<p->safe_pulses || p->turn_at_pulses[i]>p->pick_pulses)return 0U;
 return 1U;
}
static uint32_t SF_LiftDepth(uint32_t now,float *depth)
{
 VSP_Axis *a=&vsp_axis[1];
 if(!a->position_cycle || (now-a->position_tick>150U && !(vsp_group.done&SPF_LIFT)) ||
  !g_lift_ruler_controller.top_scale)return 0U;
 *depth=(a->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 /* Retain the backend's 45 motor-degree settled window (400 virtual pulses).
  * Early close still requires the tighter incoming target bound below. */
 return *depth>=0.0f && *depth<=(float)SPF_MAX_LIFT_PULSES+400.0f;
}
static uint32_t SF_ExtZero(void)
{
 return (vsp_group.done&SPF_EXT) && vsp_axis[0].samples>=3U &&
  fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=2.0f;
}
/* All axes are prepared once. Only their release is delayed; a timer never
 * establishes height. Started axes keep being polled while another waits. */
static void SF_ReleaseGates(uint32_t now)
{
 float depth;uint32_t item=g_stage2_fast_item;
 if(g_stage2_fast_state!=S2F_CARRY || !sf.issued || !VehicleSourceParallel_Owns() ||
  !SF_LiftDepth(now,&depth))return;
 if(depth<=(float)sf.profile.retract_at_pulses[item])vsp_group.release_mask|=SPF_EXT;
 if(depth<=(float)sf.profile.turn_at_pulses[item] &&
  (!(vsp_group.command.axes_mask&SPF_EXT) || SF_ExtZero()))vsp_group.release_mask|=SPF_DM;
}
static uint32_t SF_Move(uint32_t axes,float yaw,float ext,uint32_t depth,uint32_t delayed,uint32_t kind)
{
 SPF_Command *c=&sf.command;
 memset(c,0,sizeof(*c));c->run_id=sf.profile.run_id;c->pose_version=sf.profile.pose_version;
 c->sequence=++g_it_sequence;c->item=g_stage2_fast_item;c->kind=kind;c->axes_mask=axes;
 c->goal.dm_deg=yaw;c->goal.extension_mm=ext;c->goal.lift_pulses=(float)depth;
 c->dm_deg_s=SPF_DM_DEG_S;c->extension_rpm=SPF_EXT_RPM;c->lift_rpm=SPF_LIFT_RPM;
 c->stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c->keep_vm_on=1U;
 if(kind==SPF_CMD_ROUTE_AND_STOW)c->route_stage=3U;
 if(!VehicleArm_BeginSourceParallel(c,1U,kind==SPF_CMD_ROUTE_AND_STOW?1U:0U))return 0U;
 vsp_group.delayed_mask=delayed;vsp_group.release_mask=axes&~delayed;
 /* Only this commissioned fast path uses the already verified ruler lift
  * acceleration. Node6 keeps its verified 600 RPM/s value. */
 vsp_fast_lift=1U;sf.issued=1U;return 1U;
}
static uint32_t SF_MoveDone(void)
{return sf.issued && !VehicleSourceParallel_Owns() && g_source_parallel_state==3U;}
static void SF_SyncSafeLift(void)
{
 /* A real, settled, in-range group completed. Preserve the existing top
  * reference; publish the observed safe pose for subsequent ordinary IT use. */
 int64_t raw=(int64_t)(vsp_axis[1].current*(float)g_lift_ruler_controller.top_scale+0.5f);
 g_lift_ruler_controller.position_after_raw=raw;
 g_lift_ruler_controller.safe_position_raw=raw;
 g_lift_ruler_controller.backoff_valid=1U;g_lift_ruler_controller.operation=3U;
}
uint32_t Stage2Fast_IsActive(void)
{return (g_stage2_fast_state>=S2F_AIM && g_stage2_fast_state<=S2F_DEPART) || sf.stop_pending;}
uint32_t Stage2Fast_Start(const S2Fast_Profile *p,const S2Fast_Hooks *hooks,uint32_t now)
{
 float safe;
 if(Stage2Fast_IsActive() || g_stage2_fast_state==S2F_FAULT || !SF_ProfileValid(p) || !hooks ||
  !hooks->placed || !hooks->route_begin || !hooks->route_poll || !hooks->route_stop ||
  (sf.configured && sf.profile.run_id==p->run_id) || !SF_Healthy() || !it_at_top() ||
  !SingleTeach_ReferencesReady() || SourceStage2_IsActive() || VehicleArm_Busy() ||
  g_it_request || g_it_state==1U || g_it_state==9U || g_lift_ruler_request ||
  g_lift_ruler_controller.stop_pending || g_vehicle_arm_stop_pending ||
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f)return 0U;
 safe=SingleTeach_TopDeg()+(float)p->safe_pulses*(360.0f/3200.0f);
 if(fabsf(safe-g_vehicle_lift_current_deg)>1.0f)return 0U;
 memset(&sf,0,sizeof(sf));sf.profile=*p;sf.hooks=*hooks;sf.configured=1U;sf.begin_ms=now;
 g_stage2_fast_item=g_stage2_fast_loaded_mask=g_stage2_fast_fault=0U;
 g_stage2_fast_release_ms=g_stage2_fast_departure_ms=0U;
 SF_State(S2F_AIM,now);
 if(jiazhua(210.0f)!=1U){Stage2Fast_Cancel(1U);return 0U;}
 return 1U;
}
void Stage2Fast_Cancel(uint32_t reason)
{
 if(!sf.configured)return;
 if(!g_stage2_fast_fault)g_stage2_fast_fault=reason?reason:1U;
 g_stage2_fast_state=S2F_FAULT;sf.stop_pending=1U;
 if(VehicleSourceParallel_Owns())VehicleSourceParallel_Cancel();
 /* Do not release a potentially loaded gripper or cut VM. */
}
void Stage2Fast_Service(uint32_t now)
{
 uint32_t item,result;float depth;
 if(!sf.configured || !Stage2Fast_IsActive())return;
 if(sf.stop_pending){
  if(VehicleSourceParallel_Owns())VehicleSourceParallel_Cancel();
  result=!sf.route_attempted || sf.hooks.route_stop(sf.hooks.context,now)==1U;
  if(!VehicleSourceParallel_Owns() && !g_source_parallel_stop_mask && result)sf.stop_pending=0U;
  return;
 }
 if(!SF_Healthy() || g_source_parallel_cause || g_source_parallel_state==4U){Stage2Fast_Cancel(2U);return;}
 /* The route has its own timeout; the arm and evidence phases are bounded. */
 if(g_stage2_fast_state!=S2F_DEPART && now-sf.phase_ms>=15000U){Stage2Fast_Cancel(3U);return;}
 item=g_stage2_fast_item;
 SF_ReleaseGates(now);
 switch(g_stage2_fast_state){
 case S2F_AIM:
  if(!sf.issued){
   if(!SF_Move(SPF_DM,sf.profile.pick_deg[item],0.0f,sf.profile.safe_pulses,0U,SPF_CMD_TRANSFER))
    Stage2Fast_Cancel(4U);
  }else if(SF_MoveDone())SF_State(S2F_APPROACH,now);
  break;
 case S2F_APPROACH:
  if(!sf.issued){
   sf.closed=sf.tray=0U;
   if(jiazhua(160.0f)!=1U ||
    !SF_Move(SPF_LIFT|(sf.profile.extension_mm[item]>0.0f?SPF_EXT:0U),
     sf.profile.pick_deg[item],sf.profile.extension_mm[item],sf.profile.pick_pulses,0U,SPF_CMD_PICK)){
    Stage2Fast_Cancel(4U);break;
   }
  }
  if(!sf.tray && ((vsp_group.started&SPF_LIFT) || SF_MoveDone())){
   if(!CarrierPwm_Write(500U+(uint32_t)((float)item*120.0f*2000.0f/270.0f+0.5f))){Stage2Fast_Cancel(5U);break;}
   sf.tray=1U;sf.tray_ms=now;
  }
  if(!sf.closed && SF_LiftDepth(now,&depth) &&
   (depth<=(float)sf.profile.pick_pulses+1.0f || (vsp_group.done&SPF_LIFT)) &&
   depth>=(float)(sf.profile.pick_pulses-sf.profile.close_lead_pulses) &&
   (!(vsp_group.command.axes_mask&SPF_EXT) || (vsp_group.done&SPF_EXT))){
   if(jiazhua(140.0f)!=1U){Stage2Fast_Cancel(5U);break;}
   sf.closed=1U;sf.close_ms=now;
  }
  if(SF_MoveDone() && sf.closed && now-sf.close_ms>=sf.profile.close_ms &&
   sf.tray && now-sf.tray_ms>=sf.profile.carrier_ms)SF_State(S2F_CARRY,now);
  break;
 case S2F_CARRY:
  if(!sf.issued){
   uint32_t axes=SPF_LIFT|SPF_DM|(sf.profile.extension_mm[item]>0.0f?SPF_EXT:0U);
   if(!SF_Move(axes,sf.profile.slot_deg,0.0f,sf.profile.safe_pulses,axes&~SPF_LIFT,SPF_CMD_TRANSFER))
    Stage2Fast_Cancel(4U);
  }else if(SF_MoveDone()){
   SF_SyncSafeLift();
   if(jiazhua(210.0f)!=1U){Stage2Fast_Cancel(5U);break;}
   SF_State(S2F_RELEASE,now);
  }
  break;
 case S2F_RELEASE:
  if(now-sf.phase_ms<sf.profile.release_ms)break;
  result=sf.hooks.placed(sf.hooks.context,sf.profile.run_id,item+1U,now);
  if(result==2U){Stage2Fast_Cancel(6U);break;}
  if(result!=1U)break;
  g_stage2_fast_loaded_mask|=1U<<item;
  if(item<2U){g_stage2_fast_item++;SF_State(S2F_AIM,now);}
  else {
   g_stage2_fast_release_ms=now-sf.begin_ms;
   SF_State(S2F_DEPART,now);
   /* Prepare the proven zero-degree sweep, then submit route in this SAME
    * service; do not wait for stow completion or visit 180 degrees. */
   if(!SF_Move(SPF_DM,0.0f,0.0f,sf.profile.safe_pulses,0U,SPF_CMD_ROUTE_AND_STOW)){
    Stage2Fast_Cancel(4U);break;
   }
   sf.route_attempted=1U;
   if(sf.hooks.route_begin(sf.hooks.context,&sf.command,now)!=1U){Stage2Fast_Cancel(7U);break;}
   sf.route_started=1U;g_stage2_fast_departure_ms=now-sf.begin_ms;
  }
  break;
 case S2F_DEPART:
  result=sf.hooks.route_poll(sf.hooks.context,&sf.command,now);
  if(result==2U){Stage2Fast_Cancel(8U);break;}
  if(result==1U)sf.route_done=1U;
  if(sf.route_done && SF_MoveDone())SF_State(S2F_DONE,now);
  break;
 default:Stage2Fast_Cancel(9U);break;
 }
}
#else
uint32_t Stage2Fast_Start(const S2Fast_Profile *p,const S2Fast_Hooks *h,uint32_t now)
{(void)p;(void)h;(void)now;return 0U;}
uint32_t Stage2Fast_IsActive(void){return 0U;}
void Stage2Fast_Cancel(uint32_t why){(void)why;}
void Stage2Fast_Service(uint32_t now){(void)now;}
#endif
