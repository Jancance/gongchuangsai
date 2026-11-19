/* Exactly one region owns mechanical commands and the text UART observation. */
#include "source_stage2_body_port.h"
#if SOURCE_CYCLE_TEST_215
volatile uint32_t g_region_dispatch_area=3U,g_region_dispatch_rejected;
#else
volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;
#endif
volatile uint32_t g_region_dispatch_test_enable=1U;
static RegionModuleRequest rm_task;
static volatile RegionModuleStatus rm_status;
static uint32_t rm_bound,rm_wait_dm_stop;

static uint32_t RM_Idle(void)
{
 return S3_Platform() && !g_source_cycle_fault && !g_stage3_fault && !g_it_cause &&
  !g_low_fault && g_low_state!=9U && !g_startup_axes_cause && !g_lift_ruler_cause &&
  !g_dm_jog_first_fault && !g_source_body_step_reason &&
  (!g_source_cycle_state || g_source_cycle_state==8U) &&
  (!g_stage3_state || g_stage3_state==6U || g_stage3_state==11U) &&
  !g_it_request && !g_low_request && !g_source_cycle_request && !g_stage3_request &&
  !g_stage3_teach_request && !s3_teach_pending && !g_stage3_manual_state && !g_stage3_replay_enable &&
  !g_source_body_step_request && !SourceBodyStep_WheelWindow() && !g_chassis_auto_active &&
  !g_lift_ruler_request && !g_lift_ruler_stop_pending && !VehicleArm_Busy() && !g_vehicle_arm_stop_pending &&
  !VehicleSourceParallel_Owns() && !Stage2Body_Owns() && !Stage2Fast_IsActive() && !SourceStage2_IsActive() &&
  !g_dm_jog_request && !DM_CAN2_JogIsBusy() && g_it_auto_state!=1U && g_it_auto_state!=9U &&
  g_region_state!=1U && g_pzq_near_state!=1U;
}
static uint32_t RM_Valid(const RegionModuleRequest *r)
{
 uint32_t i,j;
 if(!r->run_id || r->batch_id<1U || r->batch_id>2U ||
    r->module<REGION_MODULE_SOURCE || r->module>REGION_MODULE_STACK)return 0U;
 for(i=0U;i<3U;i++){
  uint32_t slot=0U;
  if(r->colors[i]<1U || r->colors[i]>6U || r->slots[i]<1U || r->slots[i]>6U ||
     r->rings[i]<1U || r->rings[i]>3U)return 0U;
  for(j=0U;j<i;j++)if(r->colors[j]==r->colors[i] || r->rings[j]==r->rings[i])return 0U;
  for(j=0U;j<3U;j++)slot+=r->slots[j]==r->colors[i];
  if(slot!=1U)return 0U;
 }
 if(r->module==REGION_MODULE_SOURCE)return 1U;
 if(!(g_stage3_pick_depth>=3200.0f && g_stage3_pick_depth<=SPF_STAGE3_MAX_LIFT_PULSES &&
      g_stage3_carrier_yaw>=0.0f && g_stage3_carrier_yaw<=270.0f))return 0U;
 for(i=0U;i<3U;i++){
  j=r->rings[i]-1U;
  if(!(g_stage3_target_yaw[j]>=0.0f && g_stage3_target_yaw[j]<=270.0f &&
       g_stage3_target_x[j]>=0.0f && g_stage3_target_x[j]<=170.0f &&
       g_stage3_target_depth[j]>=3200.0f && g_stage3_target_depth[j]<=SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
  if(r->module==REGION_MODULE_ROUGH &&
     (!(g_stage3_recovery_depth[j]>=3200.0f && g_stage3_recovery_depth[j]<=SPF_STAGE3_MAX_LIFT_PULSES) ||
      (i==2U && g_stage3_target_depth[j]<3200.0f+S3_FAST_HANDOFF_OFFSET_PULSES)))return 0U;
  if(r->module==REGION_MODULE_STACK && g_stage3_target_depth[j]-60.0f*SOURCE_LIFT_PULSES_PER_MM<3200.0f)return 0U;
 }
 return 1U;
}
uint32_t RegionDispatch_Begin(const RegionModuleRequest *r)
{
 uint32_t ok=0U;
 if(!r)return 0U;
 taskENTER_CRITICAL();
 if((rm_status==REGION_MODULE_UNKNOWN || rm_status==REGION_MODULE_DONE ||
     (rm_status==REGION_MODULE_STOPPED && !rm_bound)) &&
    (!rm_task.run_id || (int32_t)(r->run_id-rm_task.run_id)>0) && RM_Idle() && RM_Valid(r)){
  rm_task=*r;rm_bound=rm_wait_dm_stop=0U;rm_status=REGION_MODULE_BUSY;ok=1U;
 }
 taskEXIT_CRITICAL();
 return ok;
}
RegionModuleStatus RegionDispatch_Poll(uint32_t run_id)
{
 RegionModuleStatus state;
 taskENTER_CRITICAL();
 state=run_id && run_id==rm_task.run_id?rm_status:REGION_MODULE_UNKNOWN;
 taskEXIT_CRITICAL();
 return state;
}
static uint32_t RM_Stop(uint32_t run_id,uint32_t active)
{
 uint32_t ok=0U;
 taskENTER_CRITICAL();
 if((active || (run_id && run_id==rm_task.run_id)) &&
    (rm_status==REGION_MODULE_BUSY || rm_status==REGION_MODULE_STOPPING)){
  if(rm_bound==1U){
   if(rm_task.module==REGION_MODULE_SOURCE)g_source_cycle_request=9U;
   else g_stage3_request=9U;
  }
  rm_status=REGION_MODULE_STOPPING;ok=1U;
 }
 taskEXIT_CRITICAL();
 return ok;
}
uint32_t RegionDispatch_Stop(uint32_t run_id){return RM_Stop(run_id,0U);}
uint32_t RegionDispatch_StopActive(void){return RM_Stop(0U,1U);}

static void RM_Start(uint32_t now)
{
 uint32_t i;
 if(rm_task.module==REGION_MODULE_SOURCE){
  g_region_dispatch_area=1U;sc_bound_batch=rm_task.batch_id;
  for(i=0U;i<3U;i++){g_source_cycle_colors[i]=rm_task.colors[i];sc_bound_slots[i]=rm_task.slots[i];}
  g_source_cycle_run=rm_task.run_id;g_source_cycle_prep=g_source_cycle_item=sc_pending=sc_retract_pending=0U;
  sc_beep_started=g_source_cycle_beeps=g_source_cycle_beep_started=g_source_cycle_beep_done=0U;
  g_source_cycle_pick_beeps=g_source_cycle_pick_beep_pending=g_source_cycle_scheduled=0U;
  g_source_cycle_done=g_source_cycle_body_steps=0U;
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
  g_source_cycle_started=now;SC_Pose();g_source_cycle_state=1U;
 }else{
  g_region_dispatch_area=3U;s3_module_run=rm_task.run_id;
  for(i=0U;i<3U;i++){
   g_stage3_colors[i]=rm_task.colors[i];g_stage3_positions[i]=rm_task.rings[i];g_stage3_slot_colors[i]=rm_task.slots[i];
  }
  g_stage3_cycle_mode=rm_task.module==REGION_MODULE_STACK?2U:1U;
  g_stage3_test_first_item=0U;g_stage3_test_count=3U;
  g_stage3_task_run=rm_task.run_id;g_stage3_task_batch=rm_task.batch_id;
  g_stage3_run=rm_task.run_id-1U;g_stage3_request=1U;
 }
 rm_bound=1U;
}
static uint32_t RM_StopSettled(void)
{
 return !g_it_request && g_it_state!=1U && !g_low_request && !Low_Busy() &&
  !g_source_body_step_request && !SourceBodyStep_Busy() && !g_chassis_auto_active &&
  !g_lift_ruler_request && !g_lift_ruler_stop_pending && !VehicleArm_Busy() && !g_vehicle_arm_stop_pending &&
  !VehicleSourceParallel_Owns() && !g_dm_jog_request && !DM_CAN2_JogIsBusy() &&
  (!rm_wait_dm_stop || (!g_dm_posvel_holding && g_dm_jog_stop_result==1U));
}
uint32_t RegionDispatch_Arrive(uint32_t area)
{
 if(rm_status==REGION_MODULE_BUSY || rm_status==REGION_MODULE_STOPPING || (area!=1U && area!=3U) ||
    (g_source_cycle_state && g_source_cycle_state!=8U) ||
    (g_stage3_state && g_stage3_state!=6U && g_stage3_state!=11U) ||
    g_it_request || g_it_state==1U || g_it_state==9U || Low_Busy() ||
    SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || g_chassis_auto_active){
  g_region_dispatch_rejected++;return 0U;
 }
 g_region_dispatch_area=area;
 s3_module_run=0U;
 if(area==1U)g_source_cycle_request=1U;else Stage3_Arrive();
 return 1U;
}
void RegionDispatch_KeyEvent(void)
{
 if(RegionDispatch_StopActive())return;
 if(g_region_dispatch_area==1U){SourceCycle_KeyEvent();return;}
 if(g_region_dispatch_test_enable){g_region_dispatch_area=3U;Stage3_KeyEvent();}
}
void RegionDispatch_ScanService(void)
{if(g_region_dispatch_area==1U)SourceCycle_ScanService();}
static void RegionDispatch_Service(uint32_t now)
{
 uint32_t source,done,fault;
 taskENTER_CRITICAL();
 if(rm_status==REGION_MODULE_STOPPING && !rm_bound)rm_status=REGION_MODULE_STOPPED;
 if(rm_status==REGION_MODULE_BUSY && !rm_bound){
  if(!RM_Idle() || !RM_Valid(&rm_task))rm_status=REGION_MODULE_FAULT;
  else RM_Start(now);
 }
 if(rm_status==REGION_MODULE_STOPPING && rm_bound==1U){
  rm_wait_dm_stop=g_dm_posvel_holding || DM_CAN2_JogIsBusy();
  if(rm_task.module==REGION_MODULE_SOURCE){SC_Fail(2U);if(g_it_request)g_it_request=9U;}
  else S3_Fail(2U);
  if(g_source_body_step_request)g_source_body_step_request=9U;
  if(rm_wait_dm_stop && g_it_request!=9U)g_it_request=9U;
  rm_bound=2U;
 }
 taskEXIT_CRITICAL();
 if(g_region_dispatch_area==1U)SourceCycle_Service(now);else if(g_region_dispatch_area==3U)Stage3_Service(now);
 if(rm_status==REGION_MODULE_STOPPING){
  if(rm_bound==2U && !g_source_cycle_request && !g_stage3_request && RM_StopSettled())rm_status=REGION_MODULE_STOPPED;
  return;
 }
 if(rm_status!=REGION_MODULE_BUSY || rm_bound!=1U)return;
 source=rm_task.module==REGION_MODULE_SOURCE;
 fault=source?(g_source_cycle_fault || g_source_cycle_state==9U):
       (g_stage3_fault || g_stage3_state==9U || g_stage3_run!=rm_task.run_id);
 done=source?(g_source_cycle_run==rm_task.run_id && g_source_cycle_state==8U && g_source_cycle_item==3U):
      (g_stage3_state==6U && g_stage3_locked_run==rm_task.run_id && g_stage3_placed_count==3U &&
       (rm_task.module==REGION_MODULE_STACK ? !g_stage3_recovered_count && g_stage3_phase==36U :
        g_stage3_recovered_count==3U && g_stage3_phase==32U));
 taskENTER_CRITICAL();
 if(rm_status==REGION_MODULE_BUSY){if(fault)rm_status=REGION_MODULE_FAULT;else if(done)rm_status=REGION_MODULE_DONE;}
 taskEXIT_CRITICAL();
}
static uint32_t RegionDispatch_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{return g_region_dispatch_area==3U?Stage3_SharedByte(b,now,binary_active):SourceCycle_SharedByte(b,now,binary_active);}
static void RegionDispatch_Telemetry(uint32_t now)
{if(g_region_dispatch_area==3U)Stage3_Telemetry(now);else SourceCycle_Telemetry(now);}
