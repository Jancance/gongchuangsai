/* Included in existing vehicle_mission.inc AFTER the real body port. */
#include "source_station.h"
#include "source_station_site.h"
volatile uint32_t g_source_station_state,g_source_station_reason,g_source_station_run;
volatile uint32_t g_source_station_scan_count,g_source_station_pose;
static uint32_t ss_alignment_sequence,ss_scan_baseline,ss_scan_open_ms,ss_body_session;
static char ss_task_code[16];
static uint32_t ss_task_valid;

void SourceStation_Cancel(uint32_t reason)
{
 if(g_source_station_state==SS_IDLE)return;
 if(!g_source_station_reason)g_source_station_reason=reason?reason:1U;
 g_source_station_state=SS_FAULT;ss_task_valid=0U;SAOMA_DisableReceive();
 Stage2Body_Cancel();
 /* The CAN service observes lost station permission and issues its own STOP.
  * Never operate actuators from TaskA or fabricate a loaded batch. */
}
uint32_t SourceStation_HoldsBody(void)
{
 return SOURCE_STATION_ENABLE && g_source_station_run==g_vehicle_run_id &&
  g_vehicle_stage==2U && g_vehicle_state==8U &&
  g_source_station_state>=SS_WAIT_QR && g_source_station_state<=SS_DONE;
}
uint32_t SourceStation_ManualWait(void)
{return SOURCE_STATION_ENABLE && g_source_station_state==SS_WAIT_QR;}
static uint32_t SS_Parked(void)
{
 ChassisAutoStatus body;(void)ChassisAuto_ReadStatus(&body);
 return !body.active && !body.cause && body.result==1U && body.rotation_stop_mask==15U &&
  !s_motion_session_open && !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched &&
  g_vehicle_run_id==g_source_station_run && g_vehicle_stage==2U && g_vehicle_state==8U &&
  !g_vehicle_cause && StartupAxes_Holding();
}
uint32_t SourceStation_PickAllowed(uint32_t run)
{
 ChassisAutoStatus body;uint32_t ok;taskENTER_CRITICAL();
 (void)ChassisAuto_ReadStatus(&body);
 ok=SOURCE_STATION_ENABLE && SOURCE_STATION_PZQ_POSE_VERIFIED==1U && ss_task_valid &&
  run==g_source_station_run && (g_source_station_state==SS_READY || g_source_station_state==SS_PICKING) &&
  g_vehicle_alignment_state==2U && SS_Parked() && body.session==ss_body_session;
 taskEXIT_CRITICAL();return ok;
}
uint32_t SourceStation_ReadTask(char code[16],uint32_t *run,uint32_t *batch)
{
 uint32_t ok=0U;if(!code || !run || !batch)return 0U;taskENTER_CRITICAL();
 if(ss_task_valid && SourceStation_HoldsBody() && g_source_station_state>=SS_READY){
  memcpy(code,ss_task_code,16U);*run=g_source_station_run;*batch=1U;ok=1U;
 }
 taskEXIT_CRITICAL();return ok;
}
uint32_t SourceStation_Service(uint32_t now)
{
 ChassisAutoStatus body;S2_Event aligned;uint32_t status,loaded[3];
 if(!SOURCE_STATION_ENABLE)return 0U;
 if(g_source_station_state==SS_IDLE && g_vehicle_stage==2U && g_vehicle_state==8U){
  if(!g_vehicle_run_id || !StartupAxes_Holding())return 1U;
  g_source_station_run=g_vehicle_run_id;g_source_station_pose=SOURCE_STATION_POSE_VERSION;
  g_source_station_state=SS_WAIT_PROFILE;ss_task_valid=0U;g_source_station_reason=0U;
 }
 if(g_source_station_state==SS_IDLE)return 0U;
 if(g_source_station_state==SS_FAULT)return 1U;
 if(g_source_station_run!=g_vehicle_run_id || g_vehicle_stage!=2U || g_vehicle_state!=8U ||
  g_vehicle_cause || g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched){
  SourceStation_Cancel(1U);return 1U;
 }
 if(g_source_station_state==SS_WAIT_PROFILE){
  /* Missing field data is a visible wait, not a made-up calibrated profile. */
  if(!Stage2Site_ActivateOnce()){g_source_station_reason=2U;return 1U;}
  if(!SourceStation_MechanicsReady()){g_source_station_reason=8U;return 1U;}
  if(!Stage2Body_BeginAlignment(g_source_station_run,++ss_alignment_sequence,now)){
   g_source_station_reason=3U;return 1U;
  }
  g_source_station_reason=0U;g_source_station_state=SS_ALIGN;return 1U;
 }
 if(g_source_station_state==SS_ALIGN){
  status=Stage2Body_ReadAlignment(g_source_station_run,ss_alignment_sequence,&aligned);
  if(status==2U){SourceStation_Cancel(4U);return 1U;}
  if(status!=1U)return 1U;
  if(!SS_Parked()){SourceStation_Cancel(5U);return 1U;}
  (void)ChassisAuto_ReadStatus(&body);ss_body_session=body.session;
  g_vehicle_alignment_state=2U;ss_scan_baseline=g_saoma_accept_count;ss_scan_open_ms=now;
  ss_task_valid=0U;memset(ss_task_code,0,sizeof(ss_task_code));
  SAOMA_EnableReceive();g_source_station_state=SS_WAIT_QR;return 1U;
 }
 (void)ChassisAuto_ReadStatus(&body);
 if(!SS_Parked() || body.session!=ss_body_session){SourceStation_Cancel(6U);return 1U;}
 if(g_source_station_state==SS_WAIT_QR){
  char current[16];uint32_t count,begin,end,result;
  taskENTER_CRITICAL();count=g_saoma_accept_count;begin=g_saoma_frame_start_ms;
  end=g_saoma_frame_end_ms;result=g_saoma_last_result;memcpy(current,SAOMA_Data,16U);taskEXIT_CRITICAL();
  if(count==ss_scan_baseline || result!=1U || (uint32_t)(begin-ss_scan_open_ms)>=0x80000000U ||
   (uint32_t)(now-end)>=VEHICLE_STAGE1_SCAN_FRESH_MS || !FieldMap_TaskCodeValid(current))return 1U;
  taskENTER_CRITICAL();memcpy(ss_task_code,current,16U);ss_task_valid=1U;
  g_source_station_scan_count=count;g_source_station_state=SS_READY;taskEXIT_CRITICAL();
  SAOMA_DisableReceive();return 1U;
 }
 if(g_source_station_state==SS_READY || g_source_station_state==SS_PICKING){
  extern volatile uint32_t g_it_auto_state,g_it_auto_vision_mode;
  if(g_it_auto_vision_mode && g_it_auto_state==1U)g_source_station_state=SS_PICKING;
  if(g_it_auto_vision_mode && g_it_auto_state==9U){SourceStation_Cancel(7U);return 1U;}
  taskENTER_CRITICAL();status=Vision_ReadLoadedBatch(g_source_station_run,1U,loaded);taskEXIT_CRITICAL();
  /* Complete only after the actual provider committed all3, not servo/DONE. */
  if(status && g_it_auto_state==2U){g_source_station_state=SS_DONE;g_stage2_ready=1U;}
 }
 return 1U; /* Route3 intentionally stays separate in this commissioning phase. */
}
