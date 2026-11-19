/* 仅看护观测入口，由原CAN任务调用；不写底盘路线、毫米对准或库存状态。
 * 请求1：打开本次真实扫码窗口；请求2：用户明确模拟到位；请求9：停止。
 * 状态1等扫码，2等模拟到位，3等转臂，4等伸出，5等新圆心，6有新像素观测。
 * 6不是毫米纠偏完成。当前阶段不自动下降、抓取或启动车轮。
 */
#include "saoma.h"
#include "field_map.h"
#include "vehicle_tuning.h"
volatile uint32_t g_source_view_request,g_source_view_state,g_source_view_reason;
volatile uint32_t g_source_view_scan_revision,g_source_view_center_cycle;
char g_source_view_task_code[16];
static uint32_t sv_scan_baseline,sv_open_ms,sv_sequence,sv_center_baseline,sv_pose_ms;

static uint32_t SV_Ready(void)
{
 return it_platform() && it_at_top() && SingleTeach_ReferencesReady() &&
  !g_it_cause && g_it_state!=9U && !g_lift_ruler_cause && !g_startup_axes_cause &&
  !g_dm_jog_first_fault && !g_source_parallel_cause && !g_source_parallel_stop_mask &&
  !g_lift_ruler_stop_pending && !g_vehicle_arm_stop_pending &&
  g_it_auto_state!=1U && g_it_auto_state!=9U;
}
static void SV_Stop(uint32_t reason)
{
 g_source_view_reason=reason;g_source_view_state=9U;
 SAOMA_DisableReceive();g_it_request=9U;
}
static void SourceView_Service(uint32_t now)
{
 uint32_t request=g_source_view_request;g_source_view_request=0U;
 if(request==9U || (g_it_request==9U && g_source_view_state)){
  SV_Stop(9U);return;
 }
 if(request==1U){
  /* 不抢占原动作，不清原故障，不复用上次扫码。 */
  if((g_source_view_state && g_source_view_state!=9U) || !SV_Ready() ||
     g_it_request || g_it_state==1U || VehicleArm_Busy() ||
     Stage2Fast_IsActive() || SourceStage2_IsActive() ||
     fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f){
   g_source_view_reason=1U;return;
  }
  taskENTER_CRITICAL();sv_scan_baseline=g_saoma_accept_count;sv_open_ms=now;
  memset(g_source_view_task_code,0,sizeof(g_source_view_task_code));
  g_source_view_scan_revision=0U;g_source_view_center_cycle=0U;
  taskEXIT_CRITICAL();SAOMA_EnableReceive();
  g_source_view_state=1U;g_source_view_reason=0U;
 }
 if(!g_source_view_state || g_source_view_state==9U)return;
 if(!SV_Ready()){SV_Stop(2U);return;}
 if(g_source_view_state==1U){
  char code[16];uint32_t count,begin,end,result;
  taskENTER_CRITICAL();count=g_saoma_accept_count;begin=g_saoma_frame_start_ms;
  end=g_saoma_frame_end_ms;result=g_saoma_last_result;
  memcpy(code,SAOMA_Data,16U);taskEXIT_CRITICAL();
  if(count!=sv_scan_baseline && result==1U &&
     (uint32_t)(begin-sv_open_ms)<0x80000000U &&
     (uint32_t)(now-end)<VEHICLE_STAGE1_SCAN_FRESH_MS && FieldMap_TaskCodeValid(code)){
   memcpy(g_source_view_task_code,code,16U);g_source_view_scan_revision=count;
   SAOMA_DisableReceive();g_source_view_state=2U;
  }
  return;
 }
 if(request==2U){
  if(g_source_view_state!=2U || g_it_request || g_it_state==1U || VehicleArm_Busy() ||
     Stage2Fast_IsActive() || SourceStage2_IsActive() ||
     fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f){
   g_source_view_reason=3U;return;
  }
  /* 顺序展开：收回状态转臂，再伸出。原IT后端仍独立核真实参考和到位。 */
  g_it_axes=SPF_DM;g_it_dm_deg=180.0f;g_it_extension_mm=0.0f;
  sv_sequence=g_it_sequence;g_it_request=3U;g_source_view_state=3U;
  g_source_view_reason=0U;return;
 }
 if(g_source_view_state==3U || g_source_view_state==4U){
  if(g_it_rejected || g_it_cause || g_it_state==9U){SV_Stop(4U);return;}
  if(g_it_request || g_it_state!=2U || g_it_action!=3U || g_it_sequence==sv_sequence)return;
  if(g_source_view_state==3U){
   g_it_axes=SPF_EXT;g_it_dm_deg=180.0f;g_it_extension_mm=65.0f;
   sv_sequence=g_it_sequence;g_it_request=3U;g_source_view_state=4U;return;
  }
  if(g_vis_command_request)return;
  sv_center_baseline=g_vis_center_cycle;sv_pose_ms=now;
  g_vis_command_request=5U;g_source_view_state=5U;return;
 }
 if(g_source_view_state==5U && g_vis_center_cycle!=sv_center_baseline &&
    g_vis_center_valid && (uint32_t)(g_vis_center_tick-sv_pose_ms)<0x80000000U &&
    (uint32_t)(now-g_vis_center_tick)<150U){
  g_source_view_center_cycle=g_vis_center_cycle;g_source_view_state=6U;
 }
}
