#include "source_body_step.h"
#include "boot_ready_link.h"
#include "stage3_cycle.h"
#include "region_dispatch.h"
#include "app_zone2.h"
#if APP_ZONE2_MISSION_ENABLE
#include "usart.h"
#include <stdio.h>
#include "region_placement_feedback.inc"
#endif
static uint32_t RegionDispatch_SharedByte(uint8_t,uint32_t,uint32_t);
static void RegionDispatch_Telemetry(uint32_t);
static void RegionDispatch_Service(uint32_t);
static uint32_t SourceCycle_SharedByte(uint8_t,uint32_t,uint32_t);
static void SourceCycle_Telemetry(uint32_t);
#include "usart.h"
#include "stage2_fast_cycle.h"
#include "vehicle_mission.h"
#include "source_vision_runtime.h"
#include "source_stage2_owner.h"
#include "source_observer.h"
#include "source_parallel_flow.h"
static uint32_t Low_Busy(void);
static uint32_t Low_Loaded(void);
static uint32_t Low_PoseAllowed(void);
static uint32_t Low_PickAllowed(void);
static uint32_t Low_XYAllowed(void);
static uint32_t Low_TransferGate(uint32_t now,float depth,uint32_t fresh);
static uint32_t Region_Busy(void);
static uint32_t Region_PoseAllowed(void);
static uint32_t Region_ObserveYawAllowed(void);
static uint32_t Region_ObservePickAllowed(void);
/* 单件人工看护示教入口；所有服务由原唯一CAN任务调用。
 * 1真实顶部回零，2伸缩真实回零，3锁存位姿，4带料/空爪回顶，
 * 5闭爪，6松爪，7只读DM，8载盘0度，9停止。无自动循环、无库存伪确认。 */
volatile uint32_t g_it_request,g_it_state,g_it_cause,g_it_rejected,g_it_action,g_it_sequence;
volatile uint32_t g_it_grip_prepared;
volatile uint32_t g_it_lead5_grip_done;
volatile uint32_t g_it_axes=2U,g_it_elapsed_ms,g_it_lift_permit;
volatile uint32_t g_it_ext_rpm_test;  /* 2026-11-08 debug: 0 = use 106 rpm, else this rpm */
volatile uint32_t g_it_place_depth_limit_pulses=0U; /* 放料新示教前为0，不用历史高度。 */
volatile float g_it_dm_deg=180.0f,g_it_extension_mm=48.0f,g_it_lift_pulses=(float)SPF_MAX_LIFT_PULSES;
/* 三槽载盘目标角(0/120/240 对应 1/2/3 号槽)，仅请求10使用。 */
volatile float g_it_carrier_deg=0.0f;
volatile float g_it_carrier_auto_deg=-1.0f;
volatile uint32_t g_it_carrier_auto_done=0U;
static float it_carrier_latched_deg=-1.0f;
static uint32_t it_carrier_pwm_tick;
/* 载盘动作结束时会调 it_done() 清 g_it_lift_permit；但载盘常常是在下降途中执行的，
 * 清掉许可会让升降被判 not ready 直接 STOP(cause1)。这里存旧值、完成时还原。 */
static uint32_t it_carrier_permit=0U;
static uint32_t it_begin,it_phase,it_lift_samples,it_dm_cycle,it_seen_samples;
static SPF_Command it_command,it_stage3_place;
static uint32_t it_stage3_carry,it_stage3_grip100,it_stage3_grip_tick;

/* ===== 视觉取件目标事务（离线版）=====
 * 视觉侧（现在是调试脚本扮演，后续是串口解析）写入下面这组字段并让 revision+1；
 * 固件在每件开始前经 IT 请求11 消费一次：校验新鲜度/范围/序号，拒绝越界和过期，且不重复消费。
 * 结果码：0 未提交 1 接受 2 无新事务 3 过期 4 越界 5 序号重复 */
volatile uint32_t g_src_txn_revision=0U,g_src_txn_consumed=0U,g_src_txn_seq=0U;
volatile uint32_t g_src_txn_tick=0U,g_src_txn_result=0U,g_src_txn_age_ms=0U,g_src_slot=1U;
volatile uint32_t g_src_now_ms=0U,g_src_txn_takes=0U;
volatile float g_src_extension_mm=0.0f;
volatile float g_src_pick_deg=180.0f,g_src_chassis_dx_mm=0.0f,g_src_chassis_dy_mm=0.0f;
static uint32_t g_src_txn_last_seq=0U;
/* 3秒停盘窗口；取件角允许 0~270（源区分支），底盘纠偏暂限 ±20mm。 */
uint32_t SrcTxn_Take(uint32_t now)
{
 float deg,dx,dy;
 g_src_txn_takes++;
 if(g_src_txn_revision==0U || g_src_txn_revision==g_src_txn_consumed){g_src_txn_result=2U;return 0U;}
 deg=g_src_pick_deg;dx=g_src_chassis_dx_mm;dy=g_src_chassis_dy_mm;
 if(!(deg>=0.0f && deg<=270.0f) || !(dx>=-20.0f && dx<=20.0f) || !(dy>=-20.0f && dy<=20.0f) ||
    g_src_slot<1U || g_src_slot>3U || !(g_src_extension_mm>=0.0f && g_src_extension_mm<=170.0f)){g_src_txn_result=4U;return 0U;}
 if(g_src_txn_seq==0U || g_src_txn_seq==g_src_txn_last_seq){g_src_txn_result=5U;return 0U;}
 g_src_txn_age_ms=(uint32_t)(now-g_src_txn_tick);
 if(g_src_txn_age_ms>3000U){g_src_txn_result=3U;return 0U;}
 g_src_txn_last_seq=g_src_txn_seq;
 g_src_txn_consumed=g_src_txn_revision;
 g_src_txn_result=1U;
 return 1U;
}
/* ===== 视觉串口解析（USART6 PC6/PC7 115200 8N1）=====
 * 帧：CC func ... DD。长度按功能码定：0x02 -> 6字节(左/右/下颜色)，0x05 -> 7字节(转盘中心 x,y u16小端)。
 * 解析按长度驱动，不靠 0xCC/0xDD 做重同步——因为坐标的低/高字节本身可能就是 0xCC/0xDD。
 * 调试注入：写 g_vis_test_byte 后把 g_vis_test_push 置1，poll 会把它当成一个收到的字节，
 * 这样不接香橙派也能验证帧状态机（不能证明电气层，只证明解析）。 */
volatile uint32_t g_vis_rx_bytes=0U,g_vis_rx_bad=0U,g_vis_rx_armed=0U;
volatile uint32_t g_vis_test_byte=0U,g_vis_test_push=0U;
volatile uint32_t g_vis_mat_cycle=0U,g_vis_mat_left=0U,g_vis_mat_right=0U,g_vis_mat_down=0U,g_vis_mat_tick=0U;
volatile uint32_t g_vis_center_cycle=0U,g_vis_center_x=0U,g_vis_center_y=0U,g_vis_center_tick=0U;
/* 视觉区 -> 机械臂取件姿态的标定表（左/右/下三个抓取区），taught_mask 每位置示教完置位 */
volatile float g_vis_region_deg[3]={180.0f,180.0f,180.0f};
volatile float g_vis_region_ext_mm[3]={0.0f,0.0f,0.0f};
volatile uint32_t g_vis_taught_mask=0U,g_vis_pick_region=0U;
/* 像素->毫米：与底盘纠偏参考中心；mm_per_px=0 表示未标定，纠偏保持 0 而不猜数 */
volatile float g_vis_mm_per_px=0.0f,g_vis_ref_x=0.0f,g_vis_ref_y=0.0f;
volatile uint32_t g_vis_txn_result=0U;
volatile uint32_t g_vis_material_valid=0U,g_vis_center_valid=0U;
volatile uint32_t g_vis_mat_consumed=0U,g_vis_metric_valid=0U;
volatile uint32_t g_vis_command_request=0U,g_vis_command_sent=0U,g_vis_command_rejected=0U;
static uint32_t vis_last_byte_ms=0U,vis_errors_seen=0U;

/* Task order is independent of region and carrier slot. No actuator access. */
volatile uint32_t g_vis_task_valid=0U,g_vis_task_batch=0U,g_vis_task_item=0U,g_vis_task_run_id=0U;
volatile uint32_t g_vis_task_colors[3]={0U,0U,0U};
volatile uint32_t g_vis_pending_seq=0U,g_vis_pending_slot=0U,g_vis_pending_grasped=0U;
volatile uint32_t g_vis_loaded_mask=0U,g_vis_batch_complete=0U;
static char vis_task_code[16];

uint32_t Vision_SetTask(const char *code,uint32_t batch,uint32_t run)
{
 uint32_t i,j,offset;uint32_t first=0U,second=0U,rings1=0U,rings2=0U;
 if(!code || !run || (batch!=1U && batch!=2U) || g_vis_pending_seq)return 0U;
 for(i=0U;i<15U;i++){
  if(i==3U || i==7U || i==11U){if(code[i]!='+')return 0U;}
  else if(code[i]<'1' || code[i]>'6')return 0U;
 }
 if(code[15]!='\0')return 0U;
 for(i=0U;i<3U;i++){
  first|=1U<<(code[i]-'1');second|=1U<<(code[8U+i]-'1');
  if(code[4U+i]>'3' || code[12U+i]>'3')return 0U;
  rings1|=1U<<(code[4U+i]-'1');rings2|=1U<<(code[12U+i]-'1');
  for(j=0U;j<i;j++)if(code[i]==code[j] || code[8U+i]==code[8U+j])return 0U;
 }
 if(first!=second || rings1!=7U || rings2!=7U)return 0U;
 if(g_vis_task_valid && g_vis_task_run_id==run && g_vis_task_batch==batch && !memcmp(vis_task_code,code,16U))return 1U;
 memcpy(vis_task_code,code,16U);offset=batch==1U?0U:8U;
 for(i=0U;i<3U;i++)g_vis_task_colors[i]=(uint32_t)(code[offset+i]-'0');
 g_vis_task_valid=1U;g_vis_task_run_id=run;g_vis_task_batch=batch;g_vis_task_item=0U;
 g_vis_loaded_mask=g_vis_batch_complete=0U;g_vis_material_valid=0U;
 return 1U;
}

/* Only the existing mission/CAN owner calls these functions. External screen,
 * sensor or vision providers must supply actual evidence, not motor ACKs. */
uint32_t Vision_ReportGrasp(uint32_t run,uint32_t sequence,uint32_t source_clear,uint32_t held,
 uint32_t evidence_ms,uint32_t now)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || source_clear!=1U || held!=1U ||
    (uint32_t)(now-evidence_ms)>1000U)return 0U;
 g_vis_pending_grasped=1U;return 1U;
}

uint32_t Vision_ReportPlaced(uint32_t run,uint32_t sequence,uint32_t slot,uint32_t released,
 uint32_t present,uint32_t stable,uint32_t evidence_ms,uint32_t now)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||
    !g_vis_pending_grasped || released!=1U || present!=1U || stable!=1U ||
    (uint32_t)(now-evidence_ms)>1000U || g_vis_task_item>=3U)return 0U;
 g_vis_loaded_mask|=1U<<g_vis_task_item;g_vis_task_item++;
 g_vis_pending_seq=g_vis_pending_slot=g_vis_pending_grasped=0U;
 g_vis_material_valid=0U;
 if(g_vis_task_item==3U && g_vis_loaded_mask==7U)g_vis_batch_complete=1U;
 return 1U;
}

uint32_t Vision_RejectUnstarted(uint32_t run,uint32_t sequence)
{
 if(run!=g_vis_task_run_id || !g_vis_pending_seq || sequence!=g_vis_pending_seq ||
    g_vis_pending_grasped)return 0U;
 g_vis_pending_seq=g_vis_pending_slot=0U;
 /* The consumed frame remains consumed; retry requires a fresh observation. */
 return 1U;
}

uint32_t Vision_ReadLoadedBatch(uint32_t run,uint32_t batch,uint32_t colors[3])
{
 uint32_t i;
 if(!colors || !g_vis_task_valid || run!=g_vis_task_run_id || batch!=g_vis_task_batch ||
    !g_vis_batch_complete || g_vis_loaded_mask!=7U || g_vis_task_item!=3U || g_vis_pending_seq)return 0U;
 for(i=0U;i<3U;i++)colors[i]=g_vis_task_colors[i];
 return 1U;
}

uint32_t Vision_ReadLoadedTask(uint32_t run,uint32_t batch,uint32_t colors[3],uint32_t positions[3],uint32_t slots[3])
{
 uint32_t i,offset;
 if(!colors || !positions || !slots || !Vision_ReadLoadedBatch(run,batch,slots))return 0U;
 offset=batch==1U?0U:8U;
 for(i=0U;i<3U;i++){colors[i]=g_vis_task_colors[i];positions[i]=(uint32_t)(vis_task_code[offset+4U+i]-'0');}
 return 1U;
}

static uint8_t vis_buf[10];
static uint32_t vis_len=0U,vis_expect=0U;

static void Vision_Byte(uint8_t b)
{
 uint32_t tk=(uint32_t)xTaskGetTickCount();
 if(vis_len && (uint32_t)(tk-vis_last_byte_ms)>150U){vis_len=vis_expect=0U;g_vis_rx_bad++;}
 vis_last_byte_ms=tk;
 if(vis_len==0U){if(b==0xCCU){vis_buf[0]=b;vis_len=1U;}return;}
 if(vis_len>=sizeof(vis_buf)){vis_len=0U;vis_expect=0U;g_vis_rx_bad++;return;}
 vis_buf[vis_len++]=b;
 if(vis_len==2U)
 {
  if(b==0x02U)vis_expect=6U;
  else if(b==0x05U)vis_expect=7U;
  else {vis_len=0U;vis_expect=0U;g_vis_rx_bad++;return;}
 }
 if(vis_expect && vis_len==vis_expect)
 {
  if(vis_buf[vis_expect-1U]==0xDDU)
  {
   if(vis_buf[1]==0x02U)
   {
    if(vis_buf[2]>6U || vis_buf[3]>6U || vis_buf[4]>6U)g_vis_rx_bad++;
    else{g_vis_mat_left=vis_buf[2];g_vis_mat_right=vis_buf[3];g_vis_mat_down=vis_buf[4];
     g_vis_mat_tick=tk;g_vis_mat_cycle++;g_vis_material_valid=1U;}
   }
   else
   {g_vis_center_x=(uint32_t)vis_buf[2]|((uint32_t)vis_buf[3]<<8);
    g_vis_center_y=(uint32_t)vis_buf[4]|((uint32_t)vis_buf[5]<<8);
    g_vis_center_tick=tk;g_vis_center_cycle++;g_vis_center_valid=1U;}
  }else g_vis_rx_bad++;
  vis_len=0U;vis_expect=0U;
 }
}
void Vision_Poll(void)
{
 uint8_t b,command[3];uint32_t budget=128U,request;
#if APP_ZONE2_MISSION_ENABLE
 AppZone2Station station;
 if(AppZone2_StationRead(&station))
  (void)Vision_SetTask(station.task_code,station.batch,station.ticket);
#endif
 if(!g_vis_rx_armed){USART6_VisionEnableReceive();g_vis_rx_armed=1U;}
 if(vis_errors_seen!=g_uart6_rx_errors){
  vis_errors_seen=g_uart6_rx_errors;vis_len=vis_expect=0U;
  g_vis_material_valid=g_vis_center_valid=0U;
 }
 while(budget-- && USART6_VisionReadByte(&b)){g_vis_rx_bytes++;
#if APP_ZONE2_MISSION_ENABLE
  if(RegionPlacement_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U))continue;
#endif
  if(!RegionDispatch_SharedByte(b,(uint32_t)xTaskGetTickCount(),vis_len!=0U))Vision_Byte(b);}
 RegionDispatch_Telemetry((uint32_t)xTaskGetTickCount());
 if(g_vis_test_push){g_vis_test_push=0U;Vision_Byte((uint8_t)g_vis_test_byte);}
 request=g_vis_command_request;
 if(request){
  if(request!=1U && request!=2U && request!=5U && request!=0x14U && request!=0x15U){
   g_vis_command_request=0U;g_vis_command_rejected++;
  }else{
   command[0]=0xaaU;command[1]=(uint8_t)request;command[2]=0xbbU;
   if(USART6_VisionSend(command,3U)){
    g_vis_command_request=0U;g_vis_command_sent++;
    if(request==2U)g_vis_material_valid=0U;
    if(request==5U)g_vis_center_valid=0U;
   }
  }
 }
}

static uint32_t Vision_TakeTarget(uint32_t now)
{
 uint32_t region=g_vis_pick_region,color;
 float dx=0.0f,dy=0.0f;
 g_vis_txn_result=0U;g_vis_metric_valid=0U;
 if(region>2U || !(g_vis_taught_mask&(1U<<region))){g_vis_txn_result=6U;return 0U;}
 if(!g_vis_material_valid || !g_vis_mat_cycle || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 if(g_vis_mat_cycle==g_vis_mat_consumed){g_vis_txn_result=8U;return 0U;}
 color=region==0U?g_vis_mat_left:(region==1U?g_vis_mat_right:g_vis_mat_down);
 if(!color){g_vis_txn_result=9U;return 0U;}
 if(g_vis_mm_per_px>0.0f && g_vis_center_valid && g_vis_center_cycle &&
    (uint32_t)(now-g_vis_center_tick)<=3000U){
  dx=((float)g_vis_center_x-g_vis_ref_x)*g_vis_mm_per_px;
  dy=((float)g_vis_center_y-g_vis_ref_y)*g_vis_mm_per_px;
  g_vis_metric_valid=1U;
 }
 g_src_pick_deg=g_vis_region_deg[region];g_src_extension_mm=g_vis_region_ext_mm[region];
 g_src_chassis_dx_mm=dx;g_src_chassis_dy_mm=dy;
 g_src_txn_seq++;g_src_txn_tick=now;g_src_txn_revision++;
 if(!SrcTxn_Take(now))return 0U;
 g_vis_mat_consumed=g_vis_mat_cycle;return 1U;
}

static uint32_t Vision_TakeTaskTarget(uint32_t now)
{
 uint32_t colors[3],i,count=0U,region=0U,desired;
 if(!g_vis_task_valid || g_vis_task_item>=3U){g_vis_txn_result=10U;return 0U;}
 if(g_vis_pending_seq){g_vis_txn_result=11U;return 0U;}
 if(!g_vis_material_valid || (uint32_t)(now-g_vis_mat_tick)>3000U){g_vis_txn_result=7U;return 0U;}
 desired=g_vis_task_colors[g_vis_task_item];
 colors[0]=g_vis_mat_left;colors[1]=g_vis_mat_right;colors[2]=g_vis_mat_down;
 for(i=0U;i<3U;i++)if(colors[i]==desired){count++;region=i;}
 if(count!=1U){g_vis_txn_result=count?12U:9U;return 0U;}
 g_vis_pick_region=region;g_src_slot=g_vis_task_item+1U;
 if(!Vision_TakeTarget(now))return 0U;
 g_vis_pending_seq=g_src_txn_seq;g_vis_pending_slot=g_src_slot;
 g_vis_pending_grasped=0U;return 1U;
}

static uint32_t it_platform(void)
{return g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched && !g_chassis_auto_active;}
/* Holding keeps its real health checks during the explicit body-step owner.
 * Ordinary mechanical command admission still uses it_platform unchanged. */
static uint32_t it_hold_platform(void)
{return it_platform() || (SourceBodyStep_WheelWindow() && StartupAxes_Holding() &&
 g_safety_ready && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault &&
 !g_can_ctrl_fault_latched);}

static uint32_t it_at_top(void)
{return g_lift_ruler_controller.top_valid && g_lift_ruler_controller.backoff_valid &&
 g_vehicle_lift_feedback_valid && g_lift_ruler_controller.top_scale &&
 fabsf(g_vehicle_lift_current_deg-(float)g_lift_ruler_controller.safe_position_raw/
       (float)g_lift_ruler_controller.top_scale)<=SOURCE_TOP_TOLERANCE_DEG;}
static void it_fail(uint32_t why)
{
 if(!g_it_cause)g_it_cause=why;g_it_state=9U;g_it_lift_permit=0U;
 if(VehicleSourceParallel_Owns())VehicleSourceParallel_Cancel();
 if(g_lift_ruler_state>0U && g_lift_ruler_state<4U)g_lift_ruler_request=2U;
 if(g_startup_axes_state>=1U && g_startup_axes_state<=3U)StartupAxes_Cancel();
 if(g_dm_posvel_holding || DM_CAN2_JogIsBusy())g_dm_jog_request=3U;
}
static void it_done(void){g_it_state=2U;g_it_lift_permit=0U;}
/* Only a real accepted descent may schedule its latched tray slot. */
static void it_service_lift_command(uint32_t now)
{
 uint32_t complete=(g_lift_ruler_samples!=it_lift_samples && g_lift_ruler_state==LRC_DONE);
 if(g_lift_ruler_rejected){it_fail(5U);return;}
 if(g_it_action==1U && it_carrier_latched_deg>=0.0f){
  if(!g_it_carrier_auto_done){
   if(!((g_lift_ruler_controller.operation==1U &&
         g_lift_ruler_state==LRC_MOVING && g_lift_ruler_controller.moves_sent) ||
        (complete && g_lift_ruler_controller.operation==2U)))return;
   if(CarrierPwm_Write((uint32_t)(500.0f+it_carrier_latched_deg*(2000.0f/270.0f)+0.5f))!=1U){
    it_fail(9U);return;
   }
   it_carrier_pwm_tick=(uint32_t)xTaskGetTickCount();
   g_it_carrier_auto_done=1U;
   return; /* now predates the PWM write in this service. */
  }
  /* Same 700 ms commanded settling interval as IT10; no position sensor. */
  if((uint32_t)(now-it_carrier_pwm_tick)<700U)return;
 }
 if(complete)it_done();
}
static uint32_t it_dm_fresh(void)
{return g_dm_can2_feedback_valid && (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U;}

/* Supervised EMPTY commissioning only: accepted at the taught left yaw,
 * referenced safe top and retracted X. No tray, clamp or body operation. */
static uint32_t it_empty_overlap_begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 if(!SingleTeach_ReferencesReady() || (!it_at_top() && !Low_PickAllowed()) || !it_dm_fresh() ||
  !(Low_PoseAllowed() || Region_PoseAllowed() || g_it_dm_deg==180.0f || g_it_dm_deg==195.0f || g_it_dm_deg==165.0f) ||
  fabsf(g_dm_can2_logical_position_deg-g_it_dm_deg)>1.0f ||
  (fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>2.0f && !Region_ObservePickAllowed() && !Low_PickAllowed()) ||
  !(Low_PoseAllowed() || Region_PoseAllowed() || (g_it_dm_deg==180.0f && g_it_extension_mm==0.0f) ||
    (g_it_dm_deg==195.0f && g_it_extension_mm==156.0f) ||
    (g_it_dm_deg==165.0f && g_it_extension_mm==160.0f)) ||
  g_it_lift_pulses!=29700.0f || g_lift_ruler_request || g_lift_ruler_stop_pending ||
  VehicleArm_Busy())return 0U;
 memset(&it_command,0,sizeof(it_command));
 it_command.sequence=++g_it_sequence;it_command.kind=SPF_CMD_PICK;
 it_command.axes_mask=SPF_LIFT|(!Low_PickAllowed() && g_it_extension_mm>0.0f?SPF_EXT:0U);
 it_command.goal.dm_deg=g_it_dm_deg;it_command.goal.extension_mm=g_it_extension_mm;
 it_command.goal.lift_pulses=g_it_lift_pulses;
 it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=SPF_EXT_RPM;
 it_command.lift_rpm=SPF_LIFT_RPM;it_command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;
 it_command.keep_vm_on=1U;
 if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U))return 0U;
 vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;
 it_carrier_latched_deg=g_it_carrier_auto_deg;
 if(jiazhua(160.0f)!=1U){VehicleSourceParallel_Cancel();return 0U;}
 return 1U;
#endif
}

static void it_empty_overlap_publish(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
 /* Actual settled position from this completed EXT|LIFT group. Preserve top
  * reference, invalidate at-top backoff, and let existing return own reuse. */
 g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*
  (float)g_lift_ruler_controller.top_scale+0.5f);
 g_lift_ruler_controller.backoff_valid=0U;g_lift_ruler_controller.operation=2U;
 g_lift_ruler_controller.samples++;
#endif
}
/* Left single-piece loaded commissioning: the operator confirmed actual
 * clearance after 5mm rise with the other source positions and tray empty.
 * Retract at that real height, keep rising, and yaw only at full safe height.
 * No general three-piece path permission is established by this entry. */
static uint32_t it_loaded_overlap_begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 if(!SingleTeach_ReferencesReady() || !it_dm_fresh() ||
  !(Low_PoseAllowed() || Region_PoseAllowed() || g_it_dm_deg==180.0f || g_it_dm_deg==195.0f || g_it_dm_deg==165.0f) ||
  fabsf(g_dm_can2_logical_position_deg-g_it_dm_deg)>1.0f ||
  !(Low_PoseAllowed() || Region_PoseAllowed() || (g_it_dm_deg==180.0f && g_it_extension_mm==0.0f) ||
    (g_it_dm_deg==195.0f && g_it_extension_mm==156.0f) ||
    (g_it_dm_deg==165.0f && g_it_extension_mm==160.0f)) ||
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg-
   STARTUP_EXTENSION_EXTEND_SIGN*g_it_extension_mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)>SOURCE_EXTENSION_DONE_DEG ||
  g_lift_ruler_controller.operation!=2U || g_lift_ruler_controller.samples==0U ||
  g_lift_compare_target_pulses!=29700U || g_it_lift_pulses!=1800.0f ||
  g_lift_ruler_request || g_lift_ruler_stop_pending || VehicleArm_Busy())return 0U;
 memset(&it_command,0,sizeof(it_command));
 it_command.sequence=++g_it_sequence;it_command.kind=SPF_CMD_TRANSFER;
 it_command.axes_mask=SPF_LIFT|SPF_DM|(g_it_extension_mm>0.0f?SPF_EXT:0U);
 it_command.goal.dm_deg=37.0f;
 it_command.goal.extension_mm=0.0f;it_command.goal.lift_pulses=1800.0f;
 it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=SPF_EXT_RPM;
 it_command.lift_rpm=SPF_LIFT_RPM;it_command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;
 it_command.keep_vm_on=1U;
 if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U))return 0U;
 vsp_group.delayed_mask=it_command.axes_mask&~SPF_LIFT;vsp_group.release_mask=SPF_LIFT;
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
 if(Low_Loaded()){
  uint32_t gate_now=(uint32_t)xTaskGetTickCount();
  if(Low_TransferGate(gate_now,depth,(uint32_t)(gate_now-z->position_tick)<=150U))vsp_group.release_mask|=SPF_EXT|SPF_DM;
  return;
 }
 if(depth>=0.0f && depth<=27700.0f)vsp_group.release_mask|=SPF_EXT;
 /* User-authorized carry overlap: at the last5mm height, yaw may start
  * while retraction finishes. Release still waits for ALL axes. */
 if(depth>=0.0f && depth<=3800.0f)
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

static void it_lead5_grip_service(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1];float depth;
 if(g_it_state!=1U || g_it_action!=14U || g_it_lead5_grip_done ||
  !(g_source_parallel_started_mask&SPF_LIFT) || !z->position_cycle ||
  ((uint32_t)(now-z->position_tick)>150U && !(vsp_group.done&SPF_LIFT)) ||
  ((it_command.axes_mask&SPF_EXT) && !(vsp_group.done&SPF_EXT)))return;
 depth=(z->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 if(depth<27700.0f || depth>29800.0f)return;
 if(jiazhua(140.0f)!=1U){it_fail(7U);return;}
 g_it_lead5_grip_done=1U;
#else
 (void)now;
#endif
}

#include "integration13_immediate.inc"
#include "pzq_near_cycle.inc"
#include "pzq_region_batch.inc"
#include "pzq_commit_overlap.inc"
#include "source_mcu_cycle.inc"
#include "stage3_cycle.inc"
#include "region_dispatch.inc"
static uint32_t S3_TouchdownAllowed(void)
{
 return Stage3_Owns() && g_region_dispatch_test_enable && g_stage3_state==7U &&
  g_stage3_align_done && g_low_wheel_hold_ready && !g_stage3_fault &&
  !g_stage3_manual_state && !g_stage3_manual_request && !s3_pending &&
  !g_stage3_request && g_it_request!=9U && GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) &&
  !g_source_body_step_request &&
  !SourceBodyStep_Busy() && !SourceBodyStep_WheelWindow() && !g_source_align_chain_active &&
  g_lift_ruler_request!=LRC_REQUEST_STOP && !g_lift_ruler_controller.stop_pending &&
  g_lift_ruler_controller.top_scale && SingleTeach_ReferencesReady() && it_platform() &&
  !g_dm_jog_first_fault && !g_dm_jog_request && g_dm_posvel_holding;
}
#if !APP_MOTOR_OFFLINE_ONLY
/* ��λԭ�㿿�����ֳ���ȡȷ�ϣ������þɵķ�����Ч��־�� */
static uint32_t it_stage3_tray_x_ready(void)
{
 return VSP_Position(6U,&vsp_axis[0]) &&
  fabsf(vsp_axis[0].current-g_extension_home_reference_deg)<=SOURCE_EXTENSION_ZERO_DONE_DEG;
}
/* ������ VSP ��������λ�ݲ� 22.5 ��һ�¡� */
static uint32_t it_stage3_tray_depth_ready(float depth)
{return depth>=s3_pick-2000.0f && depth<=s3_pick+22.5f*(3200.0f/360.0f);}
static uint32_t it_stage3_tray_grip(void)
{
 float depth;
 if(!S3_AnticipatoryOwned(5U) || s3_pending!=3U || it_command.axes_mask!=SPF_LIFT ||
    !(g_source_parallel_started_mask&SPF_LIFT))return 1U;
 if(Stage3_FastTransferStopPending() || !SingleTeach_ReferencesReady())return 0U;
 if(s3_tray_grip_sent)return 1U;
 if(!VehicleArm_Stage3LiftDepthFresh(it_command.sequence,&depth) &&
    (g_source_parallel_state!=3U || !VSP_Position(5U,&vsp_axis[1]) ||
     !VehicleArm_Stage3LiftDepthFresh(it_command.sequence,&depth)))return 2U;
 if(!it_stage3_tray_depth_ready(depth))return 2U;
 if(!it_stage3_tray_x_ready())return 2U;
 if(Stage3_FastTransferStopPending() || !SingleTeach_ReferencesReady())return 0U;
 if(!VehicleArm_Stage3LiftDepthFresh(it_command.sequence,&depth) || !it_stage3_tray_depth_ready(depth))return 2U;
 if(jiazhua(140.0f)!=1U)return 0U;
 s3_tray_grip_tick=(uint32_t)xTaskGetTickCount();s3_tray_grip_sent=1U;
 return 1U;
}
#endif
void Integration10_Service(void)
{
 uint32_t request=g_it_request,now=(uint32_t)xTaskGetTickCount();
 Low_Service(now);
 if(g_region_dispatch_area==3U && S3_ManualTeach()){
  g_it_request=0U;g_stage3_request=0U;g_stage3_teach_request=0U;
  g_src_now_ms=now;Vision_Poll();StartupAxes_Service(it_hold_platform(),g_can_guard_ready);return;
 }
 if(g_stage3_manual_state!=6U)RegionDispatch_Service((uint32_t)xTaskGetTickCount());
 if(Low_Busy()){
  uint32_t pending=g_it_request;g_it_request=0U;g_src_now_ms=now;Vision_Poll();
  if(pending==9U)Low_Fail(2U);
  else if(pending)g_it_rejected=1U;
  return;
 }
 Region_Service(now);PZQNear_Service(now);request=g_it_request;
 g_src_now_ms=now;
 Vision_Poll();
 g_it_request=0U;
 if(SourceBodyStep_WheelWindow() && request){
  if(request==9U)g_source_body_step_request=9U;
  else {g_it_rejected=13U;request=0U;}
 }
 /* 仅转发真实回零/运动反馈，不把旧完成值覆盖到当前位姿。 */
 if(g_lift_ruler_samples!=it_seen_samples){
  it_seen_samples=g_lift_ruler_samples;
  if(g_lift_ruler_controller.top_valid && g_lift_ruler_controller.scale){
   g_vehicle_lift_reference_valid=1U;g_vehicle_lift_feedback_valid=1U;
   g_vehicle_lift_current_deg=(float)g_lift_ruler_controller.position_after_raw/
                              (float)g_lift_ruler_controller.scale;
  }
 }
 if(request==9U){if(g_it_auto_state==1U){g_it_auto_state=9U;g_it_auto_fault=2U;}Stage2Fast_Cancel(2U);SourceStage2_Cancel(2U);it_fail(2U);request=0U;}
 if(g_stage3_replay_state>=2U && g_stage3_replay_state<=4U && request){
  g_it_rejected=13U;request=0U;
 }
 if(request==16U){
  if(!it_auto_start(now))g_it_rejected=12U;request=0U;
 }
 if(request){
  if(Stage2Fast_IsActive() || SourceStage2_IsActive() || g_it_state==1U || g_it_state==9U || (request>15U && !((request==17U || request==18U) && Stage3_Owns()))){g_it_rejected=1U;}
  else {
   it_stage3_carry=it_stage3_grip100=0U;
   g_it_rejected=0U;g_it_action=request;it_phase=0U;it_begin=now;g_it_elapsed_ms=0U;
   if(!it_platform())g_it_rejected=2U;
   else if(request==1U){
    /* 2026-11-08: LiftRuler_Start already accepts IDLE or DONE (calibration.c:35),
     * so allow the same states here. Otherwise request 1 could only ever build
     * the first top reference and every later descent was rejected with 3. */
    if((g_lift_ruler_state!=0U && g_lift_ruler_state!=4U) || VehicleArm_Busy())g_it_rejected=3U;
    else {
    g_it_lift_permit=1U;it_lift_samples=g_lift_ruler_samples;
    /* Fast three-piece candidate: pre-close at 160 degrees only for a
     * descent after a valid top reference. First reference build stays open. */
    g_it_grip_prepared=0U;g_it_carrier_auto_done=0U;it_carrier_latched_deg=-1.0f;
    if(g_lift_ruler_controller.state==LRC_DONE && g_lift_ruler_controller.top_valid){
     if(!jiazhua(160.0f)){g_it_rejected=7U;}
     else {
      float slot=g_it_carrier_auto_deg;
      g_it_grip_prepared=1U;
      if((g_lift_ruler_controller.operation==1U || g_lift_ruler_controller.operation==3U) &&
         slot>=0.0f && slot<=270.0f)it_carrier_latched_deg=slot;
     }
    }
    if(!g_it_rejected){g_lift_ruler_request=1U;g_it_state=1U;}
   }
   }else if(request==15U){
    if(it_loaded_overlap_begin())g_it_state=1U;else g_it_rejected=11U;
   }else if(request==14U){
    if(it_empty_overlap_begin())g_it_state=1U;else g_it_rejected=10U;
   }else if(request==13U){
    if(Vision_TakeTaskTarget(now))g_it_state=1U;else g_it_rejected=1U;
   }else if(request==12U){
    if(Vision_TakeTarget(now))g_it_state=1U;else g_it_rejected=1U;
   }else if(request==11U){
    /* 消费视觉取件目标事务；不通过就直接拒绝，绝不用默认值继续动作。 */
    if(SrcTxn_Take(now))g_it_state=1U;else g_it_rejected=1U;   /* 原因看 g_src_txn_result */
   }else if(request==10U){
    /* 载盘转位：只写伺服脉宽，无位置反馈；700ms 是120度行程窗口，不等于到位确认。 */
    it_carrier_permit=g_it_lift_permit;
    if(!(g_it_carrier_deg>=0.0f && g_it_carrier_deg<=270.0f))g_it_rejected=9U;
    else if(CarrierPwm_Write((uint32_t)(500.0f+g_it_carrier_deg*(2000.0f/270.0f)+0.5f))!=1U)g_it_rejected=9U;
    else g_it_state=1U;
   }else if(!g_lift_ruler_controller.top_valid || g_lift_ruler_cause ||
             g_lift_ruler_state!=4U || VehicleArm_Busy())g_it_rejected=4U;
   else if(request==2U){
    if(!it_at_top() || g_startup_axes_state!=0U || !StartupAxes_Begin())g_it_rejected=5U;
    else g_it_state=1U;
   }else if(request==3U || request==18U){
    uint32_t touchdown=request==18U;
    uint32_t axes=g_it_axes;
    uint32_t stage3_scope=Stage3_Owns();
    uint32_t stage3_sync=S3_SyncPlaceOwned() && axes==(SPF_EXT|SPF_LIFT);
    uint32_t stage3_return=stage3_sync && g_stage3_phase==S3_SYNC_RETURN_PHASE;
    uint32_t stage3_carry=request==3U && S3_AnticipatoryOwned(7U) && s3_pending==3U &&
       axes==(SPF_DM|SPF_EXT|SPF_LIFT);
    uint32_t stage3_recover=S3_FastRecoveryOwned() && axes==(SPF_DM|SPF_EXT|SPF_LIFT);
    uint32_t stage3_handoff=S3_EmptyHandoffOwned() && axes==(SPF_DM|SPF_EXT|SPF_LIFT);
    uint32_t stage3_low=S3_LowHandoffOwned() &&
       axes==(g_stage3_phase==17U?SPF_DM:SPF_EXT);
    uint32_t lift_limit=stage3_scope?SPF_STAGE3_MAX_LIFT_PULSES:SPF_MAX_LIFT_PULSES;
    float turn_extension_zero_tolerance_deg = stage3_scope ? SOURCE_EXTENSION_ZERO_DONE_DEG : 2.0f;
    if((touchdown && (axes!=SPF_LIFT || !S3_TouchdownAllowed())) ||
       !SingleTeach_ReferencesReady() || !axes || (axes&~7U) ||
       !(g_it_dm_deg>=0 && g_it_dm_deg<=270) ||
       !(g_it_extension_mm>=0 && g_it_extension_mm<=170) ||
       !(g_it_lift_pulses>=0 && g_it_lift_pulses<=(float)lift_limit) ||
       (stage3_carry && (!S3_TrayGripElapsed() ||
          g_it_extension_mm!=0.0f || g_it_dm_deg!=s3_yaw[s3_positions[g_stage3_item]-1U] ||
          g_it_lift_pulses!=S3_SafeTopPulses())) ||
       (stage3_recover && (g_it_extension_mm!=0.0f || g_it_dm_deg!=s3_carrier ||
                          g_it_lift_pulses!=S3_SafeTopPulses())) ||
       (stage3_handoff && (g_it_extension_mm!=s3_x[s3_positions[s3_first]-1U] ||
           g_it_dm_deg!=s3_yaw[s3_positions[s3_first]-1U] ||
           g_it_lift_pulses!=s3_depth[s3_positions[g_stage3_item]-1U]-S3_FAST_HANDOFF_OFFSET_PULSES)) ||
       ((axes&3U) && !stage3_recover && !stage3_handoff && !stage3_carry && ((!it_at_top() && !Low_XYAllowed() && !stage3_return && !stage3_low) ||
                     ((axes&4U) && !stage3_sync))) ||
       ((axes&4U) && g_it_lift_pulses<3200 && !stage3_return && !stage3_recover && !stage3_carry) ||
       /* 2026-11-08: this depth guard is place-path specific (arm at logical 180,
        * extension near the place offset). Running it unconditionally rejected every
        * pick extend that happened at any other arm angle with rejected=6. Apply it
        * only when a place depth limit is actually armed. */
       ((axes&4U) && !stage3_scope && g_it_place_depth_limit_pulses &&
          !(fabsf(g_dm_can2_logical_position_deg-180.0f)<=3.0f &&
          fabsf(g_startup_extension_current_deg-(g_extension_home_reference_deg+STARTUP_EXTENSION_EXTEND_SIGN*48.0f*3.6f))<=3.6f) &&
          (g_it_place_depth_limit_pulses>SPF_MAX_LIFT_PULSES ||
           g_it_lift_pulses>(float)g_it_place_depth_limit_pulses)) ||
       ((axes&1U) && !stage3_recover && !stage3_handoff && !stage3_carry && !Low_XYAllowed() && !Region_ObserveYawAllowed() && !SC_InitialObserveAllowed() &&
          (fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>turn_extension_zero_tolerance_deg ||
                     ((axes&2U) && g_it_extension_mm>0.01f))))g_it_rejected=6U;
    else {
     memset(&it_command,0,sizeof(it_command));
     it_command.kind=touchdown?SPF_CMD_STAGE3_TOUCHDOWN:
                      (stage3_handoff?SPF_CMD_STAGE3_EMPTY_HANDOFF:
                       (stage3_recover?SPF_CMD_STAGE3_LOADED_RETURN:
                        (stage3_sync?SPF_CMD_STAGE3_FAST_TRANSFER:SPF_CMD_TRANSFER)));
     it_command.sequence=++g_it_sequence;
     it_command.lift_limit_pulses=stage3_scope?SPF_STAGE3_MAX_LIFT_PULSES:0U;
     it_command.axes_mask=axes;it_command.goal.dm_deg=g_it_dm_deg;
     it_command.goal.extension_mm=g_it_extension_mm;it_command.goal.lift_pulses=g_it_lift_pulses;
     it_command.dm_deg_s=SPF_DM_DEG_S;
     it_command.extension_rpm=stage3_sync?SPF_EXT_RPM:
         (stage3_scope?SPF_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM));
     it_command.lift_rpm=touchdown?SPF_STAGE3_TOUCHDOWN_RPM:SPF_LIFT_RPM;
     /* 2026-11-08: 45055 is not a legal stepper acceleration for these drives -- it made
     * the extension axis run away (0 -> -670 deg in 0.18s, overshooting the 156mm
     * target by 108 deg and tripping the VSP overshoot check). The lift ruler uses
     * 600 for the same hardware, so match that. */
     it_command.stepper_accel_rpm_s=600;it_command.keep_vm_on=1U;
     if(stage3_carry){
      uint32_t ring=s3_positions[g_stage3_item]-1U;
      it_stage3_carry=1U;it_command.kind=SPF_CMD_STAGE3_CARRY_RISE;
      it_command.run_id=g_stage3_locked_run;it_command.item=g_stage3_item;
      it_stage3_place=it_command;it_stage3_place.kind=SPF_CMD_STAGE3_CARRY_PLACE;
      it_command.lift_rpm=SPF_STAGE3_CARRY_RISE_LIFT_RPM;
      it_stage3_place.axes_mask=SPF_EXT|SPF_LIFT;
      it_stage3_place.goal.extension_mm=s3_x[ring];it_stage3_place.goal.lift_pulses=s3_depth[ring];
     }
     if(!g_dm_posvel_holding){it_dm_cycle=g_dm_can2_read_cycle;g_dm_can2_read_request=2U;it_phase=1U;}
     else it_phase=2U;
     g_it_state=1U;
    }
   }else if(request==4U){
    g_it_lift_permit=1U;it_lift_samples=g_lift_ruler_samples;
    g_lift_ruler_request=3U;g_it_state=1U;
   }else if(request==5U || request==6U || request==17U){
    if(jiazhua(request==5U?140.0f:(request==17U?160.0f:210.0f))!=1U)g_it_rejected=7U;
    else {
     it_stage3_grip100=s3_pending==request &&
        ((request==5U && S3_AnticipatoryOwned(20U)) ||
         (request==17U && (S3_AnticipatoryOwned(11U) || S3_AnticipatoryOwned(26U))));
     if(it_stage3_grip100)it_stage3_grip_tick=(uint32_t)xTaskGetTickCount();
     g_it_state=1U;
    }
   }else if(request==7U){
    it_dm_cycle=g_dm_can2_read_cycle;
    if(!g_dm_posvel_holding){g_dm_can2_read_request=2U;it_phase=1U;}
    else it_phase=2U;
    g_it_state=1U;
   }else if(request==8U){
    if(!it_at_top() || CarrierPwm_Write(500U)!=1U)g_it_rejected=8U;
    else g_it_state=1U;
   }
  }
 }
 /* 即使已失败也保留原后端STOP重试和真实反馈服务。 */
 /* DM参数读取窗口内暂缓伸缩保持状态查询，取消/失败STOP仍服务。 */
 if(g_startup_axes_state>=5U || !(g_dm_can2_read_request || (g_dm_can2_state>=1U && g_dm_can2_state<=3U)))
  StartupAxes_Service(it_hold_platform(),g_can_guard_ready);
 if(g_it_state==1U && g_it_action==14U && !g_it_carrier_auto_done &&
  it_carrier_latched_deg>=0.0f && it_carrier_latched_deg<=270.0f &&
  (g_source_parallel_started_mask&SPF_LIFT)){
  if(!CarrierPwm_Write((uint32_t)(500.0f+it_carrier_latched_deg*2000.0f/270.0f+0.5f))){it_fail(9U);return;}
  it_carrier_pwm_tick=(uint32_t)xTaskGetTickCount();g_it_carrier_auto_done=1U;
 }
 it_loaded_overlap_gates((uint32_t)xTaskGetTickCount());
 if(VehicleSourceParallel_Owns())VehicleArm_Service();
 SourceStage2_Service(now);
 Stage2Fast_Service((uint32_t)xTaskGetTickCount());
 it_lead5_grip_service((uint32_t)xTaskGetTickCount());
 if(g_it_state!=1U){it_auto_service((uint32_t)xTaskGetTickCount());return;}
 g_it_elapsed_ms=now-it_begin;
 if(!it_platform() || g_lift_ruler_cause || g_startup_axes_cause || g_dm_jog_first_fault){it_fail(3U);return;}
 if(!(APP_ZONE2_MISSION_ENABLE && RegionDispatch_WantsService()) &&
    g_it_elapsed_ms>=(g_it_action==1U || g_it_action==4U?60000U:15000U)){it_fail(4U);return;}
 switch(g_it_action){
 case 1U:case 4U:
  it_service_lift_command(now);break;
 case 2U:
  if(g_startup_axes_state==4U && g_extension_home_reference_valid && g_extension_hold_confirmed)it_done();
  break;
 case 3U:case 18U:
  if(it_phase==1U){
   if(g_dm_can2_read_cycle==it_dm_cycle)break; /* 不消费上一轮失败。 */
   if(g_dm_can2_state==5U){it_fail(6U);return;}
   if(g_dm_can2_read_cycle==it_dm_cycle || !it_dm_fresh())break;
   it_phase=2U;
  }
  if(it_phase==2U){
   if(!it_dm_fresh())break;
   if(g_it_action==18U && !S3_TouchdownAllowed()){it_fail(2U);return;}
   if(it_stage3_carry){
#if !APP_MOTOR_OFFLINE_ONLY
    if(Stage3_FastTransferStopPending()){it_fail(2U);return;}
    if(!it_stage3_tray_x_ready())break;
    if(Stage3_FastTransferStopPending()){it_fail(2U);return;}
#endif
    if(!VehicleArm_BeginStage3CarryPlace(&it_command,&it_stage3_place)){it_fail(7U);return;}
   }else if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}
#if !APP_MOTOR_OFFLINE_ONLY
   if(Stage3_Owns() && (it_command.axes_mask&SPF_LIFT) &&
      it_command.kind!=SPF_CMD_STAGE3_TOUCHDOWN)vsp_fast_lift=1U;
   if(S3_SyncPlaceOwned() && it_command.axes_mask==(SPF_EXT|SPF_LIFT))vsp_pick_sync=1U;
   if(it_command.kind==SPF_CMD_STAGE3_LOADED_RETURN){
    vsp_group.delayed_mask=SPF_DM;vsp_group.release_mask=SPF_EXT|SPF_LIFT;vsp_pick_sync=1U;
   }
   if(g_source_cycle_state==1U && g_source_cycle_prep==4U &&
      it_command.axes_mask==(SPF_DM|SPF_EXT) && it_command.goal.dm_deg==180.0f &&
      it_command.goal.extension_mm==65.0f){vsp_pick_sync=1U;g_dm_stage2_sync=1U;g_dm_initial_observe=SC_InitialObserveAllowed();}
#endif
   it_phase=3U;break;
  }
  if(g_source_parallel_state==4U){it_fail(8U);return;}
#if !APP_MOTOR_OFFLINE_ONLY
  {
   uint32_t grip=it_stage3_tray_grip();
   if(!grip){it_fail(7U);return;}
   if(grip==2U && g_source_parallel_state==3U)return;
   if(it_stage3_carry && vsp_carry_sent){
    if(Stage3_FastTransferStopPending()){it_fail(2U);return;}
    if(!S3_PrefetchNextTray(g_stage3_item+1U,(uint32_t)xTaskGetTickCount()))return;
   }
   if(it_stage3_carry && g_source_parallel_state==3U &&
      (VehicleSourceParallel_Owns() || (g_source_parallel_done_mask&7U)!=7U)){it_fail(8U);return;}
  }
#endif
  if((it_command.axes_mask&SPF_DM) && (g_source_parallel_started_mask&SPF_DM) &&
     !S3_PrecloseOnDmStarted()){it_fail(7U);return;}
  if(g_source_parallel_state==3U){
#if !APP_MOTOR_OFFLINE_ONLY
   if(Stage3_Owns() && (it_command.axes_mask&SPF_LIFT)){
    g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*(float)g_lift_ruler_controller.top_scale);
    if((it_command.kind==SPF_CMD_STAGE3_FAST_TRANSFER &&
        g_stage3_phase==S3_SYNC_RETURN_PHASE && !s3_low_handoff) ||
       it_command.kind==SPF_CMD_STAGE3_LOADED_RETURN){
     if(VehicleSourceParallel_Owns() ||
        (g_source_parallel_done_mask&it_command.axes_mask)!=it_command.axes_mask ||
        fabsf(vsp_axis[1].current-(float)g_lift_ruler_controller.safe_position_raw/
              (float)g_lift_ruler_controller.top_scale)>SOURCE_TOP_TOLERANCE_DEG){
      it_fail(8U);return;
     }
     g_lift_ruler_controller.backoff_valid=1U;
     g_lift_ruler_controller.operation=3U;
    }else{
     g_lift_ruler_controller.backoff_valid=0U;
     g_lift_ruler_controller.operation=2U;
     if(it_command.kind==SPF_CMD_STAGE3_EMPTY_HANDOFF){
      if(!S3_EmptyHandoffOwned() || VehicleSourceParallel_Owns() || !SingleTeach_ReferencesReady() ||
         !g_vehicle_lift_feedback_valid || !g_lift_ruler_controller.top_scale ||
         g_source_parallel_done_mask!=(SPF_DM|SPF_EXT|SPF_LIFT) ||
         fabsf(vsp_axis[1].current-vsp_axis[1].goal)>22.5f ||
         !(fabsf(vsp_axis[0].current-vsp_axis[0].goal)<=
           (it_command.goal.extension_mm==0.0f?SOURCE_EXTENSION_ZERO_DONE_DEG:SOURCE_EXTENSION_DONE_DEG)) ||
         !S3_HandoffYawReady(s3_positions[s3_first]-1U)){
       it_fail(8U);return;
      }
      s3_low_handoff=2U;
     }
    }
    g_lift_ruler_controller.samples++;
   }
#endif
   it_done();}
  break;
 case 15U:
  if(g_source_parallel_state==4U){it_fail(8U);return;}
  if(g_source_parallel_state==3U && !VehicleSourceParallel_Owns() &&
   (g_source_parallel_done_mask&it_command.axes_mask)==it_command.axes_mask){it_loaded_overlap_publish();it_done();}break;
 case 14U:
  if(g_source_parallel_state==4U){it_fail(8U);return;}
  if(g_source_parallel_state==3U && !VehicleSourceParallel_Owns() &&
   (g_source_parallel_done_mask&it_command.axes_mask)==it_command.axes_mask &&
   (it_carrier_latched_deg<0.0f || (g_it_carrier_auto_done &&
    (uint32_t)(now-it_carrier_pwm_tick)>=700U))){it_empty_overlap_publish();it_done();}break;
 case 5U:case 6U:case 17U:
  /* User-selected 200 ms from close PWM request; no opening feedback. */
  if(it_stage3_grip100){
   if((uint32_t)((uint32_t)xTaskGetTickCount()-it_stage3_grip_tick)>=100U)it_done();
  }else if(g_it_elapsed_ms>=((Stage3_Owns() && g_it_action==5U)?200U:600U))it_done();
  break;
 case 7U:
  if(it_phase==1U && g_dm_can2_read_cycle==it_dm_cycle)break;
  if(g_dm_can2_state==5U){it_fail(6U);return;}
  if((it_phase==2U || g_dm_can2_read_cycle!=it_dm_cycle) && it_dm_fresh())it_done();
  break;
 case 8U:
  if(g_it_elapsed_ms>=1500U)it_done();break;
 case 11U:case 12U:case 13U:
  /* Observation/transaction completion owns no lift admission. */
  g_it_state=2U;break;
 case 10U:
  if(s3_pending==10U && S3_AnticipatoryOwned(0U)){it_done();g_it_lift_permit=it_carrier_permit;break;}
  if(it_phase==0U){it_phase=1U;break;}
  if(g_it_elapsed_ms>=700U){it_done();g_it_lift_permit=it_carrier_permit;}break;
 default:it_fail(9U);break;
 }
 it_auto_service((uint32_t)xTaskGetTickCount());
}
