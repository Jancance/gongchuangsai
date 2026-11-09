#include "lift_ruler_adapter.h"
#include "app_config.h"
#include "vehicle_arm.h"
#include "startup_axes.h"
#include "dm_can2.h"
#include "power.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
LiftRulerController g_lift_ruler_controller;
volatile uint32_t g_lift_ruler_request,g_lift_ruler_rejected;
volatile uint32_t g_lift_move_last_result,g_lift_move_last_reply[8];
volatile uint32_t g_lift_exchange_first_valid,g_lift_exchange_first_cmd,g_lift_exchange_first_result;
volatile uint32_t g_lift_exchange_first_phase,g_lift_exchange_first_raw[8];
volatile uint32_t g_lift_diag_request,g_lift_diag_cycle,g_lift_diag_done;
volatile uint32_t g_lift_diag_result[6],g_lift_diag_raw[6][8];
volatile uint32_t g_lift_compare_backoff_valid;
volatile int32_t g_lift_compare_safe_raw;
volatile uint32_t g_lift_compare_operation,g_lift_compare_top_valid,g_lift_compare_home_flags;
volatile uint32_t g_lift_compare_home_seen,g_lift_compare_home_reported,g_lift_compare_home_exit;
volatile uint32_t g_lift_compare_speed_raw,g_lift_compare_move_elapsed_ms;
volatile int32_t g_lift_compare_top_raw;
volatile uint32_t g_lift_compare_target_pulses=118800U; /* 与 DOWN_TARGET_PULSES 同步 */ /* configuration, not a motion request */
volatile uint32_t g_lift_ruler_state,g_lift_ruler_phase,g_lift_ruler_cause,g_lift_ruler_samples;
volatile uint32_t g_lift_ruler_options,g_lift_ruler_encoder_before,g_lift_ruler_encoder_after;
volatile uint32_t g_lift_ruler_stop_pending,g_lift_ruler_stop_unconfirmed,g_lift_ruler_vm_request;
volatile int32_t g_lift_ruler_before_raw,g_lift_ruler_after_raw,g_lift_ruler_target_raw;
extern volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
extern volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
/* 4310 挂在 CAN2，升降挂在 CAN1，两套独立控制器与收发器，物理上可并行。
 * 原实现要求 !g_dm_jog_request 且 DM 静止/保持，于是"升到安全高度后同时转臂"会被判 not ready，
 * LiftRuler_DebugSafety 随即发 STOP(cause2 GUARD)。现在只要求 DM 链路健康：无故障、状态有效、反馈新鲜；
 * 从未启动过 DM 动作(jog_state==0)时照旧放行，保持原有启动路径不变。 */
static uint32_t dm_parallel_ok(void)
{
 if (g_dm_jog_first_fault) return 0U;
 if (g_dm_can2_state==5U) return 0U;
 if (g_dm_jog_state==8U && g_dm_jog_result!=2U) return 0U;
 if (g_dm_jog_state==0U) return 1U;
 /* 运动中(4/5)：由 DM 自身状态机看护(JogPoll超时/BOFF/超差都会转成 first_fault)，
  * 这里不再因单次反馈采样间隙判 not ready，否则升降会被误 STOP(cause1 CANCEL)。 */
 if (g_dm_jog_state==4U || g_dm_jog_state==5U) return 1U;
 if (!g_dm_can2_feedback_valid) return 0U;
 if ((uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick>=100U) return 0U;
 return 1U;
}
static uint32_t ruler_ready(void)
{
 return g_safety_ready==1U && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
  !g_chassis_auto_active && g_it_lift_permit && (g_vehicle_arm_state==0U || g_vehicle_arm_state==2U) &&
  (g_startup_axes_state==0U || (g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_hold_confirmed)) &&
  dm_parallel_ok();
}
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t ruler_exchange(void*unused,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{
 CanRxMsg rx;uint32_t result;(void)unused;memset(&rx,0,sizeof(rx));
 if(n==2U && q[0]==0x22U && q[1]==0x6bU && rn==17U)
  return LiftRuler_ReadHomeParameters(r)==1U;
 if(n>15 || rn>8)return 0;
 result=VehicleArm_Exchange(5U,q,(uint8_t)n,(uint8_t)rn,&rx);
 if(q[0]==0xfdU){uint32_t i;g_lift_move_last_result=result;for(i=0;i<8U;i++)g_lift_move_last_reply[i]=rx.Data[i];}
 if(!g_lift_exchange_first_valid && (result!=1U ||
    ((q[0]==0xfdU || q[0]==0xf3U || q[0]==0x9aU || q[0]==0x9cU) &&
      rx.Data[1]!=2U && rx.Data[1]!=0x9fU))){
  uint32_t i;g_lift_exchange_first_cmd=q[0];g_lift_exchange_first_result=result;
  g_lift_exchange_first_phase=g_lift_ruler_controller.phase;
  for(i=0;i<8U;i++)g_lift_exchange_first_raw[i]=rx.Data[i];
  g_lift_exchange_first_valid=1U;
 }
 if(result!=1U)return result;
 memcpy(r,rx.Data,8);return 1;
}
#endif
static void ruler_publish(void)
{
 g_lift_ruler_phase=g_lift_ruler_controller.phase;g_lift_ruler_cause=g_lift_ruler_controller.cause;
 g_lift_ruler_samples=g_lift_ruler_controller.samples;g_lift_ruler_options=g_lift_ruler_controller.options;
 g_lift_ruler_encoder_before=g_lift_ruler_controller.encoder_before;g_lift_ruler_encoder_after=g_lift_ruler_controller.encoder_after;
 g_lift_ruler_before_raw=(int32_t)g_lift_ruler_controller.command_before_raw;
 g_lift_ruler_after_raw=(int32_t)g_lift_ruler_controller.position_after_raw;
 g_lift_ruler_target_raw=(int32_t)g_lift_ruler_controller.driver_target_raw;
 g_lift_ruler_stop_pending=g_lift_ruler_controller.stop_pending;
 g_lift_ruler_stop_unconfirmed=g_lift_ruler_controller.stop_unconfirmed;
 g_lift_ruler_vm_request=g_lift_ruler_controller.vm_request;
 g_lift_compare_operation=g_lift_ruler_controller.operation;
 g_lift_compare_top_valid=g_lift_ruler_controller.top_valid;
 g_lift_compare_backoff_valid=g_lift_ruler_controller.backoff_valid;
 g_lift_compare_safe_raw=(int32_t)g_lift_ruler_controller.safe_position_raw;
 g_lift_compare_top_raw=(int32_t)g_lift_ruler_controller.top_position_raw;
 g_lift_compare_home_flags=g_lift_ruler_controller.home_flags;
 g_lift_compare_home_seen=g_lift_ruler_controller.home_seen_running;
 g_lift_compare_home_reported=g_lift_ruler_controller.home_reported;
 g_lift_compare_home_exit=g_lift_ruler_controller.home_exit_confirmed;
 g_lift_compare_speed_raw=g_lift_ruler_controller.speed_raw;
 g_lift_compare_move_elapsed_ms=g_lift_ruler_controller.move_elapsed_ms;
 g_lift_ruler_state=g_lift_ruler_controller.state;
}
void LiftRuler_DebugKey(void)
{
 if(!LIFT_RULER_SERVICE_ENABLED)return;
 taskENTER_CRITICAL();
 g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state==LRC_DONE) &&
  !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;
 taskEXIT_CRITICAL();
}
void LiftRuler_DebugService(void)
{
 uint32_t request;
#if !APP_MOTOR_OFFLINE_ONLY
 uint32_t ready,now;PowerCommandStatus power;
#endif
 if(!LIFT_RULER_SERVICE_ENABLED)return;
 taskENTER_CRITICAL();request=g_lift_ruler_request;g_lift_ruler_request=0;taskEXIT_CRITICAL();
#if APP_MOTOR_OFFLINE_ONLY
 if(request)g_lift_ruler_rejected=2U;
 g_vm_test_request=0U;ruler_publish();return;
#else
 now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;ready=ruler_ready();
 if(request==LRC_REQUEST_START || request==3U)g_lift_ruler_rejected=0U;
 if(request==LRC_REQUEST_START){
  if(!LiftRuler_SetDownTarget(&g_lift_ruler_controller,g_lift_compare_target_pulses))g_lift_ruler_rejected=4U;
  else if(!LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 }
 if(request==4U){g_lift_ruler_rejected=0U;if(!LiftRuler_StartRelease(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;}
 if(request==3U && !LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 if(request==LRC_REQUEST_STOP)LiftRuler_Stop(&g_lift_ruler_controller,now);
 Power_ReadCommandStatus(&power);
 /* Read-only driver diagnosis. No reference, motion, enable or clear writes. */
 if(g_lift_diag_request==1U){
  static const uint8_t cmd[6]={0x3a,0x3b,0x35,0x36,0x1a,0x3f};
  static const uint8_t rn[6]={3,3,5,7,4,4};
  uint32_t i,j;CanRxMsg rx;uint8_t q[2];
  g_lift_diag_request=0U;g_lift_diag_done=0U;
  if(power.active && g_can_guard_ready && !g_safety_fault && !g_rtos_fault &&
     !g_can_ctrl_fault_latched && !request &&
     (g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state>=LRC_DONE) &&
     !g_lift_ruler_controller.stop_pending){
   for(i=0;i<6U;i++){
    memset(&rx,0,sizeof(rx));q[0]=cmd[i];q[1]=0x6bU;
    g_lift_diag_result[i]=VehicleArm_Exchange(5U,q,2U,rn[i],&rx);
    for(j=0;j<8U;j++)g_lift_diag_raw[i][j]=rx.Data[j];
   }
   g_lift_diag_done=1U;
  }else{for(i=0;i<6U;i++)g_lift_diag_result[i]=9U;}
  g_lift_diag_cycle++;
 }
 /* 2026-11-08: the mechanism axis must not depend on the wheel guard.
  * 'ready' already carries safety/can_ctrl/arm/extension health. */
 LiftRuler_Service(&g_lift_ruler_controller,now,ready,power.active,1U,ruler_exchange,0);
 /* 2026-11-07 integration: in the full-vehicle profile the controller never clears
  * another owner VM; it only raises its own request. Isolated profile keeps the old single-owner rule. */
 if(VEHICLE_LIFT_RULER_ONLY)g_vm_test_request=g_lift_ruler_controller.vm_request;ruler_publish();
#endif
}
void LiftRuler_DebugSafety(void)
{
 if(!LIFT_RULER_SERVICE_ENABLED)return;
 if(!ruler_ready() && g_lift_ruler_controller.state!=LRC_IDLE && g_lift_ruler_controller.state<LRC_DONE)
  g_lift_ruler_request=LRC_REQUEST_STOP;
 /* Keep ordinary platform Safety_Update. No new fault path cuts VM. */
 if(VEHICLE_LIFT_RULER_ONLY)g_vm_test_request=g_lift_ruler_controller.vm_request;
 else if(g_lift_ruler_controller.vm_request)g_vm_test_request=1U;
}
uint32_t LiftRuler_DebugCanAllowed(uint8_t address,const uint8_t*q,uint32_t n)
{
 /* 2026-11-08 fix: the isolated-profile guard blocked every node5 command once the
  * project moved to the full-vehicle profile (VEHICLE_LIFT_RULER_ONLY=0), so the
  * 0xF3 enable returned 9 and the lift could never start (LRC_CAUSE_COMM).
  * Gate on the service macro instead so both profiles use the same whitelist. */
 return (LIFT_RULER_SERVICE_ENABLED && address==5U && LiftRuler_CommandAllowed(&g_lift_ruler_controller,q,n)) || VehicleArm_SourceParallelCanAllowed(address,q,n);
}
