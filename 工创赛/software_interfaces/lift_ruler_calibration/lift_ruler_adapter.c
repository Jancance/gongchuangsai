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
volatile uint32_t g_lift_ruler_state,g_lift_ruler_phase,g_lift_ruler_cause,g_lift_ruler_samples;
volatile uint32_t g_lift_ruler_options,g_lift_ruler_encoder_before,g_lift_ruler_encoder_after;
volatile uint32_t g_lift_ruler_stop_pending,g_lift_ruler_stop_unconfirmed,g_lift_ruler_vm_request;
volatile int32_t g_lift_ruler_before_raw,g_lift_ruler_after_raw,g_lift_ruler_target_raw;
extern volatile uint32_t g_safety_ready,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
extern volatile uint32_t g_can_test_result,g_can_guard_ready,g_vm_test_request,g_chassis_auto_active;
static uint32_t ruler_ready(void)
{
 return g_safety_ready==1U && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
  !g_chassis_auto_active && g_vehicle_arm_state==0U && g_startup_axes_state==0U && !g_dm_jog_request && g_dm_jog_state==0U;
}
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t ruler_exchange(void*unused,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{
 CanRxMsg rx;uint32_t result;(void)unused;memset(&rx,0,sizeof(rx));
 if(n>15 || rn>8)return 0;
 result=VehicleArm_Exchange(5U,q,(uint8_t)n,(uint8_t)rn,&rx);
 if(result!=1U)return 0;
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
 g_lift_ruler_state=g_lift_ruler_controller.state;
}
void LiftRuler_DebugKey(void)
{
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 taskENTER_CRITICAL();
 g_lift_ruler_request=(g_lift_ruler_controller.state==LRC_IDLE && !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;
 taskEXIT_CRITICAL();
}
void LiftRuler_DebugService(void)
{
 uint32_t request;
#if !APP_MOTOR_OFFLINE_ONLY
 uint32_t ready,now;PowerCommandStatus power;
#endif
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 taskENTER_CRITICAL();request=g_lift_ruler_request;g_lift_ruler_request=0;taskEXIT_CRITICAL();
#if APP_MOTOR_OFFLINE_ONLY
 if(request)g_lift_ruler_rejected=2U;
 g_vm_test_request=0U;ruler_publish();return;
#else
 now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;ready=ruler_ready();
 if(request==LRC_REQUEST_START && !LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 if(request==LRC_REQUEST_STOP)LiftRuler_Stop(&g_lift_ruler_controller,now);
 Power_ReadCommandStatus(&power);
 LiftRuler_Service(&g_lift_ruler_controller,now,ready,power.active,g_can_guard_ready,ruler_exchange,0);
 g_vm_test_request=g_lift_ruler_controller.vm_request;ruler_publish();
#endif
}
void LiftRuler_DebugSafety(void)
{
 if(!VEHICLE_LIFT_RULER_ONLY)return;
 if(!ruler_ready() && g_lift_ruler_controller.state!=LRC_IDLE && g_lift_ruler_controller.state<LRC_DONE)
  g_lift_ruler_request=LRC_REQUEST_STOP;
 /* Keep ordinary platform Safety_Update. No new fault path cuts VM. */
 g_vm_test_request=g_lift_ruler_controller.vm_request;
}
uint32_t LiftRuler_DebugCanAllowed(uint8_t address,const uint8_t*q,uint32_t n)
{
 return VEHICLE_LIFT_RULER_ONLY && address==5U && LiftRuler_CommandAllowed(&g_lift_ruler_controller,q,n);
}
