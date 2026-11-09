#include <stdint.h>
#include <stdio.h>
uint32_t g_can_ctrl_fault_latched;
uint32_t g_can_test_result;
uint32_t g_chassis_auto_active;
uint32_t g_dm_can2_feedback_tick;
uint32_t g_dm_jog_request;
uint32_t g_dm_jog_result;
uint32_t g_dm_jog_state;
uint32_t g_dm_posvel_holding;
uint32_t g_extension_hold_confirmed;
uint32_t g_it_lift_permit;
uint32_t g_rtos_fault;
uint32_t g_safety_fault;
uint32_t g_safety_ready;
uint32_t g_startup_axes_cause;
uint32_t g_startup_axes_state;
uint32_t g_vehicle_arm_state;
uint32_t xTaskGetTickCount(void){return 1000U;}
static uint32_t ruler_ready(void)
{
 return g_safety_ready==1U && g_can_test_result==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
  !g_chassis_auto_active && g_it_lift_permit && (g_vehicle_arm_state==0U || g_vehicle_arm_state==2U) &&
  (g_startup_axes_state==0U || (g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_hold_confirmed)) &&
  !g_dm_jog_request && (g_dm_jog_state==0U || (g_dm_jog_state==7U && g_dm_jog_result==1U && g_dm_posvel_holding &&
   (uint32_t)xTaskGetTickCount()-g_dm_can2_feedback_tick<100U));
}
int main(void){unsigned checks=0,fail=0;
#define C(v) do{checks++;if(!(v))fail++;}while(0)
 g_safety_ready=g_can_test_result=g_it_lift_permit=1;
 C(ruler_ready());g_it_lift_permit=0;C(!ruler_ready());g_it_lift_permit=1;
 g_vehicle_arm_state=1;C(!ruler_ready());g_vehicle_arm_state=2;C(ruler_ready());
 g_startup_axes_state=4;C(!ruler_ready());g_extension_hold_confirmed=1;C(ruler_ready());
 g_dm_jog_state=7;C(!ruler_ready());g_dm_jog_result=g_dm_posvel_holding=1;
 g_dm_can2_feedback_tick=901;C(ruler_ready());g_dm_can2_feedback_tick=900;C(!ruler_ready());
 g_dm_can2_feedback_tick=1000;C(ruler_ready());g_startup_axes_cause=1;C(!ruler_ready());g_startup_axes_cause=0;
 g_safety_fault=1;C(!ruler_ready());g_safety_fault=0;g_can_ctrl_fault_latched=1;C(!ruler_ready());
 printf("ACTUAL_LIFT_READINESS checks=%u failures=%u\n",checks,fail);return fail?1:0;}
