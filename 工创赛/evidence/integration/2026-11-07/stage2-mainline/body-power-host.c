
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define APP_MOTOR_OFFLINE_ONLY 0
#define APP_TEST_MODE 3
#define HR_IDLE 0
#define MOTION_DONE 2
typedef struct{unsigned active,result,cut_vm,cause,rotation_stop_mask;}ChassisAutoStatus;
typedef struct{unsigned active;}PowerCommandStatus;
static ChassisAutoStatus status;
static PowerCommandStatus power;
static unsigned keeps,source_ok=1,exclusive=1,holding=1;
static unsigned s_motion_session_open=1,s_combined_keep_vm=1,s_combined_end_ms,s_motion_list_submit=1,g_vm_test_request=1;
static unsigned g_motion_plan_valid=1,g_translation_hold_selftest=1,s_serial_proto_ok=1,g_heading_exec_trip,g_can_ctrl_fault_latched;
static struct{unsigned state,request,completed,cause,index;}g_combined_test;
static struct{unsigned phase,request,trip;}g_forward_hold;
static struct{float rate_dps;}s_heading_source;
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus*s){*s=status;return 1;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){*p=power;}
static uint32_t Stage2Body_KeepsPower(void){return keeps;}
static uint32_t MotionGetState(void){return MOTION_DONE;}
static uint32_t xTaskGetTickCount(void){return 100;}
static uint32_t StartupAxes_Holding(void){return holding;}
static uint32_t CAN1_EmmSettingsConfirmed(void){return 1;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return source_ok;}
static uint32_t App_HeadingRuntimeExclusive(void){return exclusive;}
uint32_t MotionFinish(void)
{
    ChassisAutoStatus status;
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status);
    if (s_motion_session_open && MotionGetState() == MOTION_DONE && status.result == 1U &&
        !status.active && !status.cause && status.rotation_stop_mask == 15U &&
        ((g_combined_test.state == 0U) || s_motion_list_submit))
    {
        s_combined_end_ms = (uint32_t)xTaskGetTickCount();
        s_motion_session_open = s_combined_keep_vm = 0U;
        if(!Stage2Body_KeepsPower())g_vm_test_request = 0U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
static uint32_t App_SerialIdleBlocked(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)now;return 1U;
#else
    ChassisAutoStatus status;PowerCommandStatus power;uint32_t mask=0U;
    if(!CAN1_EmmSettingsConfirmed())mask|=1U;
    (void)ChassisAuto_ReadStatus(&status);Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if(g_combined_test.state || g_combined_test.request || g_motion_plan_valid!=1U)mask|=2U;
#endif
    if(g_forward_hold.phase!=HR_IDLE || g_forward_hold.request || (g_vm_test_request && !Stage2Body_KeepsPower()) ||
       (power.active && !StartupAxes_Holding()))mask|=4U;
    if(status.active || status.result || status.cut_vm)mask|=8U;
    if(g_translation_hold_selftest!=1U || s_serial_proto_ok!=1U)mask|=16U;
    if(!App_HeadingRuntimeSource(now))mask|=32U;
    if(!App_HeadingRuntimeExclusive())mask|=64U;
    if(g_can_ctrl_fault_latched)mask|=128U;
    if(!(s_heading_source.rate_dps>=-3.0f && s_heading_source.rate_dps<=3.0f))mask|=256U;
    return mask;
#endif
}
int main(void){unsigned n=0;
#define CK(v) do{n++;if(!(v)){printf("power FAIL %d\n",__LINE__);return 1;}}while(0)
 power.active=1;status.rotation_stop_mask=15;g_vm_test_request=1;
 CK(App_SerialIdleBlocked(0)&4);keeps=1;CK(App_SerialIdleBlocked(0)==0);
 source_ok=0;CK(App_SerialIdleBlocked(0)&32);source_ok=1;
 exclusive=0;CK(App_SerialIdleBlocked(0)&64);exclusive=1;
 status.active=1;CK(App_SerialIdleBlocked(0)&8);status.active=0;
 status.result=1;status.rotation_stop_mask=7;CK(!MotionFinish());
 status.rotation_stop_mask=15;CK(MotionFinish() && g_vm_test_request==1 && !s_motion_session_open);
 s_motion_session_open=1;keeps=0;CK(MotionFinish() && !g_vm_test_request);
 s_motion_session_open=1;g_vm_test_request=1;status.cause=8;CK(!MotionFinish() && g_vm_test_request==1);
 printf("source-scoped power admission checks=%u failures=0\n",n);return 0;}
