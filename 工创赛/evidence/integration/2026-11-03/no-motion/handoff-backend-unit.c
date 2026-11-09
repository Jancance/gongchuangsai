
#include "chassis_auto_test.c"
#include <stdio.h>
#include <string.h>
uint32_t test_tick=1000,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t hardware_calls,checks,failures;
static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed line check %u\n",checks);}}
uint32_t StartupAxes_Holding(void){return 1;}
void Power_ReadCommandStatus(PowerCommandStatus *p){memset(p,0,sizeof(*p));p->active=1;p->enable_seen=1;p->enable_tick=0;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){(void)a;hardware_calls++;p->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){(void)a;(void)p;hardware_calls++;*sign=0;*raw=0;return 1;}
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t v,CanRxMsg *p){(void)a;(void)v;(void)p;hardware_calls++;return 1;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t r,uint8_t ac,CanRxMsg *p){(void)a;(void)d;(void)r;(void)ac;(void)p;hardware_calls++;return 1;}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){(void)a;(void)p;hardware_calls++;return 1;}

#include "motion_control.h"
#define AppCompetitionEntry_MotionAllowed() 0U
static struct {uint32_t state;} g_combined_test;
static uint32_t g_vm_test_request,s_motion_list_submit,s_motion_has_action,s_motion_session_open;
static uint32_t g_translation_hold_selftest=1,s_combined_turn,s_combined_begin_ms,s_combined_powered_segment,s_combined_keep_vm,s_combined_stage_ms;
static uint32_t s_motion_timing_count,s_motion_timing_cursor;
uint32_t MotionGetState(void){return MOTION_IDLE;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return 1;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static uint32_t App_MotionAccept_Before(void)
{
#if APP_FINE_RPM_OFFLINE_ONLY
    return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t state = MotionGetState();
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed()) return 0U;
    if ((state != MOTION_IDLE && state != MOTION_DONE) ||
        (g_combined_test.state >= 3U) ||
        ((g_combined_test.state != 0U) && !s_motion_list_submit) ||
        !App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount()) ||
        !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
        (g_translation_hold_selftest != 1U)) return 0U;
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (!s_motion_session_open)
    {
        if (s_motion_has_action || g_vm_test_request || (power.active && !StartupAxes_Holding()) ||
            !ChassisAuto_BeginSequence()) return 0U;
#if APP_SERIAL_MOTION_CONTROL
        s_motion_timing_count = s_motion_timing_cursor = 0U;
#endif
        s_motion_session_open = 1U;
        if (!s_motion_list_submit) s_combined_begin_ms = (uint32_t)xTaskGetTickCount();
        /* 在同一临界区发布首段VM请求，避免Safety把新组当作断电重置。 */
        g_vm_test_request = 1U;
        s_combined_powered_segment = 1U;
    }
    else
    {
        if (g_vm_test_request != 1U || power.active != 1U ||
            !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)) ||
            !ChassisAuto_ReleaseCompleted(status.session)) return 0U;
        s_combined_powered_segment = 1U;
    }
    s_combined_keep_vm = 1U;
    s_combined_stage_ms = (uint32_t)xTaskGetTickCount();
    return 1U;
#endif
}
static uint32_t App_MotionAccept(void)
{
#if APP_FINE_RPM_OFFLINE_ONLY
    return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t state = MotionGetState();
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed()) return 0U;
    if ((state != MOTION_IDLE && state != MOTION_DONE) ||
        (g_combined_test.state >= 3U) ||
        ((g_combined_test.state != 0U) && !s_motion_list_submit) ||
        !App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount()) ||
        !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
        (g_translation_hold_selftest != 1U)) return 0U;
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (!s_motion_session_open)
    {
        if (s_motion_has_action || g_vm_test_request || (power.active && !StartupAxes_Holding()) ||
            !ChassisAuto_BeginSequence()) return 0U;
#if APP_SERIAL_MOTION_CONTROL
        s_motion_timing_count = s_motion_timing_cursor = 0U;
#endif
        s_motion_session_open = 1U;
        if (!s_motion_list_submit) s_combined_begin_ms = (uint32_t)xTaskGetTickCount();
        /* 在同一临界区发布首段VM请求，避免Safety把新组当作断电重置。 */
        g_vm_test_request = 1U;
        /* 联合机构已持VM时不再等待上电；同批发布后端请求，避免先提交后被旧0拒绝。
         * 仅同步本次已通过准入的请求，不写健康标志，不清故障，不在此发CAN。 */
        ChassisAuto_SafetyUpdate(g_vm_test_request);
        s_combined_powered_segment = 1U;
    }
    else
    {
        if (g_vm_test_request != 1U || power.active != 1U ||
            !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)) ||
            !ChassisAuto_ReleaseCompleted(status.session)) return 0U;
        s_combined_powered_segment = 1U;
    }
    s_combined_keep_vm = 1U;
    s_combined_stage_ms = (uint32_t)xTaskGetTickCount();
    return 1U;
#endif
}
static void reset(void){
 test_tick=1000;test_depth=test_primask=0;hardware_calls=0;g_vm_test_request=s_motion_list_submit=s_motion_has_action=s_motion_session_open=g_combined_test.state=0;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=g_motion_diag_fault_latched=0;
 g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 g_safety_ready=g_can_guard_ready=1;ChassisAuto_Init();
}
static uint32_t prepare(ChassisAutoPreparedTranslation *p){
 MotionProfile profile;uint32_t rpm,ms;
 if(!MotionRoute_Build(0,150,200,&profile,&rpm,&ms))return 0;
 return ChassisAuto_PrepareRouteTranslation(0,rpm,ms,150,200,p);
}
int main(void){
 ChassisAutoPreparedTranslation p;uint32_t before,after;
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ck(!s_vm_requested);
 before=ChassisAuto_BeginPreparedTranslation(&p,0);
 ck(!before && g_chassis_auto_result==9 && g_chassis_auto_session==0 && !g_chassis_auto_active && !hardware_calls);
 printf("REPRO result=%u backend_result=%u session=%u internal_vm=%u\n",before,g_chassis_auto_result,g_chassis_auto_session,s_vm_requested);
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));
 /* 与应用修复一致：提交时同步当前请求，不伪造平台健康或清锁存。 */
 ChassisAuto_SafetyUpdate(1);after=ChassisAuto_BeginPreparedTranslation(&p,0);
 ck(after && g_chassis_auto_session==1 && g_chassis_auto_active && !hardware_calls);
 printf("ORDERED result=%u backend_result=%u session=%u internal_vm=%u\n",after,g_chassis_auto_result,g_chassis_auto_session,s_vm_requested);
 for(unsigned fault=0;fault<7;fault++){
  reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));
  if(fault==0)g_rtos_fault=1;if(fault==1)g_safety_fault=1;if(fault==2)g_can_guard_ready=0;
  if(fault==3)g_motion_diag_fault_latched=1;if(fault==4)g_motion_auto_stop_cut_vm=1;
  if(fault==5)g_wheel_test_active=1;if(fault==6)g_motion_auto_stop_state=1;
  ChassisAuto_SafetyUpdate(1);ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && !g_chassis_auto_active && !hardware_calls);
 }
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ChassisAuto_SafetyUpdate(0);
 ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && !g_chassis_auto_active && !hardware_calls);
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ChassisAuto_SafetyUpdate(1);
 ck(!ChassisAuto_BeginPreparedTranslation(&p,1) && !g_chassis_auto_active && !hardware_calls);

 reset();test_depth=1;ck(App_MotionAccept_Before());test_depth=0;
 ck(g_vm_test_request==1 && s_vm_requested==0);ck(prepare(&p));ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && g_chassis_auto_result==9);
 reset();test_depth=1;ck(App_MotionAccept());ck(test_depth==1);test_depth=0;
 ck(g_vm_test_request==1 && s_vm_requested==1);ck(prepare(&p));
 test_depth=1;ck(ChassisAuto_BeginPreparedTranslation(&p,0));ck(test_depth==1);test_depth=0;
 ck(g_chassis_auto_active && g_chassis_auto_session==1 && !hardware_calls);
 printf("HANDOFF checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
