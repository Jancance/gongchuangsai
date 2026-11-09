
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

static float plant_pos[4],plant_velocity[4],plant_target[4],plant_gain=1.0f;
static uint32_t plant_enabled[4],injection,injected;
static uint32_t PlantEndpointAllowed(uint32_t wheel);
uint32_t StartupAxes_Holding(void){return 1;}
void Power_ReadCommandStatus(PowerCommandStatus *p){memset(p,0,sizeof(*p));p->active=1;p->enable_seen=1;p->enable_tick=0;}

uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){hardware_calls++;ck(!test_depth);p->Data[1]=plant_enabled[a-1];return 1;}

uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}

uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){
 (void)p;hardware_calls++;ck(!test_depth);
 float driver=plant_pos[a-1]*(g_chassis_forward_direction[a-1]?-1:1);
 *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1;
}


uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t a,CanRxMsg *p,float *rpm){
 (void)p;hardware_calls++;ck(!test_depth);
 float logical=roundf(plant_velocity[a-1]);*rpm=g_chassis_forward_direction[a-1]?-logical:logical;
 if(injection && a==1 && ((injected && injection==2) || (!injected && PlantEndpointAllowed(a-1)))){
  *rpm=(s_direction_guard[a-1].target_deci>0?-1.0f:1.0f)*(injection==3?3.0f:2.0f);injected++;
 }
 return 1;
}


uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t v,CanRxMsg *p){
 (void)p;hardware_calls++;ck(!test_depth);plant_enabled[a-1]=v;if(!v)plant_target[a-1]=0;return 1;
}


uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t r,uint8_t ac,CanRxMsg *p){
 (void)p;hardware_calls++;ck(!test_depth && r<=1040 && ac==236);
 plant_target[a-1]=(d==g_chassis_forward_direction[a-1]?1.0f:-1.0f)*(float)r*0.1f;return 1;
}


uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){(void)p;hardware_calls++;ck(!test_depth);plant_target[a-1]=0;return 1;}


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

static uint32_t PlantEndpointAllowed(uint32_t wheel){
 return s_forward_hold && s_distance_mode && s_route_trial && s_profile_active &&
  !s_finish_requested && !s_stop_due && !s_distance_reached && s_translation_base_rpm<=2 &&
  s_distance_progress_mm>=s_distance_target_mm-10 && s_distance_progress_mm<=s_distance_target_mm &&
  s_distance_reference.acceleration_mm_s2<=0 && s_direction_guard[wheel].target_deci &&
  abs(s_direction_guard[wheel].target_deci)<=20 && test_tick-s_direction_guard[wheel].direction_ms>=500;
}
static void stage(uint32_t direction,float distance,float speed){
 ChassisAutoPreparedTranslation p;MotionProfile profile;uint32_t cap,ms,steps;
 ck(MotionRoute_Build(direction,distance,speed,&profile,&cap,&ms));
 ck(ChassisAuto_PrepareRouteTranslation(direction,cap,ms,distance,speed,&p));
 ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
 for(steps=0;steps<4500 && g_chassis_auto_active;steps++){
  test_tick+=10;
  for(unsigned i=0;i<4;i++){
   float demand=plant_enabled[i]?plant_target[i]*plant_gain:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(0.01f/0.03f);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77.0f/60)*0.01f;
  }
  (void)ChassisAuto_UpdateForwardHold(0.0f);
  ChassisAuto_SafetyUpdate(1);ChassisAuto_CanStep();ck(!test_depth);
 }
 printf("STAGE,%u,%.0f,%.0f,%u,%u,%u,%u,%.3f,%u\n",direction,distance,speed,steps*10,g_chassis_auto_result,g_chassis_auto_cause,g_chassis_auto_fault_diagnostic.reason,s_distance_progress_mm,injected);
 ck(steps<4500);
 if(injection>=2){ck(g_chassis_auto_result==2 && g_chassis_auto_cut_vm && !s_distance_reached);}
 else ck(g_chassis_auto_result==1 && s_distance_reached && g_chassis_auto_stop_mask[2]==15);
}
int main(void){
 for(unsigned scenario=0;scenario<=3;scenario++){
  injection=scenario;plant_gain=scenario?0.97f:1.0f;
  reset();injected=0;memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));
  memset(plant_target,0,sizeof(plant_target));memset(plant_enabled,0,sizeof(plant_enabled));
  ck(App_MotionAccept());stage(0,150,200);
  if(injection)ck(injected>0);
  if(injection<=1){
   ck(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));
   injection=0;plant_gain=1.0f;stage(1,1050,400); /* no second physical test, model only */
   
  }
 }
 printf("DYNAMIC checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
