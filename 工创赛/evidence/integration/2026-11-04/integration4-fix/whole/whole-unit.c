int g_dm_startup_raw_mdeg=0,g_dm_startup_work_mdeg=0;
#define VEHICLE_PARAMETER_TABLES

#include "chassis_auto_test.c"
#include <stdio.h>
#include <string.h>
uint32_t test_tick=1000,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t hardware_calls,checks,failures,fixture_vm,fixture_can_failure;

static void ck_at(uint32_t v,uint32_t line){checks++;if(!v){failures++;printf("failed line %u check %u\n",line,checks);}}
#define ck(v) ck_at((v),__LINE__)

static float plant_pos[4],plant_velocity[4],plant_target[4],plant_gain=1.0f;
static uint32_t plant_enabled[4],injection,injected;
static uint32_t PlantEndpointAllowed(uint32_t wheel);

void Power_ReadCommandStatus(PowerCommandStatus *p){memset(p,0,sizeof(*p));p->active=fixture_vm;p->enable_seen=1;p->enable_tick=0;}

uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){hardware_calls++;ck(!test_depth);p->Data[1]=plant_enabled[a-1];return 1;}

uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}

uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){
 (void)p;hardware_calls++;ck(!test_depth);
 float driver=plant_pos[a-1]*(g_chassis_forward_direction[a-1]?-1:1);
 *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1;
}


uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t a,CanRxMsg *p,float *rpm){
 (void)p;hardware_calls++;ck(!test_depth);if(fixture_can_failure)return 6;
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
 (void)p;hardware_calls++;ck(!test_depth && r<=VEHICLE_STAGE1_BLEND_MAX_RPM*10U && ac==236);
 plant_target[a-1]=(d==g_chassis_forward_direction[a-1]?1.0f:-1.0f)*(float)r*0.1f;return 1;
}


uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){(void)p;hardware_calls++;ck(!test_depth);plant_target[a-1]=0;return 1;}



#include "route_plan.h"
#include "vehicle_mission.h"
#include "field_map.h"
#include "route_heading_reference.h"
#include "motion_control.h"
#define DEBUG_PLAN_DEFAULT 0U
#undef MOTION_ACTION_COUNT
static uint32_t s_motion_action_count;
#define MOTION_ACTION_COUNT (s_motion_action_count)
#define HR_IDLE 0U
#define HS_IDLE 0U
#define SCREEN_OK 1U
typedef struct{uint32_t state,request,index,cause,completed;} CombinedTestDebug;
typedef struct{uint32_t phase,request,trip;} ForwardHoldDebug;
static CombinedTestDebug g_combined_test;static ForwardHoldDebug g_forward_hold;
static uint32_t s_motion_session_open,s_motion_has_action,s_combined_turn,s_combined_powered_segment,s_combined_keep_vm;
static uint32_t g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request,g_heading_exec_trip,s_heading_exec;
static uint32_t s_motion_timing_count,s_motion_timing_cursor,g_debug_plan_id,g_motion_plan_valid;
static uint32_t s_serial_start_permit,g_serial_motion_state,g_serial_motion_result,g_stage2_ready,g_vm_test_request;
static uint32_t g_radar_zone_ready,g_radar_zone_selected;



static float s_perimeter_targets[9][3];
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];static RouteCheck g_route_check;
static uint32_t g_saoma_accept_count,g_saoma_last_result,g_saoma_frame_start_ms,g_saoma_frame_end_ms;
static uint32_t g_scan_display_result,g_scan_display_count,g_stage1_stop_requested,g_stage1_motion_skipped;
static char SAOMA_Data[16],displayed[16];static uint32_t display_calls,arm_calls,starts,stop_sent;

static RouteHeadingReference s_route_heading_reference;
static float fixture_heading_yaw;
static uint32_t RadarZoneSwitch_Lock(void){return g_radar_zone_selected;}
static void HeadingSession_Init(uint32_t *p){*p=0;}

#define HS_SOURCE_MS 100U
typedef struct {uint32_t healthy,sample_ms,valid_mask,angle_ms,rate_ms;float rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready=1,g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
static uint32_t g_can_jog_request,s_can_jog_preparing,g_can_feedback_enable,g_can_poll_enable=1,g_can_test_result=1;
static uint32_t g_motion_diag_hold_report,g_motion_diag_hold_stop_service,g_motion_diag_fail_stop_once;
static uint32_t g_translation_hold_selftest=1,s_serial_proto_ok=1;
static uint32_t model_gate_started,model_gate_until,model_gate_injected;
uint32_t CAN1_EmmSettingsConfirmed(void){return 1;}
static uint32_t App_CanJogOtherRequest(void){return 0;}
#ifndef MOTION_STOP_IDLE
#define MOTION_STOP_IDLE 0U
#endif
static uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit)
{return (s->valid_mask&bit)?now-((bit==1)?s->angle_ms:s->rate_ms):0xffffffffU;}
static uint32_t App_HeadingRuntimeExclusive(void)
{
    return ((g_chassis_auto_request == 0U) &&
        (g_can_jog_request == 0U) && (s_can_jog_preparing == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U) &&
        (App_CanJogOtherRequest() == 0U) && (g_can_feedback_enable == 0U) &&
        (g_can_poll_enable == 1U) && (g_can_test_result == 1U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_motion_diag_hold_report == 0U) &&
        (g_motion_diag_hold_stop_service == 0U) &&
        (g_motion_diag_fail_stop_once == 0U)) ? 1U : 0U;
}
static uint32_t App_HeadingRuntimeSource(uint32_t now)
{
    return ((s_heading_source_ready == 1U) && (s_heading_source.healthy == 1U) &&
        ((uint32_t)(now - s_heading_source.sample_ms) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,1U)) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,2U)) < HS_SOURCE_MS) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
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
    if(g_forward_hold.phase!=HR_IDLE || g_forward_hold.request || g_vm_test_request ||
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
static uint32_t App_SerialIdleReady(uint32_t now)
{return App_SerialIdleBlocked(now)==0U;}

void MotionStop(void){ChassisAuto_Abort();g_combined_test.state=5;g_vm_test_request=0;s_motion_session_open=0;}
static uint32_t Screen_ShowTaskCode(const char *s){display_calls++;strcpy(displayed,s);return 1;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){(void)now;(void)ms;return 1;}
static uint16_t App_SerialCrc(const char *s,uint32_t n){uint16_t c=0xffff;for(uint32_t i=0;i<n;i++){c^=(uint16_t)(uint8_t)s[i]<<8;for(uint32_t j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}return c;}
static uint32_t model_last_tx_tick;
static uint32_t App_DebugSendString(const char *s){if(model_last_tx_tick==test_tick)return 0;model_last_tx_tick=test_tick;printf("%s",s);return 1;}
static uint32_t App_MotionPlanValid(void){
 for(unsigned i=0;i<s_motion_action_count;i++)if(!MotionRoute_ShapeAllowed(s_motion_actions[i].kind-MOTION_FORWARD_DISTANCE,s_motion_actions[i].base_rpm,s_motion_actions[i].run_ms,s_motion_actions[i].distance_mm,s_motion_actions[i].reference_mm_s))return 0;
 return s_motion_action_count>0 && s_motion_action_count<=8;
}

#define DM_CAN2_POSVEL_SPEED_DEG_S VEHICLE_DM_SPEED_DEG_S
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state,g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;

float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg;
uint32_t g_dm_can2_read_cycle,g_dm_can2_feedback_tick;
uint32_t g_dm_jog_stop_result;
volatile uint32_t g_dm_integrated_owner,g_dm_source_smooth,g_dm_source_go;
volatile float g_dm_source_accel_deg_s2;
static uint32_t model_source_dm,model_source_dm_started,model_source_fault,model_source_queued;
static float model_source_position=-25.6f,model_source_target=-25.6f;

static uint32_t model_home_tick,model_extension_timeout,model_ext_enabled=1,model_home_drop_once;
static uint32_t model_rebound;
static uint32_t model_home_disable,model_keep_disabled,model_drop_late,model_hold_protect,model_enable_fail,model_position_jump;
static uint32_t model_enable_attempts,model_service_injected;
static uint32_t model_settle_case,model_pre_reads,model_speed_interrupt;
uint32_t StartupPose_GroupValid(void){return 1;}
uint32_t StartupPose_GroupCommand(void){return 1;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 (void)busy;
 if(g_dm_jog_request==3 || !ready){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}

 if(g_dm_jog_request==4 || g_dm_jog_request==5){g_dm_jog_request=0;model_source_queued=1;g_dm_jog_state=8;g_dm_jog_result=2;g_dm_jog_stop_result=1;g_dm_posvel_holding=0;return;}
 if(model_source_queued){model_source_queued=0;model_source_dm=1;model_source_dm_started=0;g_dm_jog_state=3;g_dm_can2_motor_state=1;g_dm_posvel_holding=0;g_dm_can2_read_cycle++;}
 if(model_source_dm){
  g_dm_can2_read_cycle++;g_dm_can2_feedback_tick=test_tick;
  if(model_source_fault==1){g_dm_jog_first_fault=7;return;}
  if(!g_dm_source_go)return;
  if(!model_source_dm_started)model_source_dm_started=test_tick;
  unsigned done=test_tick-model_source_dm_started>=4000;
  g_dm_jog_state=done?7:4;g_dm_jog_result=done;g_dm_posvel_holding=done;
  g_dm_can2_logical_position_deg=done?180:180.0f*(test_tick-model_source_dm_started)/4000;return;
 }
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){
  extern volatile uint32_t g_startup_group_release_ms;
  uint32_t done=test_tick-g_startup_group_release_ms>=300;
  g_dm_jog_state=done?7:4;g_dm_jog_result=done;g_dm_posvel_holding=done;
 }
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_n,void *out){
 (void)length;(void)reply_n;CanRxMsg *rx=out;memset(rx,0,sizeof(*rx));rx->Data[0]=q[0];rx->Data[1]=2;
 if(q[0]==0x1a)rx->Data[2]=0x80;
 if(q[0]==0x3a){
  if(model_extension_timeout && g_chassis_auto_active){test_tick+=30;return 5;}
  if(model_drop_late)model_ext_enabled=0;
  rx->Data[1]=2U|(model_ext_enabled?1U:0U);
  if(model_hold_protect && model_home_tick && test_tick-model_home_tick>=100)rx->Data[1]|=8U;
 }
 if(q[0]==0x3b){
  extern volatile uint32_t g_extension_home_done_notice;
  rx->Data[1]=model_home_tick && test_tick-model_home_tick<100?7:3;
  if(model_home_tick && test_tick-model_home_tick>=100){
   g_extension_home_done_notice=1;
   if(model_home_disable && !model_home_drop_once){model_ext_enabled=0;model_home_drop_once=1;}
  }
 }
 if(q[0]==0x35){
  extern volatile uint32_t g_extension_home_state,g_extension_hold_samples;
  uint32_t rpm=0;
  if(g_extension_home_state==9 && !model_enable_attempts){
   model_pre_reads++;
   if(model_settle_case==18 || ((model_settle_case==17 || model_settle_case==22) && model_pre_reads<=2))rpm=model_settle_case==22?256:123;
  }
  if(g_extension_home_state==9 && model_enable_attempts){
   if(model_settle_case==20)rpm=300;
   if(model_settle_case==19 && g_extension_hold_samples==1 && !model_speed_interrupt){rpm=1;model_speed_interrupt=1;}
   if(model_settle_case==21)return 5;
  }
  rx->Data[1]=1;rx->Data[2]=rpm>>8;rx->Data[3]=rpm&255;
 }
 if(q[0]==0x36){uint32_t raw=model_rebound?(model_enable_attempts?30U:2270U):2560U+(model_position_jump && model_enable_attempts?100U:0U);rx->Data[1]=1;rx->Data[2]=raw>>24;rx->Data[3]=raw>>16;rx->Data[4]=raw>>8;rx->Data[5]=raw;}
 if(q[0]==0x9a)model_home_tick=test_tick;
 if(q[0]==0xf3 && q[2]){model_enable_attempts++;if(model_enable_fail)return 5;if(!model_keep_disabled)model_ext_enabled=1;}
 return 1;
}
uint32_t StartupAxis_ReadHomeParameters(uint8_t p[17]){
 memset(p,0,17);p[0]=0x22;p[16]=0x6b;p[2]=VEHICLE_EXTENSION_HOME_DIRECTION;
 p[3]=VEHICLE_EXTENSION_HOME_RPM>>8;p[4]=VEHICLE_EXTENSION_HOME_RPM;
 p[5]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>24;p[6]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>16;
 p[7]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS>>8;p[8]=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS&255;
 p[9]=VEHICLE_EXTENSION_HOME_DETECT_RPM>>8;p[10]=VEHICLE_EXTENSION_HOME_DETECT_RPM&255;
 p[11]=VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA>>8;p[12]=VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA&255;
 p[13]=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS>>8;p[14]=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS;
 return 1;
}

#define DM_CAN2_H
uint32_t DM_CAN2_SourceTransitionPending(void){return model_source_queued && g_dm_jog_state==8 && g_dm_jog_result==2 && g_dm_jog_stop_result==1 && g_dm_source_smooth && !g_dm_source_go;}
static float model_source_grip=145;
uint8_t jiazhua(float deg){if(model_source_fault==3)return 0;model_source_grip=deg;return deg>=145 && deg<=180;}
uint32_t CarrierPwm_Write(uint32_t pulse){(void)pulse;return 1;}
#include "vehicle_arm.c"
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t length,uint8_t rn,void *out){
 CanRxMsg *p=out;uint32_t raw;memset(p,0,sizeof(*p));p->Data[0]=q[0];p->Data[1]=2;
 if(address!=6 || model_source_fault==2)return 5;
 if(q[0]==0x1a)p->Data[2]=0x80;
 if(q[0]==0x33){raw=(uint32_t)(fabsf(model_source_target)*100+.5f);p->Data[1]=model_source_target<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;}
 if(q[0]==0x36){raw=(uint32_t)(fabsf(model_source_position)*100+.5f);p->Data[1]=model_source_position<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;}
 if(q[0]==0x35){p->Data[1]=0;p->Data[3]=fabsf(model_source_position-model_source_target)>.1f;}
 if(q[0]==0x3a)p->Data[1]=model_source_fault==4?9:(fabsf(model_source_position-model_source_target)<.1f?3:1);
 if(q[0]==0xfd){
  raw=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  ck(length==15 && raw<=18000 && q[12]==2);
  model_source_target=model_source_position+(q[1]?-1:1)*(float)raw/100;
 }
 if(q[0]==0xfe)model_source_target=model_source_position;
 (void)rn;return 1;
}

#include "startup_group.inc"
static void model_group_reset(void){
 g_startup_axes_vm_request=g_group_extension_state=g_group_extension_cause=g_group_extension_vm_request=0;
 g_extension_home_state=g_extension_home_result=g_extension_home_reported=g_extension_home_flags=0;
 g_extension_home_capture=g_extension_home_done_notice=g_extension_home_elapsed_ms=0;
 eh_begin=eh_motion=eh_last=eh_poll=eh_started=eh_seen=eh_stop_pending=eh_phase=0;
 sg_begin=sg_last=sg_dm_started=0;g_startup_group_go=g_startup_group_release_ms=g_startup_group_pwm_result=0;
 g_dm_startup_go=g_dm_jog_first_fault=g_dm_point_enable=g_dm_jog_request=g_dm_jog_state=0;
 g_dm_can2_motor_state=g_dm_jog_result=g_dm_posvel_holding=0;
 g_eh_first_reason=g_eh_first_command=g_eh_first_exchange=g_eh_first_flags=g_eh_first_tick=g_eh_first_service_gap=g_eh_first_phase=0;
 eh_last_command=eh_last_exchange=eh_last_flags=model_home_tick=model_extension_timeout=0;
 model_ext_enabled=1;model_home_drop_once=model_home_disable=model_keep_disabled=model_drop_late=model_hold_protect=0;
 model_enable_fail=model_position_jump=model_enable_attempts=model_service_injected=0;
 model_settle_case=model_pre_reads=model_speed_interrupt=0;
 memset((void *)&g_source_prepare_fault,0,sizeof(g_source_prepare_fault));
 model_source_dm=model_source_dm_started=model_source_fault=model_source_queued=g_dm_jog_stop_result=0;model_source_position=model_source_target=-25.6f;model_source_grip=145;
 g_dm_integrated_owner=g_dm_source_smooth=g_dm_source_go=g_dm_can2_read_cycle=g_dm_can2_feedback_tick=0;
 g_dm_can2_logical_position_deg=0;
 g_source_prepare_state=g_source_prepare_cause=g_source_prepare_done_mask=g_source_prepare_release_ms=g_source_prepare_elapsed_ms=g_source_prepare_segment=0;
 g_vehicle_arm_state=g_vehicle_arm_cause=g_vehicle_arm_axis_owned=g_vehicle_arm_dm_owned=g_vehicle_arm_stop_pending=0;

#ifdef EXTENSION_HOLD_HANDSHAKE_V1
 g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=g_extension_hold_ready_tick=g_extension_hold_delta_mdeg=0;
 eh_hold_begin=0;eh_hold_position=0;
#endif
}

void SAOMA_EnableReceive(void){}
void SAOMA_DisableReceive(void){}
void HWT101_Update(void){}
void App_HeadingSourcePublish(void){}
#include "vehicle_mission.inc"
static uint32_t PlantEndpointAllowed(uint32_t wheel){(void)wheel;return 0;}

/* Adapter model for main.c scheduling/dispatch. Actual mission and actual CAN
 * backend execute together; startup axes, HWT, UART and plant are models. */


static void begin_leg(void){
 ChassisAutoPreparedTranslation p;MotionAction *a=&s_motion_actions[g_combined_test.index];
 ck(StartupAxes_Holding() && g_extension_hold_confirmed && g_extension_hold_samples==3);
 if(g_vehicle_stage==1 && g_vehicle_scan_search_state!=1U && g_vehicle_scan_search_state!=2U){
  ck(ChassisAuto_PrepareRouteBlend(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&p));
  ck(ChassisAuto_BeginPreparedBlend(&p,g_chassis_auto_session));
 }else{
  if(g_vehicle_stage==2){
   
   ck(a->reference_mm_s==400 && s_motion_action_count==1 && g_combined_test.index==0);
   float dx=s_vehicle_stage2_leg_target.x_mm-s_vehicle_handoff.x_mm;
   float dy=s_vehicle_stage2_leg_target.y_mm-s_vehicle_handoff.y_mm;
   if(g_vehicle_route_phase==2)ck(a->kind==(dy>0?MOTION_FORWARD_DISTANCE:MOTION_BACKWARD_DISTANCE) && fabsf(a->distance_mm-fabsf(dy))<.01f);
   else ck(a->kind==(dx>0?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE) && fabsf(a->distance_mm-fabsf(dx))<.01f);

  }
  ck(ChassisAuto_PrepareRouteTranslation(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&p));
  ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
 }
 starts++;
}
static void dispatch(void){
 if(g_combined_test.request==1 && !g_combined_test.state){
  g_combined_test.request=0;g_combined_test.index=g_combined_test.completed=0;
  ck(RouteHeading_Ensure(&s_route_heading_reference,fixture_heading_yaw,1));
  ck(ChassisAuto_BeginSequence());g_vm_test_request=1;ChassisAuto_SafetyUpdate(1);
  begin_leg();g_combined_test.state=1;s_motion_session_open=1;
 }
 if(g_chassis_auto_active){
  if(g_vehicle_stage==1 && g_stage1_stop_requested && g_chassis_auto_state>=CHASSIS_AUTO_START && !stop_sent){ck(ChassisAuto_RequestStop());stop_sent=1;}
  for(unsigned i=0;i<4;i++){
   float demand=fixture_vm&&plant_enabled[i]?plant_target[i]:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(.001f/.05f);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77/60)*.001f;
  }
  (void)ChassisAuto_UpdateForwardHold(0);ChassisAuto_SafetyUpdate(g_vm_test_request);ChassisAuto_CanStep();
 }
 if(g_combined_test.state==1 && !g_chassis_auto_active){
  if(g_chassis_auto_result==1){
   ck(g_chassis_auto_stop_mask[2]==15);if(g_vehicle_stage==2)ck(s_distance_reached);
   if(s_distance_reached)g_combined_test.completed++;
   if(++g_combined_test.index<s_motion_action_count){ck(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));begin_leg();}
   else{g_combined_test.state=3;g_vm_test_request=0;s_motion_session_open=0;ChassisAuto_SafetyUpdate(0);}
  }else{g_combined_test.state=4;g_combined_test.cause=8;g_vm_test_request=0;s_motion_session_open=0;}
 }
}
static void reset_case(unsigned zone){
 test_tick=1000;test_depth=test_primask=fixture_vm=fixture_can_failure=0;
 hardware_calls=display_calls=arm_calls=starts=stop_sent=0;
 memset(&s_route_heading_reference,0,sizeof(s_route_heading_reference));fixture_heading_yaw=.25f;
 memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));memset(plant_target,0,sizeof(plant_target));memset(plant_enabled,0,sizeof(plant_enabled));
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=g_motion_diag_fault_latched=0;
 g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 g_safety_ready=g_can_guard_ready=g_radar_zone_ready=1;g_radar_zone_selected=zone;
 ChassisAuto_Init();memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));
 s_motion_session_open=s_motion_has_action=g_vm_test_request=0;s_motion_action_count=1;
 g_vehicle_state=g_vehicle_cause=g_vehicle_key=g_vehicle_run_id=g_startup_axes_state=g_startup_axes_cause=0;
 g_vehicle_arm_state=g_vehicle_arm_cause=g_heading_exec_trip=0;g_motion_plan_valid=1;
 g_saoma_accept_count=g_saoma_last_result=g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;SAOMA_Data[0]=0;
 model_group_reset();g_can_feedback_enable=0;s_heading_source_ready=1;
 s_heading_source=(AppHeadingSource){1,test_tick,3,test_tick,test_tick,0};g_vehicle_key=1;VehicleMission_Service();
}



static void dynamic_geometry_checks(void){
 RouteRow rows[8];FieldPoint goal;uint32_t n;uint32_t original_calls=hardware_calls;
 ck(VEHICLE_STAGE1_FORWARD_DISTANCE_CM==15 && FIELD_SCAN_Y_MM==300);
 s_vehicle_handoff_valid=1;g_vehicle_route_phase=1;s_vehicle_handoff=(FieldPoint){1320,390};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && n==1 && rows[0].kind==MOTION_LEFT_DISTANCE && fabsf(rows[0].amount-12)<.001f);
 ck(s_vehicle_stage2_leg_target.x_mm==1200 && s_vehicle_stage2_leg_target.y_mm==390);
 /* 横移后的Y读回改变，下一条距离必须随之改变，不能沿用首轮计划。 */
 g_vehicle_route_phase=2;s_vehicle_handoff=(FieldPoint){1200,395};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_FORWARD_DISTANCE && fabsf(rows[0].amount-80.5f)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,390};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-81)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,300};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-90)<.001f);
 s_vehicle_handoff=(FieldPoint){1200,240};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && fabsf(rows[0].amount-96)<.001f);
 g_vehicle_route_phase=3;s_vehicle_handoff=(FieldPoint){1205,1203};
 ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_RIGHT_DISTANCE && fabsf(rows[0].amount-85.5f)<.001f && goal.x_mm==2060 && goal.y_mm==1200);
 s_vehicle_handoff=(FieldPoint){1200,1230};ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 s_vehicle_handoff=(FieldPoint){2060,1200};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==2 && !n);
 g_vehicle_route_phase=1;s_vehicle_handoff=(FieldPoint){1208,390};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==2);
 s_vehicle_handoff=(FieldPoint){1080,390};ck(VehicleMission_Stage2Rows(rows,&n,&goal)==1 && rows[0].kind==MOTION_RIGHT_DISTANCE && fabsf(rows[0].amount-12)<.001f);
 s_vehicle_handoff=(FieldPoint){1450,300};ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 s_vehicle_handoff_valid=0;ck(!VehicleMission_Stage2Rows(rows,&n,&goal));
 ck(hardware_calls==original_calls);
}

int main(void){
 dynamic_geometry_checks();
 ck(VEHICLE_CHAIN_STAGE12 && VEHICLE_TEST_STAGE==1 && VEHICLE_TEST_START_ZONE==1 && !VEHICLE_RUN_ALL && VEHICLE_STAGE2_ROUTE_ONLY);
 ck(VEHICLE_STAGE2_GOAL_OFFSET_X_MM==-100 && !VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE);
 for(unsigned inputzone=1;inputzone<=2;inputzone++)for(unsigned scenario=0;scenario<47;scenario++){
  reset_case(inputzone);model_rebound=scenario>=42;if(model_rebound)model_source_position=model_source_target=-.3f;unsigned mailbox_injected=0;unsigned sent=0,seen1=0,seen2=0,corrupted=0,rerun=0,second_stage_seen=0;
  uint32_t test_max_queue_wait=0,test_first_queue_mask=0,test_stage2_corrupted=0;
  stop_sent=0;model_gate_started=model_gate_until=model_gate_injected=0;
  g_can_feedback_enable=0;s_heading_source_ready=1;
  s_heading_source=(AppHeadingSource){1,test_tick,3,test_tick,test_tick,0};
  for(unsigned step=0;step<120000 && g_vehicle_state<4;step++){
   test_tick+=1;
   s_heading_source.healthy=1;s_heading_source.valid_mask=3;s_heading_source.rate_dps=0;
   s_heading_source.sample_ms=s_heading_source.angle_ms=s_heading_source.rate_ms=test_tick;

   model_settle_case=scenario==15?18:scenario==16?17:scenario==17?19:0;model_enable_fail=scenario==14;
   fixture_vm=(g_startup_axes_vm_request||g_vm_test_request);
   StartupAxes_Safety(!g_safety_fault&&!g_rtos_fault&&!g_can_ctrl_fault_latched);

   StartupAxes_Service(g_safety_ready,g_can_guard_ready);
   if(scenario==43 && g_source_prepare_state==2 && g_source_prepare_segment==2)model_source_fault=4;
   if(model_source_fault!=5){
    float delta=model_source_target-model_source_position;
    float step=fminf(fabsf(delta),.15f);model_source_position+=delta<0?-step:step;
   }
   if(StartupAxes_Holding())DM_CAN2_ModuleService(1,0);
   if(VehicleArm_Busy() || g_vehicle_arm_stop_pending)VehicleArm_Service();
   if(StartupAxes_Holding())DM_CAN2_ModuleService(1,0);
   if(scenario>=44 && !mailbox_injected && g_chassis_auto_active && g_chassis_auto_state==CHASSIS_AUTO_RUN){
    mailbox_injected=1;uint32_t saved_tick=s_heading_command_tick;
    s_heading_command_tick=test_tick-(scenario==44?101U:scenario==45?499U:500U);
    Auto_CheckLocked(test_tick);
    if(scenario==46){ChassisAutoFaultDiagnostic d;ChassisAuto_ReadFaultDiagnostic(&d);ck(!g_chassis_auto_active && g_chassis_auto_cause==3 && d.reason==1);}
    else ck(g_chassis_auto_active);
    s_heading_command_tick=saved_tick;
   }
   if(!VehicleArm_Busy())dispatch();

   if(g_vehicle_scan_search_state==1)seen1=1;if(g_vehicle_scan_search_state==2)seen2=1;
   unsigned stopped=g_combined_test.state==3 && g_vehicle_stage==1;
   unsigned moving=g_chassis_auto_active && s_distance_progress_mm>50;
   if(!sent && g_vehicle_stage==1 && (
      ((scenario==0 || scenario>=7 && scenario!=28 && scenario<40) && stopped && !g_vehicle_scan_search_state) ||
      ((scenario==1 || scenario==28) && g_vehicle_scan_search_state==1 && moving) ||
      (scenario==2 && g_vehicle_scan_search_state==2 && moving) ||
      (scenario==3 && !g_vehicle_scan_search_state && g_chassis_auto_active && s_distance_progress_mm>900) ||
      (scenario==4 && !g_vehicle_scan_search_state && g_chassis_auto_active && s_distance_progress_mm>200) ||
      (scenario==6 && stopped && !g_vehicle_scan_search_state))){
    strcpy(SAOMA_Data,scenario==6?"216+231+354+123":"156+123+516+231");
    g_saoma_accept_count++;g_saoma_last_result=1;g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;sent=1;
   }

   if(!sent && ((scenario==40 && g_vehicle_state==1) ||
       ((scenario==41 || scenario>=42) && g_vehicle_stage==1 && g_chassis_auto_active && s_distance_progress_mm>400))){
    strcpy(SAOMA_Data,"156+123+516+231");g_saoma_accept_count++;g_saoma_last_result=1;
    g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;sent=1;
   }
   if(scenario==41 && s_vehicle_code_valid){
    strcpy(SAOMA_Data,"216+231+612+123");g_saoma_accept_count++;
    g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;
   }
   if(s_vehicle_code_valid && g_vehicle_stage==1){
    ck(!g_stage1_stop_requested && !g_stage1_motion_skipped && !stop_sent);
    ck(!strcmp(s_vehicle_task_code,"156+123+516+231"));
   }
   if(g_vehicle_stage==2){second_stage_seen=1;ck(s_vehicle_handoff_valid);}
   if((scenario==7 || scenario==18) && g_vehicle_stage==1 && moving){if(scenario==7)g_vehicle_key=1;else g_safety_fault=1;}
   if(scenario==8 && g_vehicle_stage==2 && moving)fixture_can_failure=1;
   if(scenario==9 && g_vehicle_stage==2 && moving)model_extension_timeout=1;
   if((scenario==10 || scenario==11 || scenario==12) && stopped && !corrupted){
    if(scenario==10)s_endpoints[0].end_mask=7;
    if(scenario==11)s_endpoints[0].session++;
    if(scenario==12)for(unsigned i=0;i<4;i++)s_endpoints[0].end_ms[i]=test_tick-1001;
    corrupted=1;
   }
   if(scenario==13 && g_vehicle_stage==2 && moving)g_vehicle_key=1;
   if(scenario>=37 && g_vehicle_stage==2 && g_combined_test.state==3 && !test_stage2_corrupted){
    if(scenario==37)for(unsigned i=0;i<4;i++)s_endpoints[s_endpoint_index].end_ms[i]=test_tick-1001;
    if(scenario==38)s_endpoints[s_endpoint_index].end_mask=7;
    if(scenario==39)s_endpoints[s_endpoint_index].session++;
    test_stage2_corrupted=1;
   }

   if(scenario>=30 && g_source_prepare_state==2){
    if(scenario<=34)model_source_fault=scenario-29;
    if(scenario==35)g_vehicle_key=1;
    if(scenario==36)g_safety_fault=1;
   }
   if(g_source_prepare_state==1 || g_source_prepare_state==2){
    ck(g_vehicle_stage==2 && g_vehicle_state==3 && !g_vehicle_alignment_state && !g_chassis_auto_active && !g_vm_test_request);
   }


   if(scenario>=20 && scenario<30 && stopped && !model_gate_injected && (scenario!=28 || (sent && s_vehicle_search_used))){
    model_gate_started=test_tick;model_gate_injected=1;
    model_gate_until=test_tick+((scenario==20 || scenario==28)?10:scenario==21?700:scenario==23?100:scenario==27?999:2000);
   }
   if(model_gate_injected && test_tick<model_gate_until){
    if(scenario==23)s_heading_source.rate_dps=4;
    else if(scenario==24)g_can_feedback_enable=1;
    else{
     uint32_t age=scenario==29?99:100;
     s_heading_source.angle_ms=test_tick-(age-1);s_heading_source.rate_ms=test_tick-age;
     s_heading_source.sample_ms=test_tick-94;
    }
   }
#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
   if(g_vehicle_queue_state==1 && scenario==25)g_safety_fault=1;
   if(g_vehicle_queue_state==1 && scenario==26)g_vehicle_key=1;
#endif
   VehicleMission_Service();fixture_vm=(g_startup_axes_vm_request||g_vm_test_request);
   if(g_combined_test.request)stop_sent=0;
   if(g_vehicle_queue_wait_ms>test_max_queue_wait)test_max_queue_wait=g_vehicle_queue_wait_ms;
   if(g_vehicle_queue_first_mask)test_first_queue_mask=g_vehicle_queue_first_mask;

#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
   if(g_vehicle_queue_state==1)ck(!g_combined_test.request && !g_vm_test_request && !g_chassis_auto_active && fixture_vm);
#endif

   if(scenario==19 && g_vehicle_state==4 && !rerun){
    rerun=1;g_vehicle_key=1;VehicleMission_Service();
   }
  }
  if(scenario>=37 && scenario<40)ck(test_stage2_corrupted && g_vehicle_cause==10 && !g_source_prepare_release_ms && fixture_vm);

  if(scenario==0 || scenario==3 || scenario==4 || scenario==40 || scenario==41)ck(!seen1 && !seen2);
  if(scenario==1)ck(seen1 && !seen2);
  if(scenario==2)ck(seen1 && seen2);
  VehicleAlignmentRequest req;ck(!arm_calls && !test_depth && s_vehicle_zone==1);
  if(g_vehicle_state==4)ck(g_vehicle_route_phase==4 && g_source_prepare_state==3 && g_source_prepare_done_mask==7 && fabsf(g_source_prepare_extension_mm-70)<.2f && fabsf(model_source_grip-180)<.1f);
  if(scenario>=30 && scenario<37)ck(g_source_prepare_release_ms && g_source_prepare_state==4 && g_vehicle_alignment_state==3 && fixture_vm);

  if(scenario==0 || scenario==1 || scenario==2 || scenario==3 || scenario==16 || scenario==17 || scenario==19 || scenario==20 || scenario==21 || scenario==23 || scenario==27 || scenario==28 || scenario==29 || scenario==4 || scenario==40 || scenario==41 || scenario==42 || scenario==44 || scenario==45){
   float wx=2250+(-plant_pos[0]+plant_pos[1]+plant_pos[2]-plant_pos[3])*.25f;
   float wy=150+(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
   ck(g_vehicle_stage==2 && g_vehicle_state==4 && (g_vehicle_cause==(scenario==19?19U:0U)) && g_vehicle_stage_mask==3 && second_stage_seen);
   ck(display_calls==1 && !strcmp(displayed,"156+123+516+231") && fixture_vm && !g_chassis_auto_active);
   ck(VehicleMission_ReadAlignmentRequest(&req) && req.goal_cell==14 && req.start_id==1 && req.nominal_target_x_mm==2060 && req.nominal_target_y_mm==1200 && !req.position_executor_enabled);
   ck(fabsf(wx-2060)<=20.06f && fabsf(wy-1200)<=10.06f);
   printf("CHAIN_MODEL_ENDPOINT case%u x%.3f y%.3f handoff%.3f,%.3f\n",scenario,wx,wy,s_vehicle_handoff.x_mm,s_vehicle_handoff.y_mm);
   if(scenario==1)ck(seen1 && !seen2);if(scenario==2)ck(seen1 && seen2);if(scenario==19)ck(g_vehicle_run_id==1 && g_vehicle_cause==19);
  }else{
   ck((g_vehicle_state==6 || g_vehicle_state==7) && fixture_vm && !VehicleMission_ReadAlignmentRequest(&req));

   if(scenario==5 || scenario==6)ck(!second_stage_seen && g_vehicle_cause==16 && !display_calls);
   if(scenario==10 || scenario==11 || scenario==12)ck(!second_stage_seen && g_vehicle_cause==10);
  }

#ifdef VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS
  if(scenario==20 || scenario==21 || scenario==27 || scenario==28)ck(g_vehicle_state==4 && test_max_queue_wait>0 && test_max_queue_wait<1000);
  if(scenario==22)ck(g_vehicle_state==6 && g_vehicle_cause==7 && g_vehicle_queue_state==3 && g_vehicle_queue_first_mask==32 && g_vehicle_queue_wait_ms>=1000 && !second_stage_seen);
  if(scenario==23)ck(g_vehicle_state==4 && test_first_queue_mask==256 && test_max_queue_wait>=100);
  if(scenario==24)ck(g_vehicle_state==6 && g_vehicle_cause==7 && g_vehicle_queue_first_mask==64 && !second_stage_seen);
  if(scenario==25)ck(g_vehicle_state==6 && g_vehicle_cause==4 && !second_stage_seen);
  if(scenario==26)ck(g_vehicle_state==7 && g_vehicle_cause==2 && !second_stage_seen);
  if(scenario==29)ck(g_vehicle_state==4 && !g_vehicle_queue_first_mask && !g_vehicle_queue_wait_ms);
#endif
  if(scenario==42 || scenario==43){ck(g_extension_home_reference_valid && fabsf(g_extension_home_reference_deg+.3f)<.01f);if(scenario==42)ck(g_source_prepare_done_mask==7);else ck(g_source_prepare_cause==24 && g_source_prepare_segment==2);}
  printf("CHAIN input%u case%u state%u cause%u stage%u sessions%u searches%u/%u mask%u\n",inputzone,scenario,g_vehicle_state,g_vehicle_cause,g_vehicle_stage,starts,seen1,seen2,g_vehicle_stage_mask);
  VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();
 }
 printf("CHAIN checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
