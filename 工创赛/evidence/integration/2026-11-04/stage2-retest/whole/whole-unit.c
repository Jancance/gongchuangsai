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
static uint32_t g_radar_zone_ready,g_radar_zone_selected,g_vehicle_arm_state,g_vehicle_arm_cause;

static uint32_t g_vehicle_lift_feedback_valid;

static float g_vehicle_lift_current_deg,s_perimeter_targets[9][3];
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];static RouteCheck g_route_check;
static uint32_t g_saoma_accept_count,g_saoma_last_result,g_saoma_frame_start_ms,g_saoma_frame_end_ms;
static uint32_t g_scan_display_result,g_scan_display_count,g_stage1_stop_requested,g_stage1_motion_skipped;
static char SAOMA_Data[16],displayed[16];static uint32_t display_calls,arm_calls,starts,stop_sent;

static RouteHeadingReference s_route_heading_reference;
static float fixture_heading_yaw;
static uint32_t VehicleArm_Busy(void){return 0;}
static void VehicleArm_Cancel(void){g_vehicle_arm_state=3;}
static uint32_t VehicleArm_Begin(const void *r,uint32_t item){(void)r;(void)item;arm_calls++;return 0;}
static uint32_t RadarZoneSwitch_Lock(void){return g_radar_zone_selected;}
static void HeadingSession_Init(uint32_t *p){*p=0;}
static uint32_t App_SerialIdleReady(uint32_t now){(void)now;return !g_combined_test.state && !g_chassis_auto_active && !g_chassis_auto_result && g_safety_ready && !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched;}
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
float g_dm_posvel_speed_deg_s;
static uint32_t model_home_tick,model_extension_timeout,model_ext_enabled=1,model_home_drop_once;
static uint32_t model_home_disable,model_keep_disabled,model_drop_late,model_hold_protect,model_enable_fail,model_position_jump;
static uint32_t model_enable_attempts,model_service_injected;
static uint32_t model_settle_case,model_pre_reads,model_speed_interrupt;
uint32_t StartupPose_GroupValid(void){return 1;}
uint32_t StartupPose_GroupCommand(void){return 1;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 (void)busy;
 if(g_dm_jog_request==3 || !ready){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){
  extern volatile uint32_t g_startup_group_release_ms;
  uint32_t done=test_tick-g_startup_group_release_ms>=300;
  g_dm_jog_state=done?7:4;g_dm_jog_result=done;g_dm_posvel_holding=done;
 }
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_n,void *out){
 (void)length;(void)reply_n;CanRxMsg *rx=out;memset(rx,0,sizeof(*rx));rx->Data[0]=q[0];rx->Data[1]=2;
 if(q[0]==0x1a)rx->Data[2]=0;
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
 if(q[0]==0x36){uint32_t raw=256U+(model_position_jump && model_enable_attempts?100U:0U);rx->Data[1]=1;rx->Data[2]=raw>>24;rx->Data[3]=raw>>16;rx->Data[4]=raw>>8;rx->Data[5]=raw;}
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
#ifdef EXTENSION_HOLD_HANDSHAKE_V1
 g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=g_extension_hold_ready_tick=g_extension_hold_delta_mdeg=0;
 eh_hold_begin=0;eh_hold_position=0;
#endif
}

#include "vehicle_mission.inc"
static uint32_t PlantEndpointAllowed(uint32_t wheel){(void)wheel;return 0;}

/* Adapter model for main.c scheduling/dispatch. Actual mission and actual CAN
 * backend execute together; startup axes, HWT, UART and plant are models. */

static void begin_leg(void){
 ChassisAutoPreparedTranslation p;MotionAction *a=&s_motion_actions[g_combined_test.index];
 ck(StartupAxes_Holding() && g_extension_hold_confirmed && g_extension_hold_samples==3);
 ck(a->reference_mm_s==400 && a->distance_mm==(g_combined_test.index?960:900));
 ck(a->kind==(g_combined_test.index?MOTION_RIGHT_DISTANCE:MOTION_FORWARD_DISTANCE));
 ck(ChassisAuto_PrepareRouteTranslation(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&p));
 ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));starts++;
}
static void dispatch(void){
 if(g_combined_test.request==1 && !g_combined_test.state){
  ck(s_motion_action_count==2);g_combined_test.request=0;g_combined_test.index=g_combined_test.completed=0;
  ck(RouteHeading_Ensure(&s_route_heading_reference,fixture_heading_yaw,1));
  ck(ChassisAuto_BeginSequence());g_vm_test_request=1;ChassisAuto_SafetyUpdate(1);
  begin_leg();g_combined_test.state=1;s_motion_session_open=1;
 }
 if(g_chassis_auto_active){
  for(unsigned i=0;i<4;i++){
   float demand=fixture_vm&&plant_enabled[i]?plant_target[i]:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(.001f/.05f);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77/60)*.001f;
  }
  (void)ChassisAuto_UpdateForwardHold(0);ChassisAuto_SafetyUpdate(g_vm_test_request);ChassisAuto_CanStep();
 }
 if(g_combined_test.state==1 && !g_chassis_auto_active){
  if(g_chassis_auto_result==1){
   ck(s_distance_reached && g_chassis_auto_stop_mask[2]==15);g_combined_test.completed++;
   if(++g_combined_test.index<s_motion_action_count){
    ck(ChassisAuto_ReleaseCompleted(g_chassis_auto_session));begin_leg();
   }else{g_combined_test.state=3;g_vm_test_request=0;s_motion_session_open=0;ChassisAuto_SafetyUpdate(0);}
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
 model_group_reset();g_vehicle_key=1;VehicleMission_Service();
}

int main(void){
 ck(VEHICLE_TEST_STAGE==2 && !VEHICLE_RUN_ALL && VEHICLE_STAGE2_ROUTE_ONLY && !VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE);
 for(unsigned zone=1;zone<=2;zone++)for(unsigned scenario=0;scenario<12;scenario++){
  reset_case(zone);unsigned injected_case=0;
  for(unsigned step=0;step<50000 && g_vehicle_state<4;step++){
   test_tick+=1;
   model_settle_case=scenario==5?17:scenario==6?19:scenario==4?18:0;
   model_enable_fail=scenario==7;
   fixture_vm=(g_startup_axes_vm_request||g_vm_test_request)&&!g_safety_fault&&!g_rtos_fault;
   StartupAxes_Safety(!g_safety_fault&&!g_rtos_fault&&!g_can_ctrl_fault_latched);
   StartupAxes_Service(g_safety_ready,g_can_guard_ready);
   dispatch();
   unsigned moving=g_chassis_auto_active && s_distance_progress_mm>200;
   if(scenario==1 && moving)g_vehicle_key=1;
   if(scenario==2 && moving && g_combined_test.index==1)fixture_can_failure=1;
   if(scenario==3 && moving)model_drop_late=1;
   if(scenario==8 && moving)g_safety_fault=1;
   if(scenario==9 && !injected_case){g_vehicle_key=1;injected_case=1;}
   if(scenario==10 && moving)model_extension_timeout=1;
   if(scenario==11 && moving && !injected_case){test_tick+=VEHICLE_STAGE_TIMEOUT_MS;injected_case=1;}
   VehicleMission_Service();fixture_vm=(g_startup_axes_vm_request||g_vm_test_request)&&!g_safety_fault&&!g_rtos_fault;
  }
  VehicleAlignmentRequest req;
  ck(!arm_calls && !display_calls && !test_depth);
  if(scenario==0 || scenario==5 || scenario==6){
   float dx=(-plant_pos[0]+plant_pos[1]+plant_pos[2]-plant_pos[3])*.25f;
   float dy=(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
   ck(g_vehicle_state==4 && !g_vehicle_cause && g_vehicle_stage_mask==2 && starts==2);
   ck(g_combined_test.completed==2 && !g_chassis_auto_active && fixture_vm);
   /* Existing conventional-model envelope is 10.06 mm per axis (stage2-speed).
    * This assumed plant is not a physical endpoint acceptance test. */
   printf("ENDPOINT_MODEL dx%f dy%f\n",dx,dy);
   ck(fabsf(dx-960)<=10.06f && fabsf(dy-900)<=10.06f);
   ck(VehicleMission_ReadAlignmentRequest(&req) && req.goal_cell==14 && req.nominal_target_x_mm==2160 && req.nominal_target_y_mm==1200 && !req.position_executor_enabled);
   VehicleMission_Service();ck(g_vehicle_state==4 && g_vehicle_stage==2 && !arm_calls);
  }else{
   ck((g_vehicle_state==6 || g_vehicle_state==7) && !fixture_vm && !VehicleMission_ReadAlignmentRequest(&req));
   if(scenario==1 || scenario==9)ck(g_vehicle_state==7 && g_vehicle_cause==2);
   if(scenario==4)ck(!starts && !g_extension_hold_confirmed && g_eh_first_reason==3);
   if(scenario==7)ck(!starts && g_eh_first_reason==4 && g_eh_first_command==0xf3);
  }
  printf("STAGE2 zone%u case%u state%u cause%u legs%u hold%u alignment%u\n",zone,scenario,g_vehicle_state,g_vehicle_cause,starts,g_extension_hold_confirmed,g_vehicle_alignment_state);
  VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();
 }
 printf("STAGE2 checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
