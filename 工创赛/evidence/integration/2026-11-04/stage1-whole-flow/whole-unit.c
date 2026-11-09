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
volatile uint32_t g_startup_axes_state;
static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed line check %u\n",checks);}}

static float plant_pos[4],plant_velocity[4],plant_target[4],plant_gain=1.0f;
static uint32_t plant_enabled[4],injection,injected;
static uint32_t PlantEndpointAllowed(uint32_t wheel);
uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4 && fixture_vm;}
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
volatile uint32_t g_startup_axes_cause,g_startup_extension_feedback_valid;
static uint32_t g_vehicle_lift_feedback_valid;
volatile float g_startup_extension_current_deg;
static float g_vehicle_lift_current_deg,s_perimeter_targets[9][3];
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];static RouteCheck g_route_check;
static uint32_t g_saoma_accept_count,g_saoma_last_result,g_saoma_frame_start_ms,g_saoma_frame_end_ms;
static uint32_t g_scan_display_result,g_scan_display_count,g_stage1_stop_requested,g_stage1_motion_skipped;
static char SAOMA_Data[16],displayed[16];static uint32_t display_calls,arm_calls,starts,stop_sent;
static uint32_t startup_begin_tick;
uint32_t StartupAxes_Begin(void){g_startup_axes_state=1;fixture_vm=1;startup_begin_tick=test_tick;return 1;}
void StartupAxes_Cancel(void){g_startup_axes_state=6;fixture_vm=0;}
static uint32_t VehicleArm_Busy(void){return 0;}
static void VehicleArm_Cancel(void){g_vehicle_arm_state=3;}
static uint32_t VehicleArm_Begin(const void *r,uint32_t item){(void)r;(void)item;arm_calls++;return 0;}
static uint32_t RadarZoneSwitch_Lock(void){return g_radar_zone_selected;}
static void HeadingSession_Init(uint32_t *p){*p=0;}
static uint32_t App_SerialIdleReady(uint32_t now){(void)now;return !g_combined_test.state && !g_chassis_auto_active && !g_chassis_auto_result && g_safety_ready && !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched;}
void MotionStop(void){ChassisAuto_Abort();g_combined_test.state=5;g_vm_test_request=0;s_motion_session_open=0;}
static uint32_t Screen_ShowTaskCode(const char *s){display_calls++;strcpy(displayed,s);return 1;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){(void)now;(void)ms;return 0;}
static uint16_t App_SerialCrc(const char *s,uint32_t n){(void)s;(void)n;return 0;}
static uint32_t App_DebugSendString(const char *s){(void)s;return 1;}
static uint32_t App_MotionPlanValid(void){
 for(unsigned i=0;i<s_motion_action_count;i++)if(!MotionRoute_ShapeAllowed(s_motion_actions[i].kind-MOTION_FORWARD_DISTANCE,s_motion_actions[i].base_rpm,s_motion_actions[i].run_ms,s_motion_actions[i].distance_mm,s_motion_actions[i].reference_mm_s))return 0;
 return s_motion_action_count>0 && s_motion_action_count<=8;
}
#include "vehicle_mission.inc"
static uint32_t PlantEndpointAllowed(uint32_t wheel){(void)wheel;return 0;}

/* Adapter model for main.c scheduling/dispatch. Actual mission and actual CAN
 * backend execute together; startup axes, HWT, UART and plant are models. */
static void dispatch(void){
 if(g_combined_test.request==1 && !g_combined_test.state){
  ChassisAutoPreparedTranslation prepared;MotionAction *a=&s_motion_actions[0];
  uint32_t toward_other=g_radar_zone_selected==1?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE;
  uint32_t toward_own=g_radar_zone_selected==1?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE;
  ck(a->reference_mm_s==450);
  if(!g_vehicle_scan_search_state)ck(a->kind==toward_other && a->distance_mm==1050);
  else if(g_vehicle_scan_search_state==1)ck(a->kind==toward_other && a->distance_mm==150);
  else ck(g_vehicle_scan_search_state==2 && a->kind==toward_own && a->distance_mm==300);
  g_combined_test.request=0;stop_sent=0;
  if(g_stage1_stop_requested){g_stage1_motion_skipped=1;g_combined_test.state=3;g_combined_test.completed=0;return;}
  ck(StartupAxes_Holding());ck(ChassisAuto_BeginSequence());g_vm_test_request=1;
  ChassisAuto_SafetyUpdate(1);
  if(!g_vehicle_scan_search_state)ck(ChassisAuto_PrepareRouteBlend(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&prepared));
  else ck(ChassisAuto_PrepareRouteTranslation(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&prepared));
  if(!g_vehicle_scan_search_state)ck(ChassisAuto_BeginPreparedBlend(&prepared,g_chassis_auto_session));
  else ck(ChassisAuto_BeginPreparedTranslation(&prepared,g_chassis_auto_session));
  s_motion_session_open=s_motion_has_action=1;g_combined_test.state=1;starts++;
 }
 if(g_chassis_auto_active){
  if(g_stage1_stop_requested && !stop_sent && g_chassis_auto_state>=CHASSIS_AUTO_START){ck(ChassisAuto_RequestStop());stop_sent=1;}
  for(unsigned i=0;i<4;i++){
   float demand=fixture_vm&&plant_enabled[i]?plant_target[i]:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(.01f/.05f);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77/60)*.01f;
  }
  (void)ChassisAuto_UpdateForwardHold(0);ChassisAuto_SafetyUpdate(g_vm_test_request);ChassisAuto_CanStep();
 }
 if(g_combined_test.state==1 && !g_chassis_auto_active){
  if(g_chassis_auto_result==1){
   g_combined_test.state=3;g_combined_test.completed=s_distance_reached?1:0;
  }else{g_combined_test.state=4;g_combined_test.cause=8;}
  s_motion_session_open=0;g_vm_test_request=0;
 }
}
static void reset_case(unsigned zone){
 test_tick=1000;test_depth=test_primask=fixture_vm=fixture_can_failure=0;
 hardware_calls=display_calls=arm_calls=starts=stop_sent=0;
 memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));memset(plant_target,0,sizeof(plant_target));memset(plant_enabled,0,sizeof(plant_enabled));
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=g_motion_diag_fault_latched=0;
 g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 g_safety_ready=g_can_guard_ready=g_radar_zone_ready=1;g_radar_zone_selected=zone;
 ChassisAuto_Init();memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));
 s_motion_session_open=s_motion_has_action=g_vm_test_request=0;s_motion_action_count=1;
 g_vehicle_state=g_vehicle_cause=g_vehicle_key=g_vehicle_run_id=g_startup_axes_state=g_startup_axes_cause=0;
 g_vehicle_arm_state=g_vehicle_arm_cause=g_heading_exec_trip=0;g_motion_plan_valid=1;
 g_saoma_accept_count=g_saoma_last_result=g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;SAOMA_Data[0]=0;
 g_vehicle_key=1;VehicleMission_Service();
}
int main(void){
 FILE *trace=fopen("whole-flow.csv","w"),*summary=fopen("whole-summary.csv","w");
 fprintf(trace,"zone,scenario,time_ms,mission_state,search_state,backend_state,blend_lateral_mm,blend_forward_mm,vm_model,display_count,world_x_encoder_model_mm,world_y_encoder_model_mm,leg_target_mm,leg_progress_mm\n");
 fprintf(summary,"zone,scenario,state,cause,search_state,route_sessions,display_count,elapsed_ms,world_x_encoder_model_mm,world_y_encoder_model_mm\n");
 for(unsigned zone=1;zone<=2;zone++)for(unsigned scenario=0;scenario<7;scenario++){
  unsigned sent=0,seen_search1=0,seen_search2=0;reset_case(zone);
  for(unsigned step=0;step<5000 && g_vehicle_state<4;step++){
   test_tick+=10;
   if(g_startup_axes_state==1 && test_tick-startup_begin_tick>=500)g_startup_axes_state=4;
   dispatch();
   if(g_vehicle_scan_search_state==1)seen_search1=1;
   if(g_vehicle_scan_search_state==2)seen_search2=1;
   unsigned nominal=(g_combined_test.state==3 && !g_vehicle_scan_search_state);
   unsigned searching=(g_chassis_auto_active && s_distance_progress_mm>=50);
   if(!sent && ((scenario==0 && nominal) || (scenario==1 && g_vehicle_scan_search_state==1 && searching) ||
       (scenario==2 && g_vehicle_scan_search_state==2 && searching) || (scenario==4 && nominal))){
    strcpy(SAOMA_Data,scenario==4?"216+231+354+123":"156+123+516+231");
    g_saoma_accept_count++;g_saoma_last_result=1;g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;sent=1;
   }
   if(scenario==5 && g_stage1_lateral_mm>200 && g_chassis_auto_active)g_vehicle_key=1;
   if(scenario==6 && g_vehicle_scan_search_state==1 && searching)fixture_can_failure=1;
   VehicleMission_Service();
   float wx=(zone==1?2250.0f:150.0f)+(-plant_pos[0]+plant_pos[1]+plant_pos[2]-plant_pos[3])*.25f;
   float wy=150+(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
   if(!(step%5))fprintf(trace,"%u,%u,%u,%u,%u,%u,%.6f,%.6f,%u,%u,%.6f,%.6f,%.6f,%.6f\n",zone,scenario,test_tick-1000,g_vehicle_state,g_vehicle_scan_search_state,g_chassis_auto_state,g_stage1_lateral_mm,g_stage1_forward_mm,fixture_vm,display_calls,wx,wy,s_distance_target_mm,s_distance_progress_mm);
  }
  ck(!arm_calls && !test_depth && !g_vehicle_correct_pick_count && !g_vehicle_correct_place_count);
  if(scenario<=2){ck(g_vehicle_state==4 && !g_vehicle_cause && display_calls==1 && !strcmp(displayed,"156+123+516+231"));ck(g_chassis_auto_stop_mask[2]==15 && !g_chassis_auto_active && fixture_vm);}
  if(scenario==0)ck(starts==1 && !seen_search1 && !seen_search2);
  if(scenario==1)ck(starts==2 && seen_search1 && !seen_search2);
  if(scenario==2)ck(starts==3 && seen_search1 && seen_search2);
  if(scenario==3 || scenario==4)ck(g_vehicle_state==6 && g_vehicle_cause==16 && starts==3 && !display_calls && !fixture_vm);
  if(scenario==5)ck(g_vehicle_state==7 && g_vehicle_cause==2 && !fixture_vm);
  if(scenario==6)ck(g_vehicle_state==6 && g_vehicle_cause==8 && !display_calls && !fixture_vm);
  float wx=(zone==1?2250.0f:150.0f)+(-plant_pos[0]+plant_pos[1]+plant_pos[2]-plant_pos[3])*.25f;
  float wy=150+(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
  if(scenario==0)ck(fabsf(wx-1200)<=5 && fabsf(wy-300)<=5);
  if(scenario==3 || scenario==4)ck(fabsf(wx-(zone==1?1350:1050))<=5 && fabsf(wy-300)<=5);
  if(scenario<=4)ck(wx>=1045 && wx<=1355);
  fprintf(summary,"%u,%u,%u,%u,%u,%u,%u,%u,%.6f,%.6f\n",zone,scenario,g_vehicle_state,g_vehicle_cause,g_vehicle_scan_search_state,starts,display_calls,test_tick-1000,wx,wy);
  printf("WHOLE zone%u case%u state%u cause%u searches%u sessions%u display%u\n",zone,scenario,g_vehicle_state,g_vehicle_cause,g_vehicle_scan_search_state,starts,display_calls);
 }
 fclose(trace);fclose(summary);printf("WHOLE checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
