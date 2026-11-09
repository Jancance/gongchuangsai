
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "field_map.h"
#include "app_entry.h"
#include "route_plan.h"
extern volatile RouteCheck g_route_check;
volatile uint32_t checks,failures,failure_checks[16];
float exported_centers[25][2],exported_parks[25][2],exported_plan[7],exported_actions[2][4];
uint32_t exported_config,exported_run_ms,exported_rpm;
char accepted_frames[1024];uint32_t accepted_used;
static void ck(uint32_t good){checks++;if(!good){if(failures<16)failure_checks[failures]=checks;failures++;}}
typedef struct {uint32_t trip;} ForwardHoldDebug;
typedef struct {uint32_t request,state,index,cause,completed;} CombinedTestDebug;
typedef struct {uint32_t active,result,cause,rotation_stop_mask,session;} ChassisAutoStatus;
typedef struct {uint32_t active;} PowerCommandStatus;
typedef struct {uint32_t valid;} RouteHeadingReference;
static uint32_t tick,ready,rearm_ok,stop_calls,tx_ok;
volatile CombinedTestDebug g_combined_test;
volatile ForwardHoldDebug g_forward_hold;
static ChassisAutoStatus fixture_backend;static PowerCommandStatus fixture_power;
uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request,s_heading_exec;
uint32_t s_motion_session_open,s_motion_has_action,s_combined_turn,s_combined_powered_segment,s_combined_keep_vm;
uint32_t s_serial_start_permit,g_serial_motion_state,g_serial_motion_result,g_serial_motion_accepted,g_serial_motion_link_lost;
uint32_t s_route_heading_turn_active,s_perimeter_odom_started,s_perimeter_completed_seen,g_perimeter_records[9];
float s_perimeter_targets[9][3];RouteHeadingReference s_route_heading_reference;
uint32_t g_radar_zone_ready,g_radar_zone_selected,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
uint32_t g_vm_test_request,g_saoma_accept_count,g_scan_display_count,g_saoma_last_result,g_scan_display_result;
uint32_t g_saoma_frame_start_ms,g_saoma_frame_end_ms,g_chassis_auto_session;
char SAOMA_Data[16];
#define MOTION_ACTION_COUNT 2U
#define HR_IDLE 0U
#define HS_IDLE 0U
#define SCREEN_OK 1U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *p){*p=fixture_backend;return 1;}
static void Power_ReadCommandStatus(PowerCommandStatus *p){*p=fixture_power;}
static uint32_t ChassisAuto_RearmCompleted(uint32_t session){if(!rearm_ok)return 0;fixture_backend.result=0;return 1;}
static void HeadingSession_Init(void *p){memset(p,0,4);}
static uint32_t App_SerialIdleReady(uint32_t now){return !APP_MOTOR_OFFLINE_ONLY && ready && !g_combined_test.state;}
static void MotionStop(void){stop_calls++;g_combined_test.state=5;g_vm_test_request=0;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t ms){return 1;}
static uint16_t App_SerialCrc(const char *p,uint32_t n){uint16_t c=0xffff;uint32_t i,j;for(i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;for(j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}return c;}
static uint32_t App_DebugSendString(const char *p){uint32_t n=strlen(p);if(tx_ok && accepted_used+n<sizeof(accepted_frames)){memcpy(accepted_frames+accepted_used,p,n);accepted_used+=n;accepted_frames[accepted_used]=0;}return tx_ok;}
#include "stage1_runtime.inc"
static void reset_fixture(void){
 memset((void*)&g_combined_test,0,sizeof(g_combined_test));memset((void*)&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&fixture_backend,0,sizeof(fixture_backend));memset(&fixture_power,0,sizeof(fixture_power));
 g_stage1_state=g_stage1_cause=g_stage1_key_request=g_stage1_denied=g_stage1_scan_valid=g_stage1_run_id=0;
 g_heading_exec_trip=g_serial_motion_link_lost=g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 s_motion_session_open=g_vm_test_request=stop_calls=0;g_radar_zone_ready=1;g_radar_zone_selected=1;
 g_saoma_accept_count=g_scan_display_count=10;g_saoma_last_result=1;g_scan_display_result=SCREEN_OK;
 g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;ready=rearm_ok=tx_ok=1;tick=10;
 strcpy(SAOMA_Data,"156+123+516+231");
}
static void start(void){g_stage1_key_request=1;App_Stage1Service();}
static void arrive(void){g_combined_test.request=0;g_combined_test.state=3;g_combined_test.completed=2;g_combined_test.index=1;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;tick=100;App_Stage1Service();}
void test_entry(void){
 uint32_t i,row,col,count;FieldPoint p,q;FieldStage1Plan plan;MotionAction action[8];
 for(i=0;i<25;i++){
  row=i/5;col=i%5;if(row&1)col=4-col;
  ck(FieldMap_CellCenter(i,&p) && p.x_mm==240+col*480 && p.y_mm==240+row*480);
  exported_centers[i][0]=p.x_mm;exported_centers[i][1]=p.y_mm;
  ck(FieldMap_Park(i,&q));exported_parks[i][0]=q.x_mm;exported_parks[i][1]=q.y_mm;
 }
 ck(!FieldMap_CellCenter(25,&p));ck(!FieldMap_CellCenter(1,0));
 ck(FieldMap_Park(4,&p) && p.x_mm==2250 && p.y_mm==150);
 ck(FieldMap_Park(0,&p) && p.x_mm==150 && p.y_mm==150);
 ck(FieldMap_Stage1(&plan) && plan.left_cm==105 && plan.speed_cm_s==10);
 ck(plan.stop.x_mm==1200 && plan.stop.y_mm==300 && plan.forward_cm==15 && plan.nominal_scan_gap_mm==158.5f);
 exported_plan[0]=plan.start.x_mm;exported_plan[1]=plan.start.y_mm;
 exported_plan[2]=plan.stop.x_mm;exported_plan[3]=plan.stop.y_mm;exported_plan[4]=plan.forward_cm;exported_plan[5]=plan.left_cm;exported_plan[6]=plan.nominal_scan_gap_mm;
 exported_config=plan.config_id;
 p=(FieldPoint){240,2160};q=(FieldPoint){1200,720};ck(!FieldMap_AxisClear(p,q,150));
 p=(FieldPoint){720,240};q=(FieldPoint){720,2160};ck(!FieldMap_AxisClear(p,q,150));
 p=(FieldPoint){100,2250};q=(FieldPoint){100,1200};ck(!FieldMap_AxisClear(p,q,150));
 ck(FieldMap_TaskCodeValid("156+123+516+231"));ck(!FieldMap_TaskCodeValid("156+123+516+236"));
 ck(!FieldMap_TaskCodeValid("116+123+116+231"));ck(!FieldMap_TaskCodeValid("156+123+123+231"));
 ck(!FieldMap_TaskCodeValid("156+123+516+231x"));ck(!FieldMap_TaskCodeValid("156"));ck(!FieldMap_TaskCodeValid(0));
 ck(AppDebugPlan_Load(0,action,&count) && count==2);
 ck(action[0].kind==MOTION_FORWARD_DISTANCE && action[0].distance_mm==150 && action[0].reference_mm_s==50);
 ck(action[1].kind==MOTION_LEFT_DISTANCE && action[1].distance_mm==1050 && action[1].reference_mm_s==100);
 ck(action[0].run_ms>4000 && g_route_check.budget_ms<=VEHICLE_STAGE1_MOVE_TIMEOUT_MS && g_route_check.error==0);
 for(i=0;i<2;i++){exported_actions[i][0]=(float)action[i].kind;exported_actions[i][1]=action[i].distance_mm;exported_actions[i][2]=action[i].reference_mm_s;exported_actions[i][3]=(float)action[i].run_ms;}
 exported_run_ms=action[0].run_ms;exported_rpm=action[0].base_rpm;
 reset_fixture();ready=0;start();ck(g_stage1_state==0 && g_stage1_denied==1 && !g_combined_test.request);
#if APP_MOTOR_OFFLINE_ONLY
 ready=1;start();ck(!g_stage1_run_id && !g_combined_test.request);return;
#else
 ready=1;g_radar_zone_selected=2;start();ck(g_stage1_state==0 && !g_combined_test.request);
 g_radar_zone_selected=1;start();ck(g_stage1_state==1 && g_stage1_run_id==1 && g_combined_test.request==1 && s_serial_start_permit==1);
 ck(s_perimeter_targets[1][0]==150 && s_perimeter_targets[1][1]==0);
 ck(s_perimeter_targets[2][0]==150 && s_perimeter_targets[2][1]==1050);
 fixture_backend.active=1;arrive();ck(g_stage1_state==1);
 fixture_backend.active=0;fixture_backend.rotation_stop_mask=14;App_Stage1Service();ck(g_stage1_state==1);
 fixture_backend.rotation_stop_mask=15;g_vm_test_request=1;App_Stage1Service();ck(g_stage1_state==1);
 g_vm_test_request=0;App_Stage1Service();ck(g_stage1_state==2 && !g_stage1_scan_valid);
 g_saoma_accept_count++;g_scan_display_count++;tick=120;
 g_saoma_frame_start_ms=90;g_saoma_frame_end_ms=115;App_Stage1Service();ck(g_stage1_state==2);
 tick=700;g_saoma_frame_start_ms=110;g_saoma_frame_end_ms=120;App_Stage1Service();ck(g_stage1_state==2);
 g_saoma_frame_start_ms=640;g_saoma_frame_end_ms=650;g_scan_display_result=0;App_Stage1Service();ck(g_stage1_state==2);
 g_scan_display_result=SCREEN_OK;App_Stage1Service();ck(g_stage1_state==3 && g_stage1_scan_valid && !strcmp(g_stage1_task_code,SAOMA_Data));
 rearm_ok=0;start();ck(g_stage1_state==3 && g_combined_test.state==3);
 rearm_ok=1;ready=0;start();ck(g_stage1_state==0 && g_combined_test.state==0);
 ready=1;start();ck(g_stage1_state==1 && g_stage1_run_id==2);
 g_stage1_key_request=2;App_Stage1Service();ck(g_stage1_state==5 && stop_calls==1);
 start();ck(g_stage1_state==5 && g_stage1_run_id==2);
 reset_fixture();start();g_combined_test.state=4;App_Stage1Service();ck(g_stage1_state==4 && g_stage1_cause==2);
 start();ck(g_stage1_state==4 && g_stage1_run_id==1);
 reset_fixture();start();tick=60010;App_Stage1Service();ck(g_stage1_state==4 && g_stage1_cause==6 && stop_calls==1);
 reset_fixture();start();arrive();tick=8100;App_Stage1Service();ck(g_stage1_state==4 && g_stage1_cause==3 && !g_stage1_scan_valid);
 reset_fixture();start();arrive();tick=150;g_saoma_accept_count++;g_scan_display_count++;
 g_saoma_frame_start_ms=110;g_saoma_frame_end_ms=120;strcpy(SAOMA_Data,"156+123+516+236");App_Stage1Service();ck(g_stage1_state==2);
 strcpy(SAOMA_Data,"156+123+516+231");App_Stage1Service();ck(g_stage1_state==3);
 tick=1000;tx_ok=0;App_Stage1Telemetry();ck(!accepted_used);
 tick=1200;tx_ok=1;App_Stage1Telemetry();App_Stage1CodeTelemetry();ck(accepted_used>0);
#endif
}

int main(void){test_entry();printf("checks=%u failures=%u\n",(unsigned)checks,(unsigned)failures);return failures?1:0;}
