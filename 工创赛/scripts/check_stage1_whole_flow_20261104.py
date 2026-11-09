"""One executable: actual mission+CAN backend, modeled startup/scheduler/devices.

This is integrated software simulation, not an on-board run of every driver.
No actual firmware source is changed and no hardware tool is opened.
"""
from pathlib import Path
import subprocess,json,hashlib,os
R=Path(__file__).resolve().parents[1];A=Path(os.environ.get('STAGE1_SOURCE_ROOT','D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程'))
E=Path(os.environ.get('STAGE1_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/stage1-whole-flow-current')));E.mkdir(parents=True,exist_ok=True)
base=(R/'evidence/integration/2026-11-04/stage1-blend-search/backend-unit.c').read_text(encoding='utf-8')
base=base[:base.index('#include "motion_control.h"')]
base=base.replace('static uint32_t hardware_calls,checks,failures;',
                  'static uint32_t hardware_calls,checks,failures,fixture_vm,fixture_can_failure;\nvolatile uint32_t g_startup_axes_state;')
base=base.replace('uint32_t StartupAxes_Holding(void){return 1;}',
                  'uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4 && fixture_vm;}')
base=base.replace('p->active=1;', 'p->active=fixture_vm;')
base=base.replace(' (void)p;hardware_calls++;ck(!test_depth);\n float logical=',
                  ' (void)p;hardware_calls++;ck(!test_depth);if(fixture_can_failure)return 6;\n float logical=')
unit=base+r'''
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
volatile uint32_t g_startup_axes_cause,g_startup_extension_feedback_valid;
static uint32_t g_vehicle_lift_feedback_valid;
volatile float g_startup_extension_current_deg;
static float g_vehicle_lift_current_deg,s_perimeter_targets[9][3];
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];static RouteCheck g_route_check;
static uint32_t g_saoma_accept_count,g_saoma_last_result,g_saoma_frame_start_ms,g_saoma_frame_end_ms;
static uint32_t g_scan_display_result,g_scan_display_count,g_stage1_stop_requested,g_stage1_motion_skipped;
static char SAOMA_Data[16],displayed[16];static uint32_t display_calls,arm_calls,starts,stop_sent;
static uint32_t startup_begin_tick;
static RouteHeadingReference s_route_heading_reference;
static float fixture_heading_yaw;
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
#ifdef ROUTE_HEADING_RUN_REFERENCE_V2
  uint32_t heading_ready=RouteHeading_Ensure(&s_route_heading_reference,fixture_heading_yaw,VEHICLE_INTEGRATED_ENABLE?1U:0U);
#else
  uint32_t heading_ready=RouteHeading_Init(&s_route_heading_reference,fixture_heading_yaw);
#endif
  if(!heading_ready){printf("HEADING_REFUSED valid%u search%u\n",s_route_heading_reference.valid,g_vehicle_scan_search_state);g_combined_test.state=4;g_combined_test.cause=3;g_vm_test_request=0;return;}
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
'''
real_group=os.environ.get('STAGE1_REAL_GROUP','1')=='1'
if real_group:
    from stage1_real_group_fixture import adapt
    unit=adapt(unit)
p=E/'whole-unit.c';p.write_text(unit,encoding='utf-8');x=E/'whole-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(R/'evidence/integration/2026-11-03/no-motion/backend-stubs'),'-I'+str(A/'CHASSIS/USER'),str(p)]
cmd += [str(A/'CHASSIS/USER'/n) for n in ['field_map.c','chassis_kinematics.c','motion_profile.c','heading_controller.c']]
cmd+=['-lm','-o',str(x)];f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(x)],cwd=E,capture_output=True,text=True);print(f.stdout);(E/'whole-flow.log').write_text(f.stdout+f.stderr,encoding='utf-8')
files=['USER/vehicle_parameters.h','CHASSIS/USER/vehicle_mission.inc','CHASSIS/USER/chassis_auto_test.c','CHASSIS/USER/chassis_auto_test.h','CHASSIS/USER/main.c','CHASSIS/USER/stage1_blend.h','CHASSIS/USER/route_heading_reference.h']
if real_group:files+=['CHASSIS/USER/startup_group.inc','CHASSIS/USER/extension_home_only.inc','CHASSIS/USER/startup_axes.h']
report=dict(passed=f.returncode==0,cases=46 if real_group else 14,actual_mission_and_CAN_backend=True,actual_startup_group_and_extension=real_group,startup_axes_are_models=not real_group,scheduler_sensors_and_hardware_are_models=True,DM_driver_and_PWM_are_models=True,
            not_complete_board_firmware_execution=True,hardware_accessed=False,flashed=False,
            source_root=str(A),source_sha256={n:hashlib.sha256((A/n).read_bytes()).hexdigest() for n in files},output=f.stdout)
if real_group and f.returncode==0:
    import sys
    sys.path.insert(0,str(R/'scripts'))
    from vehicle_uart_capture import parse_extension_diag
    frames=[line.encode() for line in f.stdout.splitlines() if line.startswith(('EX1,','EX2,','EX3,'))]
    decoded=[parse_extension_diag(line) for line in frames]
    faults=[d for d in decoded if d['first_reason']]
    injected_timeouts=[d for d in faults if d['first_reason']==5 and d['command']==0x3a and d['exchange_result']==5]
    assert len(injected_timeouts)==2
    report['EX1_roundtrip_frames']=len(frames);report['injected_extension_faults_decoded']=len(faults)
(E/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');assert f.returncode==0,f.stdout
