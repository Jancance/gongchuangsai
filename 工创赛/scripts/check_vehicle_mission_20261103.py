"""Source-parameter routes and actual eight-stage state-machine host tests."""
from pathlib import Path
from vehicle_check_paths import firmware_root
import json,subprocess,sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from vehicle_uart_capture import parse_mission
R=Path(__file__).resolve().parents[1];S=firmware_root()
O=R/'evidence/integration/2026-11-03/early-scan-startup'
base=(O/'early-scan-unit.c').read_text(encoding='utf-8')
unit=base[:base.index('static void reset(void)')]
unit='#define VEHICLE_PARAMETER_TABLES\n#include <math.h>\n'+unit
unit=unit.replace('#define MOTION_ACTION_COUNT 2U', 'static uint32_t s_motion_action_count;\n#define MOTION_ACTION_COUNT (s_motion_action_count)')
unit=unit.replace('fixture_backend.result=0;return 1;', 'fixture_backend.result=0;fixture_backend.state=0;return 1;')
unit+=r'''
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t g_debug_plan_id=DEBUG_PLAN_DEFAULT,s_motion_timing_count,s_motion_timing_cursor;
static uint32_t g_vehicle_arm_state;
uint32_t g_vehicle_arm_cause,g_startup_axes_cause,g_startup_extension_feedback_valid,g_vehicle_lift_feedback_valid;
static uint32_t actor_calls,cancel_calls;
static uint32_t g_safety_ready=1U;
static float g_startup_extension_current_deg,g_vehicle_lift_current_deg;
static uint32_t VehicleArm_Busy(void){return g_vehicle_arm_state==1;}
static uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item){actor_calls++;g_vehicle_arm_state=1;return 1;}
static void VehicleArm_Cancel(void){cancel_calls++;}
'''
# One stub already exists in the combined-route harness; use its real zone fixture.
unit=unit.replace('static uint32_t RadarZoneSwitch_Lock(void){return 1;}','static uint32_t RadarZoneSwitch_Lock(void){return g_radar_zone_selected;}')
main=(S/'CHASSIS/USER/main.c').read_bytes().decode('gbk')
lo=main.index('static uint32_t App_MotionPlanValid(void)');start=main.index('{',lo);depth=1;end=start+1
while depth:depth+=(main[end]=='{')-(main[end]=='}');end+=1
unit+='#include "motion_distance_limits.h"\n'+main[lo:end]
unit+='\n#include "vehicle_mission.inc"\n'
unit+=r'''
static void reset_mission(void){
 memset((void*)&g_combined_test,0,sizeof(g_combined_test));memset((void*)&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&fixture_backend,0,sizeof(fixture_backend));memset(&fixture_power,0,sizeof(fixture_power));
 g_vehicle_state=g_vehicle_cause=g_vehicle_key=g_vehicle_arm_row=g_vehicle_item=g_vehicle_stage_mask=0;
 g_vehicle_stage=1;g_vehicle_run_id=0;g_startup_axes_state=0;g_vehicle_arm_state=0;
 s_vehicle_zone=1;g_radar_zone_selected=g_radar_zone_ready=1;ready=rearm_ok=tx_ok=1;
 g_vm_test_request=s_motion_session_open=g_forward_hold.trip=g_heading_exec_trip=0;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0;
 s_motion_action_count=2;actor_calls=cancel_calls=display_calls=0;tick=100;
 g_saoma_accept_count=10;g_saoma_last_result=1;g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;
 g_scan_display_result=SCREEN_OK;display_result=SCREEN_OK;
}
static void move_done(void){
 g_combined_test.state=3;g_combined_test.completed=s_motion_action_count;
 fixture_backend.state=9;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;
 fixture_backend.active=fixture_backend.cause=0;g_vm_test_request=s_motion_session_open=0;
}
static void test_mission(void){
 const uint32_t count[8]={2,2,1,2,2,1,2,4};uint32_t stage,zone,i,j;
 {MotionProfile profile;uint32_t rpm,ms;
 ck(MotionRoute_Build(0,10.0f,50.0f,&profile,&rpm,&ms));
 ck(!MotionRoute_Build(0,0.0f,50.0f,&profile,&rpm,&ms));
 ck(!MotionRoute_Build(0,NAN,50.0f,&profile,&rpm,&ms));}
 for(zone=1;zone<=2;zone++)for(stage=1;stage<=8;stage++){
  reset_mission();s_vehicle_zone=zone;g_startup_axes_state=4;
  ck(VehicleMission_PathClear(stage,zone));
#if !APP_MOTOR_OFFLINE_ONLY
  ck(VehicleMission_Load(stage));ck(s_motion_action_count==count[stage-1]);
  for(i=0;i<s_motion_action_count;i++)ck(s_motion_actions[i].distance_mm>=10 && s_motion_actions[i].reference_mm_s>0);
#endif
 }
 for(i=0;i<8;i++){
  for(j=0;j<32 && vehicle_arm_rows[i][j].op;j++)ck(vehicle_arm_rows[i][j].op<=9);
  ck(j<32 && vehicle_arm_repeat[i]>=1 && vehicle_arm_repeat[i]<=3);
 }
 reset_mission();g_vehicle_key=1;VehicleMission_Service();
#if APP_MOTOR_OFFLINE_ONLY
 ck(g_vehicle_state==0 && !g_combined_test.request);return;
#else
 ck(g_vehicle_state==2 && g_vehicle_stage==1 && g_vehicle_run_id==1);
 move_done();tick=1200;
#if VEHICLE_SCAN_BYPASS
 VehicleMission_Service();
#if VEHICLE_RUN_ALL
 ck(g_vehicle_stage==2 && g_vehicle_state==2); /* guarded normal stop rearm */
#else
 ck(g_vehicle_state==4 && g_vehicle_stage_mask==1);
 g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==2 && g_vehicle_run_id==2);
#endif
#else
 VehicleMission_Service();ck(g_vehicle_state==2 && !display_calls);
 strcpy(SAOMA_Data,"156+123+516+231");g_saoma_accept_count++;g_saoma_frame_start_ms=1100;g_saoma_frame_end_ms=1150;
 VehicleMission_Service();ck(display_calls==1 && !strcmp(displayed,SAOMA_Data));
#if VEHICLE_RUN_ALL
 ck(g_vehicle_stage==2 && g_vehicle_state==2);
#else
 ck(g_vehicle_state==4);
#endif
#endif
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_vehicle_key=1;VehicleMission_Service();
 ck(g_vehicle_state==7 && stop_calls>0 && cancel_calls>0);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_safety_fault=1;VehicleMission_Service();ck(g_vehicle_state==6 && g_vehicle_cause==4);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();tick+=VEHICLE_STAGE_TIMEOUT_MS;VehicleMission_Service();ck(g_vehicle_state==6 && g_vehicle_cause==5);
 /* Controller cannot manufacture a material confirmation, nor accept another transaction. */
 reset_mission();g_vehicle_state=3;g_vehicle_stage=2;g_vehicle_run_id=9;g_vehicle_arm_row=0;while(vehicle_arm_rows[1][g_vehicle_arm_row].op!=7)g_vehicle_arm_row++;uint32_t observation_row=g_vehicle_arm_row;g_vehicle_item=0;g_startup_axes_state=4;
 ck(!VehicleMission_Observe(9,0,0,0,7));ck(!VehicleMission_Observe(9,9,0,0,7));
 ck(!VehicleMission_Observe(9,2,0,32,7));ck(!VehicleMission_Observe(9,2,3,observation_row,7));
 ck(!VehicleMission_Observe(8,2,0,observation_row,7));ck(!VehicleMission_Observe(9,2,0,observation_row,8));
 VehicleMission_Service();ck(g_vehicle_arm_row==observation_row && !g_vehicle_correct_pick_count && !g_vehicle_correct_place_count);
 ck(VehicleMission_Observe(9,2,0,observation_row,7));VehicleMission_Service();ck(g_vehicle_arm_row==observation_row+1);
 /* Full chain must not pretend an early stop was the nominal scan endpoint. */
#if !VEHICLE_SCAN_BYPASS && VEHICLE_RUN_ALL
 reset_mission();g_vehicle_key=1;VehicleMission_Service();tick=200;
 strcpy(SAOMA_Data,"156+123+516+231");g_saoma_accept_count++;g_saoma_frame_start_ms=110;g_saoma_frame_end_ms=150;
 VehicleMission_Service();ck(g_stage1_stop_requested);
 move_done();g_combined_test.completed=0;VehicleMission_Service();ck(g_vehicle_state==6 && g_vehicle_cause==10);
#endif
#endif
}
int main(void){test_mission();tick+=1000;VehicleMission_Telemetry();printf("%s",accepted_frames);printf("mission checks=%u failures=%u\n",checks,failures);for(uint32_t i=0;i<failures && i<16;i++)printf("failed check %u\n",failure_checks[i]);return failures?1:0;}
'''
(O/'mission-unit.c').write_text(unit,encoding='utf-8')
reports=[]
for label,offline,bypass,chain in [('single-bypass',0,1,0),('chain-bypass',0,1,1),('single-real',0,0,0),('chain-real',0,0,1),('offline',1,1,0)]:
 exe=O/('mission-'+label+'.exe')
 cmd=['C:/mingw/mingw64/bin/gcc.exe','-DVEHICLE_STARTUP_GROUP_BEFORE_ROUTE=0','-DVEHICLE_STARTUP_GROUP_ONLY=0','-DVEHICLE_EXTENSION_HOME_ONLY=0','-DVEHICLE_DM_STARTUP_ONLY=0','-std=c99',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
      f'-DVEHICLE_SCAN_BYPASS={bypass}',f'-DVEHICLE_RUN_ALL={chain}',
      '-I'+str(S/'CHASSIS/USER'),str(O/'mission-unit.c')]
 cmd += [str(S/'CHASSIS/USER'/p) for p in ['field_map.c','app_entry.c','motion_profile.c','chassis_kinematics.c']]
 cmd += ['-lm','-o',str(exe)]
 p=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert p.returncode==0,p.stderr[-6000:]
 p=subprocess.run([str(exe)],capture_output=True,text=True);print(label,p.stdout,flush=True);assert p.returncode==0,p.stdout
 frames=[line.encode() for line in p.stdout.splitlines() if line.startswith('VM1,')]
 assert len(frames)==1
 decoded=parse_mission(frames[0]);assert 1<=decoded['stage']<=8
 reports.append(dict(case=label,passed=True,output=p.stdout,telemetry=decoded))
(O/'mission-check.json').write_text(json.dumps(dict(cases=reports,actual_c=True,peripherals_mocked=True,
 hardware_accessed=False),indent=2),encoding='utf-8')
