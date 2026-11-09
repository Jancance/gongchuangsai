"""Actual mission/CAN backend isolated straight test; modeled devices and partial scheduler."""
from pathlib import Path
import os,json,subprocess,hashlib
R=Path(__file__).resolve().parents[1];S=Path(os.environ.get('STRAIGHT_SOURCE_ROOT',str(R/'tmp/straight210_20261104')))
E=Path(os.environ.get('STRAIGHT_TEST_OUT',str(R/'evidence/integration/2026-11-04/straight210/after')));E.mkdir(parents=True,exist_ok=True)
base=(R/'evidence/integration/2026-11-04/dynamic-route/actual-final/whole-unit.c').read_text(encoding='utf-8')
prefix=base[:base.index('static void begin_leg(void)')]
main_bytes=(S/'CHASSIS/USER/main.c').read_bytes()
try:main_text=main_bytes.decode('utf-8')
except UnicodeDecodeError:main_text=main_bytes.decode('gbk')
if 'static uint32_t App_VehicleCombinedAllowed(void)' in main_text:
 from stage12_admission_fixture import function
 dispatch_gate=function(main_text,'App_VehicleCombinedAllowed')
else:
 import re
 match=re.search(r'if\((!VEHICLE_EXTENSION_HOME_ONLY[^\n]+)\) App_CombinedTestStep\(\);',main_text)
 assert match
 dispatch_gate='static uint32_t App_VehicleCombinedAllowed(void){return '+match[1]+';}\n'
prefix+='\n'+dispatch_gate
reset=base[base.index('static void reset_case(unsigned zone)'):base.index('static void dynamic_geometry_checks')]
reset=reset.replace('g_vehicle_key=1;VehicleMission_Service();','')
unit=prefix+r'''
static float test_tau=.05f;
static unsigned plant_stuck;
static void dispatch(void){
 if(!App_VehicleCombinedAllowed())return;
 if(g_combined_test.request==1 && !g_combined_test.state){
  ChassisAutoPreparedTranslation p;MotionAction *a=&s_motion_actions[0];
  ck(s_motion_action_count==1 && a->kind==MOTION_FORWARD_DISTANCE && a->distance_mm==2100 && a->reference_mm_s==400);
  ck(!g_startup_axes_vm_request && !StartupAxes_Holding());
  g_combined_test.request=0;
  ck(RouteHeading_Ensure(&s_route_heading_reference,0,1));
  ck(ChassisAuto_BeginSequence());g_vm_test_request=1;ChassisAuto_SafetyUpdate(1);
  ck(ChassisAuto_PrepareRouteTranslation(0,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&p));
  ck(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
  starts++;g_combined_test.state=1;s_motion_session_open=1;
 }
 if(g_chassis_auto_active){
  for(unsigned i=0;i<4;i++){
   float demand=fixture_vm && plant_enabled[i] && !plant_stuck?plant_target[i]:0;
   plant_velocity[i]+=(demand-plant_velocity[i])*(.001f/test_tau);
   plant_pos[i]+=plant_velocity[i]*(3.14159265358979323846f*77/60)*.001f;
  }
  (void)ChassisAuto_UpdateForwardHold(0);ChassisAuto_SafetyUpdate(g_vm_test_request);ChassisAuto_CanStep();
 }
 if(g_combined_test.state==1 && !g_chassis_auto_active){
  if(g_chassis_auto_result==1){g_combined_test.state=3;g_combined_test.completed=s_distance_reached?1:0;}
  else {g_combined_test.state=4;g_combined_test.cause=8;}
  g_vm_test_request=0;s_motion_session_open=0;ChassisAuto_SafetyUpdate(0);
 }
}
'''+reset+r'''
int main(void){
 ck(VEHICLE_STRAIGHT_TEST_ENABLE && !VEHICLE_STAGE1_BLEND_ENABLE && VEHICLE_STRAIGHT_TEST_DISTANCE_CM==210 && VEHICLE_STRAIGHT_TEST_SPEED_CM_S==40);
 ck(fabsf(CHASSIS_FORWARD_DISTANCE_SCALE-1.02182540f)<.0000001f);
 reset_case(1);ck(App_VehicleCombinedAllowed());
 for(unsigned st=1;st<=3;st++){g_startup_axes_state=st;ck(!App_VehicleCombinedAllowed());}
 g_startup_axes_state=5;ck(!App_VehicleCombinedAllowed());g_startup_axes_state=6;ck(!App_VehicleCombinedAllowed());
 g_startup_axes_state=4;g_startup_axes_vm_request=1;ck(App_VehicleCombinedAllowed());g_startup_axes_vm_request=0;ck(!App_VehicleCombinedAllowed());
 g_startup_axes_state=0;g_vehicle_arm_state=1;ck(!App_VehicleCombinedAllowed());g_vehicle_arm_state=0;
 FILE *trace=fopen("straight-model.csv","w");fprintf(trace,"case,t_ms,state,backend_state,encoder_calibrated_mm,reference_mm_s,VM_model\n");
 for(unsigned scenario=0;scenario<10;scenario++){
  reset_case(1);plant_stuck=scenario==6;test_tau=scenario==8?.1f:.05f;
  s_heading_source=(AppHeadingSource){1,test_tick,3,test_tick,test_tick,0};s_heading_source_ready=1;g_can_feedback_enable=0;
  if(scenario==5)s_heading_source.healthy=0;
  if(scenario==9)g_can_feedback_enable=1;
  g_vehicle_key=1;VehicleMission_Service();
  for(unsigned step=0;step<30000 && g_vehicle_state<4;step++){
   test_tick++;s_heading_source.sample_ms=s_heading_source.angle_ms=s_heading_source.rate_ms=test_tick;
   fixture_vm=g_vm_test_request && !g_safety_fault && !g_rtos_fault;
   dispatch();
   if(scenario==2 && s_distance_progress_mm>500)g_vehicle_key=1;
   if(scenario==3 && s_distance_progress_mm>500)fixture_can_failure=1;
   if(scenario==4 && s_distance_progress_mm>500)g_safety_fault=1;
   if(scenario==7 && g_chassis_auto_active)test_tick+=20000;
   VehicleMission_Service();fixture_vm=g_vm_test_request && !g_safety_fault && !g_rtos_fault;
   if(!(step%20))fprintf(trace,"%u,%u,%u,%u,%.6f,%.6f,%u\n",scenario,test_tick-s_vehicle_start,g_vehicle_state,g_chassis_auto_state,s_distance_progress_mm,s_distance_reference.speed_mm_s,fixture_vm);
   if((scenario==5 || scenario==9) && step>10)break;
  }
  ck(!model_home_tick && !model_source_dm && !g_source_prepare_release_ms && !display_calls && !arm_calls && !g_startup_axes_vm_request);
  if(scenario==0 || scenario==1 || scenario==8){
   float forward=(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
   ck(g_vehicle_state==4 && !g_vehicle_cause && starts==1 && g_combined_test.completed==1 && !fixture_vm);
   ck(g_chassis_auto_result==1 && !g_chassis_auto_active && g_chassis_auto_stop_mask[2]==15);
   printf("STRAIGHT_MODEL_FORWARD case%u %.6f\n",scenario,forward);
   /* 记录模型停车尾差；尚无用户实测容差，不用任意精度阈值签通过。 */
   if(scenario==1){g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==4 && g_vehicle_cause==19 && starts==1);}
  }else if(scenario==5 || scenario==9){ck(g_vehicle_state==0 && g_vehicle_cause==1 && !starts && !fixture_vm);}
  else {ck((g_vehicle_state==6 || g_vehicle_state==7) && !fixture_vm);if(scenario==2)ck(g_vehicle_state==7 && g_vehicle_cause==2);}
  printf("STRAIGHT case%u state%u cause%u starts%u progress%.3f\n",scenario,g_vehicle_state,g_vehicle_cause,starts,s_distance_progress_mm);
  VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();
 }
 fclose(trace);printf("STRAIGHT checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
p=E/'straight-unit.c';p.write_text(unit,encoding='utf-8');x=E/'straight-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(R/'evidence/integration/2026-11-03/no-motion/backend-stubs'),'-I'+str(S/'CHASSIS/USER'),'-I'+str(S/'HARDWARE/DM_CAN2'),'-I'+str(S/'HARDWARE/GRIPPER'),'-I'+str(S/'HARDWARE/CARRIER'),'-I'+str(R/'evidence/integration/2026-11-04/source-parallel/whole-stubs'),str(p)]
cmd += [str(S/'CHASSIS/USER'/n) for n in ['field_map.c','chassis_kinematics.c','motion_profile.c','heading_controller.c']]+['-lm','-o',str(x)]
f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(x)],cwd=E,capture_output=True,text=True);print(f.stdout);(E/'straight-check.log').write_text(f.stdout+f.stderr,encoding='utf-8')
files=['USER/vehicle_parameters.h']+['CHASSIS/USER/'+n for n in ['straight_test.inc','vehicle_mission.inc','main.c','chassis_auto_test.c','chassis_kinematics.c','chassis_kinematics.h']]
(E/'result.json').write_text(json.dumps(dict(passed=f.returncode==0,cases=10,actual_mission_and_CAN_backend=True,actual_main_dispatch_gate=True,devices_sensors_and_partial_scheduler_are_models=True,hardware_accessed=False,source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in files},output=f.stdout),indent=2),encoding='utf-8');assert f.returncode==0
