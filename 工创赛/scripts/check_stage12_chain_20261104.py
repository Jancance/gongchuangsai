"""Actual mission/startup/extension/CAN backend chain, with modeled devices and scheduling."""
from pathlib import Path
import os,json,subprocess,hashlib
R=Path(__file__).resolve().parents[1]
S=Path(os.environ.get('STAGE12_SOURCE_ROOT',str(R/'tmp/stage12_chain_20261104')))
E=Path(os.environ.get('STAGE12_WHOLE_OUT',str(R/'evidence/integration/2026-11-04/stage12-chain/whole')));E.mkdir(parents=True,exist_ok=True)
base=(R/'evidence/integration/2026-11-04/stage2-retest/actual-final/whole-unit.c').read_text(encoding='utf-8')
prefix=base[:base.index('static void begin_leg(void)')]
reset=base[base.index('static void reset_case(unsigned zone)'):base.index('int main(void)')]
unit=prefix+r'''
static void begin_leg(void){
 ChassisAutoPreparedTranslation p;MotionAction *a=&s_motion_actions[g_combined_test.index];
 ck(StartupAxes_Holding() && g_extension_hold_confirmed && g_extension_hold_samples==3);
 if(g_vehicle_stage==1 && !g_vehicle_scan_search_state){
  ck(ChassisAuto_PrepareRouteBlend(a->kind-MOTION_FORWARD_DISTANCE,a->base_rpm,a->run_ms,a->distance_mm,a->reference_mm_s,&p));
  ck(ChassisAuto_BeginPreparedBlend(&p,g_chassis_auto_session));
 }else{
  if(g_vehicle_stage==2){
   ck(a->reference_mm_s==400 && s_motion_action_count>=2 && s_motion_action_count<=3);
   if(g_combined_test.index==s_motion_action_count-1)ck(a->kind==MOTION_RIGHT_DISTANCE && fabsf(a->distance_mm-910)<.01f);
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
'''+reset+r'''
int main(void){
 ck(VEHICLE_CHAIN_STAGE12 && VEHICLE_TEST_STAGE==1 && VEHICLE_TEST_START_ZONE==1 && !VEHICLE_RUN_ALL && VEHICLE_STAGE2_ROUTE_ONLY);
 ck(VEHICLE_STAGE2_GOAL_OFFSET_X_MM==-50 && !VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE);
 for(unsigned inputzone=1;inputzone<=2;inputzone++)for(unsigned scenario=0;scenario<20;scenario++){
  reset_case(inputzone);unsigned sent=0,seen1=0,seen2=0,corrupted=0,rerun=0,second_stage_seen=0;
  stop_sent=0;
  for(unsigned step=0;step<120000 && g_vehicle_state<4;step++){
   test_tick+=1;
   model_settle_case=scenario==15?18:scenario==16?17:scenario==17?19:0;model_enable_fail=scenario==14;
   fixture_vm=(g_startup_axes_vm_request||g_vm_test_request)&&!g_safety_fault&&!g_rtos_fault;
   StartupAxes_Safety(!g_safety_fault&&!g_rtos_fault&&!g_can_ctrl_fault_latched);
   StartupAxes_Service(g_safety_ready,g_can_guard_ready);dispatch();
   if(g_vehicle_scan_search_state==1)seen1=1;if(g_vehicle_scan_search_state==2)seen2=1;
   unsigned stopped=g_combined_test.state==3 && g_vehicle_stage==1;
   unsigned moving=g_chassis_auto_active && s_distance_progress_mm>50;
   if(!sent && g_vehicle_stage==1 && (
      ((scenario==0 || scenario>=7) && stopped && !g_vehicle_scan_search_state) ||
      (scenario==1 && g_vehicle_scan_search_state==1 && moving) ||
      (scenario==2 && g_vehicle_scan_search_state==2 && moving) ||
      (scenario==3 && !g_vehicle_scan_search_state && g_chassis_auto_active && s_distance_progress_mm>900) ||
      (scenario==4 && !g_vehicle_scan_search_state && g_chassis_auto_active && s_distance_progress_mm>200) ||
      (scenario==6 && stopped && !g_vehicle_scan_search_state))){
    strcpy(SAOMA_Data,scenario==6?"216+231+354+123":"156+123+516+231");
    g_saoma_accept_count++;g_saoma_last_result=1;g_saoma_frame_start_ms=g_saoma_frame_end_ms=test_tick;sent=1;
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
   VehicleMission_Service();fixture_vm=(g_startup_axes_vm_request||g_vm_test_request)&&!g_safety_fault&&!g_rtos_fault;
   if(g_combined_test.request)stop_sent=0;
   if(scenario==19 && g_vehicle_state==4 && !rerun){
    memset(plant_pos,0,sizeof(plant_pos));memset(plant_velocity,0,sizeof(plant_velocity));memset(plant_target,0,sizeof(plant_target));
    sent=0;rerun=1;g_vehicle_key=1;VehicleMission_Service();
   }
  }
  VehicleAlignmentRequest req;ck(!arm_calls && !test_depth && s_vehicle_zone==1);
  if(scenario==0 || scenario==1 || scenario==2 || scenario==3 || scenario==16 || scenario==17 || scenario==19){
   float wx=2250+(-plant_pos[0]+plant_pos[1]+plant_pos[2]-plant_pos[3])*.25f;
   float wy=150+(plant_pos[0]+plant_pos[1]+plant_pos[2]+plant_pos[3])*.25f*CHASSIS_FORWARD_DISTANCE_SCALE;
   ck(g_vehicle_stage==2 && g_vehicle_state==4 && !g_vehicle_cause && g_vehicle_stage_mask==3 && second_stage_seen);
   ck(display_calls==(scenario==19?2:1) && !strcmp(displayed,"156+123+516+231") && fixture_vm && !g_chassis_auto_active);
   ck(VehicleMission_ReadAlignmentRequest(&req) && req.goal_cell==14 && req.start_id==1 && req.nominal_target_x_mm==2110 && req.nominal_target_y_mm==1200 && !req.position_executor_enabled);
   ck(fabsf(wx-2110)<=20.06f && fabsf(wy-1200)<=10.06f);
   printf("CHAIN_MODEL_ENDPOINT case%u x%.3f y%.3f handoff%.3f,%.3f\n",scenario,wx,wy,s_vehicle_handoff.x_mm,s_vehicle_handoff.y_mm);
   if(scenario==1)ck(seen1 && !seen2);if(scenario==2)ck(seen1 && seen2);if(scenario==19)ck(g_vehicle_run_id==2);
  }else{
   ck((g_vehicle_state==6 || g_vehicle_state==7) && !fixture_vm && !VehicleMission_ReadAlignmentRequest(&req));
   if(scenario==4)ck(!second_stage_seen && g_vehicle_cause==7);
   if(scenario==5 || scenario==6)ck(!second_stage_seen && g_vehicle_cause==16 && !display_calls);
   if(scenario==10 || scenario==11 || scenario==12)ck(!second_stage_seen && g_vehicle_cause==10);
  }
  printf("CHAIN input%u case%u state%u cause%u stage%u sessions%u searches%u/%u mask%u\n",inputzone,scenario,g_vehicle_state,g_vehicle_cause,g_vehicle_stage,starts,seen1,seen2,g_vehicle_stage_mask);
  VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();test_tick+=500;VehicleMission_Telemetry();
 }
 printf("CHAIN checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
p=E/'whole-unit.c';p.write_text(unit,encoding='utf-8');x=E/'whole-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(R/'evidence/integration/2026-11-03/no-motion/backend-stubs'),'-I'+str(S/'CHASSIS/USER'),str(p)]
cmd += [str(S/'CHASSIS/USER'/n) for n in ['field_map.c','chassis_kinematics.c','motion_profile.c','heading_controller.c']]+['-lm','-o',str(x)]
f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(x)],cwd=E,capture_output=True,text=True);(E/'whole.log').write_text(f.stdout+f.stderr,encoding='utf-8');print(f.stdout)
files=['USER/vehicle_parameters.h']+['CHASSIS/USER/'+n for n in ['vehicle_mission.inc','chassis_auto_test.c','startup_group.inc','extension_home_only.inc','startup_axes.h','route_heading_reference.h','grid_route.h','main.c']]
report=dict(passed=f.returncode==0,cases=40,fixed_start_zone=1,physical_switch_inputs_tested=[1,2],source_root=str(S),source_sha256={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in files},actual_mission_startup_extension_CAN_backend=True,scheduler_devices_sensors_plant_DM_PWM_are_models=True,hardware_accessed=False,output=f.stdout)
if f.returncode==0:
 from vehicle_uart_capture import parse_grid_route
 frames=[parse_grid_route(l.encode()) for l in f.stdout.splitlines() if l.startswith(('VG3,','VG4,'))]
 assert frames and all(v is not None for v in frames);report['VG3_roundtrip_frames']=len(frames)
(E/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');assert f.returncode==0
