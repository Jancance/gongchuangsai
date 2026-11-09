"""Actual task transitions for mirrored finite scanner search, hardware mocked."""
from pathlib import Path
import subprocess,json
R=Path(__file__).resolve().parents[1];S=R/'tmp/stage1_blend_search_20261104';E=R/'evidence/integration/2026-11-04/stage1-blend-search'
b=(R/'evidence/integration/2026-11-03/stage1-group-route/stage1-group-mission-unit.c').read_text(encoding='utf-8')
lo=b.index('static void test_mission(void)');hi=b.index('{',lo)+1;depth=1
while depth:depth+=(b[hi]=='{')-(b[hi]=='}');hi+=1
test=r'''
static void begin(uint32_t zone){
 reset_mission();fixture_axes_vm=0;g_safety_ready=1;g_radar_zone_selected=zone;
 g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==1 && !g_combined_test.request);
 g_startup_axes_state=4;fixture_axes_vm=1;VehicleMission_Service();
 ck(g_vehicle_state==2 && s_motion_action_count==1 && !g_vehicle_scan_search_state);
 ck(s_motion_actions[0].kind==(zone==1?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE));
 ck(s_motion_actions[0].distance_mm==1050 && s_motion_actions[0].reference_mm_s==450);
}
static void fresh(const char *code){tick+=100;strcpy(SAOMA_Data,code);g_saoma_accept_count++;g_saoma_last_result=1;g_saoma_frame_start_ms=tick;g_saoma_frame_end_ms=tick;}
static void arrive(void){g_combined_test.request=0;move_done();VehicleMission_Service();}
static void test_mission(void){
 for(uint32_t zone=1;zone<=2;zone++){
  /* No code at nominal point: search immediately 150, then cross 300. */
  begin(zone);arrive();
  ck(g_vehicle_state==2 && g_vehicle_scan_search_state==1 && !display_calls);
  ck(s_motion_actions[0].distance_mm==150 && s_motion_actions[0].reference_mm_s==450);
  ck(s_motion_actions[0].kind==(zone==1?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE));
  arrive();ck(g_vehicle_state==2 && g_vehicle_scan_search_state==2);
  ck(s_motion_actions[0].distance_mm==300 && s_motion_actions[0].kind==(zone==1?MOTION_RIGHT_DISTANCE:MOTION_LEFT_DISTANCE));
  arrive();ck(g_vehicle_state==6 && g_vehicle_cause==16 && g_vehicle_scan_search_state==3 && !fixture_axes_vm);
  /* Code while first search moving: stop requested; display only after confirmed stop. */
  begin(zone);arrive();fresh("156+123+516+231");VehicleMission_Service();
  ck(g_stage1_stop_requested && !display_calls && g_vehicle_state==2);
  arrive();ck(g_vehicle_state==4 && g_vehicle_scan_search_state==4 && display_calls==1);
  /* Code while returning search: same handling, no third search. */
  begin(zone);arrive();arrive();fresh("156+123+516+231");VehicleMission_Service();arrive();
  ck(g_vehicle_state==4 && g_vehicle_scan_search_state==4 && display_calls==1);
  /* Nominal route gets code first: no search. */
  begin(zone);fresh("156+123+516+231");VehicleMission_Service();ck(g_stage1_stop_requested);arrive();
  ck(g_vehicle_state==4 && !s_vehicle_search_used && display_calls==1);
  /* Semantically invalid code never becomes a successful task. */
  begin(zone);fresh("216+231+354+123");VehicleMission_Service();arrive();
  ck(g_vehicle_scan_search_state==1 && !display_calls && !s_vehicle_code_valid);
 }
 begin(1);arrive();tick+=VEHICLE_SCAN_SEARCH_TOTAL_MS;VehicleMission_Service();
 ck(g_vehicle_state==6 && g_vehicle_cause==18 && !fixture_axes_vm);
 begin(1);arrive();g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==7 && !fixture_axes_vm);
 begin(1);g_safety_fault=1;VehicleMission_Service();ck(g_vehicle_state==6 && !fixture_axes_vm);
 ck(!g_vehicle_correct_pick_count && !g_vehicle_correct_place_count);
}
'''
u=b[:lo]+test+b[hi:];p=E/'mission-unit.c';p.write_text(u,encoding='utf-8');x=E/'mission-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(S/'CHASSIS/USER'),str(p)]
cmd += [str(S/'CHASSIS/USER'/n) for n in ['field_map.c','app_entry.c','motion_profile.c','chassis_kinematics.c']]
cmd+=['-lm','-o',str(x)];f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(x)],capture_output=True,text=True);print(f.stdout);(E/'mission-check.log').write_text(f.stdout+f.stderr,encoding='utf-8');assert f.returncode==0,f.stdout
(E/'mission-check.json').write_text(json.dumps(dict(passed=True,actual_mission_C=True,peripherals_mocked=True,hardware_accessed=False,output=f.stdout),indent=2),encoding='utf-8')
