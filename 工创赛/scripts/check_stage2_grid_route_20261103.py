"""Actual mission and route conversion with mocked peripherals; no hardware."""
from pathlib import Path
import json,subprocess,sys
R=Path(__file__).resolve().parents[1];S=R/'tmp/stage2_grid_route_20261103'
E=R/'evidence/integration/2026-11-03/stage2-grid-route'
base=(R/'evidence/integration/2026-11-03/stage1-group-route/stage1-group-mission-unit.c').read_text(encoding='utf-8')
lo=base.index('static void test_mission(void)');hi=base.index('{',lo)+1;depth=1
while depth:depth+=(base[hi]=='{')-(base[hi]=='}');hi+=1
tests=r'''
static void test_mission(void){
 RouteRow rows[8];uint32_t count;FieldPoint start={1200,300},end;
 uint8_t nodes[3]={12,14,0};VehicleAlignmentRequest request;
 ck(VEHICLE_TEST_STAGE==2 && !VEHICLE_RUN_ALL && VEHICLE_STAGE2_ROUTE_ONLY && !VEHICLE_ALIGNMENT_POSITION_EXECUTOR_ENABLE);
 ck(GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 ck(count==2 && rows[0].kind==MOTION_FORWARD_DISTANCE && rows[0].amount==90 && rows[0].speed==10);
 ck(rows[1].kind==MOTION_RIGHT_DISTANCE && rows[1].amount==96 && end.x_mm==2160 && end.y_mm==1200);
 start.y_mm=240;ck(GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 ck(rows[0].amount==96); /* measured start must not silently become cell center */
 start.y_mm=300;
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION+1,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,10,nodes,2,10,10,rows,8,&count,&end));
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,1,&count,&end));
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,0,10,10,rows,8,&count,&end));
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,NAN,10,rows,8,&count,&end));
 nodes[0]=25;ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 nodes[0]=2;ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 nodes[0]=13;ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end)); /* diagonal */
 nodes[0]=6;ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end)); /* obstacle */
 nodes[0]=12;start.x_mm=800;
 ck(!GridRoute_Build(VEHICLE_MAP_VERSION,2,start,14,nodes,2,10,10,rows,8,&count,&end));
 for(uint32_t zone=1;zone<=2;zone++){
  reset_mission();fixture_axes_vm=0;g_safety_ready=1;g_radar_zone_selected=zone;
  ck(!VehicleMission_ReadAlignmentRequest(&request));
  g_vehicle_key=1;VehicleMission_Service();
  ck(g_vehicle_state==1 && g_vehicle_stage==2 && !g_combined_test.request && !g_vehicle_alignment_state);
  g_startup_axes_state=4;fixture_axes_vm=1;VehicleMission_Service();
  ck(g_vehicle_state==2 && g_combined_test.request && s_motion_action_count==2);
  ck(s_motion_actions[0].kind==MOTION_FORWARD_DISTANCE && s_motion_actions[0].distance_mm==900 && s_motion_actions[0].reference_mm_s==400);
  ck(s_motion_actions[1].kind==MOTION_RIGHT_DISTANCE && s_motion_actions[1].distance_mm==960 && s_motion_actions[1].reference_mm_s==400);
  ck(!VehicleMission_ReadAlignmentRequest(&request));
  g_combined_test.request=0;move_done();VehicleMission_Service();
  ck(g_vehicle_state==4 && g_vehicle_stage==2 && g_vehicle_stage_mask==2 && !actor_calls);
  ck(g_vehicle_alignment_state==1 && StartupAxes_Holding() && !g_stage1_stop_requested);
  ck(VehicleMission_ReadAlignmentRequest(&request));
  ck(request.run_id==g_vehicle_run_id && request.stage==2 && request.start_id==zone && request.goal_cell==14);
  ck(request.nominal_target_x_mm==2160 && request.nominal_target_y_mm==1200 && !request.position_executor_enabled);
  ck(!VehicleMission_ReadAlignmentRequest(0));
  VehicleMission_Service();ck(g_vehicle_state==4 && !actor_calls && !display_calls);
 }
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_vehicle_key=1;VehicleMission_Service();
 ck(g_vehicle_state==7 && g_vehicle_alignment_state==3 && stop_calls && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_safety_fault=1;VehicleMission_Service();
 ck(g_vehicle_state==6 && g_vehicle_alignment_state==3 && !VehicleMission_ReadAlignmentRequest(&request));
 reset_mission();g_vehicle_key=1;VehicleMission_Service();tick+=VEHICLE_STAGE_TIMEOUT_MS;VehicleMission_Service();
 ck(g_vehicle_state==6 && g_vehicle_cause==5 && g_vehicle_alignment_state==3);
}
'''
unit=base[:lo]+tests+base[hi:];p=E/'mission-unit.c';p.write_text(unit,encoding='utf-8');exe=E/'mission-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0','-I'+str(S/'CHASSIS/USER'),str(p)]
cmd += [str(S/'CHASSIS/USER'/n) for n in ['field_map.c','app_entry.c','motion_profile.c','chassis_kinematics.c']]
cmd+=['-lm','-o',str(exe)]
f=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert f.returncode==0,f.stderr
f=subprocess.run([str(exe)],capture_output=True,text=True);(E/'mission-check.log').write_text(f.stdout+f.stderr,encoding='utf-8');print(f.stdout);assert f.returncode==0,f.stdout
sys.path.insert(0,str(R/'scripts'))
from vehicle_uart_capture import parse_grid_route
frames=[l.encode() for l in f.stdout.splitlines() if l.startswith('VG2,')];assert len(frames)==1
parsed=parse_grid_route(frames[0]);assert parsed['source']==1 and parsed['goal_cell']==14 and not parsed['position_executor_enabled']
try:parse_grid_route(frames[0].replace(b'VG2,',b'VG2,9,',1))
except ValueError:pass
else:raise AssertionError('bad frame accepted')
(E/'contract-check.json').write_text(json.dumps(dict(passed=True,actual_mission=True,peripherals_mocked=True,hardware_accessed=False,telemetry=parsed),indent=2),encoding='utf-8')
