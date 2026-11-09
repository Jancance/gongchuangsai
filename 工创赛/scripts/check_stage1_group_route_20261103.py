"""实际第1段编排：联合完成前不排队底盘，双起点与停车持位。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/stage1_group_route_20261103'
E = R / 'evidence/integration/2026-11-03/stage1-group-route'
OLD = R / 'evidence/integration/2026-11-03/early-scan-startup'
unit = (OLD / 'mission-unit.c').read_text(encoding='utf-8')
unit = unit.replace('static uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4;}',
                    'static uint32_t fixture_axes_vm;\nstatic uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4 && fixture_axes_vm;}')
unit = unit.replace('static uint32_t StartupAxes_Begin(void){g_startup_axes_state=4;return 1;}',
                    'static uint32_t StartupAxes_Begin(void){g_startup_axes_state=1;fixture_axes_vm=1;fixture_power.active=1;return 1;}')
unit = unit.replace('static void StartupAxes_Cancel(void){g_startup_axes_state=6;}',
                    'static void StartupAxes_Cancel(void){g_startup_axes_state=6;fixture_axes_vm=0;}')
start = unit.index('static void test_mission(void)')
end = unit.index('{', start) + 1
depth = 1
while depth:
    depth += (unit[end] == '{') - (unit[end] == '}')
    end += 1
tests = r'''
static void test_mission(void){
 for(uint32_t zone=1;zone<=2;zone++){
  reset_mission();fixture_axes_vm=0;g_safety_ready=1;g_radar_zone_selected=zone;
  g_vehicle_key=1;VehicleMission_Service();
  ck(g_vehicle_state==1 && g_vehicle_stage==1 && g_vehicle_run_id==1 && !g_combined_test.request);
  g_startup_axes_state=2;VehicleMission_Service();ck(g_vehicle_state==1 && !g_combined_test.request);
  g_startup_axes_state=3;VehicleMission_Service();ck(g_vehicle_state==1 && !g_combined_test.request);
  g_startup_axes_state=4;fixture_axes_vm=0;VehicleMission_Service();ck(g_vehicle_state==1 && !g_combined_test.request);
  fixture_axes_vm=1;VehicleMission_Service();
  ck(g_vehicle_state==2 && g_combined_test.request==1 && s_motion_action_count==2 && !g_vehicle_stage_mask);
  ck(s_motion_actions[0].kind==MOTION_FORWARD_DISTANCE && s_motion_actions[0].distance_mm==150.0f && s_motion_actions[0].reference_mm_s==200.0f);
  ck(s_motion_actions[1].kind==(zone==1?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE) && s_motion_actions[1].distance_mm==1050.0f && s_motion_actions[1].reference_mm_s==400.0f);
  ck(g_vehicle_scan_bypassed && !display_calls && !g_vehicle_correct_pick_count && !g_vehicle_correct_place_count);
  g_combined_test.request=0;move_done();VehicleMission_Service();
  ck(g_vehicle_state==4 && g_vehicle_stage_mask==1 && g_vehicle_stage==1 && !actor_calls && fixture_power.active && StartupAxes_Holding());
 }
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_vehicle_key=1;VehicleMission_Service();
 ck(g_vehicle_state==7 && stop_calls>0 && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_safety_fault=1;VehicleMission_Service();
 ck(g_vehicle_state==6 && !g_combined_test.request && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_startup_axes_state=5;VehicleMission_Service();
 ck(g_vehicle_state==6 && !g_combined_test.request && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_startup_axes_state=4;VehicleMission_Service();
 ck(g_vehicle_state==2);g_startup_axes_state=5;VehicleMission_Service();ck(g_vehicle_state==6 && stop_calls>0 && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_startup_axes_state=4;VehicleMission_Service();
 g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==7 && stop_calls>0 && !fixture_axes_vm);
 reset_mission();g_vehicle_key=1;VehicleMission_Service();g_startup_axes_state=4;VehicleMission_Service();
 ready=0;g_combined_test.request=0;move_done();g_vehicle_key=1;g_vehicle_state=4;
 VehicleMission_Service();ck(g_vehicle_state==0 && g_vehicle_cause==1 && !g_combined_test.request);
}
'''
unit = unit[:start] + tests + unit[end:]
source = E / 'stage1-group-mission-unit.c'
source.write_text(unit, encoding='utf-8')
exe = E / 'stage1-group-mission.exe'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-DAPP_MOTOR_OFFLINE_ONLY=0',
       '-I' + str(S / 'CHASSIS/USER'), str(source)]
cmd += [str(S / 'CHASSIS/USER' / p) for p in
        ['field_map.c', 'app_entry.c', 'motion_profile.c', 'chassis_kinematics.c']]
cmd += ['-lm', '-o', str(exe)]
p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
assert p.returncode == 0, p.stderr
p = subprocess.run([str(exe)], capture_output=True, text=True)
print(p.stdout)
assert p.returncode == 0, p.stdout
(E / 'route-check.json').write_text(json.dumps(dict(passed=True, actual_c=True,
    peripherals_mocked=True, hardware_accessed=False, output=p.stdout), indent=2), encoding='utf-8')

# Execute the actual power decision with the actual group hold predicate.
unit = (E / 'group-unit.c').read_text(encoding='utf-8')
unit = unit[:unit.index('int main(void)')]
safety = (S / 'CHASSIS/SYSTEM/SAFETY/safety.c').read_bytes().decode('gbk')
lo = safety.index('    if ((g_safety_ready == 1U) && (vm_request == 1U ||')
hi = safety.index('    /*', lo)
unit += r'''
static uint32_t g_safety_ready=1,vm_request,power_enabled,g_motion_auto_stop_cut_vm,g_wheel_test_cut_vm,g_chassis_auto_cut_vm;
static void Power_Enable(void){power_enabled=1;}
static void Power_Disable(void){power_enabled=0;}
static void actual_power_decision(void){
''' + safety[lo:hi] + r'''
}
int main(void){
 reset();release();dm_done=1;home_flags=7;steps(4);home_flags=3;steps(100);ck(StartupAxes_Holding());
 chassis=(ChassisAutoStatus){0,9,1,0,15};g_chassis_auto_cut_vm=1;
 actual_power_decision();ck(power_enabled && StartupAxes_NormalStopHold());
 chassis.cause=1;actual_power_decision();ck(!power_enabled);chassis.cause=0;
 chassis.active=1;actual_power_decision();ck(!power_enabled);chassis.active=0;
 chassis.rotation_stop_mask=7;actual_power_decision();ck(!power_enabled);chassis.rotation_stop_mask=15;
 chassis.state=8;actual_power_decision();ck(!power_enabled);chassis.state=9;
 g_safety_ready=0;actual_power_decision();ck(!power_enabled);g_safety_ready=1;
 g_motion_auto_stop_cut_vm=1;actual_power_decision();ck(!power_enabled);g_motion_auto_stop_cut_vm=0;
 g_wheel_test_cut_vm=1;actual_power_decision();ck(!power_enabled);g_wheel_test_cut_vm=0;
 StartupAxes_Cancel();actual_power_decision();ck(!power_enabled);
 printf("group route hold checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
source = E / 'stage1-hold-unit.c'
source.write_text(unit, encoding='utf-8')
exe = E / 'stage1-hold.exe'
p = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-DAPP_MOTOR_OFFLINE_ONLY=0',
                    '-I' + str(S / 'USER'), '-I' + str(S / 'CHASSIS/USER'), str(source), '-o', str(exe)],
                   capture_output=True, text=True, errors='replace')
assert p.returncode == 0, p.stderr
p = subprocess.run([str(exe)], capture_output=True, text=True)
print(p.stdout)
assert p.returncode == 0, p.stdout
(E / 'hold-check.json').write_text(json.dumps(dict(passed=True, actual_c=True,
    peripherals_mocked=True, hardware_accessed=False, output=p.stdout), indent=2), encoding='utf-8')
