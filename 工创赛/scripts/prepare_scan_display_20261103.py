"""Prepare and verify restoring real stage-1 scan/display; no hardware access."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

R = Path(__file__).resolve().parents[1]
A = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S = R / 'tmp/scan_display_20261103'
E = R / 'evidence/integration/2026-11-03/scan-display'
E.mkdir(parents=True, exist_ok=True)
assert not S.exists(), 'Use a fresh stage; do not overwrite previous evidence'
shutil.copytree(A, S, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '.git'))
p = S / 'USER/vehicle_parameters.h'
before = p.read_bytes()
old = '#define VEHICLE_SCAN_BYPASS 1U      /* 当前按用户要求跳过扫码；不伪造任务码/计数。 */'
new = '#define VEHICLE_SCAN_BYPASS 0U      /* 恢复真实扫码：本轮合法四段码，停稳后显示；不造码。 */'
assert before.decode('utf-8').count(old) == 1
p.write_bytes(before.decode('utf-8').replace(old, new).encode('utf-8'))
(E / 'before-install/USER').mkdir(parents=True, exist_ok=True)
(E / 'before-install/USER/vehicle_parameters.h').write_bytes(before)

# Reuse the existing real mission/route fixture, replace only its scenario body.
base = (R / 'evidence/integration/2026-11-03/stage1-group-route/stage1-group-mission-unit.c').read_text(encoding='utf-8')
lo = base.index('static void test_mission(void)')
end = base.index('{', lo) + 1
depth = 1
while depth:
    depth += (base[end] == '{') - (base[end] == '}')
    end += 1
tests = r'''
static void fresh_code(const char *code){
 strcpy(SAOMA_Data,code);g_saoma_accept_count++;g_saoma_last_result=1;
 g_saoma_frame_start_ms=tick;g_saoma_frame_end_ms=tick;
}
static void begin_real(void){
 reset_mission();fixture_axes_vm=0;g_safety_ready=1;
 g_vehicle_key=1;VehicleMission_Service();
 ck(g_vehicle_state==1 && !g_vehicle_scan_bypassed && !g_combined_test.request);
}
static void axes_ready(void){
 g_startup_axes_state=4;fixture_axes_vm=1;VehicleMission_Service();
 ck(g_vehicle_state==2 && g_combined_test.request);
}
static void test_mission(void){
 ck(VEHICLE_SCAN_BYPASS==0 && VEHICLE_RUN_ALL==0 && VEHICLE_TEST_STAGE==1);
 /* Both zones: wait at the endpoint until a new valid code arrives. */
 for(uint32_t zone=1;zone<=2;zone++){
  begin_real();s_vehicle_zone=zone;axes_ready();
  ck(s_motion_actions[0].distance_mm==150 && s_motion_actions[0].reference_mm_s==200);
  ck(s_motion_actions[1].distance_mm==1050 && s_motion_actions[1].reference_mm_s==400);
  ck(s_motion_actions[1].kind==(zone==1?MOTION_LEFT_DISTANCE:MOTION_RIGHT_DISTANCE));
  tick=1000;move_done();VehicleMission_Service();ck(g_vehicle_state==2 && !display_calls);
  fresh_code("156+123+516+231");VehicleMission_Service();
  ck(g_vehicle_state==4 && display_calls==1 && !strcmp(displayed,"156+123+516+231"));
  ck(g_vehicle_stage==1 && g_vehicle_stage_mask==1 && !actor_calls && StartupAxes_Holding());
  VehicleMission_Service();ck(display_calls==1 && !g_vehicle_correct_pick_count && !g_vehicle_correct_place_count);
 }
 /* Scan during preparation: retain it, but do not display before stopped. */
 begin_real();tick=110;fresh_code("156+123+516+231");VehicleMission_Service();
 ck(s_vehicle_code_valid && !display_calls);axes_ready();ck(g_stage1_stop_requested);
 move_done();VehicleMission_Service();ck(g_vehicle_state==4 && display_calls==1);
 /* En-route scan asks for ordered stop; backend must independently confirm it. */
 begin_real();axes_ready();tick=120;fresh_code("156+123+516+231");VehicleMission_Service();
 ck(g_stage1_stop_requested && !display_calls && g_vehicle_state==2);
 move_done();VehicleMission_Service();ck(g_vehicle_state==4 && display_calls==1);
 /* Old frame, stale frame and semantically invalid frame cannot complete. */
 begin_real();axes_ready();tick=1000;move_done();
 fresh_code("156+123+516+231");g_saoma_frame_start_ms=90;VehicleMission_Service();ck(!display_calls && !s_vehicle_code_valid);
 fresh_code("156+123+516+231");tick+=VEHICLE_STAGE1_SCAN_FRESH_MS;VehicleMission_Service();ck(!display_calls && !s_vehicle_code_valid);
 fresh_code("156+123+516+236");VehicleMission_Service();ck(!display_calls && !s_vehicle_code_valid);
 fresh_code("156+123+516+231");display_result=2;VehicleMission_Service();
 ck(g_vehicle_state==6 && g_vehicle_cause==9 && !s_vehicle_display_done);
 /* Cancellation and deadline still stop the group. */
 begin_real();g_vehicle_key=1;VehicleMission_Service();ck(g_vehicle_state==7 && !fixture_axes_vm);
 begin_real();tick+=VEHICLE_STAGE_TIMEOUT_MS;VehicleMission_Service();ck(g_vehicle_state==6 && g_vehicle_cause==5);
}
'''
source = E / 'scan-display-unit.c'
source.write_text(base[:lo] + tests + base[end:], encoding='utf-8')
exe = E / 'scan-display-unit.exe'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-DAPP_MOTOR_OFFLINE_ONLY=0',
       '-I' + str(S / 'CHASSIS/USER'), str(source)]
cmd += [str(S / 'CHASSIS/USER' / name) for name in
        ['field_map.c', 'app_entry.c', 'motion_profile.c', 'chassis_kinematics.c']]
cmd += ['-lm', '-o', str(exe)]
result = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
assert result.returncode == 0, result.stderr
result = subprocess.run([str(exe)], capture_output=True, text=True)
(E / 'host-check.log').write_text(result.stdout + result.stderr, encoding='utf-8')
assert result.returncode == 0, result.stdout
print(result.stdout)
manifest = dict(actual=str(A), stage=str(S), changed=['USER/vehicle_parameters.h'],
                before_sha256=hashlib.sha256(before).hexdigest(),
                after_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                software_checks_passed=True, peripherals_mocked=True,
                hardware_accessed=False, installed=False, flashed=False)
(E / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
