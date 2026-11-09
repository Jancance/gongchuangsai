"""真实后端复现末端±2RPM反馈；模型与实车证据分开。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/endpoint_integer_feedback_20261103'
E = R / 'evidence/integration/2026-11-03/no-motion/endpoint'
parent = E.parent
base = (parent / 'handoff-backend-unit.c').read_text(encoding='utf-8')
base = base[:base.index('int main(void)')]
unit = base + r'''
static void seed(int32_t target,float speed){
 ChassisAutoPreparedTranslation p;reset();
 ck(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1);ck(prepare(&p));ck(ChassisAuto_BeginPreparedTranslation(&p,0));
 test_tick=140723;s_run_tick=138800;s_run_started=1;g_chassis_auto_state=CHASSIS_AUTO_START;
 s_translation_base_rpm=1;s_distance_progress_mm=145.572f;
 s_distance_reference=(MotionProfileSample){150,0,0,0};
 DirectionGuard_Reset(&s_direction_guard[0]);DirectionGuard_Ack(&s_direction_guard[0],target,140712);
 g_chassis_auto_driver_rpm[0]=speed;s_speed_tick[0]=test_tick;
}
int main(void){
 seed(20,-2);Auto_CheckWheelSpeedLocked(0,test_tick);
#if BEFORE
 ck(!g_chassis_auto_active && g_chassis_auto_cause==8 && g_chassis_auto_fault_diagnostic.reason==10);
 printf("OLD reproduced detail=%u target=%ld feedback=%ld\n",g_chassis_auto_fault_diagnostic.reason,(long)g_chassis_auto_fault_diagnostic.target_decirpm,(long)g_chassis_auto_fault_diagnostic.measured_decirpm);
#else
 ck(g_chassis_auto_active && s_direction_guard[0].pending && s_direction_guard[0].reverse_samples==1);
 test_tick+=100;g_chassis_auto_driver_rpm[0]=1;s_speed_tick[0]=test_tick;Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(g_chassis_auto_active && !s_direction_guard[0].pending);
 seed(20,-2);Auto_CheckWheelSpeedLocked(0,test_tick);test_tick+=100;s_speed_tick[0]=test_tick;Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==11);
 seed(20,-3);Auto_CheckWheelSpeedLocked(0,test_tick);ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==10);
 seed(30,-1);Auto_CheckWheelSpeedLocked(0,test_tick);ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==10);
 seed(20,-1);s_distance_progress_mm=139;Auto_CheckWheelSpeedLocked(0,test_tick);ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==10);
 seed(20,-1);s_distance_reference.acceleration_mm_s2=1;Auto_CheckWheelSpeedLocked(0,test_tick);ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==10);
 seed(20,-2);Auto_CheckWheelSpeedLocked(0,test_tick);test_tick+=599;Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(g_chassis_auto_active && s_direction_guard[0].reverse_samples==1);test_tick++;Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==11);
 for(int sign=-1;sign<=1;sign+=2){
  ChassisDirectionGuard d;DirectionGuard_Reset(&d);DirectionGuard_Ack(&d,sign*20,1000);
  ck(!DirectionGuard_Check(&d,1600,1600,-sign*20,1));
  DirectionGuard_Ack(&d,sign*20,1700);ck(d.pending && d.first_reverse_ms==1600);
  ck(!DirectionGuard_Check(&d,1800,1600,-sign*20,1) && d.reverse_samples==1);
  ck(DirectionGuard_Check(&d,2200,2200,sign*10,1)==DIRECTION_PERSISTENT);
 }
#endif
 ck(!hardware_calls && !test_depth);
 printf("ENDPOINT checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
path = E / 'endpoint-contract-unit.c'
path.write_text(unit, encoding='utf-8')
reports = []
for before in [1, 0]:
    # Place the old header first by copying only the backend C into the fixture
    # folder; its other headers still resolve to the actual staged source tree.
    case = E / ('old-source' if before else 'new-source')
    case.mkdir(exist_ok=True)
    (case / 'chassis_auto_test.c').write_bytes((S / 'CHASSIS/USER/chassis_auto_test.c').read_bytes())
    guard = E / 'before-install/CHASSIS/USER/chassis_direction_guard.h' if before else S / 'CHASSIS/USER/chassis_direction_guard.h'
    (case / 'chassis_direction_guard.h').write_bytes(guard.read_bytes())
    exe = E / f'endpoint-contract-{before}.exe'
    cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-DAPP_MOTOR_OFFLINE_ONLY=0', f'-DBEFORE={before}',
           '-I' + str(case), '-I' + str(parent / 'backend-stubs'), '-I' + str(S / 'CHASSIS/USER'), str(path),
           str(S / 'CHASSIS/USER/chassis_kinematics.c'), str(S / 'CHASSIS/USER/motion_profile.c'), '-lm', '-o', str(exe)]
    p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    assert p.returncode == 0, p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True)
    print(p.stdout, flush=True)
    assert p.returncode == 0, p.stdout
    reports.append(dict(before=bool(before), passed=True, output=p.stdout))
(E / 'contract-check.json').write_text(json.dumps(dict(passed=True, cases=reports,
    complete_actual_backend_with_seeded_fault_snapshot=True, hardware_accessed=False,
    no_mechanical_root_cause_claim=True), indent=2), encoding='utf-8')
