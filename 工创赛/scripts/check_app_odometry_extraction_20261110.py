"""main-B: actual old/new adapter and unchanged odometry C, ARM emulation only."""
from pathlib import Path
import hashlib
import importlib.util
import json
import re
import shutil
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
USER = ROOT / '3.stm32f407工程/CHASSIS/USER'
OUT = ROOT / '工创赛/tmp/odometry_main_b/unit'
EVIDENCE = ROOT / '工创赛/evidence/navigation-planning/2026-11-10/main-b'
BIN = Path(r'D:\Keil_v5\ARM\arm5_compiler\Bin')
BASE = '4a90de1'


def main():
    spec = importlib.util.spec_from_file_location('heading_check', ROOT / '工创赛/scripts/check_heading_source_extraction_20261110.py')
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    old = subprocess.check_output(['git', 'show', f'{BASE}:3.stm32f407工程/CHASSIS/USER/main.c'], cwd=ROOT).decode('gbk')
    for name in ('chassis_odometry.c', 'chassis_odometry.h', 'heading_source.c', 'heading_source.h'):
        original = subprocess.check_output(['git', 'show', f'{BASE}:3.stm32f407工程/CHASSIS/USER/{name}'], cwd=ROOT)
        # Git may normalize line endings; compare semantic bytes after CRLF normalization.
        assert (USER / name).read_bytes().replace(b'\r\n', b'\n') == original.replace(b'\r\n', b'\n'), name

    current = (USER / 'main.c').read_bytes().decode('gbk')
    module = (USER / 'app_odometry.c').read_bytes().decode('gbk')
    before, after, moved = map(helpers.functions, (old, current, module))
    normalized = moved['AppOdometry_Step'].replace('void AppOdometry_Step(const AppOdometryInputs *inputs)', 'static void App_OdometryStep(void)')
    for old_name, new_name in [('g_vm_test_request', '(*inputs->vm_request)'), ('s_can_jog_preparing', '(*inputs->jog_preparing)'), ('g_combined_test.state', '(*inputs->combined_state)'), ('g_combined_test.completed', '(*inputs->combined_completed)'), ('s_route_heading_reference.valid', 'inputs->route_heading_reference->valid'), ('s_route_heading_reference.origin_deg', 'inputs->route_heading_reference->origin_deg'), ('s_runtime_route.committed.id', '(*inputs->runtime_route_id)')]:
        normalized = normalized.replace(new_name, old_name)
    assert normalized == before['App_OdometryStep']
    reset = 's_perimeter_odom_started=s_perimeter_completed_seen=0U;\r\n    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));'
    checked = 0
    for name, body in before.items():
        if name == 'App_OdometryStep':
            continue
        actual = after[name].replace('AppOdometry_ResetRecords();', reset).replace('AppOdometry_Init();', 'ChassisOdometry_Init(&s_odometry);').replace('AppOdometry_Step(&s_odometry_inputs);', 'App_OdometryStep();')
        assert actual == body, name
        checked += 1
    stage = '3.stm32f407工程/CHASSIS/USER/stage1_runtime.inc'
    old_stage = subprocess.check_output(['git', 'show', f'{BASE}:{stage}'], cwd=ROOT)
    assert (ROOT / stage).read_bytes().replace(b'AppOdometry_ResetRecords();', reset.encode()) == old_stage
    assert moved['AppOdometry_Init'] == 'void AppOdometry_Init(void)\r\n{\r\n    ChassisOdometry_Init(&s_odometry);\r\n}'
    assert moved['AppOdometry_ResetRecords'] == 'void AppOdometry_ResetRecords(void)\r\n{\r\n    ' + reset + '\r\n}'
    project = (USER.parents[1] / 'gongxun-f407.uvprojx').read_text('utf-8-sig')
    for name in ('app_odometry.c', 'app_odometry.h'):
        assert project.count(f'<FileName>{name}</FileName>') == 2
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ('app_odometry.c', 'app_odometry.h', 'chassis_odometry.c', 'chassis_odometry.h', 'route_heading_reference.h', 'heading_source.h', 'heading_controller.h'):
        shutil.copyfile(USER / name, OUT / name)
    # Only external I/O is stubbed. The adapter and pure odometry are actual C.
    stubs = {
        'app_config.h': '#define APP_POSITION_DIAGNOSTIC 1\n#define APP_HEADING_SESSION_TEST 1\n#define APP_TEST_MODE 3U\n#define APP_IS_DEBUG 1\n',
        'vehicle_tuning.h': '#ifndef VEHICLE_STAGE1_ENABLE\n#define VEHICLE_STAGE1_ENABLE 1U\n#endif\n',
        'FreeRTOS.h': '#include <stdint.h>\nextern uint32_t clock_ms;\nvoid enter_lock(void);void exit_lock(void);\n#define taskENTER_CRITICAL() enter_lock()\n#define taskEXIT_CRITICAL() exit_lock()\n#define portTICK_PERIOD_MS 1U\n',
        'task.h': '#include "FreeRTOS.h"\n#define xTaskGetTickCount() clock_ms\n',
        'freertos_hooks.h': '#include <stdint.h>\nextern volatile uint32_t g_rtos_fault;\n',
        'can.h': '#include <stdint.h>\nextern volatile uint32_t g_can_ctrl_fault_latched;\n',
        'safety.h': '#include <stdint.h>\nextern volatile uint32_t g_safety_fault,g_wheel_test_active;\n',
        'power.h': '#include <stdint.h>\ntypedef struct {uint32_t active;} PowerCommandStatus;\nvoid Power_ReadCommandStatus(PowerCommandStatus *p);\n',
        'chassis_auto_test.h': '#include <stdint.h>\ntypedef struct {uint32_t mask,sign[4],raw[4],ms[4];} ChassisOdometrySamples;\nextern volatile uint32_t g_chassis_auto_active;\nvoid ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *s);\n',
        'chassis_kinematics.h': '#include <stdint.h>\ntypedef struct {float wheel_diameter_mm;} ChassisGeometry;\nextern ChassisGeometry g_chassis_geometry;\nextern const uint8_t g_chassis_forward_direction[4];\n#define CHASSIS_FORWARD_DISTANCE_SCALE 1.02182540f\n',
    }
    for name, text in stubs.items():
        (OUT / name).write_text(text)
    # Obtain the real dependency shape instead of silently replacing odometry math.
    pure = (USER / 'chassis_odometry.c').read_bytes().decode('gbk')
    assert '#include "chassis_kinematics.h"' in pure
    names = ['s_odometry', 'g_chassis_odometry', 'g_perimeter_records', 's_perimeter_odom_started', 's_perimeter_completed_seen']
    names += ['g_odometry_pose_' + n for n in ('request', 'result', 'x_mm', 'y_mm', 'heading_deg')]
    rename = ''.join(f'#define {name} old_{name}\n' for name in names)
    undo = ''.join(f'#undef {name}\n' for name in names)
    start = old.index('#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST\r\n/* CAN任务独占计算状态')
    end = old.index('\r\nstatic void App_ComboIdlePoll', start)
    harness = r'''
#include <stdint.h>
#include <string.h>
#include "app_odometry.c"
#include "chassis_odometry.c"
uint32_t clock_ms,lock_depth,lock_entries,hook_mode,hook_lock;
uint32_t trace[64],trace_count;
volatile uint32_t g_rtos_fault,g_can_ctrl_fault_latched,g_safety_fault,g_wheel_test_active,g_chassis_auto_active;
volatile uint32_t g_vm_test_request;
uint32_t s_can_jog_preparing;
struct {volatile uint32_t state,completed;} g_combined_test;
struct {struct {uint32_t id;} committed;} s_runtime_route;
RouteHeadingReference s_route_heading_reference;
ChassisGeometry g_chassis_geometry={77.0f};
const uint8_t g_chassis_forward_direction[4]={0U,1U,0U,1U};
AppHeadingSource source_input;
ChassisOdometrySamples samples_input;
uint32_t power_active;
void event(uint32_t tag){if(trace_count<64U)trace[trace_count++]=tag;}
void mutate(void){g_vm_test_request=1U;s_can_jog_preparing=1U;g_combined_test.state=1U;g_combined_test.completed=3U;s_runtime_route.committed.id=1U;s_route_heading_reference.valid=1U;s_route_heading_reference.origin_deg=17.0f;}
void enter_lock(void){lock_depth++;lock_entries++;event(10U+lock_depth);if(hook_mode==4U && lock_entries==hook_lock)mutate();}
void exit_lock(void){event(20U+lock_depth);lock_depth--;}
uint32_t App_HeadingSourceRead(AppHeadingSource *s){event(1U);enter_lock();*s=source_input;exit_lock();if(hook_mode==1U)mutate();return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *p){event(2U);p->active=power_active;if(hook_mode==2U)mutate();}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *s){event(3U);*s=samples_input;if(hook_mode==3U)mutate();}
''' + rename + '''
volatile PerimeterRecord g_perimeter_records[9];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
''' + old[start:end] + '\n' + undo + r'''
static const AppOdometryInputs inputs={&g_vm_test_request,&s_can_jog_preparing,&g_combined_test.state,&g_combined_test.completed,&s_route_heading_reference,&s_runtime_route.committed.id};
volatile uint32_t assertions,cases;
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
static void environment(uint32_t n){
 uint32_t i;
 clock_ms=1000U+n*10U;lock_depth=lock_entries=trace_count=0U;
 g_vm_test_request=(n%29U==0U);s_can_jog_preparing=(n%31U==0U);
 g_combined_test.state=(n%17U==0U)?0U:1U;g_combined_test.completed=n%11U;
 s_route_heading_reference.valid=n%19U!=0U;s_route_heading_reference.origin_deg=0.0f;
 s_runtime_route.committed.id=n%2U;
 g_chassis_auto_active=n%37U==0U;g_wheel_test_active=n%41U==0U;
 g_safety_fault=n%43U==0U;g_rtos_fault=n%47U==0U;g_can_ctrl_fault_latched=n%53U==0U;
 power_active=n%23U==0U;
 memset(&source_input,0,sizeof(source_input));source_input.healthy=n%13U!=0U;source_input.valid_mask=3U;
 source_input.angle_ms=clock_ms-(n%5U==0U?99U:n%5U==1U?100U:n%5U==2U?101U:0U);
 source_input.yaw_deg=0.0f;
 samples_input.mask=n%7U==0U?7U:15U;
 for(i=0;i<4U;i++){samples_input.sign[i]=(i%2U);samples_input.raw[i]=n*100U;samples_input.ms[i]=clock_ms;}
}
static uint32_t equal(void){
 CHECK(memcmp(&s_odometry,&old_s_odometry,sizeof(s_odometry))==0);
 CHECK(memcmp((void*)&g_chassis_odometry,(void*)&old_g_chassis_odometry,sizeof(g_chassis_odometry))==0);
 CHECK(memcmp((void*)g_perimeter_records,(void*)old_g_perimeter_records,sizeof(g_perimeter_records))==0);
 CHECK(s_perimeter_odom_started==old_s_perimeter_odom_started);
 CHECK(s_perimeter_completed_seen==old_s_perimeter_completed_seen);
 CHECK(g_odometry_pose_request==old_g_odometry_pose_request && g_odometry_pose_result==old_g_odometry_pose_result);
 CHECK(lock_depth==0U);return 0U;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t n,k,result,saved_trace[64],saved_count;ChassisOdometry saved;ChassisOdometryStatus published;
 AppOdometry_Init();ChassisOdometry_Init(&old_s_odometry);CHECK(equal()==0U);
 for(n=0;n<512U;n++){
  if(n%32U==0U){AppOdometry_Init();ChassisOdometry_Init(&old_s_odometry);}
  for(k=0;k<5U;k++){
   hook_mode=k;hook_lock=n%2U?2U:3U;
   environment(n);g_odometry_pose_request=old_g_odometry_pose_request=n%3U==0U;
   g_odometry_pose_x_mm=old_g_odometry_pose_x_mm=42.0f;g_odometry_pose_y_mm=old_g_odometry_pose_y_mm=-13.0f;
   g_odometry_pose_heading_deg=old_g_odometry_pose_heading_deg=0.0f;
   AppOdometry_Step(&inputs);saved_count=trace_count;memcpy(saved_trace,trace,sizeof(trace));
   environment(n);App_OdometryStep();
   CHECK(trace_count==saved_count && memcmp(trace,saved_trace,saved_count*4U)==0);
   result=equal();CHECK(result==0U);cases++;
  }
  if(n%11U==0U){
   saved=s_odometry;published=g_chassis_odometry;
   AppOdometry_ResetRecords();old_s_perimeter_odom_started=old_s_perimeter_completed_seen=0U;
   memset((void*)old_g_perimeter_records,0,sizeof(old_g_perimeter_records));
   CHECK(memcmp(&saved,&s_odometry,sizeof(saved))==0);CHECK(memcmp(&published,(void*)&g_chassis_odometry,sizeof(published))==0);
   CHECK(equal()==0U);CHECK(g_odometry_pose_request==old_g_odometry_pose_request);
  }
 }
 /* Explicit clean acceptance, 99/100/101ms boundary, records 0..8 and repeated completion. */
 hook_mode=0U;AppOdometry_Init();ChassisOdometry_Init(&old_s_odometry);AppOdometry_ResetRecords();
 old_s_perimeter_odom_started=old_s_perimeter_completed_seen=0U;memset((void*)old_g_perimeter_records,0,sizeof(old_g_perimeter_records));
 for(n=0;n<24U;n++){
  environment(100U+n);g_vm_test_request=s_can_jog_preparing=power_active=g_chassis_auto_active=g_wheel_test_active=0U;
  g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0U;source_input.healthy=1U;source_input.angle_ms=clock_ms;
  s_route_heading_reference.valid=1U;g_combined_test.state=1U;g_combined_test.completed=n<6U?0U:(n-6U)/2U+1U;
  samples_input.mask=15U;g_odometry_pose_request=old_g_odometry_pose_request=n<4U;
  if(n<4U)source_input.angle_ms=clock_ms-(n==0U?0U:98U+n);
  AppOdometry_Step(&inputs);App_OdometryStep();CHECK(equal()==0U);
  if(n<4U)CHECK(g_odometry_pose_result==(n<=1U?1U:2U));
  cases++;
 }
 for(n=0;n<9U;n++)CHECK(g_perimeter_records[n].captured==1U);
 return 0U;
}
'''
    (OUT / 'unit.c').write_bytes(harness.encode('gbk'))
    (OUT / 'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
    results = []
    for stage1 in (0, 1):
        commands = [[str(BIN / 'armcc.exe'), '--cpu', 'Cortex-M3', '--c99', '--split_sections', '-O1', f'-DVEHICLE_STAGE1_ENABLE={stage1}', '-I', '.', '-c', 'unit.c', '-o', 'unit.o'], [str(BIN / 'armlink.exe'), '--entry', 'test_entry', '--scatter', 'unit.sct', 'unit.o', '--library_type=microlib', '-o', 'unit.axf']]
        for command in commands:
            run = subprocess.run(command, cwd=OUT, capture_output=True, timeout=60)
            assert run.returncode == 0, (run.stdout + run.stderr).decode(errors='replace')
        sys.path.insert(0, str(ROOT / '工创赛/tmp/position_alignment_deps'))
        import unicorn as uc
        from unicorn import arm_const as reg
        blob = (OUT / 'unit.axf').read_bytes()
        elf = struct.unpack_from('<16sHHIIIIIHHHHHH', blob)
        machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
        machine.mem_map(0x08000000, 0x100000)
        machine.mem_map(0x20000000, 0x20000)
        for i in range(elf[10]):
            kind, offset, address, _, size, _, _, _ = struct.unpack_from('<IIIIIIII', blob, elf[5] + i * elf[9])
            if kind == 1 and size:
                machine.mem_write(address, blob[offset:offset + size])
        stop = 0x080FF000
        machine.reg_write(reg.UC_ARM_REG_SP, 0x2001FFF0)
        machine.reg_write(reg.UC_ARM_REG_LR, stop | 1)
        machine.emu_start(elf[4] | 1, stop, timeout=30000000, count=400000000)
        assert machine.reg_read(reg.UC_ARM_REG_PC) == stop, 'Emulator did not complete'
        result = machine.reg_read(reg.UC_ARM_REG_R0)
        assert result == 0, f'C assertion failed at generated unit.c line {result}'
        symbols = subprocess.check_output([str(BIN / 'fromelf.exe'), '--text', '-s', 'unit.axf'], cwd=OUT).decode(errors='replace')
        def value(name):
            address = int(re.search(r'\b' + name + r'\s+(0x[0-9a-fA-F]+)', symbols)[1], 16)
            return struct.unpack('<I', machine.mem_read(address, 4))[0]
        results.append(dict(stage1=stage1, cases=value('cases'), assertions=value('assertions')))
    report = dict(ok=True, baseline=BASE, active_user=str(USER), equivalent_main_functions=checked, reset_call_sites=['App_RuntimeRouteInstall', 'App_Stage1Rearm'], arm_results=results, module_sha256=hashlib.sha256((USER / 'app_odometry.c').read_bytes()).hexdigest(), hardware_accessed=False, limits='Actual old/new adapter plus unchanged pure odometry C with I/O and critical-section hooks; not RTOS scheduling, dynamics, timing, stack or vehicle acceptance. No MATLAB dynamics needed for behavior-preserving extraction.')
    (EVIDENCE / 'app_odometry_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), 'utf-8')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
