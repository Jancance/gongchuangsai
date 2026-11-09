"""复现暖VM交接时Safety尚未发布底盘请求的拒绝；真实完整后端C。"""
from pathlib import Path
import json
import shutil
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/no_motion_handoff_20261103'
E = R / 'evidence/integration/2026-11-03/no-motion'
D = E / 'backend-stubs'
D.mkdir(exist_ok=True)
for name in ['task.h', 'safety.h', 'freertos_hooks.h', 'power.h', 'can.h']:
    shutil.copyfile(R / 'scripts/p1_offline_tests/stubs' / name, D / name)
p = D / 'can.h'
t = p.read_text(encoding='utf-8').replace('CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*)', 'CAN1_ReadSpeed(uint8_t,CanRxMsg*,float*)')
t = t.replace('#endif', 'uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t,CanRxMsg*,float*);\n#endif')
p.write_text(t, encoding='utf-8')
unit = r'''
#include "chassis_auto_test.c"
#include <stdio.h>
#include <string.h>
uint32_t test_tick=1000,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t hardware_calls,checks,failures;
static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed line check %u\n",checks);}}
uint32_t StartupAxes_Holding(void){return 1;}
void Power_ReadCommandStatus(PowerCommandStatus *p){memset(p,0,sizeof(*p));p->active=1;p->enable_seen=1;p->enable_tick=0;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *p){(void)a;hardware_calls++;p->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg *p,uint8_t *sign,uint32_t *raw){(void)a;(void)p;hardware_calls++;*sign=0;*raw=0;return 1;}
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t a,CanRxMsg *p,float *rpm){(void)a;(void)p;hardware_calls++;*rpm=0;return 1;}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t v,CanRxMsg *p){(void)a;(void)v;(void)p;hardware_calls++;return 1;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t r,uint8_t ac,CanRxMsg *p){(void)a;(void)d;(void)r;(void)ac;(void)p;hardware_calls++;return 1;}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *p){(void)a;(void)p;hardware_calls++;return 1;}
static void reset(void){
 test_tick=1000;test_depth=test_primask=0;hardware_calls=0;
 g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=g_motion_diag_fault_latched=0;
 g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=g_wheel_test_active=g_wheel_test_cut_vm=0;
 g_safety_ready=g_can_guard_ready=1;ChassisAuto_Init();
}
static uint32_t prepare(ChassisAutoPreparedTranslation *p){
 MotionProfile profile;uint32_t rpm,ms;
 if(!MotionRoute_Build(0,150,200,&profile,&rpm,&ms))return 0;
 return ChassisAuto_PrepareRouteTranslation(0,rpm,ms,150,200,p);
}
int main(void){
 ChassisAutoPreparedTranslation p;uint32_t before,after;
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ck(!s_vm_requested);
 before=ChassisAuto_BeginPreparedTranslation(&p,0);
 ck(!before && g_chassis_auto_result==9 && g_chassis_auto_session==0 && !g_chassis_auto_active && !hardware_calls);
 printf("REPRO result=%u backend_result=%u session=%u internal_vm=%u\n",before,g_chassis_auto_result,g_chassis_auto_session,s_vm_requested);
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));
 /* 与应用修复一致：提交时同步当前请求，不伪造平台健康或清锁存。 */
 ChassisAuto_SafetyUpdate(1);after=ChassisAuto_BeginPreparedTranslation(&p,0);
 ck(after && g_chassis_auto_session==1 && g_chassis_auto_active && !hardware_calls);
 printf("ORDERED result=%u backend_result=%u session=%u internal_vm=%u\n",after,g_chassis_auto_result,g_chassis_auto_session,s_vm_requested);
 for(unsigned fault=0;fault<7;fault++){
  reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));
  if(fault==0)g_rtos_fault=1;if(fault==1)g_safety_fault=1;if(fault==2)g_can_guard_ready=0;
  if(fault==3)g_motion_diag_fault_latched=1;if(fault==4)g_motion_auto_stop_cut_vm=1;
  if(fault==5)g_wheel_test_active=1;if(fault==6)g_motion_auto_stop_state=1;
  ChassisAuto_SafetyUpdate(1);ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && !g_chassis_auto_active && !hardware_calls);
 }
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ChassisAuto_SafetyUpdate(0);
 ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && !g_chassis_auto_active && !hardware_calls);
 reset();ck(ChassisAuto_BeginSequence());ck(prepare(&p));ChassisAuto_SafetyUpdate(1);
 ck(!ChassisAuto_BeginPreparedTranslation(&p,1) && !g_chassis_auto_active && !hardware_calls);
 printf("HANDOFF checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
def extract_function(text, name):
    start = text.index('static uint32_t ' + name + '(')
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]

original = (E / 'before-install/CHASSIS/USER/main.c').read_bytes().decode('gbk')
current = (S / 'CHASSIS/USER/main.c').read_bytes().decode('gbk')
prelude = r'''
#include "motion_control.h"
#define AppCompetitionEntry_MotionAllowed() 0U
static struct {uint32_t state;} g_combined_test;
static uint32_t g_vm_test_request,s_motion_list_submit,s_motion_has_action,s_motion_session_open;
static uint32_t g_translation_hold_selftest=1,s_combined_turn,s_combined_begin_ms,s_combined_powered_segment,s_combined_keep_vm,s_combined_stage_ms;
static uint32_t s_motion_timing_count,s_motion_timing_cursor;
uint32_t MotionGetState(void){return MOTION_IDLE;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return 1;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){(void)p;return 1;}
'''
functions = extract_function(original, 'App_MotionAccept').replace('App_MotionAccept(', 'App_MotionAccept_Before(', 1) + '\n' + extract_function(current, 'App_MotionAccept')
unit = unit.replace('static void reset(void){', prelude + functions + '\nstatic void reset(void){')
unit = unit.replace(' test_tick=1000;test_depth=test_primask=0;hardware_calls=0;', ' test_tick=1000;test_depth=test_primask=0;hardware_calls=0;g_vm_test_request=s_motion_list_submit=s_motion_has_action=s_motion_session_open=g_combined_test.state=0;')
needle = ' printf("HANDOFF checks=%u failures=%u\\n",checks,failures);'
extra = r'''
 reset();test_depth=1;ck(App_MotionAccept_Before());test_depth=0;
 ck(g_vm_test_request==1 && s_vm_requested==0);ck(prepare(&p));ck(!ChassisAuto_BeginPreparedTranslation(&p,0) && g_chassis_auto_result==9);
 reset();test_depth=1;ck(App_MotionAccept());ck(test_depth==1);test_depth=0;
 ck(g_vm_test_request==1 && s_vm_requested==1);ck(prepare(&p));
 test_depth=1;ck(ChassisAuto_BeginPreparedTranslation(&p,0));ck(test_depth==1);test_depth=0;
 ck(g_chassis_auto_active && g_chassis_auto_session==1 && !hardware_calls);
'''
assert needle in unit
unit = unit.replace(needle, extra + needle)
(E / 'handoff-backend-unit.c').write_text(unit, encoding='utf-8')
exe = E / 'handoff-backend.exe'
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-DAPP_MOTOR_OFFLINE_ONLY=0',
       '-I' + str(D), '-I' + str(S / 'CHASSIS/USER'), str(E / 'handoff-backend-unit.c'),
       str(S / 'CHASSIS/USER/chassis_kinematics.c'), str(S / 'CHASSIS/USER/motion_profile.c'), '-lm', '-o', str(exe)]
p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
(E / 'handoff-build.log').write_text(p.stdout + p.stderr, encoding='utf-8')
assert p.returncode == 0, p.stderr
p = subprocess.run([str(exe)], capture_output=True, text=True)
print(p.stdout)
assert p.returncode == 0, p.stdout
(E / 'handoff-check.json').write_text(json.dumps(dict(passed=True,
    actual_complete_backend_source=True, CAN_RTOS_platform_mocked=True,
    hardware_accessed=False, reproduction_and_ordering_fix=p.stdout), indent=2), encoding='utf-8')
