"""实际联合编排+伸缩回零控制器；CAN/PWM为桩，不访问硬件。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/stage1_group_route_20261103'
E = R / 'evidence/integration/2026-11-03/stage1-group-route'
old = (R / 'evidence/integration/2026-11-03/extension-home-only/home-only-unit.c').read_text(encoding='utf-8')
unit = old[:old.index('static void reset(void)')]
dm = r'''
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0f
#define CHASSIS_AUTO_DONE 9U
typedef struct {uint32_t active,state,result,cause,rotation_stop_mask;} ChassisAutoStatus;
static ChassisAutoStatus chassis;
void ChassisAuto_ReadStatus(ChassisAutoStatus *p){*p=chassis;}
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state,g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
float g_dm_posvel_speed_deg_s;
static uint32_t dm_prepared,dm_done,dm_bad,dm_calls,pwm_calls,pwm_ok=1,bad_pwm_params,pwm_cancel,home_cancel,dm_run_calls;
void StartupAxes_Cancel(void);
uint32_t StartupPose_GroupValid(void){return !bad_pwm_params;}
uint32_t StartupPose_GroupCommand(void){pwm_calls++;if(pwm_cancel)StartupAxes_Cancel();return pwm_ok;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 (void)ready;(void)busy;dm_calls++;
 if(g_dm_jog_request==3){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(dm_bad){g_dm_jog_state=8;g_dm_jog_first_fault=14;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=1;}
 if(dm_prepared && !g_dm_startup_go){g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){dm_run_calls++;g_dm_jog_state=dm_done?7:4;g_dm_jog_result=dm_done;g_dm_posvel_holding=dm_done;}
}
#include "startup_group.inc"
'''
unit = unit.replace('#include "extension_home_only.inc"', dm)
unit = unit.replace('starts++;g_extension_home_capture',
                    'starts++;ck(g_startup_group_go && g_dm_startup_go && pwm_calls==1 && g_startup_axes_state==2);g_extension_home_capture')
unit = unit.replace('if(q[0]==0x9a)ck', 'if(q[0]==0x9a && home_cancel)StartupAxes_Cancel();\n if(q[0]==0x9a)ck')
unit += r'''
static void reset(void){
 tick=100;powered=1;command_count=starts=enables=aborts=stops=reads=0;
 bad_params=fail_command=speed=instant_notice=0;home_flags=motor_flags=3;
 g_startup_axes_state=g_startup_axes_cause=g_startup_axes_vm_request=0;
 g_group_extension_state=g_group_extension_cause=g_group_extension_vm_request=0;
 g_extension_home_state=g_extension_home_result=g_extension_home_reported=0;
 g_extension_home_capture=g_extension_home_done_notice=0;
 eh_begin=eh_motion=eh_last=eh_poll=eh_started=eh_seen=eh_stop_pending=eh_phase=0;
 g_startup_group_go=g_startup_group_pwm_result=g_startup_group_release_ms=0;
 sg_begin=sg_last=sg_dm_started=0;
 dm_prepared=dm_done=dm_bad=dm_calls=pwm_calls=bad_pwm_params=pwm_cancel=home_cancel=dm_run_calls=0;pwm_ok=1;
 g_dm_jog_state=g_dm_can2_motor_state=g_dm_jog_result=g_dm_posvel_holding=g_dm_jog_first_fault=0;
 g_dm_jog_request=g_dm_startup_go=0;
}
static void step(void){tick+=20;StartupAxes_Service(1,1);StartupAxes_Safety(1);}
static void steps(unsigned count){for(unsigned i=0;i<count;i++)step();}
static void release(void){dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(starts==1 && pwm_calls==1 && g_startup_group_go);}
int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 ck(!StartupAxes_Begin());StartupAxes_Service(1,1);ck(!starts && !pwm_calls && !dm_calls && !g_startup_axes_vm_request);
#else
 reset();steps(20);ck(!command_count && !pwm_calls && !dm_calls && !g_startup_axes_vm_request);
 ck(StartupAxes_Begin());ck(!StartupAxes_Begin());steps(10);
 ck(eh_phase==5 && !starts && !pwm_calls && !g_dm_startup_go);
 dm_prepared=1;step();ck(starts==1 && pwm_calls==1 && g_startup_axes_state==3);
 ck(g_dm_posvel_speed_deg_s==150.0f);
 dm_done=1;home_flags=7;steps(4);home_flags=3;steps(12);
 ck(g_extension_home_reported && g_group_extension_state==4 && !StartupAxes_Holding());
 steps(80);ck(StartupAxes_Holding() && g_dm_posvel_holding && starts==1 && pwm_calls==1);
 StartupAxes_Cancel();step();ck(g_startup_axes_state==6 && !g_startup_axes_vm_request && !g_dm_posvel_holding);ck(!StartupAxes_Begin());
 reset();bad_params=1;dm_prepared=1;ck(StartupAxes_Begin());steps(5);ck(g_startup_axes_state==5 && !starts && !pwm_calls && !g_startup_axes_vm_request);
 reset();bad_pwm_params=1;ck(!StartupAxes_Begin() && !command_count && !dm_calls && !pwm_calls);
 reset();pwm_ok=0;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(!starts && pwm_calls==1 && !g_dm_startup_go && g_startup_axes_cause==12);
 reset();dm_bad=1;ck(StartupAxes_Begin());steps(4);ck(!starts && !pwm_calls && g_startup_axes_cause==7 && !g_startup_axes_vm_request);
 reset();release();dm_bad=1;step();step();ck(!g_startup_axes_vm_request && aborts==1 && stops==1 && !g_dm_posvel_holding);
 reset();release();StartupAxes_Cancel();step();ck(aborts==1 && stops==1 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();release();StartupAxes_Safety(0);step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && stops==1);
 reset();release();tick+=151;StartupAxes_Safety(1);step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && stops==1);
 reset();dm_prepared=1;fail_command=0x9a;ck(StartupAxes_Begin());steps(8);ck(starts==1 && !g_startup_axes_vm_request && stops==1 && !g_dm_posvel_holding && !dm_run_calls);
 reset();release();home_flags=0x23;steps(5);ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();ck(StartupAxes_Begin());steps(130);ck(g_startup_axes_state==5 && !starts && !pwm_calls);
 reset();release();powered=0;step();step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();pwm_cancel=1;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(!starts && !g_startup_group_go && g_startup_axes_state==6 && !g_startup_axes_vm_request);
 reset();home_cancel=1;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(starts==1 && g_startup_axes_state==6 && !g_startup_axes_vm_request && stops==1 && !g_dm_posvel_holding && !dm_run_calls);
 reset();release();StartupAxes_Cancel();fail_command=0x9c;step();ck(eh_stop_pending && !StartupAxes_Begin());
#endif
 printf("startup group checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
(E / 'group-unit.c').write_text(unit, encoding='utf-8')
reports = []
for offline in [0, 1]:
    source = E / 'group-unit.c'
    if offline:
        offline_unit = unit[:unit.index('static void reset(void)')] + unit[unit.index('int main(void)'):]
        source = E / 'group-offline-unit.c'
        source.write_text(offline_unit, encoding='utf-8')
    exe = E / f'group-{offline}.exe'
    p = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99',
                        f'-DAPP_MOTOR_OFFLINE_ONLY={offline}', '-I' + str(S / 'USER'),
                        '-I' + str(S / 'CHASSIS/USER'), str(source), '-o', str(exe)],
                       capture_output=True, text=True, errors='replace')
    assert p.returncode == 0, p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True)
    print(offline, p.stdout, flush=True)
    assert p.returncode == 0, p.stdout
    reports.append(dict(offline=offline, passed=True, output=p.stdout))
(E / 'group-check.json').write_text(json.dumps(dict(actual_c=True,
    hardware_accessed=False, peripherals_mocked=True, cases=reports), indent=2), encoding='utf-8')
