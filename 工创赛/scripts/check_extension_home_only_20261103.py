"""真实回零控制器的主机故障注入；外设为桩，不访问车辆。"""
from pathlib import Path
import json
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/extension_home_only_20261103'
O = R / 'evidence/integration/2026-11-03/extension-home-only'
UNIT = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vehicle_parameters.h"
typedef struct {uint8_t Data[8];} CanRxMsg;
typedef struct {uint32_t active;} PowerCommandStatus;
static uint32_t tick,powered=1,command_count,starts,enables,aborts,stops,reads;
static uint32_t bad_params,fail_command,home_flags=3,speed,motor_flags=3,instant_notice;
static uint32_t checks,failures;
static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed check %u\n",checks);}}
uint32_t xTaskGetTickCount(void){return tick;}
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=powered;}
uint32_t StartupAxis_Exchange(const uint8_t*,uint8_t,uint8_t,void*);
uint32_t StartupAxis_ReadHomeParameters(uint8_t*);
#include "extension_home_only.inc"
uint32_t StartupAxis_ReadHomeParameters(uint8_t p[17]){
 reads++;memset(p,0,17);p[0]=0x22;p[3]=0;p[4]=bad_params?31:30;
 p[7]=0x27;p[8]=0x10;p[9]=1;p[10]=0x2c;p[11]=3;p[12]=0x20;p[14]=60;p[16]=0x6b;return 1;
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_length,void *out){
 CanRxMsg *p=(CanRxMsg*)out;(void)reply_length;command_count++;memset(p,0,sizeof(*p));p->Data[0]=q[0];p->Data[1]=2;
 if(q[0]==0x9a){starts++;g_extension_home_capture=1;g_extension_home_done_notice=instant_notice;}
 else if(q[0]==0xf3)enables++;
 else if(q[0]==0x9c)aborts++;
 else if(q[0]==0xfe)stops++;
 else if(q[0]==0x3b)p->Data[1]=home_flags;
 else if(q[0]==0x3a)p->Data[1]=motor_flags;
 else if(q[0]==0x35)p->Data[2]=speed;
 else if(q[0]==0x36){p->Data[1]=0;p->Data[5]=123;}
 if(q[0]==0x9a)ck(length==4 && q[1]==2 && q[2]==0 && q[3]==0x6b);
 return fail_command==q[0]?5:1;
}
static void reset(void){
 tick=100;powered=1;command_count=starts=enables=aborts=stops=reads=0;
 bad_params=fail_command=speed=instant_notice=0;home_flags=motor_flags=3;
 g_startup_axes_state=g_startup_axes_cause=g_startup_axes_vm_request=0;
 g_extension_home_state=g_extension_home_result=g_extension_home_reported=0;
 g_extension_home_capture=g_extension_home_done_notice=0;
 eh_begin=eh_motion=eh_last=eh_poll=eh_started=eh_seen=eh_stop_pending=eh_phase=0;
}
static void step(void){tick+=20;StartupAxes_Service(1,1);}
static void prepare(void){ck(StartupAxes_Begin());for(unsigned i=0;i<6;i++)step();}
static void steps(unsigned count){for(unsigned i=0;i<count;i++)step();}
int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 ck(!StartupAxes_Begin());StartupAxes_Service(1,1);ck(!command_count && !g_startup_axes_vm_request);
#else
 reset();steps(10);ck(!command_count && !starts && !g_startup_axes_vm_request);
 prepare();ck(starts==1 && reads==1 && g_startup_axes_state==3);ck(!StartupAxes_Begin());
 home_flags=7;steps(4);home_flags=3;steps(12);
 ck(g_startup_axes_state==4 && g_extension_home_state==8 && g_extension_home_result==1 && g_extension_home_reported==1);
 ck(g_startup_extension_feedback_valid && g_startup_extension_current_deg>12.29f && g_startup_extension_current_deg<12.31f);
 ck(!StartupAxes_Holding());steps(20);ck(starts==1 && g_startup_axes_vm_request==1);
 StartupAxes_Cancel();ck(g_startup_axes_state==6 && !g_startup_axes_vm_request);
 reset();instant_notice=1;prepare();steps(15);ck(g_extension_home_reported==1 && starts==1);
 reset();prepare();steps(20);ck(!g_extension_home_reported && g_startup_axes_state==3);
 tick+=11000;StartupAxes_Service(1,1);ck(g_startup_axes_state==5 && g_extension_home_result==3 && !g_startup_axes_vm_request);
 step();ck(aborts==1 && stops==1 && !eh_stop_pending);
 reset();bad_params=1;ck(StartupAxes_Begin());steps(4);ck(!starts && !enables && g_extension_home_result==7);
 reset();fail_command=0x1a;ck(StartupAxes_Begin());step();ck(!starts && g_extension_home_result==4);
 reset();powered=0;ck(StartupAxes_Begin());tick+=2500;StartupAxes_Service(1,1);ck(!starts && g_extension_home_result==3);
 reset();prepare();StartupAxes_Cancel();step();ck(aborts==1 && stops==1 && !g_startup_axes_vm_request && !g_extension_home_reported);
 ck(!StartupAxes_Begin());
 reset();fail_command=0x9a;prepare();ck(starts==1 && g_startup_axes_state==5);step();ck(aborts==1 && stops==1);
 reset();prepare();tick+=151;StartupAxes_Safety(1);ck(g_extension_home_result==8 && !g_startup_axes_vm_request);step();ck(stops==1);
 reset();prepare();StartupAxes_Safety(0);ck(g_extension_home_result==6 && !g_startup_axes_vm_request);step();ck(stops==1);
 reset();home_flags=7;ck(StartupAxes_Begin());steps(5);ck(!starts && g_extension_home_result==5);
 reset();speed=1;ck(StartupAxes_Begin());steps(6);ck(!starts && g_extension_home_result==5);
 reset();prepare();home_flags=0x23;steps(4);ck(g_extension_home_result==5 && !g_extension_home_reported);
 reset();prepare();home_flags=7;steps(4);home_flags=3;speed=1;steps(12);ck(!g_extension_home_reported);speed=0;steps(15);ck(g_extension_home_reported);
 motor_flags=9;steps(4);ck(g_startup_axes_state==5 && !g_extension_home_reported && !g_startup_axes_vm_request);
 reset();prepare();StartupAxes_Cancel();fail_command=0x9c;step();ck(eh_stop_pending && stops==1 && !StartupAxes_Begin());
#endif
 printf("home-only checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
(O / 'home-only-unit.c').write_text(UNIT, encoding='utf-8')
reports = []
for offline in [0, 1]:
    # The offline preprocessor removes private controller state used by reset.
    unit = UNIT
    if offline:
        unit = unit[:unit.index('static void reset(void)')] + unit[unit.index('int main(void)'):]
        (O / 'home-only-offline-unit.c').write_text(unit, encoding='utf-8')
    exe = O / f'home-only-{offline}.exe'
    source = O / ('home-only-offline-unit.c' if offline else 'home-only-unit.c')
    cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
           '-I' + str(S / 'USER'), '-I' + str(S / 'CHASSIS/USER'), str(source), '-o', str(exe)]
    p = subprocess.run(cmd, capture_output=True, text=True, errors='replace')
    assert p.returncode == 0, p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True)
    print(offline, p.stdout, flush=True)
    assert p.returncode == 0, p.stdout
    reports.append(dict(offline=offline, passed=True, output=p.stdout))
(O / 'home-only-check.json').write_text(json.dumps(dict(actual_c=True,
    hardware_accessed=False, peripherals_mocked=True, cases=reports), indent=2), encoding='utf-8')
