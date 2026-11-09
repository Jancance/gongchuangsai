"""Execute staged lift_home.c against mocked CAN/RTOS; does not touch hardware."""
from pathlib import Path
import subprocess
import shutil
import argparse

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--project', type=Path, default=ROOT / 'tmp/lift_home_wait_speed_20261101/project')
parser.add_argument('--host', type=Path, default=ROOT / 'tmp/lift_home_wait_speed_20261101/host')
args = parser.parse_args()
STAGE = args.project
HOST = args.host
HOST.mkdir(parents=True, exist_ok=True)
for name in ['stm32f4xx.h', 'stm32f4xx_can.h', 'FreeRTOS.h', 'task.h', 'can.h']:
    shutil.copy2(ROOT / 'tmp/arm_x_20261101/state-host' / name, HOST / name)
source = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lift_home.h"
#include "lift.h"
#include "FreeRTOS.h"
static TickType_t now;
static unsigned starts, aborts, stops, enables, fault, communication, complete;
static unsigned options, start_fail, seen_running=1;
volatile uint32_t g_lift_motion_request, g_lift_motion_state, g_lift_motion_result;
volatile uint32_t g_can_lift_home_done_notice;
LiftHomeParameters g_lift_home_params;
TickType_t xTaskGetTickCount(void) {return now;}
uint32_t Lift_ReadHomeParametersNow(void) {return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a, CanRxMsg*r) {
 assert(a==5);memset(r,0,sizeof(*r));r->Data[2]=(uint8_t)options;return 1;
}
uint32_t CAN1_ReadStatus(uint8_t a, CanRxMsg*r) {
 assert(a==5);memset(r,0,sizeof(*r));r->Data[1]=1;return 1;
}
uint32_t CAN1_LiftExchange(const uint8_t*q,uint8_t n,CanRxMsg*r) {
 (void)r;
 if(q[0]==0xFE){assert(n==4);stops++;return 1;}
 assert(q[0]==0xF3&&q[2]==1&&n==5);enables++;return 1;
}
uint32_t CAN1_StartLiftHome(CanRxMsg*r) {
 (void)r;starts++;g_can_lift_home_done_notice=0;return start_fail?5:1;
}
uint32_t CAN1_AbortLiftHome(CanRxMsg*r) {(void)r;aborts++;return 1;}
uint32_t CAN1_ReadLiftHomeStatus(CanRxMsg*r) {
 memset(r,0,sizeof(*r));
 if(g_lift_home_state==4){r->Data[1]=3;return 1;}
 if(communication)return 5;
 r->Data[1]=fault?11:((complete||!seen_running)?3:7);return 1;
}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v) {
 assert(a==5);(void)r;*v=0;return 1;
}
static void step(unsigned count) {while(count--){now+=10;LiftHome_Service(1);}}
int main(int argc,char**argv) {
 const char*s=argv[1];(void)argc;
 g_lift_home_params.direction=1;
 g_lift_home_params.rpm=LIFT_HOME_EXPECTED_RPM;
 g_lift_home_params.timeout_ms=LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS;
 g_lift_home_params.detect_rpm=300;g_lift_home_params.detect_current_ma=800;
 g_lift_home_params.detect_time_ms=60;g_lift_home_params.auto_home=0;
 if(!strcmp(s,"idle")){step(100000);assert(starts==0&&enables==0);puts(s);return 0;}
 if(!strcmp(s,"old_speed"))g_lift_home_params.rpm=30;
 if(!strcmp(s,"old_timeout"))g_lift_home_params.timeout_ms=10000;
 if(!strcmp(s,"mismatched_timeout"))g_lift_home_params.timeout_ms=LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS+1;
 if(!strcmp(s,"observed_truncation"))g_lift_home_params.timeout_ms=48928;
 if(!strcmp(s,"zero_timeout"))g_lift_home_params.timeout_ms=0;
 if(!strcmp(s,"wrong_direction"))g_lift_home_params.direction=0;
 if(!strcmp(s,"changed_current"))g_lift_home_params.detect_current_ma=1000;
 if(!strcmp(s,"auto_home"))g_lift_home_params.auto_home=1;
 if(!strcmp(s,"wrong_firmware"))options=2;
 if(!strcmp(s,"motion_busy"))g_lift_motion_state=3;
 if(!strcmp(s,"start_fail"))start_fail=1;
 if(!strcmp(s,"stale_notice")){seen_running=0;g_can_lift_home_done_notice=1;}
 g_lift_home_request=1;step(1);
 if(!strcmp(s,"prepare_timeout")) {
  now+=2600;step(1);assert(g_lift_home_state==9&&g_lift_home_result==3&&starts==0);puts(s);return 0;
 }
 step(10);
 if(!strcmp(s,"old_speed")||!strcmp(s,"old_timeout")||!strcmp(s,"mismatched_timeout")||!strcmp(s,"observed_truncation")||!strcmp(s,"zero_timeout")||
    !strcmp(s,"wrong_direction")||!strcmp(s,"changed_current")||!strcmp(s,"auto_home")) {
  assert(g_lift_home_state==9&&g_lift_home_result==7&&starts==0);puts(s);return 0;
 }
 if(!strcmp(s,"wrong_firmware")||!strcmp(s,"motion_busy")||!strcmp(s,"start_fail")) {
  assert(g_lift_home_state==9);
  if(start_fail)assert(starts==1&&aborts==1&&stops==1);
  else assert(starts==0);
  puts(s);return 0;
 }
 assert(starts==1&&LiftHome_IsBusy());
 if(!strcmp(s,"long_wait")||!strcmp(s,"repeat")||!strcmp(s,"stale_notice")) {
  for(unsigned i=0;i<40000;i++) {
   if(!strcmp(s,"repeat"))g_lift_home_request=1;
   step(1);
  }
  assert(LiftHome_IsBusy()&&starts==1&&aborts==0&&g_lift_home_result==0&&g_lift_home_reported==0);
  if(!strcmp(s,"stale_notice")){puts(s);return 0;}
 }
 if(!strcmp(s,"wrap")) {now=0xFFFFFFF0U;step(20);assert(LiftHome_IsBusy()&&aborts==0);}
 if(!strcmp(s,"stop")||!strcmp(s,"repeat")||!strcmp(s,"long_wait")||!strcmp(s,"wrap")) {
  g_lift_home_request=2;step(1);
  assert(g_lift_home_state==10&&g_lift_home_result==2&&aborts==1&&stops==1);
 } else if(!strcmp(s,"motion_stop")) {
  g_lift_motion_request=3;step(1);assert(g_lift_home_state==10&&stops==1);
 } else if(!strcmp(s,"fault")) {
  fault=1;step(10);assert(g_lift_home_state==9&&g_lift_home_result==5&&aborts==1&&stops==1);
 } else if(!strcmp(s,"communication")) {
  communication=1;step(10);assert(g_lift_home_state==9&&g_lift_home_result==4&&aborts==1);
 } else if(!strcmp(s,"power_loss")) {
  LiftHome_Service(0);assert(g_lift_home_state==9&&g_lift_home_result==6&&g_lift_home_reported==0);
 } else if(!strcmp(s,"ordinary_request")) {
  g_lift_motion_request=2;step(1);assert(LiftHome_IsBusy()&&g_lift_motion_request==0&&g_lift_motion_result==6);
 } else {
  assert(!strcmp(s,"complete"));complete=1;step(20);
  assert(g_lift_home_state==8&&g_lift_home_result==1&&g_lift_home_reported==1&&stops==0);
 }
 puts(s);return 0;
}
'''
(HOST / 'test.c').write_text(source, encoding='utf-8')
cmd = ['C:/mingw/mingw64/bin/gcc.exe', '-std=c99', '-Wall', '-Wextra', '-Werror',
       '-I' + str(HOST), '-I' + str(STAGE / 'HARDWARE/LIFT'),
       str(HOST / 'test.c'), str(STAGE / 'HARDWARE/LIFT/lift_home.c'), '-o', str(HOST / 'test.exe')]
r = subprocess.run(cmd, capture_output=True, text=True)
assert r.returncode == 0, r.stdout + r.stderr
cases = ['idle','old_speed','old_timeout','mismatched_timeout','observed_truncation','zero_timeout','wrong_direction','changed_current',
         'auto_home','wrong_firmware','motion_busy','start_fail','prepare_timeout','long_wait',
         'repeat','stale_notice','wrap','stop','motion_stop','fault','communication','power_loss',
         'ordinary_request','complete']
for case in cases:
    r = subprocess.run([str(HOST / 'test.exe'), case], capture_output=True, text=True)
    assert r.returncode == 0, case + '\n' + r.stdout + r.stderr
print('PASS: ' + str(len(cases)) + ' actual C home-state cases: ' + ', '.join(cases))
