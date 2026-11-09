"""验证实际候选C状态机及main调度包装；不接硬件。"""
from pathlib import Path
import hashlib
import json
import subprocess
import re

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/carrier_pb1_20261030'
STAGE=BASE/'project'
HOST=BASE/'host-check'
EV=ROOT/'evidence/p2-arm/2026-10-30/carrier-pb1'

MODEL=r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "carrier.h"
#include "carrier_config.h"
static unsigned init_ok=1,write_ok=1,writes,offs,last_pulse,now;
uint32_t CarrierPwm_Init(void){return init_ok;}
uint32_t CarrierPwm_Write(uint32_t p){writes++;last_pulse=p;return write_ok;}
void CarrierPwm_Off(void){offs++;}
static unsigned mode=1,dm_busy,lh_busy,eh_busy,em_busy;
unsigned g_can_test_result=1,g_safety_ready=1,g_vm_test_request=1,g_arm_menu_power_state=1,g_lift_state=4;
unsigned g_dm_jog_request,g_lift_home_request,g_lift_motion_request,g_lift_motion_state;
unsigned g_extension_home_request,g_extension_motion_request,g_lift_home_state;
unsigned g_extension_motion_state,g_extension_home_state;
unsigned ArmPower_IsIsolatedMode(void){return mode;}
unsigned DM_CAN2_JogIsBusy(void){return dm_busy;}
unsigned LiftHome_IsBusy(void){return lh_busy;}
unsigned ExtensionHome_IsBusy(void){return eh_busy;}
unsigned Extension_MotionIsBusy(void){return em_busy;}
unsigned xTaskGetTickCount(void){return now;}
#define portTICK_PERIOD_MS 1U
@@WRAPPER@@
static void start(unsigned t){g_carrier_test_enable=1;g_carrier_request=1;Carrier_Service(1,0,t);}
static int is(const char*a,const char*b){return strcmp(a,b)==0;}
int main(int argc,char**argv){
 const char*s=argv[1];(void)argc;
 if(is(s,"init_fail"))init_ok=0;
 Carrier_Init();
 assert(g_carrier_request==0&&g_carrier_test_enable==0&&writes==0&&!Carrier_IsBusy());
 if(is(s,"boot")){Carrier_Service(1,0,50);assert(writes==0);}
 else if(is(s,"disabled")){g_carrier_request=1;Carrier_Service(1,0,0);assert(writes==0&&g_carrier_result==4);}
 else if(is(s,"init_fail")){start(0);assert(writes==0&&g_carrier_result==4);}
 else if(is(s,"not_ready")){g_carrier_test_enable=1;g_carrier_request=1;Carrier_Service(0,0,0);assert(writes==0&&g_carrier_result==4);}
 else if(is(s,"nan")||is(s,"inf")||is(s,"low")||is(s,"high")){
  g_carrier_debug_angle=is(s,"nan")?NAN:is(s,"inf")?INFINITY:is(s,"low")?119.99f:150.01f;
  start(0);assert(writes==0&&g_carrier_result==3);
 }
 else if(is(s,"min")||is(s,"max")){
  g_carrier_debug_angle=is(s,"min")?120:150;start(0);
  assert(writes==1&&g_carrier_state==1&&last_pulse==(is(s,"min")?1389:1611));
 }
 else if(is(s,"write_fail")){write_ok=0;start(0);assert(writes==1&&!g_carrier_signal_active&&g_carrier_result==6);}
 else if(is(s,"unknown")){g_carrier_request=99;Carrier_Service(1,0,0);assert(writes==0&&g_carrier_result==9);}
 else if(is(s,"window")||is(s,"busy")||is(s,"no_repeat")||is(s,"post_window_pause")){
  start(100);assert(writes==1&&last_pulse==1500&&g_carrier_latched_angle==135);
  for(unsigned t=200;t<=1600;t+=100){
   if(is(s,"busy")){g_carrier_debug_angle=150;g_carrier_request=1;}
   Carrier_Service(1,0,t);
  }
  assert(writes==1&&g_carrier_state==2&&g_carrier_result==1&&!g_carrier_signal_active&&Carrier_IsBusy());
  if(is(s,"busy"))assert(g_carrier_rejected_count==15&&g_carrier_latched_angle==135);
  if(is(s,"no_repeat")){g_carrier_request=1;Carrier_Service(1,0,1700);assert(writes==1&&g_carrier_state==2);}
  if(is(s,"post_window_pause")){Carrier_Service(1,0,9000);assert(g_carrier_state==2);}
 }
 else if(is(s,"gap")||is(s,"power_loss")||is(s,"disable")||is(s,"conflict")){
  start(0);if(is(s,"disable"))g_carrier_test_enable=0;
  Carrier_Service(is(s,"power_loss")?0:1,is(s,"conflict")?1:0,is(s,"gap")?501:100);
  assert(g_carrier_state==9&&!g_carrier_signal_active&&Carrier_IsBusy());
  assert(g_carrier_result==(is(s,"gap")?8:is(s,"conflict")?5:4));
  unsigned prev_offs=offs;g_carrier_request=1;g_carrier_test_enable=1;Carrier_Service(1,0,200);
  assert(writes==1&&offs==prev_offs&&g_carrier_state==9);
 }
 else if(is(s,"abort")||is(s,"stop_idle")){
  if(is(s,"abort"))start(0);
  g_carrier_request=2;Carrier_Service(0,1,100);
  assert(!Carrier_IsBusy()&&!g_carrier_signal_active&&g_carrier_state==3&&g_carrier_test_enable==0);
 }
 else if(is(s,"wrap")){
  start(UINT32_MAX-100);for(unsigned t=0;t<=1400;t+=100)Carrier_Service(1,0,t);
  assert(g_carrier_state==2&&!g_carrier_signal_active&&g_carrier_elapsed_ms==1501);
 }
 else if(is(s,"rearm")){
  g_carrier_request=1;Carrier_Service(0,0,0);assert(g_carrier_state==9);
  g_carrier_test_enable=1;g_carrier_request=1;Carrier_Service(1,0,1);assert(writes==0);
  g_carrier_request=2;Carrier_Service(1,0,2);start(3);assert(writes==1&&g_carrier_state==1);
 }
 else if(strncmp(s,"block_",6)==0){
  if(is(s,"block_dm"))dm_busy=1;
  if(is(s,"block_lhome"))lh_busy=1;
  if(is(s,"block_ehome"))eh_busy=1;
  if(is(s,"block_emove"))em_busy=1;
  if(is(s,"block_lmove"))g_lift_motion_state=4;
  if(is(s,"block_dmreq"))g_dm_jog_request=1;
  if(is(s,"block_lmreq"))g_lift_motion_request=1;
  if(is(s,"block_lhreq"))g_lift_home_request=1;
  if(is(s,"block_emreq"))g_extension_motion_request=1;
  if(is(s,"block_ehreq"))g_extension_home_request=1;
  if(is(s,"block_lfault"))g_lift_motion_state=9;
  if(is(s,"block_lhfault"))g_lift_home_state=9;
  if(is(s,"block_efault"))g_extension_motion_state=9;
  if(is(s,"block_ehfault"))g_extension_home_state=9;
  g_carrier_test_enable=1;g_carrier_request=1;App_CarrierStep();assert(writes==0&&g_carrier_result==5);
 }
 else if(is(s,"mode")||is(s,"safety")||is(s,"vm")||is(s,"identity")){
  if(is(s,"mode"))mode=0;if(is(s,"safety"))g_safety_ready=0;
  if(is(s,"vm"))g_vm_test_request=0;if(is(s,"identity"))g_lift_state=0;
  g_carrier_test_enable=1;g_carrier_request=1;App_CarrierStep();assert(writes==0&&g_carrier_result==4);
 }
 else if(is(s,"guard_starts")||is(s,"guard_stops")){
  g_carrier_test_enable=1;g_carrier_request=1;App_CarrierStep();assert(writes==1&&Carrier_IsBusy());
  g_lift_motion_request=is(s,"guard_starts")?1:3;
  g_lift_home_request=is(s,"guard_starts")?1:2;
  g_extension_motion_request=is(s,"guard_starts")?1:3;
  g_extension_home_request=is(s,"guard_starts")?1:2;
  g_dm_jog_request=is(s,"guard_starts")?1:3;
  now=100;App_CarrierStep();assert(g_carrier_state==9&&!g_carrier_signal_active&&Carrier_IsBusy());
  assert(g_lift_motion_request==(is(s,"guard_starts")?0:3));
  assert(g_extension_motion_request==(is(s,"guard_starts")?0:3));
  assert(g_lift_home_request==(is(s,"guard_starts")?0:2));
  assert(g_extension_home_request==(is(s,"guard_starts")?0:2));
  assert(g_dm_jog_request==(is(s,"guard_starts")?0:3));
 }
 else assert(!"unknown test");
 puts(s);return 0;
}
'''

HOST.mkdir(parents=True,exist_ok=True)
t=(STAGE/'USER/main.c').read_bytes()
a=t.index(b'static void App_CarrierStep(void)')
b=t.index(b'static void App_CanTask(void *argument)',a)
wrapper=t[a:b].decode('ascii')
(HOST/'model.c').write_text(MODEL.replace('@@WRAPPER@@',wrapper),encoding='utf-8')
exe=HOST/'carrier-check.exe'
def run(args):
    p=subprocess.run(args,capture_output=True,text=True,timeout=30)
    assert p.returncode==0,(args,p.stdout,p.stderr)
    return p.stdout.strip()
run(['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra',
     '-I',str(STAGE/'HARDWARE/CARRIER'),str(HOST/'model.c'),
     str(STAGE/'HARDWARE/CARRIER/carrier.c'),'-o',str(exe)])
cases=['boot','disabled','init_fail','not_ready','nan','inf','low','high','min','max',
       'write_fail','unknown','window','busy','no_repeat','post_window_pause',
       'gap','power_loss','disable','conflict','abort','stop_idle','wrap','rearm',
       'mode','safety','vm','identity','guard_starts','guard_stops']
cases+=['block_'+s for s in ['dm','lhome','ehome','emove','lmove','dmreq','lmreq',
                           'lhreq','emreq','ehreq','lfault','lhfault','efault','ehfault']]
outputs=[run([str(exe),c]) for c in cases]
# 检查调度调用在任何轴服务前，及未改变原调度主体（除显式入口新增）。
assert t.index(b'App_CarrierStep();',t.index(b'static void App_CanTask')) < t.index(b'Lift_Service(',b)
old=(EV/'original/USER/main.c').read_bytes()
recovered=t.replace(b'#include "gripper.h"\n#include "carrier.h"',b'#include "gripper.h"',1)
recovered=recovered.replace(b'/* PB1 carrier uses the same serialized arm scheduler; no boot motion. */\n'+t[a:b],b'',1)
recovered=recovered.replace(b'        g_can_task_count++;\n        App_CarrierStep();',b'        g_can_task_count++;',1)
recovered=recovered.replace(b'    Carrier_Init();\n    Gripper_Init();',b'    Gripper_Init();',1)
assert recovered==old,'Existing main behavior changed outside declared insertions'
pwm=(STAGE/'HARDWARE/CARRIER/carrier_pwm.c').read_text(encoding='utf-8')
assert 'TIM_OC4Init(TIM3' in pwm and 'GPIO_PinSource1, GPIO_AF_TIM3' in pwm
assert 'clocks.PCLK1_Frequency' in pwm and 'timer_hz *= 2U' in pwm
manifest=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
for rel,digest in manifest['protected'].items():
    assert hashlib.sha256((STAGE/rel).read_bytes()).hexdigest()==digest,rel
review={'passed':True,'runtime_cases':len(cases),'cases':cases,
        'existing_main_recovered_byte_identical':True,
        'protected_source_count':len(manifest['protected']),
        'source_sha256':{e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in manifest['files']},
        'hardware_motion_or_waveform_verified':False}
(EV/'host-check.log').write_text('\n'.join(outputs)+'\n',encoding='utf-8')
(EV/'host-review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('PASS:',len(cases),'C runtime cases; main insertions and protected sources checked; no hardware validation.')
