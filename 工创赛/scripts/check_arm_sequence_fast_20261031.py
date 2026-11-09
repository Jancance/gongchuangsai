"""只检查本轮新增：动作组/请求仲裁、夹爪共享准入；不重跑旧机构/视觉套件。"""
from pathlib import Path
import json,hashlib,subprocess,shutil
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'tmp/arm_sequence_fast_20261031'
STAGE=BASE/'project';EV=ROOT/'evidence/p2-arm/2026-10-31/arm-sequence-fast';HOST=BASE/'host-check'
HOST.mkdir(parents=True,exist_ok=True)
(HOST/'lift.h').write_text('''#include <stdint.h>
extern volatile uint32_t g_lift_state,g_lift_motion_state,g_lift_motion_result,g_lift_stop_result,g_lift_motion_request,g_lift_up_pulses,g_lift_down_pulses;
''',encoding='utf-8')
(HOST/'FreeRTOS.h').write_text('''#ifndef FAST_RTOS_H
#define FAST_RTOS_H
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef int BaseType_t;typedef uint32_t TickType_t;typedef void*TaskHandle_t;
#define pdFAIL 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define configASSERT(x) assert(x)
#endif
''',encoding='utf-8')
(HOST/'task.h').write_text('''#include "FreeRTOS.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
BaseType_t xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*);
void vTaskDelay(TickType_t);
''',encoding='utf-8')
(HOST/'gripper.h').write_text('''#include <stdint.h>
uint8_t jiazhua(float);void Gripper_Stop(void);
''',encoding='utf-8')
text=(STAGE/'USER/main.c').read_bytes();a=text.index(b'static uint32_t App_ArmSequenceReady(void)');b=text.index(b'static void App_CarrierStep(void)',a)
wrapper=text[a:b].decode('ascii')
MODEL=r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "arm_sequence.h"
#include "gripper_task.h"
volatile uint32_t g_lift_state=4,g_lift_motion_state,g_lift_motion_result,g_lift_stop_result,g_lift_motion_request;
volatile uint32_t g_lift_up_pulses=8123,g_lift_down_pulses=40000;
volatile uint32_t g_gripper_signal_stop_count;
volatile uint32_t g_jiazhua_debug_request;
unsigned g_can_test_result=1,g_safety_ready=1,g_vm_test_request=1,g_arm_menu_power_state=1;
unsigned g_lift_home_request,g_lift_home_state,g_lift_read_request,g_lift_home_read_request;
unsigned g_extension_motion_request,g_extension_home_request,g_extension_read_request,g_extension_home_read_request;
unsigned g_dm_jog_request,g_carrier_request,now;
static unsigned access_ready,access_busy,access_exclusive,pending,ticket,done,cmd_result,moves,stop_calls,remaining;
static unsigned carrier_busy,other_busy,command_fail,open_defer,lift_fail,stop_fail,gripper_cancel;
static unsigned directions[4],pulses[4],commands;static float angles[4];
unsigned ArmPower_IsIsolatedMode(void){return 1;}
unsigned DM_CAN2_JogIsBusy(void){return other_busy;}
unsigned LiftHome_IsBusy(void){return 0;}
unsigned ExtensionHome_IsBusy(void){return 0;}
unsigned Extension_MotionIsBusy(void){return 0;}
unsigned Carrier_IsBusy(void){return carrier_busy;}
void Gripper_SetAccess(uint32_t r,uint32_t b,uint32_t e){access_ready=r;access_busy=b;access_exclusive=e;}
uint32_t Gripper_IsCommandBusy(void){return pending;}
uint32_t Gripper_ControlledCommand(float a){
 if(!access_ready||access_busy||pending)return 0;
 if(a==180&&open_defer){open_defer=0;return 0;}
 ticket++;pending=1;angles[commands++]=a;return ticket;}
uint32_t Gripper_ControlledResult(uint32_t t){return t&&t==done?cmd_result:0;}
void Gripper_CancelControlled(void){if(pending){pending=0;done=ticket;cmd_result=2;}gripper_cancel++;}
@@WRAPPER@@
static void tick(void){
 now+=20;App_ArmSequenceGuard();
 if(g_lift_home_request==2)g_lift_home_request=0;
 if(g_lift_motion_request==3){g_lift_motion_request=0;remaining=0;stop_calls++;g_lift_stop_result=stop_fail?0:1;g_lift_motion_state=stop_fail?9:8;}
 else if((g_lift_motion_request==1||g_lift_motion_request==2)&&remaining==0){
  unsigned dir=g_lift_motion_request;g_lift_motion_request=0;
  directions[moves]=dir;pulses[moves++]=dir==2?g_lift_up_pulses:g_lift_down_pulses;
  remaining=3;g_lift_motion_state=2;g_lift_motion_result=0;
 }else if(remaining){remaining--;if(!remaining){g_lift_motion_state=lift_fail?9:7;g_lift_motion_result=lift_fail?4:1;}}
 ArmSequence_Service(App_ArmSequenceReady(),carrier_busy||other_busy,now);
 if(pending){pending=0;done=ticket;cmd_result=command_fail?2:1;}
}
static void start(void){g_arm_sequence_enable=1;g_arm_grip_height_ready=1;g_arm_sequence_request=1;tick();}
static void wait_grip(void){for(unsigned i=0;i<15&&g_arm_sequence_state!=2;i++)tick();assert(g_arm_sequence_state==2);}
static int is(const char*a,const char*b){return strcmp(a,b)==0;}
int main(int argc,char**argv){const char*s=argv[1];(void)argc;
 if(is(s,"disabled")||is(s,"height_missing")){
  if(is(s,"height_missing"))g_arm_sequence_enable=1;
  g_arm_sequence_request=1;tick();assert(!commands&&!moves&&!ArmSequence_IsBusy()&&g_arm_sequence_result==3);
 }else if(is(s,"bad_config")){
  g_arm_test_lift_pulses=10001;start();assert(!commands&&!moves&&g_arm_sequence_result==4);
 }else if(is(s,"complete")||is(s,"snapshot")||is(s,"deferred_open")){
  if(is(s,"deferred_open"))open_defer=1;
  start();wait_grip();
  if(is(s,"snapshot"))g_arm_test_lift_pulses=9000;
  g_arm_grip_confirm=1;
  for(unsigned i=0;i<150&&g_arm_sequence_state!=7;i++)tick();
  assert(g_arm_sequence_state==7&&g_arm_sequence_result==1&&!ArmSequence_IsBusy());
  assert(moves==2&&directions[0]==2&&directions[1]==1&&pulses[0]==5000&&pulses[1]==5000);
  assert(commands==2&&angles[0]==140&&angles[1]==180&&g_lift_up_pulses==8123&&g_lift_down_pulses==40000);
  for(unsigned i=0;i<100;i++){tick();}assert(moves==2&&commands==2);
 }else if(is(s,"fresh_confirm")){
  g_arm_grip_confirm=1;start();wait_grip();for(unsigned i=0;i<40;i++)tick();assert(!moves&&g_arm_sequence_state==2&&g_arm_grip_confirm==0);
 }else if(is(s,"confirm_timeout")){
  start();wait_grip();for(unsigned i=0;i<760;i++)tick();assert(g_arm_sequence_state==9&&g_arm_sequence_result==7&&!moves&&ArmSequence_IsBusy());
 }else if(is(s,"gripper_fail")){
  command_fail=1;start();for(unsigned i=0;i<5;i++)tick();assert(g_arm_sequence_state==9&&g_arm_sequence_result==5&&!moves);
 }else if(is(s,"guard")){
  start();g_extension_motion_request=2;g_extension_home_request=1;g_dm_jog_request=4;g_carrier_request=1;g_lift_motion_request=1;
  App_ArmSequenceGuard();assert(!g_extension_motion_request&&!g_extension_home_request&&!g_dm_jog_request&&!g_carrier_request&&!g_lift_motion_request&&access_exclusive);
  g_extension_motion_request=3;g_extension_home_request=2;g_dm_jog_request=3;g_carrier_request=2;
  App_ArmSequenceGuard();assert(g_extension_motion_request==3&&g_extension_home_request==2&&g_dm_jog_request==3&&g_carrier_request==2);
 }else if(is(s,"abort_before_dispatch")||is(s,"external_stop_preempts")){
  start();wait_grip();g_arm_grip_confirm=1;tick();assert(g_lift_motion_request==2&&moves==0);
  if(is(s,"external_stop_preempts"))g_lift_home_request=2;else g_arm_sequence_request=2;
  tick();assert(moves==0&&stop_calls==1&&g_arm_sequence_state==8&&ArmSequence_IsBusy()&&g_arm_grip_height_ready==0);
  g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy()&&g_arm_sequence_state==0);
 }else if(is(s,"reset_active_rejected")){
  start();g_arm_sequence_request=3;tick();assert(ArmSequence_IsBusy()&&g_arm_sequence_state!=0);
 }else if(is(s,"lift_fail")){
  lift_fail=1;start();wait_grip();g_arm_grip_confirm=1;
  for(unsigned i=0;i<12;i++){tick();}assert(g_arm_sequence_state==9&&g_arm_sequence_result==6&&ArmSequence_IsBusy()&&commands==1);
 }else if(is(s,"stop_unknown")){
  start();wait_grip();g_arm_grip_confirm=1;tick();stop_fail=1;g_arm_sequence_request=2;tick();
  g_arm_sequence_request=3;tick();assert(ArmSequence_IsBusy());
  stop_fail=0;g_lift_motion_request=3;tick();g_arm_sequence_request=3;tick();assert(!ArmSequence_IsBusy());
 }else if(is(s,"ready_loss")||is(s,"height_revoked")||is(s,"gap")||is(s,"manual_gripper_stop")){
  start();wait_grip();
  if(is(s,"ready_loss"))g_vm_test_request=0;
  if(is(s,"height_revoked"))g_arm_grip_height_ready=0;
  if(is(s,"manual_gripper_stop"))g_gripper_signal_stop_count++;
  if(is(s,"gap"))now+=501;
  tick();assert(g_arm_sequence_state==9&&ArmSequence_IsBusy()&&!moves&&commands==1);
 }else assert(!"unknown case");
 puts(s);return 0;}
'''
(HOST/'sequence-test.c').write_text(MODEL.replace('@@WRAPPER@@',wrapper),encoding='utf-8')
GRIP=r'''
#include <stdint.h>
#include <setjmp.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "gripper_task.h"
static jmp_buf done;static void(*entry)(void*);static unsigned writes,stops;static float angles[5];
BaseType_t xTaskCreate(void(*fn)(void*),const char*n,unsigned st,void*a,unsigned p,TaskHandle_t*h){(void)n;(void)st;(void)a;(void)p;entry=fn;*h=(void*)1;return 1;}
void vTaskDelay(TickType_t t){(void)t;longjmp(done,1);}
uint8_t jiazhua(float a){angles[writes++]=a;return 1;}
void Gripper_Stop(void){stops++;}
int main(int argc,char**argv){uint32_t ticket=0;(void)argc;const char*s=argv[1];Gripper_TaskInit();
 if(strcmp(s,"manual_busy")==0){Gripper_SetAccess(1,1,0);g_jiazhua_debug_angle=140;g_jiazhua_debug_request=1;}
 else if(strcmp(s,"manual_ready")==0){Gripper_SetAccess(1,0,0);g_jiazhua_debug_angle=140;g_jiazhua_debug_request=1;}
 else if(strcmp(s,"owned")==0){Gripper_SetAccess(1,0,1);ticket=Gripper_ControlledCommand(140);assert(ticket);g_jiazhua_debug_angle=180;g_jiazhua_debug_request=1;}
 else if(strcmp(s,"cancel")==0){Gripper_SetAccess(1,0,1);ticket=Gripper_ControlledCommand(140);assert(ticket);Gripper_CancelControlled();}
 else if(strcmp(s,"stop")==0){Gripper_SetAccess(0,1,1);g_jiazhua_debug_request=2;}
 else assert(!"unknown grip case");
 if(setjmp(done)==0)entry(NULL);
 if(strcmp(s,"manual_busy")==0)assert(writes==1&&g_jiazhua_debug_result==4);
 if(strcmp(s,"manual_ready")==0)assert(writes==2&&angles[1]==140&&g_jiazhua_debug_result==1);
 if(strcmp(s,"owned")==0)assert(writes==2&&angles[1]==140&&Gripper_ControlledResult(ticket)==1&&g_jiazhua_debug_result==4);
 if(strcmp(s,"cancel")==0)assert(writes==1&&Gripper_ControlledResult(ticket)==2);
 if(strcmp(s,"stop")==0)assert(stops==1&&g_gripper_signal_stop_count==1&&g_jiazhua_debug_result==2);
 puts(s);return 0;}
'''
(HOST/'grip-test.c').write_text(GRIP,encoding='utf-8')
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=30)
 assert p.returncode==0,(args,p.stdout,p.stderr)
 return p.stdout.strip()
gcc='C:/mingw/mingw64/bin/gcc.exe';includes=['-I'+str(HOST),'-I'+str(STAGE/'USER'),'-I'+str(STAGE/'HARDWARE/LIFT'),'-I'+str(STAGE/'HARDWARE/GRIPPER')]
seqexe=HOST/'sequence.exe';gripexe=HOST/'grip.exe'
run([gcc,'-std=c99','-Wall','-Wextra']+includes+[str(STAGE/'USER/arm_sequence.c'),str(HOST/'sequence-test.c'),'-o',str(seqexe)])
run([gcc,'-std=c99','-Wall','-Wextra','-Werror']+includes+[str(STAGE/'USER/gripper_task.c'),str(HOST/'grip-test.c'),'-o',str(gripexe)])
seqcases=['disabled','height_missing','bad_config','complete','snapshot','deferred_open','fresh_confirm','confirm_timeout','gripper_fail','guard','abort_before_dispatch','external_stop_preempts','reset_active_rejected','lift_fail','stop_unknown','ready_loss','height_revoked','gap','manual_gripper_stop']
gripcases=['manual_busy','manual_ready','owned','cancel','stop']
logs=[run([str(seqexe),s]) for s in seqcases]+[run([str(gripexe),s]) for s in gripcases]
m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
for n,h in m['protected'].items():assert hashlib.sha256((STAGE/n).read_bytes()).hexdigest()==h,n
r={'passed':True,'new_sequence_scenarios':len(seqcases),'new_gripper_scenarios':len(gripcases),'cases':seqcases+gripcases,
 'source_sha256':{e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in m['files']},
 'protected_files':len(m['protected']),'old_suites_repeated':False,'physical_grasp_or_automatic_competition_accepted':False}
(EV/'host-review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(EV/'host-check.log').write_text('\n'.join(logs)+'\n',encoding='utf-8')
print('PASS:new sequence19+gripper5 scenarios;prior suites not repeated;no hardware.')
