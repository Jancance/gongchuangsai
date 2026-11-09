"""真实节点6回零状态机、任务调度及CAN启动/退出/通知捕获的合成验证。"""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'tmp/extension_sensorless_20261030/project'
HOST=ROOT/'tmp/extension_sensorless_20261030/host-check'
EV=ROOT/'evidence/p2-arm/2026-10-30/extension-sensorless'
MODULE=STAGE/'HARDWARE/EXTENSION'
GCC='C:/mingw/mingw64/bin/gcc.exe'


def run(cmd):
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=30)
    assert p.returncode==0,(cmd,p.stdout,p.stderr)
    return p.stdout.strip()


def extract(data,signature):
    a=data.index(signature);z=data.index(b'{',a)+1;depth=1
    while depth:
        if data[z]==123:depth+=1
        elif data[z]==125:depth-=1
        z+=1
    return data[a:z]


MODEL=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "extension.h"
static const char *scenario;
static TickType_t now;
static unsigned starts,aborts,stops,enables,options,params,polls,ended,normal_moves;
static unsigned aborted,retry_stop;
volatile uint32_t g_can_extension_home_done_notice;
static int is(const char*s){return strcmp(scenario,s)==0;}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t ticks){now+=ticks;}
void CAN1_ExtensionHomeSessionEnd(void){ended++;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==6);r->Data[0]=0x1F;return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){assert(a==6);options++;r->Data[2]=is("bad_options")?0:6;if(is("prep_timeout"))now+=2600;return 1;}
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t*p,CanRxMsg*r){
 const uint8_t sample[17]={0x22,0,0,0,30,0,0,0x27,0x10,1,0x2C,3,0x20,0,60,0,0x6B};
 params++;memcpy(p,sample,17);memset(r,0,3*sizeof(*r));
 if(is("param_fail"))return 5;
 if(is("wrong_dir"))p[2]=1;
 if(is("rpm_changed"))p[4]=31;
 if(is("timeout_changed"))p[8]=0x11;
 if(is("detect_rpm_changed"))p[10]=0x2D;
 if(is("current_changed"))p[12]=0x21;
 if(is("time_changed"))p[14]=61;
 if(is("auto_changed"))p[15]=1;
 if(is("mode_menu2"))p[1]=2;
 return 1;
}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){
 assert(a==6);r->Data[1]=(is("not_enabled")||is("enable_fail"))?2:3;
 if(is("status_fault")){r->Data[1]=7;}
 return is("status_fail")?5:1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v){
 assert(a==6);r->Data[0]=0x36;*s=0;*v=1000;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){
 assert(a==6);r->Data[0]=0x35;*v=0;
 if(is("pre_moving")&&!aborted)*v=2;
 if(is("running_speed")&&starts&&!aborted)*v=2;
 return 1;}
uint32_t CAN1_ExtensionExchange(const uint8_t*q,uint8_t n,CanRxMsg*r){
 r->Data[1]=2;
 if(q[0]==0xFE){assert(n==4&&q[1]==0x98&&q[2]==0&&q[3]==0x6B);stops++;
  return ((is("stop_fail")||is("stop_retry")||is("home_stop_failed")||is("power_loss_stopfail"))&&!retry_stop)?5:1;}
 if(q[0]==0xF3){assert(n==5&&q[1]==0xAB&&q[2]==1&&q[3]==0&&q[4]==0x6B);enables++;return is("enable_fail")?5:1;}
 assert(q[0]==0xFD&&n==12&&q[8]==160&&q[9]==2);normal_moves++;return 1;}
uint32_t CAN1_StartExtensionHome(CanRxMsg*r){
 starts++;polls=0;g_can_extension_home_done_notice=0;r->Data[1]=2;
 if(is("fast_notice")){g_can_extension_home_done_notice=1;r->Data[1]=0x9F;}
 return is("start_fail")?5:1;}
uint32_t CAN1_AbortExtensionHome(CanRxMsg*r){
 aborts++;r->Data[1]=2;
 if(is("abort_fail"))return 5;
 aborted=1;return is("abort_idle")?8:1;}
uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg*r){
 r->Data[1]=3;
 if(!starts){if(is("bad_home_ready"))r->Data[1]=1;if(is("unexpected_home")&&!aborted)r->Data[1]=7;return is("home_read_fail")?5:1;}
 if(aborted)return 1;
 if(is("abort_fail")){r->Data[1]=7;return 1;}
 if(is("feedback_fail"))return 5;
 if(is("driver_home_fail")){r->Data[1]=0x0B;return 1;}
 polls++;
 if(is("idle_only")||is("stale_notice")||is("fast_notice"))return 1;
 if(is("stuck_running")||polls==1)r->Data[1]=7;
 return 1;
}

#ifndef INTEGRATION
int main(int argc,char**argv){
 unsigned i,expected=1,before;uint32_t ready=1,busy=0;
 assert(argc==2);scenario=argv[1];if(is("wrap"))now=0xFFFFFFD0U;
 ExtensionHome_Service(1,0);assert(starts==0&&options==0&&params==0&&stops==0);
 if(is("invalid_request")){g_extension_home_request=3;ExtensionHome_Service(1,0);assert(g_extension_home_state==0&&starts==0);puts("PASS home invalid request");return 0;}
 g_extension_home_request=1;
 if(is("motion_pending"))g_extension_motion_request=1;
 if(is("motion_busy"))g_extension_motion_state=6;
 if(is("failed_motion"))g_extension_motion_state=9;
 if(is("identity_busy"))g_extension_state=2;
 if(is("stale_notice"))g_can_extension_home_done_notice=1;
 for(i=0;i<1500;i++){
  now+=10;ready=!(is("not_ready")||((is("power_loss")||is("power_loss_stopfail"))&&starts));
  busy=is("busy_start")||(is("busy_after")&&starts);
  if(is("repeat")&&ExtensionHome_IsBusy())g_extension_home_request=1;
  if((is("abort")||is("abort_fail")||is("abort_idle")||is("stop_fail")||is("stop_retry"))&&starts)g_extension_home_request=2;
  if(is("ordinary_stop")&&starts)g_extension_motion_request=3;
  if(is("block_move")&&starts)g_extension_motion_request=1;
  if(is("gap")&&starts)now+=600;
  ExtensionHome_Service(ready,busy);
  if(g_extension_home_state>=8)break;
 }
 if(is("bad_options")||is("param_fail")||is("status_fail")||is("home_read_fail")||is("enable_fail")||is("start_fail")||is("feedback_fail")||is("abort_fail")||is("stop_fail")||is("stop_retry"))expected=4;
 if(is("wrong_dir")||is("rpm_changed")||is("timeout_changed")||is("detect_rpm_changed")||is("current_changed")||is("time_changed")||is("auto_changed"))expected=7;
 if(is("status_fault")||is("bad_home_ready")||is("unexpected_home")||is("pre_moving")||is("driver_home_fail"))expected=5;
 if(is("not_ready")||is("busy_start")||is("motion_pending")||is("motion_busy")||is("failed_motion")||is("identity_busy")||is("power_loss")||is("power_loss_stopfail")||is("busy_after"))expected=6;
 if(is("idle_only")||is("stale_notice")||is("stuck_running")||is("running_speed")||is("prep_timeout"))expected=3;
 if(is("abort")||is("ordinary_stop")||is("abort_idle"))expected=2;
 if(is("gap"))expected=8;
 assert(g_extension_home_result==expected&&normal_moves==0);
 if(expected==1){assert(g_extension_home_state==8&&g_extension_home_reported==1&&starts==1&&!ExtensionHome_IsBusy());
  before=starts;for(i=0;i<8;i++){now+=10;ExtensionHome_Service(1,0);}assert(starts==before);
  if(is("invalidate")){ExtensionHome_Service(0,0);assert(g_extension_home_reported==0);}}
 else if(expected==2){assert(g_extension_home_state==10&&g_extension_home_reported==0&&aborts==1&&stops==1&&!ExtensionHome_IsBusy());}
 else{assert(g_extension_home_state==9&&g_extension_home_reported==0);before=starts;
  g_extension_home_request=1;ExtensionHome_Service(1,0);assert(starts==before);}
 if(is("not_ready")||is("busy_start")||is("motion_pending")||is("motion_busy")||is("identity_busy")||is("failed_motion"))assert(starts==0&&options==0&&aborts==0&&stops==0);
 if(is("not_enabled"))assert(enables==1);
 if(is("idle_only")||is("stale_notice"))assert(starts==1&&aborts==1&&stops==1);
 if(is("start_fail"))assert(aborts==1&&stops==1);
 if(is("stop_fail")||is("abort_fail")||is("stop_retry")){
  assert(ExtensionHome_IsBusy());g_extension_motion_request=1;Extension_MotionService(1,0);assert(normal_moves==0);
  if(is("stop_retry")){retry_stop=1;g_extension_home_request=2;ExtensionHome_Service(1,0);assert(g_extension_home_state==10&&!ExtensionHome_IsBusy());}}
 if(is("abort_idle"))assert(g_extension_home_abort_result==8&&g_extension_home_stop_result==1);
 if(is("power_loss_stopfail")){
  unsigned a_count=aborts,s_count=stops;assert(ExtensionHome_IsBusy());
  for(i=0;i<5;i++)ExtensionHome_Service(0,0);
  assert(aborts==a_count&&stops==s_count&&ExtensionHome_IsBusy());
  retry_stop=1;g_extension_home_request=2;ExtensionHome_Service(0,0);
  assert(g_extension_home_state==10&&!ExtensionHome_IsBusy());}
 printf("PASS home state %s result=%u starts=%u aborts=%u stops=%u\n",scenario,expected,starts,aborts,stops);return 0;
}
#else
unsigned g_can_test_result=1,g_safety_ready=1,g_vm_test_request=1,g_arm_menu_power_state=1;
unsigned g_lift_state=4,g_lift_motion_request,g_lift_home_request,g_lift_read_request,g_lift_home_read_request;
unsigned g_lift_motion_state,dm_busy,home_busy,dm_request,dm_started,lift_started;
uint32_t DM_CAN2_JogIsBusy(void){return dm_busy;}
uint32_t LiftHome_IsBusy(void){return home_busy;}
void Lift_Service(uint32_t ready){assert(ready);if(g_lift_motion_request==1){lift_started++;g_lift_motion_state=1;g_lift_motion_request=0;}}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){assert(ready);if(dm_request&&!busy)dm_started++;dm_request=0;}
void step(void){
__BRANCH__
}
int main(int argc,char**argv){
 unsigned i;assert(argc==2);scenario=argv[1];g_extension_home_request=1;
 if(is("dm_busy")){dm_busy=1;step();assert(starts==0&&g_extension_home_result==6);}
 else if(is("lift_busy")){g_lift_motion_state=6;step();assert(starts==0&&g_extension_home_result==6);}
 else if(is("lift_home_busy")){home_busy=1;step();assert(starts==0&&g_extension_home_result==6);}
 else if(is("simultaneous_lift")){g_lift_motion_request=1;step();assert(lift_started==1&&starts==0&&g_extension_home_result==6);}
 else if(is("simultaneous_motion")){g_extension_motion_request=1;step();assert(g_extension_home_state==9&&g_extension_motion_state==2&&starts==0);}
 else if(is("simultaneous_dm")){dm_request=4;step();assert(ExtensionHome_IsBusy()&&dm_started==0);}
 else if(is("home_stop_failed")){
  for(i=0;i<30&&!starts;i++){now+=10;step();}assert(starts==1);
  g_extension_home_request=2;step();assert(ExtensionHome_IsBusy()&&g_extension_home_state==9);
  g_lift_motion_request=1;dm_request=4;g_extension_motion_request=1;step();
  assert(lift_started==0&&dm_started==0&&normal_moves==0&&ExtensionHome_IsBusy());
 }else{
  step();assert(ExtensionHome_IsBusy());g_lift_motion_request=1;g_lift_home_request=1;
  g_lift_read_request=1;g_lift_home_read_request=1;g_extension_read_request=1;g_extension_home_read_request=1;
  g_extension_motion_request=1;dm_request=4;step();
  assert(lift_started==0&&dm_started==0&&normal_moves==0&&g_extension_motion_request==0);
  assert(g_lift_home_request==0&&g_lift_read_request==0&&g_lift_home_read_request==0&&g_extension_home_read_request==0);
  assert(params==1&&g_extension_home_state==3);
 }
 printf("PASS home integration %s\n",scenario);return 0;
}
#endif
'''
main=(STAGE/'USER/main.c').read_bytes().replace(b'\r\n',b'\n')
a=main.index(b'        if (ArmPower_IsIsolatedMode() != 0U)');a=main.index(b'{',a)+1
z=main.index(b'            g_can_task_stack_words =',a)
model=MODEL.replace('__BRANCH__',main[a:z].decode('utf-8',errors='replace'))
(HOST/'home-state-model.c').write_text(model,encoding='utf-8')
common=[GCC,'-std=c99','-Wall','-Wextra','-Werror','-I'+str(HOST),'-I'+str(MODULE)]
records=[]
cases=['normal','fast_notice','mode_menu2','not_enabled','wrap','repeat','invalidate','block_move',
       'bad_options','param_fail','wrong_dir','rpm_changed','timeout_changed','detect_rpm_changed',
       'current_changed','time_changed','auto_changed','status_fail','status_fault','bad_home_ready',
       'home_read_fail','pre_moving','unexpected_home','enable_fail','start_fail','idle_only','stale_notice',
       'stuck_running','driver_home_fail','feedback_fail','running_speed','abort','ordinary_stop',
       'abort_fail','abort_idle','stop_fail','stop_retry','power_loss','busy_after','gap','not_ready',
       'busy_start','motion_pending','motion_busy','identity_busy','failed_motion','prep_timeout','invalid_request',
       'power_loss_stopfail']
for label,defines,group_cases in [
    ('home-state',[],cases),
    ('home-integration',['-DINTEGRATION'],['dm_busy','lift_busy','lift_home_busy','simultaneous_lift',
                                        'simultaneous_motion','simultaneous_dm','home_stop_failed','home_active'])]:
    exe=HOST/(label+'.exe')
    run(common+defines+[str(MODULE/'extension.c'),str(MODULE/'extension_home.c'),
                        str(HOST/'home-state-model.c'),'-o',str(exe)])
    for case in group_cases:records.append(dict(group=label,case=case,output=run([str(exe),case])))

PORT=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "stm32f4xx_can.h"
#ifndef APP_ARM_LIFT_TEST
#define APP_ARM_LIFT_TEST 1U
#endif
#define CAN1_SINGLE_TX_WAIT_MS 100U
#define CAN1_SINGLE_REPLY_WAIT_MS 200U
#define CAN1 0
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 4U
#define CAN_RTR_Data 0U
#define CAN_TxStatus_NoMailBox 4U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 0U
static const char*scenario;
static TickType_t now;
static unsigned sent,received,flushed,cancels,aborted,queried;
static uint32_t s_can1_normal_ready;
volatile uint32_t g_can_lift_home_done_notice,g_can_extension_home_done_notice;
static uint32_t s_lift_home_capture=1,s_extension_home_capture;
static int is(const char*s){return strcmp(scenario,s)==0;}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t t){now+=t;}
uint32_t CAN1_InitMode(unsigned mode){(void)mode;return is("init_fail")?3:1;}
unsigned CAN_MessagePending(unsigned can,unsigned fifo){
 (void)can;(void)fifo;if(!sent)return is("stale_start")&&flushed<3;
 return !received&&!is("no_reply");}
void CAN_Receive(unsigned can,unsigned fifo,CanRxMsg*r){
 (void)can;(void)fifo;memset(r,0,sizeof(*r));r->IDE=4;r->DLC=3;r->ExtId=0x600;r->Data[0]=0x9A;r->Data[1]=0x9F;r->Data[2]=0x6B;
 if(!sent){flushed++;return;}
 received=1;r->Data[1]=is("notice")||is("wrong_node")?0x9F:2;
 if(is("wrong_node"))r->ExtId=0x500;
 if(is("wrong_packet"))r->ExtId=0x601;
 if(is("wrong_ide"))r->IDE=0;
 if(is("wrong_rtr"))r->RTR=1;
 if(is("wrong_command"))r->Data[0]=0x22;
 if(is("bad_length"))r->DLC=4;
 if(is("bad_tail"))r->Data[2]=0;
 if(is("bad_ack"))r->Data[1]=0xE2;
}
unsigned CAN_Transmit(unsigned can,CanTxMsg*t){
 (void)can;sent++;assert(t->ExtId==0x600&&t->IDE==4&&t->RTR==0&&t->DLC==4);
 assert(t->Data[0]==0x9A&&t->Data[1]==2&&t->Data[2]==0&&t->Data[3]==0x6B);return is("no_mailbox")?4:0;}
unsigned CAN_TransmitStatus(unsigned can,unsigned mailbox){(void)can;(void)mailbox;return is("tx_fail")?0:(is("tx_timeout")?2:1);}
void CAN_CancelTransmit(unsigned can,unsigned mailbox){(void)can;(void)mailbox;cancels++;}
uint32_t CAN1_ExchangeSingle(uint8_t a,const uint8_t*q,uint8_t n,uint8_t expected,CanRxMsg*r){
 assert(a==6&&n==3&&expected==3&&q[0]==0x9C&&q[1]==0x48&&q[2]==0x6B);aborted++;r->Data[1]=is("bad_abort_ack")?0xE2:2;return is("abort_transport_fail")?5:1;}
uint32_t CAN1_ReadParameter(uint8_t a,uint8_t c,uint8_t n,CanRxMsg*r){assert(a==6&&c==0x3B&&n==3);queried++;r->Data[1]=3;return 1;}
__FUNCTIONS__
int main(int argc,char**argv){
 CanRxMsg r;uint32_t expected=1,result;assert(argc==2);scenario=argv[1];
 if(strncmp(scenario,"capture_",8)==0){
  memset(&r,0,sizeof(r));r.IDE=4;r.ExtId=0x600;r.DLC=3;r.Data[0]=0x9A;r.Data[1]=0x9F;r.Data[2]=0x6B;
  s_extension_home_capture=1;
  if(is("capture_off"))s_extension_home_capture=0;
  if(is("capture_node5"))r.ExtId=0x500;
  if(is("capture_rtr"))r.RTR=1;
  if(is("capture_length"))r.DLC=4;
  if(is("capture_tail"))r.Data[2]=0;
  if(is("capture_ack_only"))r.Data[1]=2;
  CAN1_CaptureLiftHomeNotice(&r);
  assert(g_can_extension_home_done_notice==(is("capture_valid")?1:0));
  assert(g_can_lift_home_done_notice==(is("capture_node5")?1:0));
  puts("PASS capture node isolation and format");return 0;
 }
 if(is("session_end")){s_extension_home_capture=1;CAN1_ExtensionHomeSessionEnd();assert(!s_extension_home_capture&&sent==0);puts("PASS session end");return 0;}
 if(is("abort")||is("bad_abort_ack")||is("abort_transport_fail")){result=CAN1_AbortExtensionHome(&r);assert(result==(is("abort")?1:(is("bad_abort_ack")?8:5))&&aborted==1&&sent==0);puts("PASS abort fixed node6");return 0;}
 if(is("status")){assert(CAN1_ReadExtensionHomeStatus(&r)==1&&queried==1&&sent==0);puts("PASS status read fixed node6");return 0;}
 if(is("null")){assert(CAN1_StartExtensionHome(0)==7&&sent==0);puts("PASS null rejected");return 0;}
 if(is("wrong_node")||is("wrong_packet")||is("wrong_ide")||is("wrong_rtr")||is("wrong_command")||is("no_reply"))expected=5;
 if(is("bad_length")||is("bad_tail"))expected=6;
 if(is("bad_ack"))expected=8;
 if(is("tx_fail")||is("tx_timeout")||is("no_mailbox"))expected=4;
 if(is("init_fail"))expected=3;
 if(is("disabled"))expected=9;
 result=CAN1_StartExtensionHome(&r);assert(result==expected);
 if(is("notice"))assert(g_can_extension_home_done_notice==1);
 if(is("stale_start"))assert(flushed==3&&g_can_extension_home_done_notice==0);
 if(is("wrong_node"))assert(g_can_lift_home_done_notice==1&&g_can_extension_home_done_notice==0);
 if(is("tx_fail")||is("tx_timeout"))assert(cancels==1);
 if(expected==9||expected==3)assert(sent==0);
 printf("PASS home transport %s result=%u\n",scenario,result);return 0;
}
'''
can=(STAGE/'SYSTEM/CAN/can.c').read_bytes()
signatures=[b'static void CAN1_CaptureLiftHomeNotice(',b'uint32_t CAN1_StartExtensionHome(',
            b'uint32_t CAN1_AbortExtensionHome(',b'uint32_t CAN1_ReadExtensionHomeStatus(',
            b'void CAN1_ExtensionHomeSessionEnd(']
port=PORT.replace('__FUNCTIONS__','\n'.join(extract(can,s).decode('utf-8') for s in signatures))
(HOST/'home-port.c').write_text(port,encoding='utf-8')
exe=HOST/'home-port.exe';run(common+[str(HOST/'home-port.c'),'-o',str(exe)])
cases=['valid','notice','stale_start','wrong_node','wrong_packet','wrong_ide','wrong_rtr','wrong_command',
       'no_reply','bad_length','bad_tail','bad_ack','tx_fail','tx_timeout','no_mailbox','init_fail','null',
       'abort','bad_abort_ack','abort_transport_fail','status','session_end','capture_valid','capture_off',
       'capture_node5','capture_rtr','capture_length','capture_tail','capture_ack_only']
for case in cases:records.append(dict(group='home-port',case=case,output=run([str(exe),case])))
exe=HOST/'home-port-disabled.exe';run(common+['-DAPP_ARM_LIFT_TEST=0',str(HOST/'home-port.c'),'-o',str(exe)])
records.append(dict(group='home-port',case='disabled',output=run([str(exe),'disabled'])))
m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
r=dict(passed=True,cases=records,case_count=len(records),
       source_sha256={e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in m['files']},
       synthetic_only=True,hardware_acceptance=False)
(EV/'home-motion-review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(EV/'home-motion-host-check.log').write_text('\n'.join(e['output'] for e in records)+'\n',encoding='utf-8')
print(json.dumps(dict(passed=True,cases=len(records)),ensure_ascii=False))
