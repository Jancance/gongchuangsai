"""以合成回包运行真实节点6状态机、传输函数及隔离任务分支，不访问硬件。"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--step', type=int, choices=[16, 160], default=16)
parser.add_argument('--home-read', action='store_true')
parser.add_argument('--sensorless', action='store_true')
args = parser.parse_args()
profile = 'extension_jog_20261030' if args.step == 16 else 'extension_jog160_20261030'
evidence_name = 'extension-jog' if args.step == 16 else 'extension-jog160'
if args.home_read:
    assert args.step == 160
    profile, evidence_name = 'extension_home_read_20261030', 'extension-home-read'
if args.sensorless:
    assert args.step == 160
    profile, evidence_name = 'extension_sensorless_20261030', 'extension-sensorless'
STAGE = ROOT/'tmp'/profile/'project'
HOST = ROOT/'tmp'/profile/'host-check'
EV = ROOT/'evidence/p2-arm/2026-10-30'/evidence_name
GCC = 'C:/mingw/mingw64/bin/gcc.exe'
HOST.mkdir(parents=True, exist_ok=True)


def function(data, name):
    start = data.index(('uint32_t '+name+'(').encode())
    brace = data.index(b'{', start)
    depth = 1
    end = brace+1
    while depth:
        if data[end] == 123: depth += 1
        elif data[end] == 125: depth -= 1
        end += 1
    return data[start:end]


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    assert p.returncode == 0, (cmd, p.stdout, p.stderr)
    return p.stdout.strip()


(HOST/'stm32f4xx_can.h').write_text('''#ifndef FAKE_CAN_H
#define FAKE_CAN_H
#include <stdint.h>
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC,Data[8],FMI; } CanRxMsg;
typedef CanRxMsg CanTxMsg;
#endif
''')
(HOST/'FreeRTOS.h').write_text('''#ifndef FAKE_RTOS_H
#define FAKE_RTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(n) ((TickType_t)(n))
#define portTICK_PERIOD_MS 1U
#endif
''')
(HOST/'task.h').write_text('''#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
''')
(HOST/'can.h').write_text('''#include "stm32f4xx_can.h"
uint32_t CAN1_ReadVersion(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadMotorOptions(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*);
uint32_t CAN1_ReadPosition(uint8_t,CanRxMsg*,uint8_t*,uint32_t*);
uint32_t CAN1_ExtensionExchange(const uint8_t*,uint8_t,CanRxMsg*);
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t*,CanRxMsg*);
extern volatile uint32_t g_can_extension_home_done_notice;
uint32_t CAN1_StartExtensionHome(CanRxMsg*);
uint32_t CAN1_AbortExtensionHome(CanRxMsg*);
uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg*);
void CAN1_ExtensionHomeSessionEnd(void);
''')

MODEL = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "extension.h"
static const char *scenario;
static TickType_t now;
static unsigned reads,moves,stops,enables,moved;
static int is(const char*s){return strcmp(scenario,s)==0;}
volatile uint32_t g_extension_home_request=0;
uint32_t ExtensionHome_IsBusy(void){return 0;}
void ExtensionHome_Service(uint32_t ready,uint32_t busy){(void)ready;(void)busy;assert(g_extension_home_request==0);}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t ticks){now+=ticks;}
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t*p,CanRxMsg*r){(void)p;(void)r;assert(!"unexpected home read in motion regression");return 5;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){assert(a==6);reads++;r->Data[0]=0x1F;return 1;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){
 assert(a==6);reads++;r->Data[0]=0x1A;r->Data[2]=is("bad_options")?0:6;return 1;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){
 assert(a==6);reads++;
 if(is("read_fail")&&moved)return 5;
 r->Data[0]=0x3A;r->Data[1]=3;
 if(is("enable")&&!enables)r->Data[1]=2;
 if(is("fault")&&moved)r->Data[1]=7;
 if(is("disable")&&moved)r->Data[1]=2;
 if(is("no_arrival")&&moved)r->Data[1]=1;
 return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){
 assert(a==6);reads++;r->Data[0]=0x35;
 *v=(is("pre_moving")|| (is("no_stop")&&moved))?1:0;return 1;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v){
 assert(a==6);reads++;r->Data[0]=0x36;
 *s=0;*v=1000;
 if(is("signed")){*s=moved?0:1;*v=20;return 1;}
 if(moved&&!is("ack_only"))*v=(is("negative")?980:1020);
 return 1;}
uint32_t CAN1_ExtensionExchange(const uint8_t*b,uint8_t len,CanRxMsg*r){
 r->Data[0]=b[0];r->Data[1]=2;r->Data[2]=0x6B;
 if(b[0]==0xF3){assert(len==5&&b[2]==1&&b[3]==0&&b[4]==0x6B);enables++;return 1;}
 if(b[0]==0xFE){assert(len==4&&b[1]==0x98&&b[2]==0&&b[3]==0x6B);stops++;return is("stop_fail")?5:1;}
 assert(len==12&&b[0]==0xFD&&b[1]==(is("negative")?1:0));
 assert(b[2]==0&&b[3]==5&&b[4]==20&&b[5]==0&&b[6]==0&&b[7]==0&&b[8]==16);
 assert(b[9]==2&&b[10]==0&&b[11]==0x6B);moves++;
 if(is("move_fail"))return 5;
 moved=1;return 1;}

#ifndef INTEGRATION
int main(int argc,char**argv){
 unsigned i,expected=1,before;uint32_t ready=1,busy=0;
 assert(argc==2);scenario=argv[1];
 if(is("wrap"))now=0xFFFFFFE0U;
 Extension_Service(1,0);Extension_MotionService(1,0);
 assert(reads==0&&moves==0&&stops==0&&enables==0);
 if(is("readonly")){
  g_extension_read_request=1;Extension_Service(1,0);Extension_Service(1,0);Extension_Service(1,0);
  assert(g_extension_state==4&&reads==3&&moves==0);puts("PASS readonly preserved");return 0;}
 g_extension_motion_request=is("negative")?2:1;
 for(i=0;i<450;i++){
  now+=10;ready=!(is("not_ready")||(is("power_loss")&&moved));
  busy=is("busy")||(is("busy_during")&&moved);
  if(is("repeat")&&Extension_MotionIsBusy())g_extension_motion_request=1;
  if(is("edit_direction")&&Extension_MotionIsBusy())g_extension_motion_direction=1;
  if((is("abort")||is("stop_fail"))&&moved)g_extension_motion_request=3;
  if(is("gap")&&moved)now+=600;
  if(is("readonly_during")&&moved)g_extension_read_request=1;
  Extension_Service(ready,busy);Extension_MotionService(ready,busy);
  if(g_extension_motion_state>=7)break;
 }
 if(is("ack_only")||is("no_stop")||is("no_arrival"))expected=3;
 if(is("bad_options")||is("move_fail")||is("read_fail")||is("stop_fail"))expected=4;
 if(is("fault")||is("disable")||is("pre_moving"))expected=5;
 if(is("not_ready")||is("busy")||is("power_loss")||is("busy_during"))expected=6;
 if(is("gap"))expected=7;
 if(is("abort"))expected=2;
 assert(g_extension_motion_result==expected);
 if(expected==1){assert(g_extension_motion_state==7&&moves==1&&stops==0);
  assert(g_extension_delta_raw==(is("negative")?-20:(is("signed")?40:20)));
  before=moves;for(i=0;i<8;i++){now+=10;Extension_Service(1,0);Extension_MotionService(1,0);}assert(moves==before);}
 else if(expected==2){assert(g_extension_motion_state==8&&stops==1);}
 else{assert(g_extension_motion_state==9);before=moves;
  g_extension_motion_request=1;Extension_MotionService(1,0);assert(moves==before);}
 if(is("not_ready")||is("busy"))assert(reads==0&&moves==0&&stops==0);
 if(is("bad_options"))assert(moves==0&&stops==0);
 if(is("pre_moving"))assert(moves==0&&stops==1);
 if(is("enable"))assert(enables==1);
 if(is("readonly_during"))assert(g_extension_read_request==0&&reads<20);
 if(is("stop_fail"))assert(g_extension_stop_result==5);
 printf("PASS motion %s result=%u moves=%u stops=%u\n",scenario,expected,moves,stops);return 0;
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
 assert(argc==2);scenario=argv[1];
 if(is("dm_busy")){dm_busy=1;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("lift_busy")){g_lift_motion_state=6;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("home_busy")){home_busy=1;g_extension_motion_request=1;step();assert(moves==0&&reads==0&&g_extension_motion_result==6);}
 else if(is("simultaneous_lift")){g_lift_motion_request=1;g_extension_motion_request=1;step();assert(lift_started==1&&moves==0&&g_extension_motion_result==6);}
 else if(is("simultaneous_dm")){dm_request=4;g_extension_motion_request=1;step();assert(Extension_MotionIsBusy()&&dm_started==0);}
 else if(is("readonly_busy")){g_extension_read_request=1;g_extension_motion_request=0;step();assert(g_extension_state==2&&moves==0);}
 else{g_extension_motion_request=1;step();assert(Extension_MotionIsBusy());
  g_lift_motion_request=1;g_lift_home_request=1;g_lift_read_request=1;g_lift_home_read_request=1;dm_request=4;
  step();assert(lift_started==0&&dm_started==0&&g_lift_home_request==0&&g_lift_read_request==0&&g_lift_home_read_request==0);}
 printf("PASS integration %s\n",scenario);return 0;
}
#endif
'''
main_bytes = (STAGE/'USER/main.c').read_bytes().replace(b'\r\n', b'\n')
a = main_bytes.index(b'        if (ArmPower_IsIsolatedMode() != 0U)')
a = main_bytes.index(b'{', a)+1
z = main_bytes.index(b'            g_can_task_stack_words =', a)
model = MODEL.replace('__BRANCH__', main_bytes[a:z].decode('utf-8', errors='replace'))
if args.step == 160:
    assert model.count('b[8]==16') == 1
    model = model.replace('b[8]==16', 'b[8]==160')
(HOST/'motion-model.c').write_text(model, encoding='utf-8')
module = STAGE/'HARDWARE/EXTENSION'
common = [GCC, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(HOST), '-I'+str(module)]
records = []
for executable, defines, cases in [
    ('motion', [], ['positive','negative','enable','wrap','signed','repeat','edit_direction',
                   'ack_only','no_stop','no_arrival','bad_options','pre_moving','fault','disable',
                   'read_fail','move_fail','not_ready','busy','power_loss','busy_during','gap','abort',
                   'stop_fail','readonly','readonly_during']),
    ('integration', ['-DINTEGRATION'], ['dm_busy','lift_busy','home_busy','simultaneous_lift',
                                      'simultaneous_dm','readonly_busy','extension_busy'])]:
    exe = HOST/(executable+'.exe')
    run(common+defines+[str(module/'extension.c'), str(HOST/'motion-model.c'), '-o', str(exe)])
    for case in cases:
        records.append(dict(group=executable, case=case, output=run([str(exe),case])))

TRANSPORT = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "extension_config.h"
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
static const char *scenario;
static TickType_t now;
static unsigned sent,available,cancels,small;
static uint32_t s_can1_normal_ready;
static CanTxMsg txs[2];
static int is(const char*s){return strcmp(scenario,s)==0;}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t t){now+=t;}
uint32_t CAN1_InitMode(unsigned mode){(void)mode;return is("init_fail")?3:1;}
unsigned CAN_MessagePending(unsigned can,unsigned fifo){(void)can;(void)fifo;return available;}
void CAN_Receive(unsigned can,unsigned fifo,CanRxMsg*r){
 (void)can;(void)fifo;memset(r,0,sizeof(*r));available=0;
 r->ExtId=is("wrong_node")?0x500:0x600;r->IDE=is("wrong_ide")?0:4;r->RTR=is("wrong_rtr")?1:0;
 r->DLC=is("bad_length")?2:3;r->Data[0]=0xFD;r->Data[1]=is("bad_ack")?1:(is("notice")?0x9F:2);r->Data[2]=is("bad_tail")?0:0x6B;
}
unsigned CAN_Transmit(unsigned can,CanTxMsg*t){
 (void)can;assert(sent<2);txs[sent++]=*t;if(sent==2&&!is("no_reply"))available=1;
 return is("no_mailbox")?4:0;}
unsigned CAN_TransmitStatus(unsigned can,unsigned mailbox){
 (void)can;(void)mailbox;return is("tx_fail")?0:(is("tx_timeout")?2:1);}
void CAN_CancelTransmit(unsigned can,unsigned mailbox){(void)can;(void)mailbox;cancels++;}
uint32_t CAN1_ExchangeSingle(uint8_t a,const uint8_t*q,uint8_t n,uint8_t expected,CanRxMsg*r){
 assert(a==6&&expected==3&&((n==5&&q[0]==0xF3&&q[2]==1)||(n==4&&q[0]==0xFE)));
 small++;r->Data[1]=is("bad_small_ack")?1:2;return 1;}
__FUNCTION__
int main(int argc,char**argv){
 uint8_t q[12]={0xFD,0,0,5,20,0,0,0,16,2,0,0x6B};CanRxMsg r;uint32_t expected=1,result;
 assert(argc==2);scenario=argv[1];
 if(is("bad_direction"))q[1]=2;
 if(is("too_fast"))q[3]=6;
 if(is("high_speed_byte"))q[2]=1;
 if(is("zero_speed"))q[3]=0;
 if(is("zero_pulses"))q[8]=0;
 if(is("too_many_pulses"))q[8]=17;
 if(is("high_pulse_byte"))q[5]=1;
 if(is("bad_mode"))q[9]=1;
 if(is("synchronous"))q[10]=1;
 if(is("bad_checksum"))q[11]=0;
 if(is("too_much_accel"))q[4]=21;
 if(is("zero_accel"))q[4]=0;
 if(is("enable")||is("bad_small_ack")){uint8_t e[5]={0xF3,0xAB,1,0,0x6B};result=CAN1_ExtensionExchange(e,5,&r);assert(result==(is("enable")?1:8)&&small==1);puts("PASS transport enable ack");return 0;}
 if(is("stop")){uint8_t s[4]={0xFE,0x98,0,0x6B};assert(CAN1_ExtensionExchange(s,4,&r)==1&&small==1);puts("PASS transport stop");return 0;}
 if(is("unlock")){uint8_t e[5]={0xF3,0xAB,0,0,0x6B};assert(CAN1_ExtensionExchange(e,5,&r)==7&&small==0&&sent==0);puts("PASS transport unlock rejected");return 0;}
 if(is("null")){assert(CAN1_ExtensionExchange(0,12,&r)==7);assert(CAN1_ExtensionExchange(q,12,0)==7);puts("PASS transport null rejected");return 0;}
 if(is("short")){assert(CAN1_ExtensionExchange(q,3,&r)==7&&sent==0);puts("PASS transport short rejected");return 0;}
 if(is("bad_direction")||is("too_fast")||is("high_speed_byte")||is("zero_speed")||is("zero_pulses")||is("too_many_pulses")||is("high_pulse_byte")||is("bad_mode")||is("synchronous")||is("bad_checksum")||is("too_much_accel")||is("zero_accel"))expected=7;
 if(is("wrong_node")||is("wrong_ide")||is("wrong_rtr")||is("no_reply"))expected=5;
 if(is("bad_length")||is("bad_tail"))expected=6;
 if(is("bad_ack"))expected=8;
 if(is("tx_fail")||is("tx_timeout")||is("no_mailbox"))expected=4;
 if(is("init_fail"))expected=3;
 if(is("disabled"))expected=9;
 result=CAN1_ExtensionExchange(q,12,&r);assert(result==expected);
 if(expected==1){const uint8_t first[8]={0xFD,0,0,5,20,0,0,0};const uint8_t second[5]={0xFD,16,2,0,0x6B};
  assert(sent==2&&txs[0].ExtId==0x600&&txs[1].ExtId==0x601);
  assert(txs[0].IDE==4&&txs[0].RTR==0&&txs[0].DLC==8&&txs[1].DLC==5);
  assert(memcmp(txs[0].Data,first,8)==0&&memcmp(txs[1].Data,second,5)==0);}
 if(expected==7||expected==9||expected==3)assert(sent==0&&small==0);
 if(is("tx_timeout")||is("tx_fail"))assert(cancels==1);
 assert(now<=300);printf("PASS transport %s result=%u\n",scenario,result);return 0;
}
'''
can_bytes = (STAGE/'SYSTEM/CAN/can.c').read_bytes()
transport = TRANSPORT.replace('__FUNCTION__', function(can_bytes, 'CAN1_ExtensionExchange').decode('utf-8'))
if args.step == 160:
    for old, new in [('0,0,0,16,2,0,0x6B', '0,0,0,160,2,0,0x6B'),
                     ('q[8]=17', 'q[8]=161'),
                     ('second[5]={0xFD,16,2,0,0x6B}', 'second[5]={0xFD,160,2,0,0x6B}')]:
        assert transport.count(old) == 1, old
        transport = transport.replace(old, new)
(HOST/'transport.c').write_text(transport, encoding='utf-8')
exe = HOST/'transport.exe'
run(common+[str(HOST/'transport.c'), '-o', str(exe)])
cases = ['valid','notice','enable','stop','bad_small_ack','unlock','null','short','bad_direction',
         'too_fast','high_speed_byte','zero_speed','zero_pulses','too_many_pulses','high_pulse_byte',
         'bad_mode','synchronous','bad_checksum','too_much_accel','zero_accel','wrong_node','wrong_ide',
         'wrong_rtr','no_reply','bad_length','bad_tail','bad_ack','tx_fail','tx_timeout','no_mailbox','init_fail']
for case in cases:
    records.append(dict(group='transport', case=case, output=run([str(exe),case])))
disabled = HOST/'transport-disabled.exe'
run(common+['-DAPP_ARM_LIFT_TEST=0', str(HOST/'transport.c'), '-o', str(disabled)])
records.append(dict(group='transport', case='disabled', output=run([str(disabled),'disabled'])))
original = (ROOT/'evidence/p2-arm/2026-10-30/extension-jog/original/SYSTEM/CAN/can.c').read_bytes()
assert function(original,'CAN1_LiftExchange') == function(can_bytes,'CAN1_LiftExchange')
# 原底盘控制函数为static；节点地址保护所在的全部旧CAN内容除新增include外逐字节保持。
if not args.sensorless:
    assert can_bytes[:len(original)+len(b'\n#include "extension_config.h"')].replace(
        b'\n#include "extension_config.h"',b'',1) == original
else:
    before = (EV/'original/SYSTEM/CAN/can.c').read_bytes()
    for name in ['CAN1_ControlSingle','CAN1_LiftExchange','CAN1_ExtensionExchange',
                 'CAN1_ReadLiftHomeParameters','CAN1_ReadExtensionHomeParameters']:
        assert function(can_bytes,name)==function(before,name),name
m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
report = dict(passed=True, cases=records, case_count=len(records),
              expected_pulses=args.step,
              source_sha256={e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in m['files']},
              legacy_control_functions_preserved=True, synthetic_only=True, hardware_acceptance=False)
(EV/'host-review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
(EV/'host-check.log').write_text('\n'.join(r['output'] for r in records)+'\n', encoding='utf-8')
print(json.dumps(dict(passed=True, cases=len(records), groups={g:sum(r['group']==g for r in records) for g in ['motion','integration','transport']}),ensure_ascii=False))
