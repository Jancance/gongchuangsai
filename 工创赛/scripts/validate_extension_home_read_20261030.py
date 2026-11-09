"""验证节点6参数解码/互锁及真实CAN三帧解析；不连接硬件。"""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--sensorless', action='store_true')
args = parser.parse_args()
profile = 'extension_sensorless_20261030' if args.sensorless else 'extension_home_read_20261030'
evidence = 'extension-sensorless' if args.sensorless else 'extension-home-read'
STAGE = ROOT/'tmp'/profile/'project'
HOST = ROOT/'tmp'/profile/'host-check'
EV = ROOT/'evidence/p2-arm/2026-10-30'/evidence
GCC = 'C:/mingw/mingw64/bin/gcc.exe'


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    assert p.returncode == 0, (cmd, p.stdout, p.stderr)
    return p.stdout.strip()


def function(data, name):
    start = data.index(('uint32_t '+name+'(').encode())
    end = data.index(b'{', start)+1
    depth = 1
    while depth:
        if data[end] == 123: depth += 1
        elif data[end] == 125: depth -= 1
        end += 1
    return data[start:end]


MODEL = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "extension.h"
static const char *scenario;
static unsigned calls;
static int is(const char*s){return strcmp(scenario,s)==0;}
volatile uint32_t g_extension_home_request=0;
uint32_t ExtensionHome_IsBusy(void){return 0;}
TickType_t xTaskGetTickCount(void){return 0;}
void vTaskDelay(TickType_t t){(void)t;}
uint32_t CAN1_ReadVersion(uint8_t a,CanRxMsg*r){(void)a;(void)r;assert(0);return 5;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){(void)a;(void)r;assert(0);return 5;}
uint32_t CAN1_ReadMotorOptions(uint8_t a,CanRxMsg*r){(void)a;(void)r;assert(0);return 5;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v){(void)a;(void)r;(void)v;assert(0);return 5;}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*s,uint32_t*v){(void)a;(void)r;(void)s;(void)v;assert(0);return 5;}
uint32_t CAN1_ExtensionExchange(const uint8_t*q,uint8_t n,CanRxMsg*r){(void)q;(void)n;(void)r;assert(!"read-only path issued a control command");return 5;}
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t p[17],CanRxMsg r[3]){
 const uint8_t sample[17]={0x22,2,0,0,12,0,0,0x3A,0x98,0,90,1,0x5E,0,150,0,0x6B};
 calls++;memcpy(p,sample,17);memset(r,0,3*sizeof(*r));
 if(is("transport_fail"))return 5;
 if(is("direction1"))p[2]=1;
 if(is("auto1"))p[15]=1;
 if(is("mode0"))p[1]=0;
 if(is("mode_bad"))p[1]=6;
 if(is("direction_bad"))p[2]=2;
 if(is("auto_bad"))p[15]=2;
 if(is("rpm_bad")){p[3]=0x0B;p[4]=0xB9;}
 if(is("detect_rpm_bad")){p[9]=0x0B;p[10]=0xB9;}
 if(is("current_bad")){p[11]=0x13;p[12]=0x89;}
 if(is("head_bad"))p[0]=0x23;
 if(is("tail_bad"))p[16]=0;
 return 1;
}
int main(int argc,char**argv){
 unsigned i,expected=1;assert(argc==2);scenario=argv[1];
 Extension_HomeReadService(1,0);assert(calls==0&&g_extension_home_read_result==0);
 g_extension_home_read_request=is("invalid_request")?2:1;
 if(is("motion_busy"))g_extension_motion_state=6;
 if(is("motion_pending"))g_extension_motion_request=1;
 if(is("identity_busy"))g_extension_state=2;
 Extension_HomeReadService(!is("not_ready"),is("other_busy"));
 if(is("not_ready")||is("other_busy")||is("motion_busy")||is("motion_pending")||is("identity_busy")||is("invalid_request"))expected=7;
 if(strstr(scenario,"_bad"))expected=6;
 if(is("transport_fail"))expected=5;
 assert(g_extension_home_read_result==expected&&g_extension_home_read_request==0);
 if(expected==7)assert(calls==0);
 else assert(calls==1);
 if(expected==1){
  assert(g_extension_home_params.mode==(is("mode0")?0:2));
  assert(g_extension_home_params.direction==(is("direction1")?1:0));
  assert(g_extension_home_params.rpm==12&&g_extension_home_params.timeout_ms==15000);
  assert(g_extension_home_params.detect_rpm==90&&g_extension_home_params.detect_current_ma==350);
  assert(g_extension_home_params.detect_time_ms==150&&g_extension_home_params.auto_home==(is("auto1")?1:0));
 }else{ExtensionHomeParameters zero={0};assert(memcmp(&g_extension_home_params,&zero,sizeof(zero))==0);}
 for(i=0;i<5;i++)Extension_HomeReadService(1,0);
 assert(calls==(expected==7?0:1));
 if(is("invalidate")){Extension_HomeReadService(0,0);assert(g_extension_home_read_result==0&&g_extension_home_params.rpm==0);}
 if(is("repeat")){g_extension_home_read_request=1;Extension_HomeReadService(1,0);assert(calls==2&&g_extension_home_read_result==1);}
 assert(g_extension_motion_state==(is("motion_busy")?6:0));
 printf("PASS home module %s result=%u\n",scenario,expected);return 0;
}
'''
(HOST/'home-module.c').write_text(MODEL, encoding='utf-8')
module = STAGE/'HARDWARE/EXTENSION'
common = [GCC, '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(HOST), '-I'+str(module)]
exe = HOST/'home-module.exe'
run(common+[str(module/'extension.c'), str(HOST/'home-module.c'), '-o', str(exe)])
records = []
for case in ['valid','direction1','auto1','mode0','mode_bad','direction_bad','auto_bad','rpm_bad',
             'detect_rpm_bad','current_bad','head_bad','tail_bad','transport_fail','not_ready',
             'other_busy','motion_busy','motion_pending','identity_busy','invalid_request','invalidate','repeat']:
    records.append(dict(group='module', case=case, output=run([str(exe),case])))

TRANSPORT = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
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
static unsigned sent,index_rx,cancels,flushed;
static uint32_t s_can1_normal_ready;
static const uint8_t sample[17]={0x22,2,0,0,12,0,0,0x3A,0x98,0,90,1,0x5E,0,150,0,0x6B};
static int is(const char*s){return strcmp(scenario,s)==0;}
TickType_t xTaskGetTickCount(void){return now;}
void vTaskDelay(TickType_t n){now+=n;}
uint32_t CAN1_InitMode(unsigned mode){(void)mode;return is("init_fail")?3:1;}
unsigned CAN_MessagePending(unsigned can,unsigned fifo){
 (void)can;(void)fifo;if(!sent)return is("stale_start")&&flushed<3;
 if(is("no_reply")||is("wrap_no_reply"))return 0;
 return index_rx<(is("missing_last")?2U:3U);}
void CAN_Receive(unsigned can,unsigned fifo,CanRxMsg*r){
 unsigned p=index_rx,offset=1+7*p,chunk=p==2?2:7;
 (void)can;(void)fifo;memset(r,0,sizeof(*r));
 if(!sent){flushed++;return;}
 index_rx++;r->ExtId=(is("wrong_node")?0x500:0x600)|p;
 if(is("duplicate")&&p==1)r->ExtId=0x600;
 if(is("out_of_order")&&p==0)r->ExtId=0x601;
 r->IDE=is("wrong_ide")?0:4;r->RTR=is("wrong_rtr")?1:0;
 r->DLC=(uint8_t)(chunk+1);r->Data[0]=is("wrong_command")?0x23:0x22;
 memcpy(r->Data+1,sample+offset,chunk);
 if(is("first_length")&&p==0)r->DLC=7;
 if(is("last_length")&&p==2)r->DLC=2;
 if(is("last_tail")&&p==2)r->Data[2]=0;
}
unsigned CAN_Transmit(unsigned can,CanTxMsg*t){
 (void)can;assert(t->ExtId==0x600&&t->IDE==4&&t->RTR==0&&t->DLC==2&&t->Data[0]==0x22&&t->Data[1]==0x6B);
 sent++;return is("no_mailbox")?4:0;}
unsigned CAN_TransmitStatus(unsigned can,unsigned mailbox){
 (void)can;(void)mailbox;return is("tx_fail")?0:(is("tx_timeout")?2:1);}
void CAN_CancelTransmit(unsigned can,unsigned mailbox){(void)can;(void)mailbox;cancels++;}
__FUNCTION__
int main(int argc,char**argv){
 uint8_t p[17];CanRxMsg r[3];uint32_t result,expected=1;unsigned i;
 assert(argc==2);scenario=argv[1];
 if(is("null")){assert(CAN1_ReadExtensionHomeParameters(0,r)==7&&CAN1_ReadExtensionHomeParameters(p,0)==7&&sent==0);puts("PASS home transport null");return 0;}
 if(is("wrap_no_reply"))now=0xFFFFFFF0;
 if(is("wrong_node")||is("wrong_ide")||is("wrong_rtr")||is("wrong_command")||is("missing_last")||is("no_reply")||is("wrap_no_reply"))expected=5;
 if(is("duplicate")||is("out_of_order")||is("first_length")||is("last_length")||is("last_tail"))expected=6;
 if(is("no_mailbox")||is("tx_fail")||is("tx_timeout"))expected=4;
 if(is("init_fail"))expected=3;
 if(is("disabled"))expected=9;
 result=CAN1_ReadExtensionHomeParameters(p,r);assert(result==expected);
 if(expected==1){assert(memcmp(p,sample,17)==0);for(i=0;i<3;i++)assert(r[i].ExtId==(0x600|i)&&r[i].Data[0]==0x22);}
 if(is("stale_start"))assert(flushed==3);
 if(is("tx_fail")||is("tx_timeout"))assert(cancels==1);
 if(expected==9||expected==3)assert(sent==0);
 printf("PASS home transport %s result=%u\n",scenario,result);return 0;
}
'''
current_can = (STAGE/'SYSTEM/CAN/can.c').read_bytes()
text = TRANSPORT.replace('__FUNCTION__', function(current_can,'CAN1_ReadExtensionHomeParameters').decode('utf-8'))
(HOST/'home-transport.c').write_text(text, encoding='utf-8')
exe = HOST/'home-transport.exe'
run(common+[str(HOST/'home-transport.c'), '-o', str(exe)])
for case in ['valid','stale_start','duplicate','out_of_order','first_length','last_length','last_tail',
             'wrong_node','wrong_ide','wrong_rtr','wrong_command','missing_last','no_reply','wrap_no_reply',
             'no_mailbox','tx_fail','tx_timeout','init_fail','null']:
    records.append(dict(group='transport', case=case, output=run([str(exe),case])))
disabled = HOST/'home-transport-disabled.exe'
run(common+['-DAPP_ARM_LIFT_TEST=0', str(HOST/'home-transport.c'), '-o', str(disabled)])
records.append(dict(group='transport', case='disabled', output=run([str(disabled),'disabled'])))

old_can = (EV/'original/SYSTEM/CAN/can.c').read_bytes()
old_extension = (EV/'original/HARDWARE/EXTENSION/extension.c').read_bytes()
if not args.sensorless:
    assert current_can.startswith(old_can)
    assert (module/'extension.c').read_bytes().startswith(old_extension)
else:
    # 新增的回零互锁以外，保留原小步和参数解释；统一换行仅用于对照。
    ext=(module/'extension.c').read_bytes().replace(b'\r\n',b'\n')
    ext=ext.replace(b'if (ExtensionHome_IsBusy() || Extension_MotionIsBusy()',b'if (Extension_MotionIsBusy()',1)
    guard=b'''    if (ExtensionHome_IsBusy())
    {
        if (request == 3U) { g_extension_home_request = 2U; return; }
        other_busy = 1U;
    }
'''
    assert ext.count(guard)==1
    assert ext.replace(guard,b'',1)==old_extension.replace(b'\r\n',b'\n')
    capture=b'''    if (s_extension_home_capture && rx->IDE == CAN_Id_Extended && rx->RTR == CAN_RTR_Data &&
        rx->ExtId == 0x0600U && rx->DLC == 3U && rx->Data[0] == 0x9AU &&
        rx->Data[1] == 0x9FU && rx->Data[2] == 0x6BU)
        g_can_extension_home_done_notice = 1U;
'''
    restored=current_can.replace(b'\r\n',b'\n')
    globals_added=b'volatile uint32_t g_can_extension_home_done_notice = 0U;\nstatic uint32_t s_extension_home_capture = 0U;\n'
    assert restored.count(globals_added)==restored.count(capture)==1
    restored=restored.replace(globals_added,b'',1).replace(capture,b'',1)
    assert restored.startswith(old_can.replace(b'\r\n',b'\n'))
assert function(current_can,'CAN1_ReadLiftHomeParameters') == function(old_can,'CAN1_ReadLiftHomeParameters')
assert function(current_can,'CAN1_ExtensionExchange') == function(old_can,'CAN1_ExtensionExchange')
m = json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
r = dict(passed=True, cases=records, case_count=len(records),
         source_sha256={e['file']:hashlib.sha256((STAGE/e['file']).read_bytes()).hexdigest() for e in m['files']},
         original_commands_and_decoder_preserved=True, synthetic_only=True, hardware_acceptance=False)
(EV/'home-read-review.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(EV/'home-read-host-check.log').write_text('\n'.join(e['output'] for e in records)+'\n',encoding='utf-8')
print(json.dumps(dict(passed=True, cases=len(records)),ensure_ascii=False))
