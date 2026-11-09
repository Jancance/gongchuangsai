"""执行真实CAN传输/会话捕获函数，CAN外设为内存队列。"""
from pathlib import Path
import json
import re
import subprocess

R = Path(__file__).resolve().parents[1]
S = R / 'tmp/startup_group_20261103'
O = R / 'evidence/integration/2026-11-03/startup-group'
text = (S / 'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('gbk')


def function(name):
    match = re.search(r'^(?:static )?(?:uint32_t|void) ' + name + r'\([^;]*?\)\s*\{', text, re.M | re.S)
    assert match, name
    start = match.start()
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return re.sub(r'/\*.*?\*/', '', text[start:end], flags=re.S)


unit = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define VEHICLE_EXTENSION_HOME_ONLY 0
#define VEHICLE_STARTUP_GROUP_ONLY 1
#define CAN1 1
#define CAN_FIFO0 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_Mode_Normal 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 2
#define CAN1_SINGLE_TX_WAIT_MS 20
#define CAN1_SINGLE_REPLY_WAIT_MS 30
#define pdMS_TO_TICKS(x) (x)
typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId,IDE,RTR;uint8_t DLC,Data[8];} CanRxMsg;
typedef CanRxMsg CanTxMsg;
uint32_t tick,s_can1_normal_ready,s_can_ctrl_trace_active,s_can_ctrl_trace_tick;
uint32_t g_can_ctrl_trace_flush_count,g_can_ctrl_trace_phase,g_can_ctrl_trace_tx_ms,g_can_ctrl_trace_rx_count;
CanRxMsg g_can_ctrl_trace_last_rx;
uint32_t g_startup_axes_vm_request=1,g_startup_axes_state=2,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
volatile uint32_t g_extension_home_capture,g_extension_home_done_notice;
static CanRxMsg queue[20];static unsigned used,cursor,tx_count,packet_fault,command;
uint32_t xTaskGetTickCount(void){return tick++;}
void vTaskDelay(uint32_t n){tick+=n;}
uint32_t CAN1_InitMode(uint32_t mode){(void)mode;return 1;}
uint32_t CAN_MessagePending(uint32_t can,uint32_t fifo){(void)can;(void)fifo;return used-cursor;}
void CAN_Receive(uint32_t can,uint32_t fifo,CanRxMsg *p){(void)can;(void)fifo;*p=queue[cursor++];}
void CAN_CancelTransmit(uint32_t can,uint8_t box){(void)can;(void)box;}
uint8_t CAN_TransmitStatus(uint32_t can,uint8_t box){(void)can;(void)box;return CAN_TxStatus_Ok;}
static void notice(uint32_t id,uint32_t dlc,uint32_t tail){
 CanRxMsg p={0};p.ExtId=id;p.IDE=1;p.DLC=dlc;p.Data[0]=0x9a;p.Data[1]=0x9f;p.Data[2]=tail;queue[used++]=p;
}
uint8_t CAN_Transmit(uint32_t can,CanTxMsg *q){
 CanRxMsg p={0};(void)can;tx_count++;command=q->Data[0];
 if(command==0x22){
  uint8_t data[17]={0x22,0,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};unsigned offset=1;
  for(unsigned packet=0;packet<3;packet++){
   unsigned chunk=17-offset;if(chunk>7)chunk=7;
   memset(&p,0,sizeof(p));p.ExtId=0x600+packet;p.IDE=1;p.DLC=chunk+1;p.Data[0]=0x22;
   memcpy(p.Data+1,data+offset,chunk);offset+=chunk;
   if(packet_fault==1 && packet==1)p.ExtId=0x600;
   if(packet_fault==2 && packet==2)p.Data[chunk]=0;
   if(packet_fault==3 && packet==1)p.DLC--;
   queue[used++]=p;
  }
 }else{
  p.ExtId=q->ExtId;p.IDE=1;p.DLC=3;p.Data[0]=command;p.Data[1]=2;p.Data[2]=0x6b;
  if(command==0x9a && packet_fault==4){p.Data[1]=0x9f;}
  queue[used++]=p;
 }return 0;
}
'''
unit += '\n'.join(function(n) for n in ['StartupHome_Receive', 'CAN1_ExchangeSingle',
                                      'VehicleArm_Exchange'])
# The offline build deliberately has no parameter reader.
unit += '\n#if !APP_MOTOR_OFFLINE_ONLY\n' + function('StartupAxis_ReadHomeParameters') + '\n#endif\n'
unit += r'''
static unsigned checks,failures;
static void ck(unsigned ok){checks++;if(!ok){failures++;printf("failed check %u\n",checks);}}
static void reset(void){used=cursor=tx_count=packet_fault=tick=0;g_extension_home_capture=g_extension_home_done_notice=0;g_startup_axes_vm_request=1;g_startup_axes_state=2;g_safety_fault=0;}
int main(void){
 CanRxMsg reply;uint8_t out[17],home[4]={0x9a,2,0,0x6b},abort[3]={0x9c,0x48,0x6b};
#if !APP_MOTOR_OFFLINE_ONLY
 reset();ck(StartupAxis_ReadHomeParameters(out)==1 && out[4]==30 && out[16]==0x6b);
 for(unsigned i=1;i<=3;i++){reset();packet_fault=i;ck(StartupAxis_ReadHomeParameters(out)==6);}
 reset();g_startup_axes_vm_request=0;ck(StartupAxis_ReadHomeParameters(out)==9 && !tx_count);
 reset();notice(0x600,3,0x6b);ck(VehicleArm_Exchange(6,home,4,3,&reply)==1);
 ck(!g_extension_home_done_notice && g_extension_home_capture && tx_count==1);
 reset();packet_fault=4;ck(VehicleArm_Exchange(6,home,4,3,&reply)==1 && g_extension_home_done_notice);
 reset();g_startup_axes_vm_request=0;ck(VehicleArm_Exchange(6,home,4,3,&reply)==9 && !tx_count);
 reset();g_safety_fault=1;ck(VehicleArm_Exchange(6,home,4,3,&reply)==9 && !tx_count);
#else
 reset();ck(VehicleArm_Exchange(6,home,4,3,&reply)!=1 && !tx_count);
#endif
 reset();ck(VehicleArm_Exchange(5,home,4,3,&reply)!=1 && !tx_count);
 ck(VehicleArm_Exchange(1,home,4,3,&reply)!=1 && !tx_count);
 home[1]=1;ck(VehicleArm_Exchange(6,home,4,3,&reply)!=1 && !tx_count);home[1]=2;
 reset();g_startup_axes_vm_request=0;ck(VehicleArm_Exchange(6,abort,3,3,&reply)==1);
 for(unsigned i=0;i<4;i++){
  reset();g_extension_home_capture=1;notice(i==0?0x500:0x600,i==1?2:3,i==2?0:0x6b);
  StartupHome_Receive(&reply);ck(g_extension_home_done_notice==(i==3));
 }
 printf("home transport checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
(O / 'group-transport-unit.c').write_text(unit, encoding='utf-8')
reports = []
for offline in [0, 1]:
    exe = O / f'group-transport-{offline}.exe'
    p = subprocess.run(['C:/mingw/mingw64/bin/gcc.exe', '-std=c99',
                        f'-DAPP_MOTOR_OFFLINE_ONLY={offline}', str(O / 'group-transport-unit.c'),
                        '-o', str(exe)], capture_output=True, text=True)
    assert p.returncode == 0, p.stderr
    p = subprocess.run([str(exe)], capture_output=True, text=True)
    print(offline, p.stdout, flush=True)
    assert p.returncode == 0, p.stdout
    reports.append(dict(offline=offline, passed=True, output=p.stdout))
(O / 'group-transport-check.json').write_text(json.dumps(dict(actual_c=True,
    hardware_accessed=False, peripherals_mocked=True, cases=reports), indent=2), encoding='utf-8')
