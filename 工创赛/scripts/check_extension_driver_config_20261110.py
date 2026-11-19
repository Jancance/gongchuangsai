"""Actual node6 read-only CAN block reader and request2 parser tests; simulated bus only."""
from pathlib import Path
import subprocess,json
R=Path(__file__).resolve().parents[1];P=R/'tmp/pickup_half_descent_20261110/project';U=P/'CHASSIS/USER'
E=R/'evidence/integration/2026-11-10/half-descent-driver';E.mkdir(exist_ok=True)
src=(P/'CHASSIS/SYSTEM/CAN/can.c').read_bytes().decode('utf-8',errors='replace')
reader=src[src.index('uint32_t CAN1_ExtensionReadOnlyBlock'):src.index('uint32_t StartupAxis_Exchange',src.index('uint32_t CAN1_ExtensionReadOnlyBlock'))]
prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "extension_driver_diag.h"
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define CAN_Id_Extended 4U
#define CAN_RTR_Data 0U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 2U
#define CAN_FIFO0 0U
#define CAN1 ((void*)1)
typedef struct {uint32_t ExtId,IDE,RTR;uint8_t DLC,Data[8];} CanRxMsg;
typedef CanRxMsg CanTxMsg;
static uint32_t checks,tick,tx_calls,cancels,rx_head,rx_tail,empty_reply,bad_packet,bad_checksum,short_error,tx_status=1;
static uint32_t s_can1_normal_ready=1,g_startup_axes_vm_request=1,g_startup_axes_state=4,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static CanRxMsg queue[12];
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t xTaskGetTickCount(void){return tick;}
static void vTaskDelay(uint32_t x){tick+=x;}
static uint32_t CAN_MessagePending(void *p,uint32_t fifo){(void)p;(void)fifo;return rx_tail-rx_head;}
static void StartupHome_Receive(CanRxMsg *r){*r=queue[rx_head++];}
static uint8_t CAN_TransmitStatus(void *p,uint8_t mailbox){(void)p;(void)mailbox;return (uint8_t)tx_status;}
static void CAN_CancelTransmit(void *p,uint8_t mailbox){(void)p;(void)mailbox;cancels++;}
static void put16(uint8_t*p,uint32_t v){p[0]=v>>8;p[1]=v;}
static void put32(uint8_t*p,uint32_t v){p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v;}
static uint32_t make_reply(uint8_t command,uint8_t raw[64]){
 uint32_t n=4;memset(raw,0,64);raw[0]=command;
 switch(command){
 case 0x1f:n=6;put16(raw+1,123);put16(raw+3,0x0114);break;
 case 0x21:n=18;put32(raw+1,126640);put32(raw+5,126641);put32(raw+9,12600);put32(raw+13,18);break;
 case 0x24:put16(raw+1,24000);break;
 case 0x26:put16(raw+1,600);break;
 case 0x27:put16(raw+1,1200);break;
 case 0x42:n=36;raw[1]=37;raw[2]=24;raw[4]=1;put16(raw+15,2200);put16(raw+17,3000);raw[25]=1;put16(raw+33,8);break;
 case 0x43:n=36;raw[1]=37;raw[2]=12;break;
 default:CK(0);
 }
 raw[n-1]=bad_checksum?0:0x6b;return n;
}
static uint8_t CAN_Transmit(void *p,CanTxMsg *q){
 uint8_t request[3],raw[64];uint32_t qn,ex,n,offset=1,packet=0;(void)p;
 CK(ExtDriver_ReadSpec(q->Data[0],request,&qn,&ex));CK(q->ExtId==0x600 && q->IDE==4 && q->RTR==0 && q->DLC==qn && !memcmp(q->Data,request,qn));
 tx_calls++;rx_head=rx_tail=0;if(empty_reply)return 0;
 if(short_error){queue[0].ExtId=0x600;queue[0].IDE=4;queue[0].RTR=0;queue[0].DLC=3;queue[0].Data[0]=q->Data[0];queue[0].Data[1]=0xe2;queue[0].Data[2]=0x6b;rx_tail=1;return 0;}
 n=make_reply(q->Data[0],raw);
 while(offset<n){
  uint32_t chunk=n-offset;if(chunk>7)chunk=7;CanRxMsg*r=&queue[rx_tail++];memset(r,0,sizeof(*r));
  r->ExtId=0x600|(bad_packet&&packet==1?3:packet);r->IDE=4;r->DLC=chunk+1;r->Data[0]=raw[0];memcpy(r->Data+1,raw+offset,chunk);
  offset+=chunk;packet++;
 }
 return 0;
}
static uint32_t VehicleArm_Exchange(uint8_t addr,const uint8_t*q,uint8_t len,uint8_t rn,void *out){(void)addr;(void)q;(void)len;(void)rn;(void)out;CK(0);return 9;}
'''
test=r'''
#include "extension_readonly_diag.inc"
int main(void){
 uint8_t raw[64],q[3];uint32_t n,qn,ex;
 for(uint32_t c=0;c<256;c++){
  uint32_t allowed=c==0x1f||c==0x21||c==0x24||c==0x26||c==0x27||c==0x42||c==0x43;
  CK(ExtDriver_ReadSpec((uint8_t)c,q,&qn,&ex)==allowed);
 }
#if APP_MOTOR_OFFLINE_ONLY
 CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==0);
 g_ext_diag_request=2;ExtDiag_Service(1,1);CK(g_ext_diag_state==3 && g_ext_diag_rejected==9 && tx_calls==0);
#else
 uint8_t commands[7]={0x1f,0x21,0x24,0x26,0x27,0x42,0x43};
 for(uint32_t i=0;i<7;i++){
  uint32_t before=tx_calls;CK(CAN1_ExtensionReadOnlyBlock(commands[i],raw,&n)==1);CK(tx_calls==before+1);
  CK(ExtDriver_FormatKnown(commands[i],raw,n));
 }
 CK(CAN1_ExtensionReadOnlyBlock(0x48,raw,&n)==7);CK(CAN1_ExtensionReadOnlyBlock(0xfd,raw,&n)==7);
 uint32_t before=tx_calls;s_can1_normal_ready=0;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);s_can1_normal_ready=1;
 g_startup_axes_vm_request=0;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);g_startup_axes_vm_request=1;
 g_startup_axes_state=3;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);g_startup_axes_state=4;
 g_safety_fault=1;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==9 && tx_calls==before);g_safety_fault=0;
 bad_packet=1;CK(CAN1_ExtensionReadOnlyBlock(0x42,raw,&n)==6);bad_packet=0;
 bad_checksum=1;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==6);bad_checksum=0;
 short_error=1;CK(CAN1_ExtensionReadOnlyBlock(0x21,raw,&n)==6 && n==3 && raw[1]==0xe2);short_error=0;
 empty_reply=1;uint32_t start=tick;CK(CAN1_ExtensionReadOnlyBlock(0x42,raw,&n)==5 && tick-start==50 && n==0);empty_reply=0;
 tx_status=0;start=tick;CK(CAN1_ExtensionReadOnlyBlock(0x42,raw,&n)==4 && tick-start==10);tx_status=1;rx_head=rx_tail;
 g_ext_diag_request=2;before=tx_calls;
 for(uint32_t i=0;i<7;i++){
  uint32_t prior=tx_calls;ExtDiag_Service(1,1);CK(tx_calls==prior+1 && g_ext_diag_index==i+1);
 }
 CK(tx_calls==before+7 && g_ext_diag_state==2 && g_ext_diag_kind==2 && g_ext_diag_context==1 && g_ext_cfg_decode_mask==127);
 CK(g_ext_cfg_firmware_version==123 && g_ext_cfg_hardware_word==0x0114);
 CK(g_ext_cfg_pid[0]==126640 && g_ext_cfg_pid[1]==126641 && g_ext_cfg_pid[2]==12600 && g_ext_cfg_pid[3]==18);
 CK(g_ext_cfg_bus_mv==24000 && g_ext_cfg_bus_ma==600 && g_ext_cfg_phase_ma==1200);
 CK(g_ext_cfg_control_mode==1 && g_ext_cfg_ma_limit==2200 && g_ext_cfg_vm_limit==3000 && g_ext_cfg_prwindow_raw==8 && g_ext_cfg_scale==1);
 CK(g_ext_cfg_length[1]==18 && g_ext_cfg_length[5]==36 && g_ext_cfg_raw[5][1]==37);
 before=tx_calls;ExtDiag_Service(1,1);CK(tx_calls==before);
 g_ext_diag_request=2;ExtDiag_Service(0,1);CK(g_ext_diag_state==3 && g_ext_diag_rejected==1 && tx_calls==before && g_ext_cfg_decode_mask==0);
 g_ext_diag_request=2;ExtDiag_Service(1,1);CK(g_ext_diag_state==1);before=tx_calls;ExtDiag_Service(0,1);CK(g_ext_diag_state==3 && tx_calls==before);
 /* Reject known-length but wrong-format/Emm config without producing decoded defaults. */
 make_reply(0x42,raw);raw[2]=21;CK(!ExtDriver_FormatKnown(0x42,raw,36));
 ExtDriverBlock b;uint8_t malformed[8]={0x42,0xff,24,0,0,0,0,0};ExtDriver_BlockBegin(&b,0x42,0);
 CK(ExtDriver_BlockFeed(&b,0,malformed,8)==2 && b.length<=64);
#endif
 printf("PASS %u driver-config reader/service/parser checks offline=%u\n",checks,APP_MOTOR_OFFLINE_ONLY);return 0;
}
'''
for offline in (0,1):
 c=E/f'config-{offline}.c';exe=E/f'config-{offline}.exe';c.write_text(prefix+reader+test,encoding='utf-8')
 p=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable','-Wno-misleading-indentation',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}','-I',str(U),str(c),'-o',str(exe)],capture_output=True,text=True)
 log=p.stdout+p.stderr
 if p.returncode==0:
  p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=p.stdout+p.stderr
 (E/f'config-{offline}.log').write_text(log,encoding='utf-8');print(log,flush=True);assert p.returncode==0
assert 'CAN1_InitMode(' not in reader and 'CAN1_ResetSession(' not in reader
(E/'checks.json').write_text(json.dumps({'passed':True,'actual_can_reader':True,'actual_request2_service':True,
 'write_commands_rejected':True,'one_query_per_service':True,'no_bus_reinitialization':True,'rx_timeout_ms':50,'tx_timeout_ms':10,
 'pid_format':'X four uint32 big-endian','unknown_config_kept_raw':True,'hardware_access':False},indent=2)+'\n')
