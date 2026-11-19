"""Add strictly read-only node6 driver information to an existing half-descent candidate."""
from pathlib import Path
R=Path(__file__).resolve().parents[1];P=R/'tmp/pickup_half_descent_20261110/project';U=P/'CHASSIS/USER'
assert P.exists(), 'Wait for the coordinator to create the candidate; never copy over it.'
header=r'''#ifndef EXTENSION_DRIVER_DIAG_H
#define EXTENSION_DRIVER_DIAG_H
#include <stdint.h>
#include <string.h>
typedef struct {uint8_t raw[64];uint32_t command,expected,length,packet;} ExtDriverBlock;
static uint32_t ExtDriver_ReadSpec(uint8_t command,uint8_t q[3],uint32_t *n,uint32_t *expected)
{
 q[0]=command;q[1]=0x6bU;q[2]=0U;*n=2U;
 switch(command){
 case 0x1fU:*expected=6U;return 1U;
 case 0x21U:*expected=18U;return 1U;
 case 0x24U:case 0x26U:case 0x27U:*expected=4U;return 1U;
 case 0x42U:q[1]=0x6cU;q[2]=0x6bU;*n=3U;*expected=0U;return 1U;
 case 0x43U:q[1]=0x7aU;q[2]=0x6bU;*n=3U;*expected=0U;return 1U;
 default:return 0U;
 }
}
static void ExtDriver_BlockBegin(ExtDriverBlock *b,uint32_t command,uint32_t expected)
{memset(b,0,sizeof(*b));b->command=command;b->expected=expected;}
/* 0=more,1=complete,2=malformed. Node/IDE/RTR checked by the CAN owner. */
static uint32_t ExtDriver_BlockFeed(ExtDriverBlock *b,uint32_t packet,const uint8_t *data,uint32_t dlc)
{
 uint32_t chunk,i;
 if(!data || dlc<2U || dlc>8U || data[0]!=b->command || packet!=b->packet)return 2U;
 if(!b->packet){
  b->raw[0]=data[0];b->length=1U;
  if(!b->expected){
   if(dlc<3U || data[1]<4U || data[1]>65U){for(i=1;i<dlc;i++)b->raw[b->length++]=data[i];return 2U;}
   b->expected=(uint32_t)data[1]-1U; /* wire header includes the omitted address */
  }
 }
 if(b->expected<2U || b->expected>64U || b->length>=b->expected)return 2U;
 chunk=b->expected-b->length;if(chunk>7U)chunk=7U;
 if(dlc!=chunk+1U){for(i=1;i<dlc && b->length<64U;i++)b->raw[b->length++]=data[i];return 2U;}
 for(i=1;i<dlc;i++)b->raw[b->length++]=data[i];b->packet++;
 if(b->length==b->expected)return b->raw[b->length-1U]==0x6bU?1U:2U;
 return 0U;
}
static uint32_t ExtDriver_U16(const uint8_t *p){return ((uint32_t)p[0]<<8)|p[1];}
static uint32_t ExtDriver_U32(const uint8_t *p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
/* Known X formats only; unknown layouts remain raw, never default parameters. */
static uint32_t ExtDriver_FormatKnown(uint8_t cmd,const uint8_t *raw,uint32_t n)
{
 if(!raw || !n || raw[0]!=cmd || raw[n-1U]!=0x6bU)return 0U;
 if(cmd==0x1fU)return n==6U;
 if(cmd==0x21U)return n==18U;
 if(cmd==0x24U || cmd==0x26U || cmd==0x27U)return n==4U;
 if(cmd==0x42U)return n==36U && raw[1]==37U && raw[2]==24U;
 if(cmd==0x43U)return n==36U && raw[1]==37U && raw[2]==12U;
 return 0U;
}
#endif
'''
(U/'extension_driver_diag.h').write_bytes(header.replace('\n','\r\n').encode('utf-8'))
def patch(rel,old,new):
 p=P/rel;b=p.read_bytes();a=old.encode('utf-8');z=new.encode('utf-8');
 if b.count(z)==1:return
 assert b.count(a)==1,(rel,old[:70]);p.write_bytes(b.replace(a,z,1))
can='CHASSIS/SYSTEM/CAN/can.c'
patch(can,'#include "extension_diag_read.h"','#include "extension_diag_read.h"\r\n#include "extension_driver_diag.h"')
reader=r'''
/* Sole owner, node6 only. This function has no enable/reset/menu/motion path. */
uint32_t CAN1_ExtensionReadOnlyBlock(uint8_t command,uint8_t out[64],uint32_t *length)
{
 uint8_t q[3];uint32_t qn,expected;
 if(!out || !length)return 7U;*length=0U;memset(out,0,64U);
 if(!ExtDriver_ReadSpec(command,q,&qn,&expected))return 7U;
#if APP_MOTOR_OFFLINE_ONLY
 return 9U;
#else
 {
  CanTxMsg tx;CanRxMsg rx;TickType_t start;uint32_t result=5U,feed,flush;uint8_t mailbox,status;
  ExtDriverBlock block;
  if(!s_can1_normal_ready || !g_startup_axes_vm_request || g_startup_axes_state!=4U ||
   g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;
  ExtDriver_BlockBegin(&block,command,expected);
  for(flush=0U;flush<3U && CAN_MessagePending(CAN1,CAN_FIFO0);flush++)StartupHome_Receive(&rx);
  memset(&tx,0,sizeof(tx));tx.ExtId=0x0600U;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;
  tx.DLC=(uint8_t)qn;memcpy(tx.Data,q,qn);mailbox=CAN_Transmit(CAN1,&tx);
  if(mailbox>2U)return 4U;
  start=xTaskGetTickCount();
  for(;;){
   status=CAN_TransmitStatus(CAN1,mailbox);if(status==CAN_TxStatus_Ok)break;
   if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(10U)){
    CAN_CancelTransmit(CAN1,mailbox);return 4U;}
   vTaskDelay(pdMS_TO_TICKS(1U));
  }
  start=xTaskGetTickCount();
  /* Read-only diagnostic timeout stays below the DM holding-service budget. */
  while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(50U)){
   if(CAN_MessagePending(CAN1,CAN_FIFO0)){
    StartupHome_Receive(&rx);
    if(rx.IDE==CAN_Id_Extended && rx.RTR==CAN_RTR_Data && (rx.ExtId>>8)==6U && rx.DLC && rx.Data[0]==command){
     feed=ExtDriver_BlockFeed(&block,rx.ExtId&0xffU,rx.Data,rx.DLC);
     if(feed){result=feed==1U?1U:6U;break;}
    }
   }
   vTaskDelay(pdMS_TO_TICKS(1U));
  }
  *length=block.length;memcpy(out,block.raw,block.length);return result;
 }
#endif
}

'''
patch(can,'uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)',
 reader.replace('\n','\r\n')+'uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t n,uint8_t reply_n,void *out)')
patch('CHASSIS/SYSTEM/CAN/can.h','uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm);',
 'uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm);\r\nuint32_t CAN1_ExtensionReadOnlyBlock(uint8_t command,uint8_t out[64],uint32_t *length);')
core='CHASSIS/USER/extension_readonly_diag.inc'
patch(core,'#include <string.h>','#include <string.h>\r\n#include "extension_driver_diag.h"')
patch(core,'volatile uint32_t g_ext_diag_context;',r'''volatile uint32_t g_ext_diag_kind;
volatile uint32_t g_ext_cfg_cycle,g_ext_cfg_length[7],g_ext_cfg_raw[7][64],g_ext_cfg_decode_mask;
volatile uint32_t g_ext_cfg_firmware_version,g_ext_cfg_hardware_word,g_ext_cfg_pid[4];
volatile uint32_t g_ext_cfg_bus_mv,g_ext_cfg_bus_ma,g_ext_cfg_phase_ma;
volatile uint32_t g_ext_cfg_ma_limit,g_ext_cfg_vm_limit,g_ext_cfg_prwindow_raw,g_ext_cfg_scale,g_ext_cfg_control_mode;
volatile uint32_t g_ext_diag_context;'''.replace('\n','\r\n'))
patch(core,'if(request!=1U || g_ext_diag_state==1U)', 'if((request!=1U && request!=2U) || g_ext_diag_state==1U)')
patch(core,'  g_ext_diag_context=context;g_ext_diag_index=0U;',r'''  g_ext_diag_context=context;g_ext_diag_index=0U;g_ext_diag_kind=request;
  if(request==2U){
   g_ext_cfg_cycle=g_ext_diag_cycle;g_ext_cfg_decode_mask=0U;
   g_ext_cfg_firmware_version=g_ext_cfg_hardware_word=g_ext_cfg_bus_mv=g_ext_cfg_bus_ma=g_ext_cfg_phase_ma=0U;
   g_ext_cfg_ma_limit=g_ext_cfg_vm_limit=g_ext_cfg_prwindow_raw=g_ext_cfg_scale=g_ext_cfg_control_mode=0U;
   for(i=0U;i<4U;i++)g_ext_cfg_pid[i]=0U;
   for(i=0U;i<7U;i++){g_ext_cfg_length[i]=0U;for(j=0U;j<64U;j++)g_ext_cfg_raw[i][j]=0U;}
  }'''.replace('\n','\r\n'))
start=(U/'extension_readonly_diag.inc').read_bytes();a=start.index(b' i=g_ext_diag_index;');z=start.index(b' if(g_ext_diag_index==EXT_DIAG_ITEMS)',a)
block=r''' i=g_ext_diag_index;
 if(i<EXT_DIAG_ITEMS){
  static const uint8_t basic[7]={0x1aU,0x33U,0x34U,0x35U,0x36U,0x37U,0x3aU};
  static const uint8_t basic_lengths[7]={4U,7U,7U,5U,7U,7U,3U};
  static const uint8_t config[7]={0x1fU,0x21U,0x24U,0x26U,0x27U,0x42U,0x43U};
  if(g_ext_diag_kind==2U){
   uint8_t raw[64];uint32_t n=0U,result;uint8_t command=config[i];
   memset(raw,0,sizeof(raw));g_ext_diag_cmd[i]=command;
   result=CAN1_ExtensionReadOnlyBlock(command,raw,&n);
   g_ext_diag_result[i]=result;g_ext_diag_tick[i]=(uint32_t)xTaskGetTickCount();g_ext_cfg_length[i]=n;
   for(j=0U;j<64U;j++)g_ext_cfg_raw[i][j]=raw[j];
   for(j=0U;j<8U;j++)g_ext_diag_raw[i][j]=raw[j];
   if(result==1U && ExtDriver_FormatKnown(command,raw,n)){
    g_ext_cfg_decode_mask|=1U<<i;
    if(i==0U){g_ext_cfg_firmware_version=ExtDriver_U16(raw+1);g_ext_cfg_hardware_word=ExtDriver_U16(raw+3);}
    if(i==1U)for(j=0U;j<4U;j++)g_ext_cfg_pid[j]=ExtDriver_U32(raw+1U+4U*j);
    if(i==2U)g_ext_cfg_bus_mv=ExtDriver_U16(raw+1);
    if(i==3U)g_ext_cfg_bus_ma=ExtDriver_U16(raw+1);
    if(i==4U)g_ext_cfg_phase_ma=ExtDriver_U16(raw+1);
    if(i==5U){
     g_ext_cfg_control_mode=raw[4];g_ext_cfg_ma_limit=ExtDriver_U16(raw+15);g_ext_cfg_vm_limit=ExtDriver_U16(raw+17);
     g_ext_cfg_scale=raw[25];g_ext_cfg_prwindow_raw=ExtDriver_U16(raw+33);
    }
   }
  }else{
   uint8_t q[2]={basic[i],0x6bU};CanRxMsg reply;g_ext_diag_cmd[i]=basic[i];
   memset(&reply,0,sizeof(reply));
   g_ext_diag_result[i]=VehicleArm_Exchange(6U,q,2U,basic_lengths[i],&reply);
   g_ext_diag_tick[i]=(uint32_t)xTaskGetTickCount();
   for(j=0;j<8U;j++)g_ext_diag_raw[i][j]=reply.Data[j];
  }
  g_ext_diag_index=i+1U;
 }
'''.replace('\n','\r\n').encode('utf-8')
(U/'extension_readonly_diag.inc').write_bytes(start[:a]+block+start[z:])
print('Added request2 read-only driver data to existing half-descent candidate; no motion parameters or integration10 edits.')
