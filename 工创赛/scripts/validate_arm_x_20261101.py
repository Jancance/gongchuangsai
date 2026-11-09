from pathlib import Path
import json,hashlib,subprocess,shutil,re
R=Path(__file__).resolve().parents[1];S=R/'tmp/arm_x_20261101/project';E=R/'evidence/p2-arm/2026-11-01/arm-x';H=R/'tmp/arm_x_20261101/host';H.mkdir(parents=True,exist_ok=True)
# Extract actual affected transport and unchanged chassis functions for direct execution.
new=(S/'SYSTEM/CAN/can.c').read_bytes().decode('latin1').replace('\r\n','\n');old=(E/'original/SYSTEM/CAN/can.c').read_bytes().decode('latin1').replace('\r\n','\n')
def function(t,name):
 k=re.search(r'(?:static\s+)?(?:uint32_t|void)\s+'+name+r'\s*\(',t).start();a=t.index('{',k);level=1;b=a+1
 while level:
  level+=(t[b]=='{')-(t[b]=='}');b+=1
 return t[k:b]
chassis_names=['CAN1_ReadSpeed','CAN1_ReadPosition','CAN1_SetMotorEnable','CAN1_StopMotor','CAN1_SetEmmSpeedRaw','CAN1_ControlSingle','CAN1_ReadVersion','CAN1_ReadStatus','CAN1_ResetSession']
for name in chassis_names:assert function(new,name)==function(old,name),name
all_old_prefix=old[:old.index('uint32_t CAN1_LiftExchange(')];all_new_prefix=new[:new.index('static ArmXEncoding s_arm_x_encoding')];assert all_old_prefix==all_new_prefix
common=new[new.index('static ArmXEncoding s_arm_x_encoding'):new.index('uint32_t CAN1_ReadMotorOptions(')]
common+='\n'+function(new,'CAN1_ExtensionExchange')
chassis='\n'.join(function(new,n) for n in ['CAN1_SetMotorEnable','CAN1_StopMotor','CAN1_SetEmmSpeedRaw'])
code=r"""#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "arm_x.h"
#include "lift_config.h"
#include "extension_config.h"
typedef struct {uint32_t StdId,ExtId;uint8_t IDE,RTR,DLC,Data[8];} CanTxMsg;
typedef CanTxMsg CanRxMsg;
typedef uint32_t TickType_t;
#define APP_ARM_LIFT_TEST 1U
#define CAN_Id_Extended 4U
#define CAN_RTR_Data 0U
#define CAN_FIFO0 0U
#define CAN_Mode_Normal 0U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 0U
#define CAN_TxStatus_NoMailBox 4U
#define CAN1_SINGLE_TX_WAIT_MS 30U
#define CAN1_SINGLE_REPLY_WAIT_MS 100U
#define pdMS_TO_TICKS(x) (x)
#define CAN1 ((void*)0)
static uint32_t s_can1_normal_ready=1,now,frames,have_rx,ack=2;
static CanTxMsg sent[8];static CanRxMsg response;static uint8_t control[8],control_address,control_length;
static TickType_t xTaskGetTickCount(void){return now;}
static void vTaskDelay(TickType_t t){now+=t;}
static uint32_t CAN1_InitMode(uint8_t mode){(void)mode;return 1;}
static uint8_t CAN_MessagePending(void*c,uint8_t f){(void)c;(void)f;return (uint8_t)have_rx;}
static void CAN_Receive(void*c,uint8_t f,CanRxMsg*out){(void)c;(void)f;*out=response;have_rx=0;}
static uint8_t CAN_Transmit(void*c,CanTxMsg*tx){(void)c;assert(frames<8);sent[frames++]=*tx;
 if((tx->ExtId&255)==1){memset(&response,0,sizeof(response));response.IDE=4;response.ExtId=tx->ExtId&~255U;response.DLC=3;response.Data[0]=0xFD;response.Data[1]=(uint8_t)ack;response.Data[2]=0x6B;have_rx=1;}
 return 0;
}
static uint32_t CAN_TransmitStatus(void*c,uint8_t m){(void)c;(void)m;return 1;}
static void CAN_CancelTransmit(void*c,uint8_t m){(void)c;(void)m;}
static uint32_t CAN1_ExchangeSingle(uint8_t a,const uint8_t*q,uint8_t n,uint8_t r,CanRxMsg*out){assert(a==5||a==6);(void)q;(void)n;(void)r;memset(out,0,sizeof(*out));out->Data[1]=2;return 1;}
static uint32_t CAN1_ControlSingle(uint8_t a,const uint8_t*q,uint8_t n,CanRxMsg*out){if(a<1||a>4)return 9;control_address=a;control_length=n;memcpy(control,q,n);memset(out,0,sizeof(*out));return 1;}
@@COMMON@@
@@CHASSIS@@
static unsigned big(const uint8_t*p){return ((unsigned)p[0]<<24)|((unsigned)p[1]<<16)|((unsigned)p[2]<<8)|p[3];}
int main(void){uint8_t q[15],packet[15];CanRxMsg reply;unsigned addr,ms,scaled,dir,i,tests=0;ArmXEncoding enc;uint32_t units;
 const unsigned steps[]={1,2,4,8,16,32,64,128,256};const unsigned moves[]={16,160,1600,5000,27000,32000,50000};
 for(i=0;i<9;i++)for(scaled=0;scaled<2;scaled++)for(dir=0;dir<2;dir++){
  ms=steps[i];assert(ArmX_Configure((uint8_t)(4|(scaled?128:0)),ms,&enc));
  for(unsigned k=0;k<7;k++){
   unsigned pulses=moves[k];assert(ArmX_BuildMove(&enc,dir,pulses,50000,18464,45055,45055,q));
   assert(q[0]==0xFD&&q[1]==dir&&q[2]==0xAF&&q[3]==0xFF&&q[4]==0xAF&&q[5]==0xFF);
   assert(q[6]==0x48&&q[7]==0x20&&q[12]==2&&q[13]==0&&q[14]==0x6B);
   assert(ArmX_AngleUnits(&enc,pulses,&units)&&big(q+8)==units);tests++;
  }
 }
 for(addr=5;addr<=6;addr++)for(dir=0;dir<2;dir++)for(scaled=0;scaled<2;scaled++){
  assert(CAN1_ArmXConfigure(addr,(uint8_t)(4|(scaled?128:0)),16));
  assert(CAN1_ArmXBuildMove(addr,dir,1600,addr==5?18464:1060,45055,45055,q));
  frames=have_rx=0;ack=2;assert((addr==5?CAN1_LiftExchange(q,15,&reply):CAN1_ExtensionExchange(q,15,&reply))==1);
  assert(frames==2&&sent[0].ExtId==(addr<<8)&&sent[1].ExtId==((addr<<8)|1)&&sent[0].DLC==8&&sent[1].DLC==8);
  packet[0]=sent[0].Data[0];memcpy(packet+1,sent[0].Data+1,7);memcpy(packet+8,sent[1].Data+1,7);assert(memcmp(packet,q,15)==0);
  q[14]=0;frames=0;assert(CAN1_ArmXExchange(addr,q,15,&reply)==7&&frames==0);q[14]=0x6B;
  ack=0xE2;frames=have_rx=0;assert(CAN1_ArmXExchange(addr,q,15,&reply)==8);ack=2;tests++;
 }
 assert(!CAN1_ArmXConfigure(1,4,16)&&!CAN1_ArmXConfigure(4,4,16));
 assert(!CAN1_ArmXBuildMove(1,0,16,1000,1000,1000,q));frames=0;assert(CAN1_ArmXExchange(1,q,15,&reply)==7&&frames==0);
 assert(!ArmX_Configure(6,16,&enc));assert(!ArmX_Configure(4,0,&enc));assert(!ArmX_Configure(4,257,&enc));
 assert(ArmX_Configure(5,16,&enc)&&enc.pulses_per_turn==6400);assert(ArmX_Configure(4,16,&enc)&&enc.pulses_per_turn==3200);
 assert(ArmX_AngleUnits(&enc,32000,&units)&&units==36000);assert(ArmX_Configure(132,16,&enc));assert(ArmX_AngleUnits(&enc,32000,&units)&&units==360000);
 assert(!ArmX_BuildMove(&enc,2,160,1600,1060,45055,45055,q));assert(!ArmX_BuildMove(&enc,0,1601,1600,1060,45055,45055,q));
 for(addr=1;addr<=4;addr++)for(dir=0;dir<2;dir++){
  assert(CAN1_SetEmmSpeedRaw(addr,dir,123,20,&reply)==1);
  const uint8_t expected[7]={0xF6,(uint8_t)dir,0,123,20,0,0x6B};assert(control_address==addr&&control_length==7&&!memcmp(control,expected,7));
  assert(CAN1_SetMotorEnable(addr,1,&reply)==1&&control_length==5&&control[2]==1);
  assert(CAN1_StopMotor(addr,&reply)==1&&control_length==4&&control[0]==0xFE);tests++;
 }
 printf("PASS %u armX conversion/transport/chassis-vector groups; no chassis API changes\n",tests);return 0;}
"""
code=code.replace('@@COMMON@@',common).replace('@@CHASSIS@@',chassis)
(H/'transport.c').write_text(code,encoding='latin1')
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-Wall','-Wextra','-Werror','-I'+str(S/'SYSTEM/CAN'),'-I'+str(S/'HARDWARE/LIFT'),'-I'+str(S/'HARDWARE/EXTENSION'),str(H/'transport.c'),'-o',str(H/'transport.exe')]
z=subprocess.run(cmd,capture_output=True,text=True);assert z.returncode==0,z.stderr
z=subprocess.run([str(H/'transport.exe')],capture_output=True,text=True,timeout=10);assert z.returncode==0,z.stderr
(E/'transport-check.log').write_text(z.stdout,encoding='utf-8');m=json.loads((E/'manifest.json').read_text(encoding='utf-8'));m['transport_pass']=True;m['chassis_api_byte_identical']=chassis_names;m['all_can1_source_before_arm_exchange_byte_identical']=True;(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print(z.stdout,flush=True)
