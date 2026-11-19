#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define APP_MOTOR_OFFLINE_ONLY 0
#define SOURCE_WHEEL_HOLD_SPEED_RAW 37U
#define ZDT_EMM_MOTION_ACCELERATION 236U
#define CAN_Id_Extended 1U
#define CAN_RTR_Data 0U
#define CAN_FIFO0 0U
#define CAN_Mode_Normal 0U
#define CAN_TxStatus_Ok 1U
#define CAN_TxStatus_Failed 2U
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 60U
#define CAN1 0
#define pdMS_TO_TICKS(x) (x)
typedef uint32_t TickType_t;
typedef struct{uint32_t ExtId,IDE,RTR,DLC;uint8_t Data[8];}CanTxMsg;
typedef CanTxMsg CanRxMsg;
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t tick,transmitted,reply_pending,suppress_reply,ack=2,confirmed=1,s_can1_normal_ready=1;
static uint32_t g_startup_axes_vm_request=1,g_startup_axes_state=4,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static CanTxMsg frames[2];
static uint32_t CAN1_EmmSettingsConfirmed(void){return confirmed;}
static uint32_t CAN1_InitMode(uint32_t mode){(void)mode;return 1;}
static uint32_t CAN_MessagePending(int controller,uint32_t fifo){(void)controller;(void)fifo;return reply_pending;}
static void StartupHome_Receive(CanRxMsg *out){memset(out,0,sizeof(*out));out->ExtId=frames[0].ExtId;out->IDE=1;out->DLC=3;out->Data[0]=0xfd;out->Data[1]=(uint8_t)ack;out->Data[2]=0x6b;reply_pending=0;}
static uint32_t CAN_Transmit(int controller,CanTxMsg *frame){(void)controller;CHECK(transmitted<2);frames[transmitted++]=*frame;if(transmitted==2 && !suppress_reply)reply_pending=1;return 0;}
static uint32_t CAN_TransmitStatus(int c,uint32_t mailbox){(void)c;(void)mailbox;return 1;}
static void CAN_CancelTransmit(int c,uint32_t mailbox){(void)c;(void)mailbox;CHECK(0);}
static uint32_t xTaskGetTickCount(void){return tick;}
static void vTaskDelay(uint32_t delay){tick+=delay;}
#include "tested-wheel-hold.inc"

typedef struct {uint32_t active;}PowerCommandStatus;
#define CHASSIS_AUTO_DONE 8U
#define CHASSIS_AUTO_IDLE 0U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t s_sequence_hold,s_sequence_continue,s_used,s_vm_requested;
static uint32_t g_chassis_auto_session,g_chassis_auto_active,g_chassis_auto_cut_vm,g_chassis_auto_state,g_chassis_auto_result,g_chassis_auto_cause,g_chassis_auto_stop_mask[3];
static uint32_t powered,healthy,zero_speed;
static void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=powered;}
static uint32_t Auto_BaseHealthy(void){return healthy && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;}
static uint32_t Auto_ZeroSpeedLocked(void){return zero_speed;}
#include "tested-close-held.inc"
static void normal(void){
 s_sequence_hold=s_used=s_vm_requested=1;s_sequence_continue=0;
 g_chassis_auto_session=4;g_chassis_auto_state=CHASSIS_AUTO_DONE;g_chassis_auto_result=1;
 g_chassis_auto_active=g_chassis_auto_cut_vm=g_chassis_auto_cause=0;g_chassis_auto_stop_mask[2]=15;
 powered=healthy=zero_speed=1;g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
}
static void reset_wire(void){transmitted=reply_pending=tick=0;suppress_reply=0;ack=2;confirmed=1;}
int main(void){
 unsigned address,i;CanRxMsg reply;uint8_t decoded[12];
 for(address=1;address<=4;address++){
  reset_wire();CHECK(CAN1_HoldWheelCurrentPosition((uint8_t)address,&reply)==1);
  CHECK(transmitted==2 && frames[0].ExtId==(address<<8) && frames[1].ExtId==((address<<8)|1));
  CHECK(frames[0].DLC==8 && frames[1].DLC==5 && frames[0].Data[0]==0xfd && frames[1].Data[0]==0xfd);
  memcpy(decoded,frames[0].Data,8);memcpy(decoded+8,frames[1].Data+1,4);
  CHECK(decoded[1]==0 && decoded[2]==0 && decoded[3]==37 && decoded[4]==236);
  for(i=5;i<=8;i++)CHECK(decoded[i]==0); /* absolutely no nonzero displacement */
  CHECK(decoded[9]==2 && decoded[10]==0 && decoded[11]==0x6b);
 }
 reset_wire();CHECK(CAN1_HoldWheelCurrentPosition(0,&reply)==7 && !transmitted);
 reset_wire();confirmed=0;CHECK(CAN1_HoldWheelCurrentPosition(1,&reply)==9 && !transmitted);confirmed=1;
 reset_wire();g_safety_fault=1;CHECK(CAN1_HoldWheelCurrentPosition(1,&reply)==9 && !transmitted);g_safety_fault=0;
 reset_wire();ack=0x9f;CHECK(CAN1_HoldWheelCurrentPosition(1,&reply)==1);
 reset_wire();ack=0xe2;CHECK(CAN1_HoldWheelCurrentPosition(1,&reply)==6);
 reset_wire();suppress_reply=1;CHECK(CAN1_HoldWheelCurrentPosition(1,&reply)==5);
 normal();CHECK(ChassisAuto_CloseHeldCompleted(4)==1 && !s_sequence_hold && !s_used && !s_vm_requested);
 CHECK(g_chassis_auto_state==CHASSIS_AUTO_IDLE && g_chassis_auto_result==0 && powered==1);
 normal();g_chassis_auto_state=9;CHECK(!ChassisAuto_CloseHeldCompleted(4) && g_chassis_auto_state==9);
 normal();g_chassis_auto_cause=7;CHECK(!ChassisAuto_CloseHeldCompleted(4) && g_chassis_auto_cause==7);
 normal();g_safety_fault=1;CHECK(!ChassisAuto_CloseHeldCompleted(4) && g_safety_fault==1);
 normal();g_chassis_auto_active=1;CHECK(!ChassisAuto_CloseHeldCompleted(4));
 normal();zero_speed=0;CHECK(!ChassisAuto_CloseHeldCompleted(4));
 normal();g_chassis_auto_stop_mask[2]=7;CHECK(!ChassisAuto_CloseHeldCompleted(4));
 normal();CHECK(!ChassisAuto_CloseHeldCompleted(3));
 printf("PASS actual wheel hold/rearm: %u checks; zero distance only,4addresses, speed37 accel236, no fault/cancel clear or VM cut\n",checks);
 return 0;
}
