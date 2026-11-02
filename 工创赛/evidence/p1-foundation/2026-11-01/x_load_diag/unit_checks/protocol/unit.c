#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "src/x_load_diag.h"
uint32_t g_chassis_auto_active,g_chassis_auto_session,g_chassis_auto_state;
static uint32_t depth;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)

typedef uint32_t TickType_t;
typedef struct{uint32_t ExtId;uint8_t IDE,RTR,DLC,Data[8];} CanTxMsg;
typedef CanTxMsg CanRxMsg;
#define CAN1 0
#define CAN_FIFO0 0
#define CAN_Mode_Normal 0
#define CAN_Id_Extended 1
#define CAN_RTR_Data 0
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Failed 2
#define CAN_TxStatus_Ok 1
#define CAN1_SINGLE_TX_WAIT_MS 20U
#define CAN1_SINGLE_REPLY_WAIT_MS 30U
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1U
static uint32_t tick,s_can1_normal_ready,tx_calls,reset_calls,received,record_result,record_calls;
static uint8_t ready,head,tail,tx_mode;static CanRxMsg queue[8];static CanTxMsg sent;static XSystemSnapshot captured;
static uint32_t xTaskGetTickCount(void){return tick++;}
static uint32_t CAN1_InitMode(uint32_t mode){(void)mode;return 1;}
static void CAN1_ResetSession(void){s_can1_normal_ready=0;reset_calls++;}
static uint32_t CAN_MessagePending(uint32_t can,uint32_t fifo){(void)can;(void)fifo;return ready && head<tail;}
static void CAN_Receive(uint32_t can,uint32_t fifo,CanRxMsg *p){(void)can;(void)fifo;*p=queue[head++];received++;}
static uint8_t CAN_Transmit(uint32_t can,CanTxMsg *p){(void)can;sent=*p;tx_calls++;ready=1;return tx_mode==1?4:0;}
static uint8_t CAN_TransmitStatus(uint32_t can,uint8_t box){(void)can;(void)box;return tx_mode==2?2:1;}
static void CAN_CancelTransmit(uint32_t can,uint8_t box){(void)can;(void)box;}
void XLoad_RecordSnapshot(uint8_t address,uint32_t result,const XSystemSnapshot *p,uint32_t ms){(void)address;(void)ms;captured=*p;record_result=result;record_calls++;}
static void fresh(uint8_t address){uint8_t data[36]={0},pos=0,pkt=0,i;
 tick=tx_calls=reset_calls=received=record_result=record_calls=0;s_can1_normal_ready=ready=head=tx_mode=0;
 data[0]=0x43;data[1]=37;data[2]=12;data[3]=0x5D;data[4]=0xC0;data[7]=0x0B;data[8]=0xB8;
 data[18]=1;data[20]=1;data[21]=1;data[24]=4;data[25]=0xD2;data[33]=3;data[34]=3;data[35]=0x6B;
 while(pos<36){CanRxMsg *q=&queue[pkt];memset(q,0,sizeof(*q));q->ExtId=((uint32_t)address<<8)|pkt;q->IDE=1;q->Data[0]=0x43;q->DLC=1;
  if(!pkt){q->DLC=0;}
  while(pos<36 && q->DLC<8)q->Data[q->DLC++]=data[pos++];pkt++;
 }tail=pkt;
}

uint32_t CAN1_ReadXSystemSnapshot(uint8_t address,CanRxMsg *reply,float *rpm)

{

    CanTxMsg tx;CanRxMsg rx;XSystemSnapshot sample={0};

    TickType_t start;uint32_t result=5U;uint8_t packet=0U,length=0U,i,offset,mailbox,status;

    uint8_t data[36]={0};uint32_t raw;

    if(address<1U || address>4U || !reply || !rpm)return 7U;

    *rpm=0.0f;memset(reply,0,sizeof(*reply));

    if(!s_can1_normal_ready){result=CAN1_InitMode(CAN_Mode_Normal);if(result!=1U)goto finished;s_can1_normal_ready=1U;}

    for(i=0U;i<3U && CAN_MessagePending(CAN1,CAN_FIFO0);i++)CAN_Receive(CAN1,CAN_FIFO0,&rx);

    memset(&tx,0,sizeof(tx));tx.ExtId=(uint32_t)address<<8;tx.IDE=CAN_Id_Extended;tx.RTR=CAN_RTR_Data;

    tx.DLC=3U;tx.Data[0]=0x43U;tx.Data[1]=0x7AU;tx.Data[2]=0x6BU;

    mailbox=CAN_Transmit(CAN1,&tx);if(mailbox==CAN_TxStatus_NoMailBox){result=4U;goto finished;}

    start=xTaskGetTickCount();

    do {

        status=CAN_TransmitStatus(CAN1,mailbox);

        if(status==CAN_TxStatus_Failed || (TickType_t)(xTaskGetTickCount()-start)>=pdMS_TO_TICKS(CAN1_SINGLE_TX_WAIT_MS)){

            CAN_CancelTransmit(CAN1,mailbox);result=4U;goto finished;}

    }while(status!=CAN_TxStatus_Ok);

    start=xTaskGetTickCount();result=5U;

    while((TickType_t)(xTaskGetTickCount()-start)<pdMS_TO_TICKS(CAN1_SINGLE_REPLY_WAIT_MS)){

        if(!CAN_MessagePending(CAN1,CAN_FIFO0))continue;

        CAN_Receive(CAN1,CAN_FIFO0,&rx);

        if(rx.IDE!=CAN_Id_Extended || rx.RTR!=CAN_RTR_Data || (rx.ExtId>>8)!=address)continue;

        if(rx.DLC<2U || rx.DLC>8U || rx.Data[0]!=0x43U)continue;

        *reply=rx;

        if((rx.ExtId&0xFFU)!=packet){result=6U;break;}

        offset=packet?1U:0U;

        if((uint32_t)length+rx.DLC-offset>36U){result=6U;break;}

        for(i=offset;i<rx.DLC;i++)data[length++]=rx.Data[i];

        packet++;

        if(length==36U){

            if(data[1]!=37U || data[2]!=12U || data[35]!=0x6BU || data[13]>1U ||

               data[18]>1U || data[21]>1U || data[26]>1U){result=6U;break;}

            raw=((uint32_t)data[19]<<8)|data[20];

            sample.speed_decirpm=data[18]?-(int32_t)raw:(int32_t)raw;

            sample.position_deci_deg=((uint32_t)data[22]<<24)|((uint32_t)data[23]<<16)|((uint32_t)data[24]<<8)|data[25];

            sample.position_sign=data[21];sample.phase_ma=(uint16_t)(((uint16_t)data[7]<<8)|data[8]);

            sample.bus_mv=(uint16_t)(((uint16_t)data[3]<<8)|data[4]);

            sample.home_flags=data[33];sample.motor_flags=data[34];

            *rpm=(float)sample.speed_decirpm*0.1f;result=1U;break;

        }

    }

finished:

    XLoad_RecordSnapshot(address,result,&sample,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);

    if(result!=1U)CAN1_ResetSession();return result;

}

uint32_t test_entry(void){CanRxMsg reply;float rpm;uint32_t i;uint8_t last;
 fresh(3);CHECK(tail==5);CHECK(CAN1_ReadXSystemSnapshot(3,&reply,&rpm)==1);CHECK(rpm<-.099f && rpm>-.101f);CHECK(tx_calls==1 && sent.DLC==3 && sent.Data[0]==0x43 && sent.Data[1]==0x7A && sent.Data[2]==0x6B);CHECK(captured.phase_ma==3000 && captured.bus_mv==24000 && captured.position_sign==1 && captured.position_deci_deg==1234);CHECK(record_calls==1 && record_result==1 && tick<30);
 fresh(1);queue[1].ExtId++;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==6);CHECK(reset_calls==1 && record_result==6 && rpm==0);
 fresh(1);last=queue[4].DLC;queue[4].Data[last-1]^=1;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==6);
 fresh(1);queue[0].Data[1]=36;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==6);
 fresh(1);for(i=0;i<tail;i++)queue[i].ExtId+=0x100;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==5);CHECK(tick<60);
 fresh(1);tail=2;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==5);
 fresh(1);queue[2].Data[4]=2;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==6);
 fresh(1);tx_mode=1;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==4);
 fresh(1);tx_mode=2;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==4);
 fresh(1);CHECK(CAN1_ReadXSystemSnapshot(0,&reply,&rpm)==7);CHECK(!tx_calls);
 fresh(1);CHECK(CAN1_ReadXSystemSnapshot(5,&reply,&rpm)==7);CHECK(!tx_calls);
 fresh(1);queue[2].Data[5]=3;queue[2].Data[6]=0x21;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==1);CHECK(rpm< -80.09f && rpm> -80.11f);
 fresh(1);queue[2].Data[5]=255;queue[2].Data[6]=255;CHECK(CAN1_ReadXSystemSnapshot(1,&reply,&rpm)==1);CHECK(rpm< -6500.0f);
 CHECK(!depth);return 100U;
}
