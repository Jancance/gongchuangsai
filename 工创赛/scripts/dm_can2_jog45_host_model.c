/* 合成回复检查状态机出口与目标范围，不代替机构动力学或实机验收。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dm_can2.h"
#include "FreeRTOS.h"
CAN_TypeDef mock_can2;
volatile uint32_t g_dm_can2_read_request,g_dm_can2_read_cycle=1,g_dm_can2_feedback_tick;
volatile uint32_t g_dm_can2_state=4,g_dm_can2_result=1,g_dm_can2_valid_mask=7;
volatile uint32_t g_dm_can2_range_mask=7,g_dm_can2_feedback_valid=1;
volatile uint32_t g_dm_can2_esc_id=1,g_dm_can2_master_id=0,g_dm_can2_ctrl_mode=1;
volatile uint32_t g_dm_can2_motor_state,g_dm_can2_position_raw,g_dm_can2_velocity_raw,g_dm_can2_torque_raw;
volatile float g_dm_can2_pmax=12.5f,g_dm_can2_vmax=30,g_dm_can2_tmax=10;
volatile float g_dm_can2_position_rad=-2.0739f,g_dm_can2_position_deg;
volatile float g_dm_can2_velocity_rad_s,g_dm_can2_torque_nm;
CanRxMsg g_dm_can2_last_rx;
static uint32_t now,txcount,enable_count,disable_count,have_rx,failed,enabled;
static CanRxMsg queued;
static float position=-2.0739f;
static const float original=-2.0739f;
static const char *scenario;
static int is(const char*s){return strcmp(scenario,s)==0;}
static float ab(float f){return f<0?-f:f;}
TickType_t xTaskGetTickCount(void){return now;}
void DM_CAN2_Service(uint32_t ready,uint32_t busy){
 if(!ready||busy)return;
 if(g_dm_can2_read_request==2){
  g_dm_can2_read_request=0;
  if(is("stale"))return;
  g_dm_can2_read_cycle++;g_dm_can2_state=4;g_dm_can2_result=1;
  g_dm_can2_position_rad=position;g_dm_can2_motor_state=0;
  g_dm_can2_feedback_tick=now;
  if(is("badprofile"))g_dm_can2_ctrl_mode=2;
 }
}
void DM_CAN2_CancelRead(void){g_dm_can2_read_request=0;}
uint8_t CAN_MessagePending(CAN_TypeDef*c,uint8_t f){(void)c;(void)f;return (uint8_t)have_rx;}
void CAN_Receive(CAN_TypeDef*c,uint8_t f,CanRxMsg*r){(void)c;(void)f;*r=queued;have_rx=0;}
void CAN_ClearFlag(CAN_TypeDef*c,uint32_t f){(void)c;(void)f;}
uint8_t CAN_GetFlagStatus(CAN_TypeDef*c,uint32_t f){(void)c;(void)f;return 0;}
uint8_t CAN_TransmitStatus(CAN_TypeDef*c,uint8_t m){(void)c;(void)m;return failed?0:1;}
void CAN_CancelTransmit(CAN_TypeDef*c,uint8_t m){(void)c;(void)m;have_rx=0;}
uint8_t CAN_Transmit(CAN_TypeDef*c,CanTxMsg*t){
 uint32_t i,special=1,p,v=2047,torque=2047,state;
 (void)c;assert(t->StdId==1 && t->IDE==0 && t->RTR==0 && t->DLC==8);
 txcount++;failed=0;
 for(i=0;i<7;i++)if(t->Data[i]!=255)special=0;
 if(special){
  assert(t->Data[7]==0xFC || t->Data[7]==0xFD);
  if(t->Data[7]==0xFC){enabled=1;enable_count++;if(is("txfail"))failed=1;}
  else{enabled=0;disable_count++;}
 }else{
  float target;
  p=((uint32_t)t->Data[0]<<8)|t->Data[1];target=((float)p*2.0f/65535.0f-1.0f)*12.5f;
  if(!enabled)assert((((unsigned)(t->Data[3]&15)<<8)|t->Data[4])==0);
  else assert((((unsigned)(t->Data[3]&15)<<8)|t->Data[4])==40);
  assert(ab(target-original)<45.1f*0.01745329252f);
  assert((((unsigned)t->Data[2]<<4)|(t->Data[3]>>4))==2047);
  assert((((unsigned)(t->Data[6]&15)<<8)|t->Data[7])==2047);
  if(enabled)position=target;
  if(enabled && is("tracking"))position=original+0.26f;
  if(enabled && is("limit"))torque=2350;
  if(enabled && is("noreply")){have_rx=0;return 0;}
 }
 if(special && t->Data[7]==0xFD && is("stoplost")){have_rx=0;return 0;}
 state=enabled?1:0;
 if(enabled && !special && is("fault"))state=9;
 if(enabled && !special && is("disabled"))state=0;
 p=(uint32_t)((position+12.5f)*65535.0f/25.0f);
 memset(&queued,0,sizeof(queued));queued.DLC=8;
 queued.Data[0]=(uint8_t)((state<<4)|1);
 queued.Data[1]=(uint8_t)(p>>8);queued.Data[2]=(uint8_t)p;
 queued.Data[3]=(uint8_t)(v>>4);queued.Data[4]=(uint8_t)(((v&15)<<4)|(torque>>8));
 queued.Data[5]=(uint8_t)torque;queued.Data[6]=25;queued.Data[7]=25;
 have_rx=1;return 0;
}
int main(int argc,char**argv){
 unsigned i;uint32_t expected;
 assert(argc==2);scenario=argv[1];
 for(i=0;i<10;i++){now+=10;DM_CAN2_ModuleService(1,0);}assert(txcount==0);
 g_dm_jog_request=is("negative")?2:1;
 for(i=0;i<1400;i++){
  now+=10;
  if(is("repeat") && g_dm_jog_state==4 && i%10==0)g_dm_jog_request=1;
  if(is("abort") && g_dm_jog_state==4 && i>80)g_dm_jog_request=3;
  if(is("gap") && g_dm_jog_state==4 && i>80)now+=60;
  DM_CAN2_ModuleService(1,is("busy")?1:0);
  if(g_dm_jog_state==7 || g_dm_jog_state==8)break;
 }
 assert(i<1400);
 expected=(is("positive")||is("negative")||is("repeat"))?1:
          is("stale")?4:is("txfail")?5:(is("noreply")||is("stoplost"))?6:(is("fault")||is("disabled"))?7:
          is("tracking")?8:is("limit")?9:is("busy")?3:is("abort")?2:
          is("gap")?12:is("badprofile")?11:999;
 if(g_dm_jog_result!=expected)fprintf(stderr,"%s: state=%u result=%u stop=%u tx=%u\n",scenario,(unsigned)g_dm_jog_state,(unsigned)g_dm_jog_result,(unsigned)g_dm_jog_stop_result,txcount);
 assert(g_dm_jog_result==expected);
 if(expected==1 && !is("stoplost")){
  assert(g_dm_jog_state==7 && g_dm_jog_stop_result==1);
  assert(enable_count==1 && disable_count==1);
  assert(ab((g_dm_jog_target_deg-g_dm_jog_start_deg)-(is("negative")?-45.0f:45.0f))<0.01f);
 }else if(is("stoplost")){
  assert(g_dm_jog_state==8 && g_dm_jog_stop_result==2);
  g_dm_jog_request=1;now+=10;DM_CAN2_ModuleService(1,0);
  assert(g_dm_jog_state==8 && g_dm_jog_result==3);
  scenario="cleanup";g_dm_jog_request=3;
  for(i=0;i<30;i++){now+=10;DM_CAN2_ModuleService(1,0);if(g_dm_jog_state==8 && g_dm_jog_stop_result==1)break;}
  assert(g_dm_jog_stop_result==1);
 }else if(is("stale")||is("busy")||is("badprofile"))assert(txcount==0);
 else assert(g_dm_jog_state==8 && g_dm_jog_stop_result==1);
 printf("PASS motion scenario: %s; tx=%u; no startup/repeat queue, bounded relative targets.\n",argv[1],txcount);
 return 0;
}
