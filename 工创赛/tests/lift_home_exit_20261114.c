#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lift_ruler_calibration.h"
extern volatile int64_t g_lift_home_exit_reference_raw,g_lift_home_exit_position_raw,g_lift_home_exit_delta_raw;
extern volatile uint32_t g_lift_home_exit_limit_raw,g_lift_home_exit_cycle;
static unsigned checks,queries,motion_writes;
static int64_t position_value;
static int malformed,read_ok=1,stop_ack;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t exchange(void *context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{
 uint32_t raw;(void)context;memset(r,0,rn);
 if(q[0]==0x36U){
  CHECK(n==2U && rn==7U);queries++;
  if(!read_ok)return 0U;
  raw=(uint32_t)(position_value<0?-position_value:position_value);
  r[0]=0x36U;r[1]=malformed?2U:(position_value<0?1U:0U);
  r[2]=(uint8_t)(raw>>24);r[3]=(uint8_t)(raw>>16);r[4]=(uint8_t)(raw>>8);r[5]=(uint8_t)raw;r[6]=0x6bU;
  return 1U;
 }
 if(q[0]==0xfeU){CHECK(n==4U && rn==3U);r[0]=0xfeU;r[1]=2U;r[2]=0x6bU;return stop_ack?1U:0U;}
 motion_writes++;return 0U;
}
static void setup(LiftRulerController*c,uint32_t scale,int64_t reference)
{
 LiftRuler_Init(c);c->state=LRC_MOVING;c->phase=30U;c->scale=c->top_scale=scale;
 c->begin_ms=c->move_begin_ms=900U;c->last_service_ms=990U;c->top_position_raw=reference;
 c->home_started=c->home_exit_confirmed=1U;
 queries=motion_writes=0;read_ok=1;malformed=stop_ack=0;
}
int main(void)
{
 LiftRulerController c;uint32_t scale_index,anchor_index,delta_index;
 const uint32_t scales[2]={10,100};const int64_t anchors[3]={0,78,-1000};
 for(scale_index=0;scale_index<2;scale_index++){
  uint32_t scale=scales[scale_index];int64_t limit=(int64_t)32*360*scale/3200;
  int64_t deltas[9]={-2*limit,-limit-1,-limit,-1,0,1,limit,limit+1,2*limit};
  for(anchor_index=0;anchor_index<3;anchor_index++)for(delta_index=0;delta_index<9;delta_index++){
   int64_t reference=anchors[anchor_index],delta=deltas[delta_index];uint32_t cycle=g_lift_home_exit_cycle;
   setup(&c,scale,reference);position_value=reference+delta;
   LiftRuler_Service(&c,1000,1,1,1,exchange,0);
   CHECK(queries==1 && !motion_writes);CHECK(c.top_position_raw==reference);
   CHECK(g_lift_home_exit_cycle==cycle+1U && g_lift_home_exit_reference_raw==reference);
   CHECK(g_lift_home_exit_position_raw==position_value && g_lift_home_exit_delta_raw==delta);
   CHECK(g_lift_home_exit_limit_raw==(uint32_t)limit);
   if(delta>=-limit && delta<=limit){
    CHECK(c.cause==0 && c.phase==31 && c.backoff_active==1);
    CHECK(c.top_valid==0 && c.backoff_valid==0 && c.position_after_raw==position_value);
   }else{
    CHECK(c.state==LRC_FAULT && c.cause==LRC_CAUSE_POSITION && c.stop_pending);
    LiftRuler_Service(&c,1100,1,1,1,exchange,0);CHECK(c.stop_pending && !c.stop_confirmed);
    c.last_service_ms=1390;LiftRuler_Service(&c,1400,1,1,1,exchange,0);
    CHECK(c.stop_pending && c.stop_unconfirmed);
    stop_ack=1;LiftRuler_Service(&c,1460,1,1,1,exchange,0);
    CHECK(!c.stop_pending && c.stop_confirmed && c.cause==LRC_CAUSE_POSITION);
   }
  }
 }
 setup(&c,100,78);position_value=0;LiftRuler_Service(&c,1000,1,1,1,exchange,0);
 CHECK(c.phase==31 && !c.cause && c.top_position_raw==78);
 setup(&c,100,0);malformed=1;LiftRuler_Service(&c,1000,1,1,1,exchange,0);
 CHECK(c.cause==LRC_CAUSE_REPLY && c.stop_pending);
 setup(&c,100,0);read_ok=0;LiftRuler_Service(&c,1000,1,1,1,exchange,0);
 CHECK(c.cause==LRC_CAUSE_COMM && c.stop_pending);
 setup(&c,100,0);LiftRuler_Service(&c,1000,0,1,1,exchange,0);
 CHECK(c.cause==LRC_CAUSE_GUARD && c.stop_pending && queries==0);
 printf("PASS actual lift controller home exit: %u checks; bounded32p, both scales, original reference, larger motion/fault/STOP preserved\n",checks);
 return 0;
}
