/* Actual LRC return phase7 frames, mixed cadence, native reference/backoff. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lift_ruler_calibration.h"
static unsigned checks;static int64_t actual;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t bus(void*x,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 uint32_t p=(uint32_t)(actual<0?-actual:actual);(void)x;(void)n;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(q[0]==0x36){r[1]=actual<0;r[2]=p>>24;r[3]=p>>16;r[4]=p>>8;r[5]=p;return 1;}
 if(q[0]==0x31)return 1;
 if(q[0]==0xfd || q[0]==0xf3){r[1]=2;return 1;}return 0;
}
static void step(LiftRulerController*c,uint32_t phase,uint32_t now){c->phase=phase;c->last_service_ms=now-10;LiftRuler_Service(c,now,1,1,1,bus,0);if(c->cause)fprintf(stderr,"phase%u cause%u state%u op%u\n",phase,c->cause,c->state,c->operation);CHECK(!c->cause);}
int main(void){
 for(unsigned sc=0;sc<2;sc++)for(unsigned anchor=0;anchor<3;anchor++)for(unsigned mode=0;mode<3;mode++){
  LiftRulerController c;unsigned scale=sc?100:10;int64_t top=(int64_t[]){0,78,-1000}[anchor];
  LiftRuler_Init(&c);c.top_valid=1;c.top_scale=c.scale=scale;c.top_position_raw=top;c.state=LRC_DONE;
  for(unsigned n=1;n<=20;n++){
   unsigned third=mode==0 || (mode==2 && n%2);unsigned units=third?600:1800;
   actual=top+(int64_t)(third?52000:22000)*360*scale/3200;
   if(third)CHECK(LiftRuler_StartReturnWithLimit(&c,1000,1,82000));else CHECK(LiftRuler_StartReturn(&c,1000,1));
   CHECK(c.reuse_count==n%10 && c.reuse_reference==(n%10!=0));
   c.state=LRC_MOVING;step(&c,7,1010);
   CHECK(c.goal_raw==top+(int64_t)units*360*scale/3200 && c.top_position_raw==top);
   CHECK(c.command_speed==18464 && c.command_accel==45055 && c.move_frame[0]==0xfd);
   uint32_t encoded=((uint32_t)c.move_frame[8]<<24)|((uint32_t)c.move_frame[9]<<16)|((uint32_t)c.move_frame[10]<<8)|c.move_frame[11];
   CHECK(encoded==(uint32_t)(actual-c.goal_raw) && c.move_frame[1]==1);
   /* Exercise actual completion branch with a matching encoder sample. */
   c.position_after_raw=c.goal_raw;c.expected_encoder_mod=0;c.encoder_before=0;
   step(&c,13,1015);
   CHECK(n%10?(c.state==LRC_DONE && c.backoff_valid):(c.phase==20 && c.state==LRC_MOVING));
   /* Reuse and native cadence use the same controller state, never reset counts. */
   if(n%10==0){
    actual=top;step(&c,26,1020);CHECK(c.top_position_raw==top && !c.top_valid);
    step(&c,30,1030);CHECK(c.backoff_active && c.phase==31);
    step(&c,31,1040);CHECK(c.phase==5);step(&c,7,1050);
    CHECK(c.goal_raw==top+(int64_t)600*360*scale/3200 && c.top_position_raw==top);
    c.position_after_raw=c.goal_raw;c.expected_encoder_mod=0;c.encoder_before=0;
    step(&c,13,1060);CHECK(c.state==LRC_DONE && c.backoff_valid && c.top_valid && c.safe_position_raw==c.goal_raw && c.top_position_raw==top);
   }
   c.state=LRC_DONE;c.top_valid=1;c.backoff_active=0;
  }
 }
 /* Initial homing backoff remains 600 without changing its zero. */
 {LiftRulerController c;LiftRuler_Init(&c);c.scale=100;c.state=LRC_MOVING;c.operation=0;
  c.begin_ms=1000;actual=78;step(&c,26,1010);step(&c,30,1020);step(&c,31,1030);step(&c,7,1040);
  CHECK(c.top_position_raw==78 && c.goal_raw==78+(int64_t)600*360*100/3200);}
 printf("PASS actual LRC return: %u checks; S3 600/source1800, frames, scales10/100, mixed20 cadence, native26/30/31/backoff600, zero unchanged\n",checks);return 0;
}
