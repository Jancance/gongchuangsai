/* Actual scoped Group and LRC return controller, synthetic bus only. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
static uint32_t now=1000,done_ack,stop_ack;
static int64_t pos;
static uint32_t prepare(void*x,uint32_t b,const SPF_Command*c){(void)x;(void)b;CHECK(c->goal.lift_pulses<=c->lift_limit_pulses);return 1;}
static uint32_t start(void*x,uint32_t b,const SPF_Command*c){(void)x;(void)b;(void)c;return 1;}
static uint32_t poll(void*x,uint32_t b){(void)x;(void)b;return done_ack;}
static uint32_t stop(void*x,uint32_t b){(void)x;(void)b;return stop_ack;}
static uint32_t clockms(void*x){(void)x;return now;}
static SPF_GroupIO io={0,prepare,start,poll,stop,clockms};
static SPF_Command command(float depth,uint32_t limit){
 SPF_Command c={0};c.sequence=1;c.kind=SPF_CMD_TRANSFER;c.axes_mask=SPF_LIFT;
 c.goal.dm_deg=180;c.goal.lift_pulses=depth;c.lift_limit_pulses=limit;
 c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=limit?106:150;c.lift_rpm=1846.4f;
 c.stepper_accel_rpm_s=600;c.keep_vm_on=1;return c;
}
static uint32_t exchange(void*x,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 uint32_t raw;(void)x;(void)n;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(q[0]==0x36){raw=(uint32_t)(pos<0?-pos:pos);r[1]=pos<0;r[2]=raw>>24;r[3]=raw>>16;r[4]=raw>>8;r[5]=raw;return 1;}
 if(q[0]==0xfe){r[1]=2;return stop_ack;}
 return 0;
}
static void controller(LiftRulerController*c,uint32_t scale){
 LiftRuler_Init(c);c->state=LRC_DONE;c->top_valid=1;c->scale=c->top_scale=scale;
 c->top_position_raw=1234;c->operation=2;c->down_target_pulses=29700;
}
int main(void){
 float depths[]={3200,29700,32000,40000,60000,82000};unsigned i,j;
 CHECK(SPF_MAX_LIFT_PULSES==29700 && SPF_EXT_RPM==150);
 for(i=0;i<6;i++){
  SPF_Group g={0};SPF_Command c=command(depths[i],0);
  CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000)==(depths[i]<=29700));
  memset(&g,0,sizeof(g));c=command(depths[i],82000);
  CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
  c.lift_limit_pulses=0;c.goal.lift_pulses=0; /* original command cannot mutate owned batch */
  CHECK(g.command.lift_limit_pulses==82000 && g.command.goal.lift_pulses==depths[i]);
  done_ack=0;for(j=0;j<5;j++)SPF_GroupService(&g);
  CHECK(g.state==2 && !g.done);done_ack=1;SPF_GroupService(&g);CHECK(g.state==3 && g.done==4);
  memset(&g,0,sizeof(g));c=command(depths[i],82000);CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
  SPF_GroupService(&g);SPF_GroupCancel(&g,2);stop_ack=0;SPF_GroupService(&g);
  CHECK(g.state==4 && g.stop_mask==4);stop_ack=1;SPF_GroupService(&g);CHECK(g.stop_mask==0 && g.fault==2);
 }
 for(i=0;i<4;i++){
  float bad[]={-1,82001,INFINITY,NAN};SPF_Group g={0};SPF_Command c=command(bad[i],82000);
  CHECK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));
 }
 {SPF_Group g={0};SPF_Command c=command(32000,82000);c.extension_rpm=150;CHECK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));}
 for(i=0;i<2;i++)for(j=0;j<6;j++){
  LiftRulerController c;uint32_t scale=i?100:10;controller(&c,scale);
  CHECK(LiftRuler_StartReturnWithLimit(&c,1000,1,82000));CHECK(c.return_limit_pulses==82000);
  c.state=LRC_MOVING;c.phase=7;c.last_service_ms=1000;
  pos=c.top_position_raw+(int64_t)(depths[j]*360*scale/3200);
  LiftRuler_Service(&c,1010,1,1,1,exchange,0);
  CHECK(!c.cause && c.phase==8 && c.goal_raw==c.top_position_raw+(int64_t)1800*360*scale/3200);
  CHECK(c.command_speed==18464 && c.command_accel==45055 && c.down_target_pulses==29700);
  c.state=LRC_DONE;c.operation=2;CHECK(LiftRuler_StartReturn(&c,1020,1));
  CHECK(c.return_limit_pulses==29700); /* no scoped limit leaks into ordinary return */
 }
 {LiftRulerController c;controller(&c,10);CHECK(!LiftRuler_SetDownTarget(&c,32000));
  CHECK(LiftRuler_StartReturn(&c,1000,1));c.state=LRC_MOVING;c.phase=7;c.last_service_ms=1000;
  pos=c.top_position_raw+(int64_t)82000*360*10/3200;LiftRuler_Service(&c,1010,1,1,1,exchange,0);
  CHECK(c.cause==LRC_CAUSE_POSITION && c.stop_pending);
  stop_ack=0;LiftRuler_Service(&c,1060,1,1,1,exchange,0);CHECK(c.stop_pending);
  stop_ack=1;LiftRuler_Service(&c,1110,1,1,1,exchange,0);CHECK(!c.stop_pending && c.cause==LRC_CAUSE_POSITION);}
 printf("PASS actual deep-Z scope/group/return: %u checks; source bounds/default return preserved, current feedback wait and STOP ACK retained\n",checks);return 0;
}
