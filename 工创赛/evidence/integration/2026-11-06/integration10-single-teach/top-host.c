
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "lift_ruler_calibration.h"
static const char*test;
static uint32_t now,move_at,home_at,home_duration;
static unsigned checks,failures,moves,homes,stops,aborts,forbidden,enabled,engine;
static double motor,origin,goal,move_speed,move_accel,home_origin;
static void ck(unsigned ok,unsigned line){checks++;if(!ok){failures++;printf("FAIL %s line%u\n",test,line);}}
#define CK(x) ck((x),__LINE__)
static uint32_t scale=100;
static void update(void){
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001,x;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)x=.5*a*t*t;
  else if(t<ta+tc)x=.5*a*ta*ta+v*(t-ta);
  else if(t<2*ta+tc){double q=2*ta+tc-t;x=distance-.5*a*q*q;}
  else{x=distance;engine=0;}
  motor=origin+(goal>=origin?x:-x);
 }else if(engine==2){
  double u=(double)(uint32_t)(now-home_at)/home_duration;
  if(u>=1){motor=-12465;engine=0;}else motor=home_origin+(-12465-home_origin)*u;
 }
}
static double rpm_now(void){
 if(engine==2)return 60;
 if(engine==1){
  double distance=fabs(goal-origin),v=move_speed*6,a=move_accel*6,ta=v/a,tc,t=(uint32_t)(now-move_at)*.001;
  if(distance<v*ta){ta=sqrt(distance/a);v=a*ta;tc=0;}else tc=distance/v-ta;
  if(t<ta)return a*t/6;
  if(t<ta+tc)return v/6;
  if(t<2*ta+tc)return a*(2*ta+tc-t)/6;
 }return 0;
}
static void rawpos(uint8_t*r,double p){
 int64_t signed_raw=(int64_t)llround(p*scale);uint32_t v=(uint32_t)llabs(signed_raw);
 r[1]=signed_raw<0;r[2]=v>>24;r[3]=v>>16;r[4]=v>>8;r[5]=v;
}
static void home_params(uint8_t*r){
 const uint8_t value[17]={0x22,0,1,0,60,0,0,0x9c,0x40,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(r,value,17);if(!strcmp(test,"wrong_menu"))r[4]=30;
}
static uint32_t bus(void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 LiftRulerController*c=context;uint32_t u;double p;update();memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(!strcmp(test,"read_loss") && moves && q[0]!=0xfe && q[0]!=0x9c)return 0;
 if(n==2){
  switch(q[0]){
   case 0x1a:r[2]=scale==100?0x80:0;if(!strcmp(test,"bad_options"))r[2]|=2;break;
   case 0x22:CK(rn==17);home_params(r);break;
   case 0x36:
    p=motor;
    if(!strcmp(test,"position_frozen") && moves && !homes)p=origin;
    if(!strcmp(test,"overshoot") && moves && !homes)p=goal-100;
    rawpos(r,p);
    if(!strcmp(test,"bad_position") || (!strcmp(test,"bad_home_position") && c->phase==26))r[1]=2;
    break;
   case 0x31:
    u=(uint32_t)((12345LL+(int64_t)llround(motor*65536/360))&65535U);
    if(!strcmp(test,"bad_encoder") && moves)u=(u+1000)&65535U;
    r[1]=u>>8;r[2]=u;break;
   case 0x3a:r[1]=(enabled?1:0)|(moves && !engine?2:0);
    if(!strcmp(test,"disabled") && moves)r[1]&=~1U;
    if(!strcmp(test,"protection") && moves)r[1]|=4U;break;
   case 0x35:u=(uint32_t)llround(rpm_now()*10);
    if(!strcmp(test,"initial_moving") && !moves)u=5;
    if(!strcmp(test,"home_speed_nonzero") && homes && !engine)u=5;
    if(!strcmp(test,"overspeed") && moves && !homes)u=5000;
    r[1]=0;r[2]=u>>8;r[3]=u;break;
   case 0x33:rawpos(r,goal+(!strcmp(test,"wrong_target")?10:0));break;
   case 0x3b:r[1]=engine==2?7:3;
    if(!strcmp(test,"home_stale_idle"))r[1]=3;
    if(!strcmp(test,"home_failure") && homes)r[1]|=8;break;
   default:forbidden++;return 0;
  }return 1;
 }
 if(n==5 && q[0]==0xf3){CK(LiftRuler_CommandAllowed(c,q,n));enabled=1;r[1]=2;return 1;}
 if(n==15 && q[0]==0xfd){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[12]==2 && !q[13] && q[14]==0x6b);
  u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  move_speed=(((unsigned)q[6]<<8)|q[7])*.1;move_accel=((unsigned)q[2]<<8)|q[3];
  CK(move_accel==(((unsigned)q[4]<<8)|q[5]));
  CK((c->operation==0 && move_speed==300 && move_accel==60) ||
     (c->operation>0 && fabs(move_speed-1846.4)<1e-6 && move_accel==45055));
  CK(q[1]==(c->operation==1?0:1));
  origin=motor;goal=motor+(q[1]?-1:1)*(double)u/scale;
  if(c->operation==1)CK(fabs(goal-((double)c->top_position_raw/scale+c->down_target_pulses*360.0/3200))<.001);move_at=now;engine=1;moves++;r[1]=2;
  if(!strcmp(test,"move_ack_lost"))return 0;return 1;
 }
 if(n==4 && q[0]==0x9a){
  CK(LiftRuler_CommandAllowed(c,q,n));CK(q[1]==2 && !q[2] && c->home_params_valid);
  homes++;home_origin=motor;home_duration=c->operation==0?5000:1000;home_at=now;engine=2;r[1]=2;
  if(!strcmp(test,"home_ack_lost"))return 0;return 1;
 }
 if(n==3 && q[0]==0x9c){aborts++;if(!strcmp(test,"abort_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 if(n==4 && q[0]==0xfe){stops++;if(!strcmp(test,"stop_ack_missing"))return 0;engine=0;r[1]=2;return 1;}
 forbidden++;return 0;
}
static void reset_case(const char*name){test=name;now=0;checks+=0;moves=homes=stops=aborts=forbidden=enabled=engine=0;motor=0;scale=!strcmp(name,"unscaled")?10:100;}

int main(void){
 const char*cases[]={"normal","unscaled","wrong_menu","bad_options","bad_position","initial_moving","home_ack_lost","home_failure","home_stale_idle","home_speed_nonzero","bad_home_position","cancel_home","abort_ack_missing"};
 unsigned k,i;
 for(k=0;k<sizeof(cases)/sizeof(cases[0]);k++){
  LiftRulerController c;reset_case(cases[k]);motor=-12465;LiftRuler_Init(&c);
  CK(!LiftRuler_Start(&c,0,0));CK(LiftRuler_Start(&c,0,1));
  for(i=0;i<2500;i++){
   now=i*20;
   if((!strcmp(test,"cancel_home") || !strcmp(test,"abort_ack_missing")) && homes && !c.cause)LiftRuler_Stop(&c,now);
   LiftRuler_Service(&c,now,1,1,1,bus,&c);
   if(c.state>=LRC_DONE && !c.stop_pending)break;
  }
  CK(moves==0 && !forbidden && c.vm_request==1);
  if(k<2){
   CK(c.state==LRC_DONE && c.top_valid && c.home_seen_running && c.home_reported && homes==1);
   CK(c.operation==1);CK(LiftRuler_Start(&c,now,1));
   for(i=0;i<1000;i++){now+=20;LiftRuler_Service(&c,now,1,1,1,bus,&c);if(c.state>=LRC_DONE && !c.stop_pending)break;}
   CK(c.state==LRC_DONE && c.operation==2 && moves==1 && homes==1);
   CK(LiftRuler_Start(&c,now,1));
   for(i=0;i<2500;i++){now+=20;LiftRuler_Service(&c,now,1,1,1,bus,&c);if(c.state>=LRC_DONE && !c.stop_pending)break;}
   CK(c.state==LRC_DONE && c.operation==3 && moves==2 && homes==2 && c.top_valid);
  }else{CK(c.cause && !c.top_valid && !LiftRuler_Start(&c,now,1));}
 }
 printf("TOP_START cases=%u checks=%u failures=%u\n",k,checks,failures);return failures?1:0;
}
