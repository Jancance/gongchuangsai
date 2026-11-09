
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "lift_ruler_calibration.h"
static unsigned checks,failures,moves,stops,enable,forbidden;static uint32_t now,moved_at;
static uint32_t scale=100;static double initial=-123.4,delta=0,speed=0;static const char*test;
static void ck(unsigned v,unsigned line){checks++;if(!v){failures++;printf("FAIL %s line%u\n",test,line);}}
#define CK(v) ck((v),__LINE__)
static void rawpos(uint8_t*r,double value){int64_t p=(int64_t)llround(value*scale);uint32_t u=(uint32_t)(p<0?-p:p);r[1]=p<0;r[2]=u>>24;r[3]=u>>16;r[4]=u>>8;r[5]=u;}
static void update(void){
 double t=(double)(uint32_t)(now-moved_at)*.001,ta=.5,v=180,a=360,total=9.875;
 if(!moves){delta=speed=0;return;}
 if(t<ta){delta=.5*a*t*t;speed=a*t;}
 else if(t<9.375){delta=45+v*(t-ta);speed=v;}
 else if(t<total){double q=total-t;delta=1687.5-.5*a*q*q;speed=a*q;}
 else{delta=1687.5;speed=0;}
}
static uint32_t bus(void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 uint32_t u;double angle;LiftRulerController*c=context;
 update();memset(r,0,8);r[0]=q[0];r[rn-1]=0x6b;
 if(!strcmp(test,"power_loss") && moves && (uint32_t)(now-moved_at)>500)return 0;
 if(n==2){
  switch(q[0]){
   case 0x1a:r[2]=(scale==100?0x80:0);if(!strcmp(test,"bad_options"))r[2]|=2;break;
   case 0x36:angle=initial+delta;
    if(!strcmp(test,"position_frozen"))angle=initial;
    if(moves && !strcmp(test,"wrong_direction"))angle=initial-3;
    if(moves && !strcmp(test,"overshoot"))angle=initial+1690.5;
    rawpos(r,angle);if(!strcmp(test,"bad_position_sign"))r[1]=2;
    if(moves && !strcmp(test,"read_failure"))return 0;
    break;
   case 0x31:{uint32_t enc=(uint32_t)(16000LL+(int64_t)llround(delta*65536/360))&65535U;
    if(moves && !strcmp(test,"bad_encoder"))enc=(enc+1000U)&65535U;
    r[1]=enc>>8;r[2]=enc;break;}
   case 0x3a:r[1]=(enable?1:0)|((moves && delta>=1687.5)?2:0);
    if(moves && !strcmp(test,"disabled"))r[1]&=~1U;
    if(moves && !strcmp(test,"protection"))r[1]|=4;
    if(moves && !strcmp(test,"early_done_bit"))r[1]|=2;
    break;
   case 0x35:u=(uint32_t)llround(speed/6*10);
    if(!strcmp(test,"initial_fraction_speed") && !moves)u=5;
    if(!strcmp(test,"fraction_speed") && moves && delta>=1687.5)u=5;
    if(!strcmp(test,"overspeed") && moves)u=500;
    r[1]=0;r[2]=u>>8;r[3]=u;break;
   case 0x33:rawpos(r,initial+1687.5+(!strcmp(test,"target_mismatch")?10:0));break;
   default:forbidden++;return 0;
  }
 }else if(n==5 && q[0]==0xf3){
  CK(LiftRuler_CommandAllowed(c,q,n));enable=1;r[1]=2;
  if(!strcmp(test,"enable_ack_lost"))return 0;
 }else if(n==15 && q[0]==0xfd){
  CK(LiftRuler_CommandAllowed(c,q,n));
  u=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];
  CK(q[1]==0 && q[2]==0 && q[3]==60 && q[4]==0 && q[5]==60);
  CK((((unsigned)q[6]<<8)|q[7])==300 && u==15000U*360U*scale/3200U && q[12]==2 && q[13]==0 && q[14]==0x6b);
  moves++;moved_at=now;r[1]=2;if(!strcmp(test,"move_ack_lost"))return 0;
 }else if(n==4 && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b){
  stops++;if(!strcmp(test,"stop_ack_missing") || (!strcmp(test,"stop_then_ack") && stops<3))return 0;r[1]=2;
 }else{forbidden++;return 0;}
 return 1;
}
static void run(const char*name){
 LiftRulerController c;uint32_t i,base=0;unsigned cancel_sent=0;test=name;moves=stops=enable=forbidden=0;delta=speed=0;
 scale=!strcmp(test,"normal_unscaled")?10:100;if(!strcmp(test,"millis_wrap"))base=0xfffffd00U;
 LiftRuler_Init(&c);CK(c.state==LRC_IDLE && !c.vm_request);
 if(!strcmp(test,"boot")){
  for(i=0;i<20;i++)LiftRuler_Service(&c,i*50,1,1,1,bus,&c);
  CK(c.state==LRC_IDLE && !c.vm_request && !moves && !enable);return;
 }
 CK(!LiftRuler_Start(&c,base,0) && c.state==LRC_IDLE);
 CK(LiftRuler_Start(&c,base,1));CK(!LiftRuler_Start(&c,base,1));
 for(i=0;i<340;i++){
  uint32_t ready=1,power=1,healthy=1;now=base+i*50U;
  if(!strcmp(test,"service_gap") && i>=20 && i<28)continue;
  if(!strcmp(test,"power_wait") && i<20)power=0;
  if(!strcmp(test,"power_timeout"))power=0;
  if(!strcmp(test,"power_loss") && moves && (uint32_t)(now-moved_at)>500)power=0;
  if(!strcmp(test,"guard_lost") && i>20)ready=0;
  if(!strcmp(test,"can_health_lost") && i>20)healthy=0;
  if((!strcmp(test,"cancel") || !strcmp(test,"stop_ack_missing") || !strcmp(test,"stop_then_ack")) && moves && !cancel_sent){LiftRuler_Stop(&c,now);cancel_sent=1;}
  LiftRuler_Service(&c,now,ready,power,healthy,bus,&c);
  if(c.state==LRC_DONE || ((c.state==LRC_FAULT || c.state==LRC_CANCELLED) && !c.stop_pending))break;
 }
 CK(forbidden==0 && moves<=1 && c.moves_sent<=1);
 CK(c.vm_request==1);
 if(!strncmp(test,"normal",6) || !strcmp(test,"millis_wrap") || !strcmp(test,"power_wait") || !strcmp(test,"early_done_bit")){
  CK(c.state==LRC_DONE && c.samples==1 && moves==1 && !c.cause && !c.speed_raw);
  CK(llabs(c.position_after_raw-c.command_before_raw-(int64_t)c.move_units)<=(int64_t)(.3*scale));
  CK((uint32_t)(now-moved_at)>=9875U);
 }else{
  CK(c.state!=LRC_DONE && c.cause && !c.samples);
  if(!strcmp(test,"stop_ack_missing") || !strcmp(test,"power_loss"))CK(c.stop_pending && c.stop_unconfirmed && !c.stop_confirmed);
  else CK(c.stop_confirmed && !c.stop_pending);
 }
 if(c.state==LRC_DONE){
  CK(LiftRuler_Start(&c,now,1));CK(c.samples==1 && c.vm_request==1);
  CK(!LiftRuler_Start(&c,now,1));
 }else CK(!LiftRuler_Start(&c,now,1));
}

static void repeat_limit(void){
 LiftRulerController c;unsigned k,i;test="explicit_repeat_limit";initial=-123.4;scale=100;enable=0;
 LiftRuler_Init(&c);
 for(k=0;k<7;k++){
  moves=stops=forbidden=0;delta=speed=0;now=k*20000U;
  CK(LiftRuler_Start(&c,now,1));CK(c.vm_request==1 && c.samples==k);
  CK(!LiftRuler_Start(&c,now,1));
  for(i=0;i<340;i++){
   now=k*20000U+i*50U;
   LiftRuler_Service(&c,now,1,1,1,bus,&c);
   if(c.state>=LRC_DONE)break;
  }
  CK(c.state==LRC_DONE && c.samples==k+1 && c.vm_request==1 && moves==1 && !forbidden);
  i=moves;LiftRuler_Service(&c,now+50,1,1,1,bus,&c);CK(moves==i);
  initial+=delta;
 }
 CK(!LiftRuler_Start(&c,now+100,1));CK(c.samples==7 && c.vm_request==1);
}

int main(void){
 static const char*cases[]={"boot","normal_scaled","normal_unscaled","millis_wrap","power_wait","early_done_bit",
 "bad_options","bad_position_sign","initial_fraction_speed","enable_ack_lost","move_ack_lost","target_mismatch",
 "position_frozen","fraction_speed","disabled","protection","cancel","service_gap","read_failure",
 "stop_ack_missing","stop_then_ack","wrong_direction","overshoot","overspeed","guard_lost","power_loss",
 "can_health_lost","power_timeout"};
 unsigned i;for(i=0;i<sizeof(cases)/sizeof(cases[0]);i++)run(cases[i]);
 repeat_limit();
 printf("LIFT_RULER cases=%u checks=%u failures=%u\n",(unsigned)(sizeof(cases)/sizeof(cases[0])),checks,failures);return failures?1:0;
}
