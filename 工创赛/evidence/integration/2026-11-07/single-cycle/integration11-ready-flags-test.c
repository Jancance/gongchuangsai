#include "lift_ruler_calibration.h"
#include <stdio.h>
#include <string.h>
static int which,checks,failures,aborts,queries;static unsigned position_value;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d case%d\n",__LINE__,which);}}while(0)
static uint32_t ex(void*u,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 (void)u;(void)n;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 CK(q[0]!=0xfd && q[0]!=0xf3 && q[0]!=0x9a);
 if(q[0]==0x36){unsigned v=position_value;if(which==6 && queries>0)v+=31;
  r[2]=(v>>24)&255;r[3]=(v>>16)&255;r[4]=(v>>8)&255;r[5]=v&255;queries++;return 1;}
 if(q[0]==0x3b){r[1]=((which==1||which==2||which==3)&&!aborts)?7:3;return 1;}
 if(q[0]==0x9c){aborts++;r[1]=which==2?0xe2:2;return which==3?0:1;}
 if(q[0]==0x3a){r[1]=which==4?15:which==7?2:3;return 1;}
 if(q[0]==0x35){r[3]=which==5?1:0;return 1;}
 if(q[0]==0xfe){r[1]=2;return 1;}return 0;
}
int main(void){uint32_t now,scale;
 for(which=0;which<8;which++){
  LiftRulerController c;LiftRuler_Init(&c);c.state=LRC_MOVING;c.phase=26;c.vm_request=1;c.scale=100;
  c.home_started=c.home_seen_running=c.home_params_valid=1;c.operation=0;
  position_value=0;queries=aborts=0;
  for(now=10;now<44000;now+=10){LiftRuler_Service(&c,now,1,1,1,ex,0);if(c.state>=LRC_DONE)break;}
  if((which<2 || which==7)){CK(c.state==LRC_DONE && c.top_valid && c.home_reported && c.home_exit_confirmed && c.top_position_raw==0);CK(c.moves_sent==0);}
  else{CK(c.cause && !c.top_valid && !c.home_reported && c.samples==0 && c.stop_pending);}
  if(which==0)CK(aborts==0);if(which==1)CK(aborts==1);if(which==4)CK(c.cause==LRC_CAUSE_DRIVER);
 }
 which=0;
 for(scale=10;scale<=100;scale+=90){
  LiftRulerController c;LiftRuler_Init(&c);c.state=LRC_PREP;c.phase=7;c.vm_request=1;c.scale=c.top_scale=scale;c.operation=2;
  position_value=108400U*360U*scale/3200U;queries=aborts=0;
  LiftRuler_Service(&c,10,1,1,1,ex,0);
  CK(c.phase==8 && c.goal_raw==3200LL*360*scale/3200);
  CK(c.command_speed==18464 && c.move_units==105200U*360U*scale/3200U);
  CK(c.move_frame[1]==1 && c.move_frame[12]==2);
 }
 printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}
