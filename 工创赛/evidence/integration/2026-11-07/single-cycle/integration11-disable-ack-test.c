#include "lift_ruler_calibration.h"
#include <stdio.h>
#include <string.h>
static int scenario,checks,failures,disable_count;static LiftRulerController *active;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %d scenario %d\n",__LINE__,scenario);}}while(0)
static uint32_t ex(void*u,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 (void)u;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(q[0]==0xf3){CK(n==5 && q[2]==0);CK(LiftRuler_CommandAllowed(active,q,n));disable_count++;r[1]=scenario==1?0xe2:2;return scenario==2?0:1;}
 if(q[0]==0x36){unsigned v=scenario==6 && active->phase==29?31:0;r[5]=(uint8_t)v;return 1;}
 if(q[0]==0x3a){r[1]=scenario==3?3:scenario==4?0x0e:2;return 1;}
 if(q[0]==0x35){if(scenario==5)r[3]=1;return 1;}
 if(q[0]==0xfe){r[1]=2;return 1;}return 0;
}
int main(void){int i;uint32_t now;uint8_t off[5]={0xf3,0xab,0,0,0x6b};
 for(scenario=0;scenario<7;scenario++){
  LiftRulerController c;LiftRuler_Init(&c);active=&c;c.state=LRC_MOVING;c.phase=26;c.vm_request=1;c.scale=100;c.home_started=1;c.home_seen_running=1;c.home_params_valid=1;c.operation=0;c.move_begin_ms=0;c.last_service_ms=0;disable_count=0;
  CK(LiftRuler_CommandAllowed(&c,off,5));
  for(now=10;now<=1000;now+=10){LiftRuler_Service(&c,now,1,1,1,ex,0);if(c.state>=LRC_DONE)break;}
  CK(disable_count==1);
  if(scenario==0){CK(c.state==LRC_DONE && c.top_valid && c.home_reported && c.flags==2 && c.samples==1);}
  else{CK(c.cause && !c.top_valid && !c.home_reported && c.stop_pending && c.samples==0);CK(!LiftRuler_Start(&c,now+1,1));}
 }
 {LiftRulerController c;LiftRuler_Init(&c);c.state=LRC_MOVING;c.vm_request=1;c.home_started=c.home_seen_running=c.home_params_valid=1;
  for(i=0;i<31;i++){c.phase=i;CK(LiftRuler_CommandAllowed(&c,off,5)==(i==26));}
  c.phase=26;c.cause=9;CK(!LiftRuler_CommandAllowed(&c,off,5));c.cause=0;c.stop_pending=1;CK(!LiftRuler_CommandAllowed(&c,off,5));
 }
 printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}
