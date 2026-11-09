#include "lift_ruler_calibration.h"
#include <stdio.h>
#include <string.h>
static LiftRulerController*c;static int which,checks,failures,moves,enables,homes;static unsigned scale;static long pos,goal;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d case%d\n",__LINE__,which);}}while(0)
static void putpos(uint8_t*r,long v){unsigned a=v<0?-v:v;r[1]=v<0;r[2]=a>>24;r[3]=a>>16;r[4]=a>>8;r[5]=a;}
static uint32_t ex(void*u,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){unsigned units;
 (void)u;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(q[0]==0x9a){homes++;return 0;}
 if(q[0]==0x1a){r[2]=scale==100?0x84:4;return 1;}
 if(q[0]==0x3b){r[1]=which==2?7:3;return 1;}
 if(q[0]==0x31){r[1]=4;r[2]=0xd2;return 1;}
 if(q[0]==0x36){putpos(r,pos);return which==6?0:1;}
 if(q[0]==0x33){putpos(r,goal+(which==5?1000:0));return 1;}
 if(q[0]==0x3a){r[1]=which==1?15:3;return 1;}
 if(q[0]==0x35){if(which==7)r[3]=1;return 1;}
 if(q[0]==0xf3){enables++;CK(LiftRuler_CommandAllowed(c,q,n));r[1]=2;return 1;}
 if(q[0]==0xfd){moves++;CK(LiftRuler_CommandAllowed(c,q,n));CK(q[1]==0 && q[12]==2);
  CK(q[6]==1 && q[7]==44 && q[3]==60 && q[5]==60);
  units=((unsigned)q[8]<<24)|((unsigned)q[9]<<16)|((unsigned)q[10]<<8)|q[11];CK(units==360*scale);
  goal=pos+units;if(which!=8)pos=goal;
  r[1]=which==4?0xe2:2;return which==3?0:1;
 }
 if(q[0]==0xfe){r[1]=2;return 1;}return 0;
}
int main(void){unsigned now;
 for(scale=10;scale<=100;scale+=90)for(which=0;which<9;which++){
  LiftRulerController x;c=&x;LiftRuler_Init(c);pos=-500;goal=pos;moves=enables=homes=0;
  CK(!LiftRuler_StartRelease(c,0,0));CK(LiftRuler_StartRelease(c,0,1));
  for(now=10;now<17000;now+=10){LiftRuler_Service(c,now,1,1,1,ex,0);if(c->state>=LRC_DONE)break;}
  CK(homes==0 && moves<=1 && c->top_valid==0 && c->home_reported==0 && c->down_target_pulses==108400);
  if(which==0){CK(c->state==LRC_DONE && !c->cause && c->operation==4 && c->samples==1 && pos==-500L+360L*(long)scale);}
  else{CK(c->cause && c->stop_pending);}
  CK(!LiftRuler_StartRelease(c,now+10,1));CK(!LiftRuler_Start(c,now+10,1));
  if(which==1||which==2||which==6||which==7)CK(moves==0);
 }
 printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}
