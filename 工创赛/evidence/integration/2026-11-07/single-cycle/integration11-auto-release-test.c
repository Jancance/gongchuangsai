#include "lift_ruler_calibration.h"
#include <stdio.h>
#include <string.h>
static LiftRulerController*c;static int which,checks,failures,enabled,homebusy,homes,posthome,frames;static unsigned scale;static long pos,goal;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line%d case%d scale%u phase%u\n",__LINE__,which,scale,c->phase);}}while(0)
static void p32(uint8_t*r,long v){unsigned a=v<0?-v:v;r[1]=v<0;r[2]=a>>24;r[3]=a>>16;r[4]=a>>8;r[5]=a;}
static uint32_t ex(void*u,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){unsigned a,enc;(void)u;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 switch(q[0]){
 case 0x1a:r[2]=scale==100?0x84:4;return 1;
 case 0x22:r[1]=2;r[2]=1;r[3]=0;r[4]=60;r[5]=0;r[6]=0;r[7]=0x9c;r[8]=0x40;r[9]=1;r[10]=0x2c;r[11]=3;r[12]=0x20;r[13]=0;r[14]=60;r[15]=0;return 1;
 case 0x36:p32(r,pos);return 1;
 case 0x33:if(which==10 && c->backoff_active && c->last_service_ms-c->move_begin_ms<100U){p32(r,0);return 1;}p32(r,goal+(which==3&&c->backoff_active?1000:0));return 1;
 case 0x31:enc=(unsigned)((long long)pos*65536/(360*scale))&65535;if(which==7&&c->backoff_active&&c->phase==13)enc+=1000;r[1]=enc>>8;r[2]=enc;return 1;
 case 0x35:return 1;
 case 0x3a:r[1]=(enabled?1:0)|2;if(which==4&&posthome)r[1]=15;return 1;
 case 0x3b:
  if(which==13 && c->phase==27 && c->home_exit_confirmed){r[1]=7;return 1;}
  if(homebusy==2){homebusy=1;r[1]=7;return 1;}
  if(homebusy==1){homebusy=0;pos=0;posthome=1;if(which==1)enabled=0;}
  r[1]=3;return 1;
 case 0x9a:homes++;homebusy=2;CK(LiftRuler_CommandAllowed(c,q,n));r[1]=2;return which==8?0:1;
 case 0x9c:homebusy=0;r[1]=which==11?0xe2:2;return which==12?0:1;
 case 0xf3:CK(LiftRuler_CommandAllowed(c,q,n));r[1]=which==5&&c->backoff_active?0xe2:2;if(r[1]==2)enabled=1;return 1;
 case 0xfd:
  frames++;CK(LiftRuler_CommandAllowed(c,q,n));a=((unsigned)q[8]<<24)|((unsigned)q[9]<<16)|((unsigned)q[10]<<8)|q[11];goal=pos+(q[1]?-(long)a:(long)a);
  if(c->backoff_active){CK(q[1]==0 && a==360*scale && c->command_speed==300 && c->command_accel==60);}
  else if(c->operation==1){CK(goal==108400L*360L*(long)scale/3200L);CK(a==105200U*360U*scale/3200U);}
  else if(c->operation==2){CK(goal==3200L*360L*(long)scale/3200L);CK(q[1]==1 && c->command_speed==18464);}
  if(!(which==6&&c->backoff_active))pos=goal;
  r[1]=2;return which==2&&c->backoff_active?0:1;
 case 0xfe:r[1]=2;return 1;
 }return 0;
}
int main(void){unsigned now,k;for(scale=10;scale<=100;scale+=90)for(which=0;which<14;which++){
 LiftRulerController x;c=&x;LiftRuler_Init(c);enabled=1;homebusy=homes=posthome=frames=0;pos=goal=0;now=0;
 for(k=0;k<((which<2||which==10)?5U:1U);k++){
  if(k==1||k==3)CK(LiftRuler_Start(c,now,1));else if(k>0)CK(LiftRuler_StartReturn(c,now,1));else CK(LiftRuler_Start(c,now,1));
  for(;;){now+=10;if(which==9&&c->backoff_active&&c->phase==7)LiftRuler_Stop(c,now);LiftRuler_Service(c,now,1,1,1,ex,0);if(c->state>=LRC_DONE)break;CK(now<100000);if(now>=100000)break;}
  if(which<2||which==10){CK(!c->cause && c->state==LRC_DONE && c->samples==k+1);
   if(k==0||k==2||k==4){CK(c->top_valid && c->backoff_valid && !c->backoff_active && c->home_reported);CK(c->top_position_raw==0 && c->safe_position_raw==(long long)360*scale && pos==c->safe_position_raw);}
   else{CK(c->top_valid && c->top_position_raw==0 && pos==108400L*360L*(long)scale/3200L);}
  }else{CK(c->cause && !c->top_valid && !c->backoff_valid && (c->stop_pending || c->stop_confirmed));CK(!LiftRuler_Start(c,now+1,1));}
  now+=10;
 }
 if(which<2||which==10)CK(homes==3 && frames==7);
 if(which>=2 && which!=10)CK(homes==1 && frames<=1);
 }
 printf("checks=%d failures=%d\n",checks,failures);return failures?1:0;
}
