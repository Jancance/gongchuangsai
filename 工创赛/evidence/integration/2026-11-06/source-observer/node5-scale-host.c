
#include <stdint.h>
#include <stdio.h>
#include <math.h>
typedef struct {uint8_t Data[8];} Reply;
static Reply s_rx;static float s_current;static uint8_t s_address,s_options;
static float g_vehicle_lift_current_deg;static uint32_t g_vehicle_lift_feedback_valid;
static uint32_t query(uint8_t cmd,uint8_t n){return cmd==0x36 && n==7;}
static uint32_t position(void)
{
    uint32_t raw;
    if(!query(0x36,7U) || s_rx.Data[1]>1U)return 0U;
    raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
    s_current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw/((s_options&0x80U)?100.0f:10.0f);
    if(s_address==5U){g_vehicle_lift_current_deg=s_current;g_vehicle_lift_feedback_valid=1U;}
    return 1U;
}
int main(void){unsigned checks=0;uint32_t raw[]={0,27000,144017,1219509};
 for(unsigned addr=5;addr<=6;addr++)for(unsigned scale=10;scale<=100;scale+=90)
 for(unsigned sign=0;sign<2;sign++)for(unsigned i=0;i<4;i++){
  s_address=addr;s_options=scale==100?0x80:0;s_rx.Data[1]=sign;
  s_rx.Data[2]=raw[i]>>24;s_rx.Data[3]=raw[i]>>16;s_rx.Data[4]=raw[i]>>8;s_rx.Data[5]=raw[i];
  if(!position() || fabs(s_current-(sign?-1.:1.)*raw[i]/scale)>.01)return 1;checks++;
 }
 puts("NODE5_SCALE checks=32 failures=0");return 0;
}
