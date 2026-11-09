#include "lift_ruler_calibration.h"
#include <string.h>
#define LRC_STEP_PULSES 5000U
#define LRC_SPEED_DECI_RPM 300U
#define LRC_ACCEL_RPM_S 60U
static int64_t lrc_abs(int64_t v){return v<0?-v:v;}
static uint32_t lrc_position(const uint8_t*r,int64_t*out)
{
 uint32_t v=((uint32_t)r[2]<<24)|((uint32_t)r[3]<<16)|((uint32_t)r[4]<<8)|r[5];
 if(r[1]>1U)return 0U;
 *out=r[1]?-(int64_t)v:(int64_t)v;return 1U;
}
static void lrc_fail(LiftRulerController*c,uint32_t why,uint32_t now)
{
 if(!c->cause)c->cause=why;
 c->state=why==LRC_CAUSE_CANCEL?LRC_CANCELLED:LRC_FAULT;
 if(!c->stop_pending){c->stop_pending=1;c->stop_begin_ms=now;c->last_stop_ms=now-50U;}
 c->stop_confirmed=0U; /* VM不因故障自动切断；只能STOP且不重发位移。 */
}
void LiftRuler_Init(LiftRulerController*c){if(c)memset(c,0,sizeof(*c));}
uint32_t LiftRuler_Start(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 if(!c || ready!=1U || c->state!=LRC_IDLE || c->moves_sent || c->samples || c->stop_pending)return 0U;
 c->state=LRC_POWER;c->vm_request=1;c->begin_ms=c->last_service_ms=now;return 1U;
}
void LiftRuler_Stop(LiftRulerController*c,uint32_t now)
{if(c && c->state!=LRC_IDLE)lrc_fail(c,LRC_CAUSE_CANCEL,now);}
uint32_t LiftRuler_CommandAllowed(const LiftRulerController*c,const uint8_t*q,uint32_t n)
{
 static const uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
 if(!c || !q || c->vm_request!=1U || c->cause || c->stop_pending || (c->state!=LRC_PREP && c->state!=LRC_MOVING))return 0U;
 if(n==5U && c->phase==5U && !memcmp(q,enable,5U))return 1U;
 return n==15U && c->phase==8U && c->moves_sent==1U && !memcmp(q,c->move_frame,15U);
}
static uint32_t lrc_exchange(LiftRulerExchange exchange,void*context,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{
 memset(r,0,8U);
 return exchange && exchange(context,q,n,rn,r)==1U && r[0]==q[0] && r[rn-1U]==0x6bU;
}
static uint32_t lrc_query(LiftRulerExchange exchange,void*context,uint8_t cmd,uint32_t rn,uint8_t*r)
{uint8_t q[2];q[0]=cmd;q[1]=0x6b;return lrc_exchange(exchange,context,q,2,rn,r);}
void LiftRuler_Service(LiftRulerController*c,uint32_t now,uint32_t ready,uint32_t power,uint32_t healthy,
 LiftRulerExchange exchange,void*context)
{
 uint8_t r[8],enable[5]={0xf3,0xab,1,0,0x6b},stop[4]={0xfe,0x98,0,0x6b};
 uint32_t value;int64_t pos;int32_t modulo_delta;
 if(!c)return;
 if(c->stop_pending){
  if((uint32_t)(now-c->stop_begin_ms)>=300U)c->stop_unconfirmed=1U;
  if((uint32_t)(now-c->last_stop_ms)<50U)return;
  c->last_stop_ms=now;
  if(lrc_exchange(exchange,context,stop,4,3,r) && r[1]==2U){c->stop_pending=0;c->stop_confirmed=1U;}
  return;
 }
 if(c->state==LRC_IDLE || c->state>=LRC_DONE)return;
 if(ready!=1U){lrc_fail(c,LRC_CAUSE_GUARD,now);return;}
 if((uint32_t)(now-c->last_service_ms)>300U){lrc_fail(c,LRC_CAUSE_SERVICE,now);return;}
 c->last_service_ms=now;
 if((uint32_t)(now-c->begin_ms)>=8000U){lrc_fail(c,LRC_CAUSE_TIMEOUT,now);return;}
 if(c->state==LRC_POWER){
  if((uint32_t)(now-c->begin_ms)>=5000U){lrc_fail(c,LRC_CAUSE_POWER,now);return;}
  if(power==1U && healthy==1U){c->state=LRC_PREP;c->phase=0;c->prep_ms=now;}
  return;
 }
 if(power!=1U || healthy!=1U){lrc_fail(c,LRC_CAUSE_GUARD,now);return;}
 if(c->state==LRC_PREP && (uint32_t)(now-c->prep_ms)>=2000U){lrc_fail(c,LRC_CAUSE_PREP,now);return;}
 switch(c->phase){
 case 0:
  if(!lrc_query(exchange,context,0x1a,4,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[2]&3U){lrc_fail(c,LRC_CAUSE_OPTIONS,now);return;}
  c->options=r[2];c->scale=(r[2]&0x80U)?100U:10U;break;
 case 1:
  if(!lrc_query(exchange,context,0x36,7,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(!lrc_position(r,&pos)){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->position_before_raw=c->command_before_raw=c->position_after_raw=pos;c->last_position_ms=now;break;
 case 2:
  if(!lrc_query(exchange,context,0x31,4,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  c->encoder_before=((uint32_t)r[1]<<8)|r[2];break;
 case 3:case 6:case 10:
  if(!lrc_query(exchange,context,0x3a,3,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  c->flags=r[1];
  if((c->flags&12U) || (c->phase>=6U && !(c->flags&1U))){lrc_fail(c,LRC_CAUSE_DRIVER,now);return;}
  break;
 case 4:
  if(!lrc_query(exchange,context,0x35,5,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->speed_raw=((uint32_t)r[2]<<8)|r[3];
  if(c->speed_raw){c->stable=0;c->phase=3;return;}
  if(++c->stable<3U){c->phase=3;return;}c->stable=0;break;
 case 5:
  if(!lrc_exchange(exchange,context,enable,5,3,r) || r[1]!=2U){lrc_fail(c,LRC_CAUSE_COMM,now);return;}break;
 case 7:
  if(!lrc_query(exchange,context,0x36,7,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(!lrc_position(r,&pos)){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->command_before_raw=c->position_after_raw=pos;c->last_position_ms=now;
  c->move_units=LRC_STEP_PULSES*360U*c->scale/3200U;c->goal_raw=pos+(int64_t)c->move_units;
  if(lrc_abs(c->goal_raw)>0x7fffffffLL){lrc_fail(c,LRC_CAUSE_POSITION,now);return;}
  memset(c->move_frame,0,15);c->move_frame[0]=0xfd;c->move_frame[1]=0; /* 已验向下：正位置方向。 */
  c->move_frame[2]=c->move_frame[4]=(uint8_t)(LRC_ACCEL_RPM_S>>8);
  c->move_frame[3]=c->move_frame[5]=(uint8_t)LRC_ACCEL_RPM_S;
  c->move_frame[6]=(uint8_t)(LRC_SPEED_DECI_RPM>>8);c->move_frame[7]=(uint8_t)LRC_SPEED_DECI_RPM;
  c->move_frame[8]=(uint8_t)(c->move_units>>24);c->move_frame[9]=(uint8_t)(c->move_units>>16);
  c->move_frame[10]=(uint8_t)(c->move_units>>8);c->move_frame[11]=(uint8_t)c->move_units;
  c->move_frame[12]=2;c->move_frame[14]=0x6b;break;
 case 8:
  if(c->moves_sent || (uint32_t)(now-c->last_position_ms)>=400U){lrc_fail(c,LRC_CAUSE_POSITION,now);return;}
  c->moves_sent=1U;
  if(!lrc_exchange(exchange,context,c->move_frame,15,3,r) || (r[1]!=2U && r[1]!=0x9fU))
  {lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  c->state=LRC_MOVING;break;
 case 9:
  if(!lrc_query(exchange,context,0x33,7,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(!lrc_position(r,&pos)){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->driver_target_raw=pos;
  if(lrc_abs(pos-c->goal_raw)>(int64_t)c->scale){lrc_fail(c,LRC_CAUSE_TARGET,now);return;}break;
 case 11:
  if(!lrc_query(exchange,context,0x36,7,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(!lrc_position(r,&pos)){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->position_after_raw=pos;c->last_position_ms=now;
  if(pos<c->command_before_raw-2*(int64_t)c->scale || pos>c->goal_raw+2*(int64_t)c->scale)
  {lrc_fail(c,LRC_CAUSE_POSITION,now);return;}break;
 case 12:
  if(!lrc_query(exchange,context,0x35,5,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){lrc_fail(c,LRC_CAUSE_REPLY,now);return;}
  c->speed_raw=((uint32_t)r[2]<<8)|r[3];
  if(c->speed_raw>450U){lrc_fail(c,LRC_CAUSE_SPEED,now);return;}
  if((uint32_t)(now-c->last_position_ms)<400U && !c->speed_raw && (c->flags&2U) &&
     lrc_abs(c->position_after_raw-c->goal_raw)<=(int64_t)((c->scale*3U)/10U))c->stable++;else c->stable=0;
  if(c->stable<3U){c->phase=10;return;}break;
 default:
  if(!lrc_query(exchange,context,0x31,4,r)){lrc_fail(c,LRC_CAUSE_COMM,now);return;}
  c->encoder_after=((uint32_t)r[1]<<8)|r[2];
  value=(c->encoder_after-c->encoder_before)&65535U;
  modulo_delta=(int32_t)((value-36864U)&65535U);if(modulo_delta>32767)modulo_delta-=65536;
  if(modulo_delta>365 || modulo_delta< -365){lrc_fail(c,LRC_CAUSE_ENCODER,now);return;}
  c->samples=1;c->state=LRC_DONE;return;
 }
 c->phase++;
}
