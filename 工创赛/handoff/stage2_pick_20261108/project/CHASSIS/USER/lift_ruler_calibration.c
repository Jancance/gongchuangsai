#include "lift_ruler_calibration.h"
#include <string.h>
#define PRELIFT_PULSES 90000U
#define DOWN_TARGET_PULSES 118800U
#define RETURN_TOP_MARGIN_PULSES 7200U
#define REFERENCE_SPEED 18464U
#define REFERENCE_ACCEL 45055U
static int64_t absolute(int64_t v){return v<0?-v:v;}
static uint32_t be16(const uint8_t*p){return ((uint32_t)p[0]<<8)|p[1];}
static uint32_t be32(const uint8_t*p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
static uint32_t position(const uint8_t*r,int64_t*p)
{if(r[1]>1U)return 0U;*p=r[1]?-(int64_t)be32(r+2):(int64_t)be32(r+2);return 1U;}
static uint32_t exchange(LiftRulerExchange fn,void*ctx,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r)
{memset(r,0,rn);return fn && fn(ctx,q,n,rn,r)==1U && r[0]==q[0] && r[rn-1U]==0x6bU;}
static uint32_t query(LiftRulerExchange fn,void*ctx,uint8_t cmd,uint32_t rn,uint8_t*r)
{uint8_t q[2];q[0]=cmd;q[1]=0x6b;return exchange(fn,ctx,q,2U,rn,r);}

/* 2026-11-08: this driver answers the 0x22 homing-parameter read only while the
 * motor is enabled; a disabled driver replies with an all-zero payload, which
 * home_parameters_valid() rejects (LRC_CAUSE_HOME_PARAMS). Enable once per phase
 * before that read. The phase-5 enable is unchanged and LiftRuler_Start re-arms
 * the flag, so a new transfer always re-enables the driver first. */
static uint32_t s_hp_enable_phase=0xFFFFFFFFU;
static uint32_t hp_enable_before_params(LiftRulerExchange fn,void*ctx,uint32_t phase,uint8_t*r)
{
 static const uint8_t hp_enable[5]={0xf3,0xab,1,0,0x6b};
 if(s_hp_enable_phase==phase)return 1U;
 if(!exchange(fn,ctx,hp_enable,5U,3U,r) || r[1]!=2U)return 0U;
 s_hp_enable_phase=phase;return 1U;
}
static void fail(LiftRulerController*c,uint32_t cause,uint32_t now)
{
 if(!c->cause)c->cause=cause;
 c->backoff_valid=0;
 c->state=cause==LRC_CAUSE_CANCEL?LRC_CANCELLED:LRC_FAULT;c->top_valid=0;c->home_reported=0;
 if(!c->stop_pending){c->stop_pending=1;c->stop_begin_ms=now;c->last_stop_ms=now-50U;}
 c->stop_confirmed=0; /* Keep VM. Never clear a fault to continue the comparison. */
}
void LiftRuler_Init(LiftRulerController*c){if(c){memset(c,0,sizeof(*c));c->down_target_pulses=DOWN_TARGET_PULSES;}}
uint32_t LiftRuler_SetDownTarget(LiftRulerController*c,uint32_t pulses)
{
 if(!c || c->cause || c->stop_pending || (c->state!=LRC_IDLE && c->state!=LRC_DONE) ||
    pulses<100000U || pulses>DOWN_TARGET_PULSES)return 0U;
 c->down_target_pulses=pulses;return 1U;
}
uint32_t LiftRuler_Start(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 if(!c || ready!=1U || c->cause || c->stop_pending || c->operation>3U)return 0;
 if(c->state!=LRC_IDLE && c->state!=LRC_DONE)return 0;
 if(c->operation && !c->top_valid)return 0;
 if(!c->down_target_pulses)c->down_target_pulses=DOWN_TARGET_PULSES;
 if(c->down_target_pulses<100000U || c->down_target_pulses>DOWN_TARGET_PULSES)return 0;
 /* A completed return can start the next explicitly requested down test.
  * No fault is cleared and no reference is synthesized. */
 if(c->operation==3U)c->operation=1U;
 c->backoff_active=0U;if(c->operation!=1U)c->backoff_valid=0U;
 c->phase=c->stable=c->moves_sent=c->stop_confirmed=c->stop_unconfirmed=0;
 c->home_started=c->home_seen_running=c->home_reported=c->home_params_valid=c->home_exit_confirmed=0;
 s_hp_enable_phase=0xFFFFFFFFU;
 c->state=LRC_POWER;c->vm_request=1;c->begin_ms=c->last_service_ms=now;return 1;
}
uint32_t LiftRuler_StartReturn(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 uint32_t previous;
 if(!c || !ready || c->state!=LRC_DONE || !c->top_valid || c->cause || c->stop_pending)return 0U;
 c->reuse_count=(c->reuse_count>=9U)?0U:(c->reuse_count+1U);
 c->reuse_reference=c->top_valid?((c->reuse_count!=0U)?1U:0U):0U;
 previous=c->operation;c->operation=2U;
 if(LiftRuler_Start(c,now,ready))return 1U;
 c->operation=previous;return 0U;
}
uint32_t LiftRuler_StartRelease(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 if(!c || ready!=1U || c->state!=LRC_IDLE || c->operation || c->cause || c->stop_pending)return 0;
 c->operation=4U;c->state=LRC_POWER;c->vm_request=1U;
 c->phase=c->stable=c->moves_sent=c->stop_confirmed=c->stop_unconfirmed=0U;
 c->top_valid=c->home_reported=0U;
 c->begin_ms=c->last_service_ms=now;return 1U;
}
void LiftRuler_Stop(LiftRulerController*c,uint32_t now)
{if(c && c->state!=LRC_IDLE)fail(c,LRC_CAUSE_CANCEL,now);}
uint32_t LiftRuler_CommandAllowed(const LiftRulerController*c,const uint8_t*q,uint32_t n)
{
 static const uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
 static const uint8_t home[4]={0x9a,2,0,0x6b};
 if(!c || !q || !c->vm_request || c->cause || c->stop_pending)return 0;
 if(c->state!=LRC_PREP && c->state!=LRC_MOVING)return 0;
 if(c->phase==5U && n==5U && !memcmp(q,enable,5U))return 1;
 if((c->phase==2U || c->phase==20U) && n==5U && !memcmp(q,enable,5U))return 1;
 if(c->phase==8U && n==15U && c->moves_sent==1U && !memcmp(q,c->move_frame,15U))return 1;
 return c->phase==23U && n==4U && c->home_params_valid && c->home_started && !memcmp(q,home,4U);
}
static void done(LiftRulerController*c)
{c->operation++;c->samples++;c->state=LRC_DONE;}
static uint32_t home_parameters_valid(const uint8_t*r)
{
 return r[1]<=5U && r[2]==1U && be16(r+3)==60U && be32(r+5)==40000U &&
  be16(r+9)==300U && be16(r+11)==800U && be16(r+13)==60U && r[15]==0U;
}
void LiftRuler_Service(LiftRulerController*c,uint32_t now,uint32_t ready,uint32_t power,uint32_t healthy,
 LiftRulerExchange fn,void*ctx)
{
 uint8_t r[17],enable[5]={0xf3,0xab,1,0,0x6b},stop[4]={0xfe,0x98,0,0x6b};
 uint8_t home[4]={0x9a,2,0,0x6b},abort_home[3]={0x9c,0x48,0x6b};
 int64_t p,delta;uint32_t units,modulo;int32_t residual;
 if(!c)return;
 if(c->stop_pending){
  if((uint32_t)(now-c->stop_begin_ms)>=300U)c->stop_unconfirmed=1;
  if((uint32_t)(now-c->last_stop_ms)<50U)return;c->last_stop_ms=now;
  if(c->home_started && !c->home_exit_confirmed){
   if(exchange(fn,ctx,abort_home,3,3,r) && (r[1]==2U || r[1]==0x9fU))c->home_exit_confirmed=1;
   /* Continue sending STOP too when abort acknowledgement is missing. */
  }
  if(exchange(fn,ctx,stop,4,3,r) && r[1]==2U)c->stop_confirmed=1;
  if(c->stop_confirmed && (!c->home_started || c->home_exit_confirmed))c->stop_pending=0;
  return;
 }
 if(c->state==LRC_IDLE || c->state>=LRC_DONE)return;
 if(ready!=1U || (c->state!=LRC_POWER && (power!=1U || healthy!=1U))){fail(c,LRC_CAUSE_GUARD,now);return;}
 if((uint32_t)(now-c->last_service_ms)>300U){fail(c,LRC_CAUSE_SERVICE,now);return;}
 c->last_service_ms=now;
 if(c->phase>=20U && (uint32_t)(now-c->move_begin_ms)>=43000U){fail(c,LRC_CAUSE_TIMEOUT,now);return;}
 if(c->phase<20U && (uint32_t)(now-c->begin_ms)>=16000U){fail(c,LRC_CAUSE_TIMEOUT,now);return;}
 if(c->state==LRC_POWER){
  if((uint32_t)(now-c->begin_ms)>=5000U){fail(c,LRC_CAUSE_POWER,now);return;}
  if(power==1U && healthy==1U){c->state=LRC_PREP;c->phase=0;c->prep_ms=now;}
  return;
 }
 if(c->state==LRC_PREP && (uint32_t)(now-c->prep_ms)>=2000U){fail(c,LRC_CAUSE_PREP,now);return;}
 switch(c->phase){
 case 0:
  if(!query(fn,ctx,0x1a,4,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[2]&3U){fail(c,LRC_CAUSE_OPTIONS,now);return;}
  c->options=r[2];c->scale=(r[2]&0x80U)?100U:10U;
  if(c->operation && c->operation!=4U && c->scale!=c->top_scale){fail(c,LRC_CAUSE_OPTIONS,now);return;}break;
 case 1:case 7:
  if(!query(fn,ctx,0x36,7,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!position(r,&p)){fail(c,LRC_CAUSE_REPLY,now);return;}
  c->command_before_raw=c->position_after_raw=p;c->last_position_ms=now;
  if(c->phase==1U){c->position_before_raw=p;break;}
  if(c->backoff_active){
   c->goal_raw=c->top_position_raw+(int64_t)2400U*360*c->scale/3200; /* 1.5mm backoff */
   /* backoff was 30RPM/6RPM-s and crawled 2.7s. Raise speed AND accel by the same
      10x so the profile shape is unchanged; the speed check threshold rises with
      command_speed, so no overshoot fault. */
   c->command_speed=3000U;c->command_accel=600U;
  }else if(c->operation==4U){
   c->goal_raw=p+(int64_t)3200U*360*c->scale/3200;
   c->command_speed=300U;c->command_accel=60U;
  }else if(c->operation==0U){
   c->goal_raw=p-(int64_t)PRELIFT_PULSES*360*c->scale/3200;
   c->command_speed=3000U;c->command_accel=60U;
  }else{
   units=c->operation==1U?c->down_target_pulses:RETURN_TOP_MARGIN_PULSES;
   c->goal_raw=c->top_position_raw+(int64_t)units*360*c->scale/3200;
   c->command_speed=REFERENCE_SPEED;c->command_accel=REFERENCE_ACCEL;
  }
  delta=c->goal_raw-p;
  if(c->backoff_active && (delta<=0 || delta>(int64_t)(360U*c->scale+c->scale*3U/10U))){fail(c,LRC_CAUSE_POSITION,now);return;}
  if(!delta || absolute(delta)>(int64_t)DOWN_TARGET_PULSES*360*c->scale/3200+2*(int64_t)c->scale ||
     absolute(c->goal_raw)>0x7fffffffLL){fail(c,LRC_CAUSE_POSITION,now);return;}
  c->move_units=(uint32_t)absolute(delta);
  memset(c->move_frame,0,15);c->move_frame[0]=0xfd;c->move_frame[1]=delta<0;
  c->move_frame[2]=c->move_frame[4]=(uint8_t)(c->command_accel>>8);
  c->move_frame[3]=c->move_frame[5]=(uint8_t)c->command_accel;
  c->move_frame[6]=(uint8_t)(c->command_speed>>8);c->move_frame[7]=(uint8_t)c->command_speed;
  c->move_frame[8]=(uint8_t)(c->move_units>>24);c->move_frame[9]=(uint8_t)(c->move_units>>16);
  c->move_frame[10]=(uint8_t)(c->move_units>>8);c->move_frame[11]=(uint8_t)c->move_units;
  c->move_frame[12]=2;c->move_frame[14]=0x6b;
  c->expected_encoder_mod=(uint32_t)((int64_t)(delta*65536/(360*c->scale))&65535U);break;
 case 2:
  if(c->operation==4U){
   if(!query(fn,ctx,0x3b,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}
   if((r[1]&3U)!=3U || (r[1]&0x3cU)){fail(c,LRC_CAUSE_HOME_FLAGS,now);return;}
  }
  if(c->operation==0U){
   if(!hp_enable_before_params(fn,ctx,2U,r) || !query(fn,ctx,0x22,17,r)){fail(c,LRC_CAUSE_COMM,now);return;}
   memcpy(c->home_parameters,r,17);
   if(!home_parameters_valid(r)){fail(c,LRC_CAUSE_HOME_PARAMS,now);return;}
   c->home_params_valid=1;
  }
  if(!query(fn,ctx,0x31,4,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->encoder_before=be16(r+1);break;
 case 3:case 6:case 10:
  if(!query(fn,ctx,0x3a,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->flags=r[1];
  if((c->flags&12U) || (c->phase>=6U && !(c->flags&1U))){fail(c,LRC_CAUSE_DRIVER,now);return;}
  /* 本次用户已确认顶部：只低速回零，不发�?0000脉冲预升�?*/
  if(c->phase==6U && c->operation==0U && !c->backoff_active){c->phase=20U;c->move_begin_ms=now;c->state=LRC_MOVING;return;}break;
 case 4:
  if(!query(fn,ctx,0x35,5,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){fail(c,LRC_CAUSE_REPLY,now);return;}
  c->speed_raw=be16(r+2);
  if(c->speed_raw){c->stable=0;c->phase=3;return;}
  if(++c->stable<3U){c->phase=3;return;}c->stable=0;break;
 case 5:
  if(!exchange(fn,ctx,enable,5,3,r) || r[1]!=2U){fail(c,LRC_CAUSE_COMM,now);return;}break;
 case 8:
  if(c->moves_sent || (uint32_t)(now-c->last_position_ms)>=400U){fail(c,LRC_CAUSE_POSITION,now);return;}
  c->moves_sent=1;c->move_begin_ms=now;
  if(!exchange(fn,ctx,c->move_frame,15,3,r) || (r[1]!=2U && r[1]!=0x9fU)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->state=LRC_MOVING;break;
 case 9:
  if(!query(fn,ctx,0x33,7,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!position(r,&p)){fail(c,LRC_CAUSE_REPLY,now);return;}c->driver_target_raw=p;
  if(absolute(p-c->goal_raw)>(int64_t)c->scale){
   /* Backoff after native homing: 0x33 may still hold the homing target or -1/-2
      until the drive latches the new frame. Re-send the same absolute frame inside
      a bounded window measured from the phase-30 timestamp; verification is kept. */
   if(c->backoff_active && (uint32_t)(now-c->begin_ms)<3000U){
    if((uint32_t)(now-c->last_position_ms)>=250U){c->last_position_ms=now;c->moves_sent=0U;c->phase=8U;}
    return;
   }
   fail(c,LRC_CAUSE_TARGET,now);return;
  }break;
 case 11:
  if(!query(fn,ctx,0x36,7,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!position(r,&p)){fail(c,LRC_CAUSE_REPLY,now);return;}c->position_after_raw=p;c->last_position_ms=now;
  /* Up to0.5mm transient endpoint deviation, then normal settling; severe
   * excursion still stops. A single small fast-stop transient is not DONE. */
  delta=c->goal_raw-c->command_before_raw;
  if((delta>=0 && (p<c->command_before_raw-90*(int64_t)c->scale || p>c->goal_raw+90*(int64_t)c->scale)) ||
     (delta<0 && (p>c->command_before_raw+90*(int64_t)c->scale || p<c->goal_raw-90*(int64_t)c->scale)))
  {fail(c,LRC_CAUSE_POSITION,now);return;}break;
 case 12:
  if(!query(fn,ctx,0x35,5,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){fail(c,LRC_CAUSE_REPLY,now);return;}c->speed_raw=be16(r+2);
  if(c->speed_raw>c->command_speed+c->command_speed/2U){fail(c,LRC_CAUSE_SPEED,now);return;}
  if((uint32_t)(now-c->last_position_ms)<400U && !c->speed_raw && (c->flags&2U) &&
     absolute(c->position_after_raw-c->goal_raw)<=(int64_t)(c->scale*3U/10U))c->stable++;else c->stable=0;
  if(c->stable<3U){c->phase=10;return;}c->move_elapsed_ms=now-c->move_begin_ms;break;
 case 13:
  if(!query(fn,ctx,0x31,4,r)){fail(c,LRC_CAUSE_COMM,now);return;}c->encoder_after=be16(r+1);
  modulo=(c->encoder_after-c->encoder_before)&65535U;
  residual=(int32_t)((modulo-c->expected_encoder_mod)&65535U);if(residual>32767)residual-=65536;
  if(residual>365 || residual< -365){fail(c,LRC_CAUSE_ENCODER,now);return;}
  if(c->backoff_active){
   c->backoff_active=0U;c->backoff_valid=1U;
   c->safe_position_raw=c->position_after_raw;c->top_valid=1U;c->home_reported=1U;
   done(c);return;
  }
  if(c->operation==4U){
   c->state=LRC_DONE;c->samples++;c->top_valid=c->home_reported=0U;return;
  }
  if(c->operation==1U){done(c);return;}
  /* Reuse the still-valid top reference: skip the slow native homing and the
     extra backoff. A real homing is forced every 5th return. */
  if(c->operation==2U && c->reuse_reference){
   c->safe_position_raw=c->position_after_raw;c->backoff_active=0U;c->backoff_valid=1U;
   c->top_valid=1U;c->home_reported=1U;done(c);return;}
  c->phase=20U;c->move_begin_ms=now;return;
 case 20:
  hp_enable_before_params(fn,ctx,20U,r); /* best effort: params may already be enabled */
  if(!query(fn,ctx,0x22,17,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  memcpy(c->home_parameters,r,17);
  if(!home_parameters_valid(r)){fail(c,LRC_CAUSE_HOME_PARAMS,now);return;}
  c->home_params_valid=1;break;
 case 21:
  if(!query(fn,ctx,0x3a,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!(r[1]&1U) || (r[1]&12U)){fail(c,LRC_CAUSE_DRIVER,now);return;}break;
 case 22:case 24:
  if(!query(fn,ctx,0x3b,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}c->home_flags=r[1];
  if((r[1]&3U)!=3U || (r[1]&0x38U) || (c->phase==22U && (r[1]&4U))){fail(c,LRC_CAUSE_HOME_FLAGS,now);return;}
  if(c->phase==22U)break;
  if(r[1]&4U)c->home_seen_running=1;
  if((r[1]&4U) || !c->home_seen_running)return;
  c->phase=25U;return;
 case 23:
  c->home_started=1;c->stable=0;
  if(!exchange(fn,ctx,home,4,3,r) || (r[1]!=2U && r[1]!=0x9fU)){fail(c,LRC_CAUSE_COMM,now);return;}
  break;
 case 25:
  if(!query(fn,ctx,0x35,5,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){fail(c,LRC_CAUSE_REPLY,now);return;}c->speed_raw=be16(r+2);
  if(c->speed_raw)c->stable=0;else c->stable++;
  if(c->stable<3U){c->phase=24U;return;}break;
 case 26:
  if(!query(fn,ctx,0x36,7,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!position(r,&p)){fail(c,LRC_CAUSE_REPLY,now);return;}
  c->position_after_raw=c->top_position_raw=p;c->top_scale=c->scale;c->top_valid=0U;c->backoff_valid=0U;
  c->stable=0;break;
 case 27:
  /* Explicitly terminate the native homing command before ordinary motion.
   * 3B running-clear alone does not prove that its command context was released. */
  if(!c->home_exit_confirmed){
   if(!exchange(fn,ctx,abort_home,3,3,r) || r[1]!=2U){fail(c,LRC_CAUSE_REPLY,now);return;}
   c->home_exit_confirmed=1U;return;
  }
  if(!query(fn,ctx,0x3b,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->home_flags=r[1];
  if((r[1]&3U)!=3U || (r[1]&0x38U)){fail(c,LRC_CAUSE_HOME_FLAGS,now);return;}
  if(r[1]&4U)return;
  break;
 case 28:
  if(!query(fn,ctx,0x3a,3,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->flags=r[1];
  if(!(c->flags&2U) || (c->flags&12U)){fail(c,LRC_CAUSE_DRIVER,now);return;}
  break;
 case 29:
  if(!query(fn,ctx,0x35,5,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(r[1]>1U){fail(c,LRC_CAUSE_REPLY,now);return;}c->speed_raw=be16(r+2);
  if(c->speed_raw)c->stable=0;else c->stable++;
  if(c->stable<3U){c->phase=28U;return;}break;
 case 30:
  if(!query(fn,ctx,0x36,7,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  if(!position(r,&p)){fail(c,LRC_CAUSE_REPLY,now);return;}
  if(absolute(p-c->top_position_raw)>(int64_t)(c->scale*3U/10U)){
   fail(c,LRC_CAUSE_POSITION,now);return;}
  /* Keep the measured homing reference; no extra backoff or shifted pickup. */
  c->position_after_raw=p;c->backoff_active=1U;c->backoff_valid=0U;
  c->stable=0U;c->moves_sent=0U;c->begin_ms=c->move_begin_ms=now;
  c->phase=31U;return;
 case 31:
  if(!query(fn,ctx,0x31,4,r)){fail(c,LRC_CAUSE_COMM,now);return;}
  c->encoder_before=be16(r+1);c->phase=5U;return;
 default:fail(c,LRC_CAUSE_REPLY,now);return;
 }
 c->phase++;
}
