#ifndef TURN_BRAKE_H
#define TURN_BRAKE_H
#include <stdint.h>
#include <math.h>
#include "heading_controller.h"
#include "app_config.h"
/* Pure remaining-angle controller. Units: degrees, seconds; no hardware access. */
typedef struct {
    uint32_t tick, start_tick, stable_ms, moved;
    float start_yaw, angle, direction, speed_limit;
    float feedback_angle, feedback_rate, speed, acceleration;
    float lookahead_s;
} TurnBrake;
static __inline float TurnBrake_Clip(float x,float limit)
{ return x>limit ? limit : (x < -limit ? -limit : x); }
static __inline void TurnBrake_Init(TurnBrake *s,uint32_t now,float yaw,float angle,float speed)
{
    TurnBrake empty={0}; *s=empty;
    s->tick=s->start_tick=now; s->start_yaw=yaw;
    s->angle=fabsf(angle);s->direction=angle>0.0f?1.0f:-1.0f;
    s->speed_limit=speed;s->lookahead_s=0.7f;
}
/* Return0=running,1=stable,2=fault. Caller retains freshness/epoch/backend guards. */
static __inline uint32_t TurnBrake_Step(TurnBrake *s,uint32_t now,float yaw,float rate,float *command)
{
    uint32_t elapsed=now-s->tick;
    float dt,observed,remaining,ahead,distance,envelope,want,desired,alpha,rpm;
    *command=0;
    if(!(yaw>=-3600.0f && yaw<=3600.0f && rate>=-200.0f && rate<=200.0f) ||
       elapsed>=100U || now-s->start_tick>=30000U ||
       !(s->lookahead_s>=0.7f && s->lookahead_s<=0.9f)) return 2U;
    if(!elapsed) return 0U;
    dt=(float)elapsed/1000.0f;s->tick=now;
    observed=s->direction*HeadingController_ErrorDeg(yaw,s->start_yaw);
    /* Raw overshoot remains an abort even while the filtered controller settles. */
    if(observed>s->angle+1.0f || observed< -2.0f) return 2U;
    alpha=dt/(0.08f+dt);
    s->feedback_angle+=(observed-s->feedback_angle)*alpha;
    s->feedback_rate+=(s->direction*rate-s->feedback_rate)*alpha;
    remaining=s->angle-s->feedback_angle;
    ahead=remaining-s->lookahead_s*(s->feedback_rate>0.0f?s->feedback_rate:0.0f);
    distance=ahead>0.25f?ahead-0.25f:0.0f;
    envelope=sqrtf(144.0f+2.0f*VEHICLE_TURN_ACCEL_DEG_S2*distance)-12.0f;
    want=envelope;
    if(want>2.0f*distance)want=2.0f*distance;
    if(want>s->speed_limit)want=s->speed_limit;
    desired=TurnBrake_Clip((want-s->speed)*5.0f,VEHICLE_TURN_ACCEL_DEG_S2);
    s->acceleration+=TurnBrake_Clip(desired-s->acceleration,VEHICLE_TURN_JERK_DEG_S3*dt);
    s->speed+=s->acceleration*dt;
    if(s->speed<0.0f){s->speed=0.0f;s->acceleration=0.0f;}
    if(s->speed>s->speed_limit){s->speed=s->speed_limit;s->acceleration=0.0f;}
    if(fabsf(remaining)<=(s->stable_ms?0.9f:0.8f) &&
       fabsf(s->feedback_rate)<=0.6f && fabsf(rate)<=1.0f &&
       fabsf(s->angle-observed)<=1.0f) s->stable_ms+=elapsed;
    else s->stable_ms=0U;
    if(s->stable_ms>=700U)return 1U;
    rpm=s->speed/(6.0f*77.0f/418.0f);
    *command=floorf(rpm*10.0f+0.5f)*0.1f;
    if(*command>(int32_t)APP_TURN_BRAKE_MAX_RPM)*command=(int32_t)APP_TURN_BRAKE_MAX_RPM;
    if(*command) s->moved=1U;
    *command=s->direction*(*command);
    return 0U;
}
#endif
