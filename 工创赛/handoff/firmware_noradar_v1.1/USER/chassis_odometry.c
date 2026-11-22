#include "chassis_odometry.h"
#include "chassis_kinematics.h"
#include <math.h>
#include <string.h>
#include <float.h>

#define ODOM_PI 3.14159265358979323846f
static uint32_t finite_value(float x) { return x>=-FLT_MAX && x<=FLT_MAX; }
static float absolute(float x) { return x<0.0f ? -x : x; }
static float wrap(float x)
{
    /* All callers bound the input to a small number of revolutions. */
    while (x>180.0f) x-=360.0f;
    while (x<=-180.0f) x+=360.0f;
    return x;
}
static ChassisOdomPoint latest(const ChassisOdomHistory *h)
{ return h->points[(h->head+ODOM_HISTORY-1U)%ODOM_HISTORY]; }
static void remember(ChassisOdomHistory *h, uint32_t ms, float value)
{
    h->points[h->head].ms=ms; h->points[h->head].value=value;
    h->head=(h->head+1U)%ODOM_HISTORY;
    if(h->count<ODOM_HISTORY) h->count++;
}
static uint32_t at(const ChassisOdomHistory *h,uint32_t ms,float *value)
{
    uint32_t j, have=0U;
    ChassisOdomPoint upper={0U,0.0f};
    for(j=0U;j<h->count;j++)
    {
        ChassisOdomPoint p=h->points[(h->head+ODOM_HISTORY-1U-j)%ODOM_HISTORY];
        int32_t offset=(int32_t)(p.ms-ms);
        if(!offset) { *value=p.value; return 1U; }
        if(offset>0) { upper=p; have=1U; }
        else {
            uint32_t span=upper.ms-p.ms;
            if(!have || !span || span>=ODOM_WHEEL_MAX_AGE_MS) return 0U;
            *value=p.value+(upper.value-p.value)*(float)(ms-p.ms)/(float)span;
            return 1U;
        }
    }
    return 0U;
}
void ChassisOdometry_Init(ChassisOdometry *c)
{
    if(!c) return;
    memset(c,0,sizeof(*c)); c->status.reasons=ODOM_WAITING;
    c->status.age_ms=0xFFFFFFFFU; c->status.local_origin=1U;
}
uint32_t ChassisOdometry_SetPose(ChassisOdometry *c,float x,float y,float heading,float yaw)
{
    if(!c || !finite_value(x) || !finite_value(y) || absolute(x)>1000000.0f ||
       absolute(y)>1000000.0f || !finite_value(heading) || absolute(heading)>180.0f ||
       !finite_value(yaw) || absolute(yaw)>180.0f) return 0U;
    ChassisOdometry_Init(c); c->configured=1U;
    c->status.x_mm=x; c->status.y_mm=y; c->status.heading_deg=heading;
    c->heading_offset=wrap(heading-yaw); c->status.local_origin=0U;
    return 1U;
}
void ChassisOdometry_Invalidate(ChassisOdometry *c)
{
    if(!c) return;
    c->status.valid=0U;
    if(c->based) { c->status.continuity_lost=1U; c->status.reasons|=ODOM_GAP; }
}
static void reject(ChassisOdometry *c)
{
    ChassisOdometry_Invalidate(c); c->rejected=1U;
    c->status.reasons|=ODOM_BAD_SAMPLE;
}
void ChassisOdometry_Wheel(ChassisOdometry *c,uint32_t i,uint32_t sign,uint32_t raw,uint32_t ms)
{
    int64_t current;
    float delta;
    if(!c) return;
    if(i>=4U || sign>1U) { reject(c); return; }
    current=(int64_t)raw; if(sign) current=-current;
    if(g_chassis_forward_direction[i]) current=-current;
    if(c->raw_mask & (1UL<<i))
    {
        ChassisOdomPoint old=latest(&c->wheel[i]);
        uint32_t dt=ms-old.ms;
        if(!dt) { if(current!=c->raw[i]) reject(c); return; }
        if((int32_t)dt<0) { reject(c); return; }
        delta=(float)(current-c->raw[i])*ODOM_PI*g_chassis_geometry.wheel_diameter_mm/65536.0f;
        /* 1000mm/s wheel travel + 2mm quantisation/noise allowance; not a motor limit. */
        if(dt>=ODOM_WHEEL_MAX_AGE_MS || absolute(delta)>1.0f*(float)dt+2.0f)
        {
            reject(c); c->wheel[i].count=c->wheel[i].head=0U;
            /* Rebase this encoder without integrating the missing interval. */
        }
        else c->wheel_mm[i]+=delta;
    }
    c->raw[i]=current; c->raw_mask|=1UL<<i;
    remember(&c->wheel[i],ms,c->wheel_mm[i]);
}
void ChassisOdometry_Yaw(ChassisOdometry *c,float yaw,uint32_t ms)
{
    float unwrapped=yaw;
    if(!c) return;
    if(!finite_value(yaw) || absolute(yaw)>180.0f) { reject(c); return; }
    if(c->yaw.count)
    {
        ChassisOdomPoint old=latest(&c->yaw);
        uint32_t dt=ms-old.ms;
        float delta=wrap(yaw-wrap(old.value));
        if(!dt) { if(absolute(delta)>0.001f) reject(c); return; }
        if((int32_t)dt<0) { reject(c); return; }
        if(dt>=ODOM_WHEEL_MAX_AGE_MS || absolute(delta)>0.12f*(float)dt+1.0f)
        { reject(c); c->yaw.count=c->yaw.head=0U; }
        else unwrapped=old.value+delta;
    }
    remember(&c->yaw,ms,unwrapped);
}
void ChassisOdometry_Update(ChassisOdometry *c,uint32_t now)
{
    uint32_t i,common=0U,oldest=0U,dt;
    float wheel[4],yaw,dx,dy,angle,half,sinc;
    if(!c) return;
    c->status.valid=0U;
    c->status.age_ms=c->based ? now-c->status.sample_ms : 0xFFFFFFFFU;
    c->status.reasons=c->status.continuity_lost ? ODOM_GAP : ODOM_WAITING;
    if(c->rejected) c->status.reasons|=ODOM_BAD_SAMPLE;
    if(c->raw_mask!=15U || !c->yaw.count) return;
    for(i=0U;i<5U;i++)
    {
        ChassisOdomPoint p=latest(i<4U ? &c->wheel[i] : &c->yaw);
        uint32_t age=now-p.ms;
        if(age>=(i<4U ? ODOM_WHEEL_MAX_AGE_MS : ODOM_YAW_MAX_AGE_MS))
        { ChassisOdometry_Invalidate(c); c->status.reasons|=ODOM_STALE; return; }
        if(i==0U || age>oldest) { oldest=age; common=p.ms; }
    }
    if(c->based && (int32_t)(common-c->last_ms)<=0)
    {
        if(now-c->last_ms>=ODOM_WHEEL_MAX_AGE_MS) ChassisOdometry_Invalidate(c);
        else if(!c->status.continuity_lost && !c->rejected)
        { c->status.valid=1U; c->status.reasons=0U; }
        return;
    }
    for(i=0U;i<4U;i++) if(!at(&c->wheel[i],common,&wheel[i])) return;
    if(!at(&c->yaw,common,&yaw)) return;
    if(!c->based)
    {
        if(!c->configured) c->heading_offset=wrap(-wrap(yaw));
        /* Before the first baseline no displacement has been claimed. */
        c->rejected=0U;
        c->based=1U;
    }
    else
    {
        dt=common-c->last_ms;
        if(dt>=ODOM_WHEEL_MAX_AGE_MS || c->rejected || c->status.continuity_lost)
        {
            /* Freeze pose after lost continuity until explicit SetPose. */
            ChassisOdometry_Invalidate(c); return;
        }
        for(i=0U;i<4U;i++) wheel[i]-=c->previous_mm[i];
        dx=(wheel[0]+wheel[1]+wheel[2]+wheel[3])*0.25f;
        dy=(wheel[0]-wheel[1]-wheel[2]+wheel[3])*0.25f;
        /* Explicit first-stage odometry calibration; no command trim subtraction. */
        dx*=CHASSIS_FORWARD_DISTANCE_SCALE;
        half=(yaw-c->previous_yaw)*ODOM_PI/360.0f;
        sinc=absolute(half)<0.0001f ? 1.0f : sinf(half)/half;
        angle=(wrap(c->previous_yaw)+c->heading_offset)*ODOM_PI/180.0f+half;
        c->status.x_mm+=sinc*(dx*cosf(angle)-dy*sinf(angle));
        c->status.y_mm+=sinc*(dx*sinf(angle)+dy*cosf(angle));
        for(i=0U;i<4U;i++) wheel[i]+=c->previous_mm[i];
    }
    for(i=0U;i<4U;i++) c->previous_mm[i]=wheel[i];
    c->previous_yaw=yaw; c->last_ms=common;
    c->status.heading_deg=wrap(wrap(yaw)+c->heading_offset);
    c->status.sample_ms=common; c->status.age_ms=now-common; c->status.updates++;
    c->status.valid=(!c->status.continuity_lost && !c->rejected);
    c->status.reasons=c->status.valid ? 0U : (ODOM_GAP|ODOM_BAD_SAMPLE);
}
