#ifndef STAGE1_BLEND_H
#define STAGE1_BLEND_H
#include "motion_profile.h"
#include "app_config.h"
#include <math.h>
#include <string.h>
/* Project x/y are field coordinates; here x is positive travel toward scan,
 * y is positive forward. All positions below are encoder estimates, not SLAM. */
typedef struct {
    MotionProfile x,y;
    float x_factor,y_factor,duration;
    float corr[2],rate[2],last_x,last_y;
    uint32_t tick,initialized;
} Stage1Blend;
static __inline float Blend_Clip(float v,float cap){return v>cap?cap:(v< -cap?-cap:v);}
static __inline uint32_t Stage1Blend_Build(uint32_t mode,float distance,float speed,Stage1Blend *p)
{
    MotionProfileLimits lx,ly;
    if(!p || (mode!=1U && mode!=2U) || !(distance>=100 && distance<=2100) ||
       !(speed>0 && speed<=VEHICLE_STAGE1_LATERAL_CM_S*10.0f) ||
       !(VEHICLE_STAGE1_TRACK_MIN_MM>=5 && VEHICLE_STAGE1_TRACK_MAX_MM>=VEHICLE_STAGE1_TRACK_MIN_MM &&
         VEHICLE_STAGE1_TRACK_MAX_MM<=100 && VEHICLE_STAGE1_TRACK_LAG_MS>=0 && VEHICLE_STAGE1_TRACK_LAG_MS<=200 &&
         VEHICLE_STAGE1_TRACK_MARGIN_MM>=0 && VEHICLE_STAGE1_TRACK_MARGIN_MM<=50 &&
         VEHICLE_STAGE1_FINAL_WAIT_MS>=500 && VEHICLE_STAGE1_FINAL_WAIT_MS<=5000))return 0U;
    memset(p,0,sizeof(*p));
    if(!Chassis_CalibrateDistanceMm(mode,3.14159265358979323846f*VEHICLE_WHEEL_DIAMETER_MM/60,&p->x_factor) ||
       !Chassis_CalibrateDistanceMm(0,3.14159265358979323846f*VEHICLE_WHEEL_DIAMETER_MM/60,&p->y_factor))return 0U;
    lx=(MotionProfileLimits){speed,VEHICLE_ROUTE_ACCEL_MM_S2*.5f-p->x_factor*4,
        VEHICLE_ROUTE_JERK_MM_S3*.5f-p->x_factor*12};
    ly=(MotionProfileLimits){VEHICLE_STAGE1_FORWARD_CM_S*10.0f,
        VEHICLE_ROUTE_ACCEL_MM_S2*.5f-p->y_factor*4,VEHICLE_ROUTE_JERK_MM_S3*.5f-p->y_factor*12};
    if(!MotionProfile_Prepare(&p->x,distance,&lx) ||
       !MotionProfile_Prepare(&p->y,VEHICLE_STAGE1_FORWARD_DISTANCE_CM*10.0f,&ly))return 0U;
    p->duration=fmaxf(p->x.duration_s,p->y.duration_s);return 1U;
}
static __inline float Stage1Blend_TrackLimit(float reference_speed)
{
    float limit=fmaxf(VEHICLE_STAGE1_TRACK_MIN_MM,
        VEHICLE_STAGE1_TRACK_MARGIN_MM+fabsf(reference_speed)*VEHICLE_STAGE1_TRACK_LAG_MS*.001f);
    return fminf(limit,VEHICLE_STAGE1_TRACK_MAX_MM);
}
/* 0 running, 1 reached, 2 tracking/input fault, 3 end deadline. */
static __inline uint32_t Stage1Blend_Step(Stage1Blend *p,float now,float observed,
    float x,float y,uint32_t tick,float *xrpm,float *yrpm)
{
    MotionProfileSample ref[2],old[2];float pos[2]={x,y},factor[2]={p->x_factor,p->y_factor};
    const MotionProfile *plans[2]={&p->x,&p->y};uint32_t done=1U;
    float dt=p->initialized?(float)(uint32_t)(tick-p->tick)*.001f:0;
    if(!(now>=0 && observed>=0 && observed<=now+.001f && dt>=0 && dt<=.4f &&
         x>=-5 && x<=p->x.distance_mm+50 && y>=-5 && y<=p->y.distance_mm+50))return 2U;
    for(uint32_t i=0;i<2;i++){
        float error,desired,value;
        if(!MotionProfile_Sample(plans[i],now,&ref[i]) || !MotionProfile_Sample(plans[i],observed,&old[i]))return 2U;
        error=old[i].position_mm-pos[i];
        if(fabsf(error)>Stage1Blend_TrackLimit(old[i].speed_mm_s))return 2U;
        desired=Blend_Clip(error/factor[i],2);
        if(dt>0){
            float step=fminf(dt,.1f),z=p->corr[i]-desired,b=p->rate[i]+z,decay=expf(-step);
            p->corr[i]=desired+(z+b*step)*decay;p->rate[i]=(p->rate[i]-b*step)*decay;
        }
        value=ref[i].speed_mm_s/factor[i]+p->corr[i];
        if(now<plans[i]->duration_s){if(value<0)value=0;done=0;}
        else if(fabsf(plans[i]->distance_mm-pos[i])<=2){value=0;p->corr[i]=p->rate[i]=0;}
        else{value=Blend_Clip((plans[i]->distance_mm-pos[i])/factor[i],2);done=0;}
        if(i==0)*xrpm=value;else *yrpm=value;
    }
    p->tick=tick;p->initialized=1U;p->last_x=x;p->last_y=y;
    if(done)return 1U;
    return now>=p->duration+VEHICLE_STAGE1_FINAL_WAIT_MS*.001f?3U:0U;
}
/* Aligned wheel samples have already been projected onto lateral travel. */
static __inline void Stage1Blend_Project(uint32_t mode,const float mm[4],float *x,float *y,float *spread)
{
    const int8_t signs[2][4]={{1,-1,-1,1},{-1,1,1,-1}};
    float lo=0,hi=0;*x=*y=0;
    for(uint32_t i=0;i<4;i++){*x+=mm[i]*.25f;*y+=mm[i]*signs[mode-1][i]*.25f;}
    for(uint32_t i=0;i<4;i++){
        float residual=mm[i]-*x-signs[mode-1][i]*(*y);
        if(!i || residual<lo)lo=residual;if(!i || residual>hi)hi=residual;
    }
    *spread=hi-lo;
}
static __inline uint32_t Stage1Blend_Commands(uint32_t mode,float x,float y,float heading,ChassisMotorCommand out[4])
{
    ChassisWheelSpeeds v;float side=mode==1?x:-x;
    if((mode!=1 && mode!=2) || !(fabsf(x)<=VEHICLE_STAGE1_BLEND_MAX_RPM && fabsf(y)<=60 && fabsf(heading)<=2))return 0U;
    v.rpm[0]=y+side-heading;v.rpm[1]=y-side+heading;
    v.rpm[2]=y-side-heading;v.rpm[3]=y+side+heading;
    return Chassis_EncodeEmm(&v,VEHICLE_STAGE1_BLEND_MAX_RPM,out);
}
#endif
