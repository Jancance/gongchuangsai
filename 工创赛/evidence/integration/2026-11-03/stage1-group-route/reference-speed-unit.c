
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_config.h"
#include "motion_route_trial.h"
#include "motion_trial20.h"
#include "zdt_emm_protocol.h"
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1U
static uint32_t s_route_trial=1,s_distance_max_rpm,s_profile_follow_tick;
static float s_profile_mm_per_rpm_s,s_profile_follow_rpm,s_profile_follow_rate;
static MotionProfileSample s_distance_reference;
static MotionProfile s_distance_profile;
static uint32_t Auto_ProfileFollowRpmLocked(float elapsed, float progress, TickType_t now, float *rpm)
{
    float c = s_route_trial ? ROUTE_TRIAL_FOLLOW_RPM : MotionDistanceFollowRpm(s_distance_max_rpm), w = APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
    float desired = (s_route_trial ? ROUTE_TRIAL_FOLLOW_KP : MotionDistanceFollowKp(s_distance_max_rpm)) *
        (s_distance_reference.position_mm-progress) / s_profile_mm_per_rpm_s;
    TickType_t ticks = (TickType_t)(now-s_profile_follow_tick);
    float value;
    if (!(desired >= -1.0e6f && desired <= 1.0e6f) ||
        !(s_profile_follow_rpm >= -c-0.001f && s_profile_follow_rpm <= c+0.001f) ||
        !(s_profile_follow_rate >= -2.0f*c*w-0.001f && s_profile_follow_rate <= 2.0f*c*w+0.001f)) return 0U;
    if (desired > c) desired = c;
    if (desired < -c) desired = -c;
    if (ticks >= pdMS_TO_TICKS(10U))
    {
        float dt = (float)ticks * portTICK_PERIOD_MS / 1000.0f;
        float y, b, decay;
        /* 调度迟到时不回放多步；较长缺口仍由原源/位置保护处理。 */
        if (dt > 0.1f) dt = 0.1f;
        y = s_profile_follow_rpm-desired;
        b = s_profile_follow_rate+w*y;
        decay = expf(-w*dt);
        value = desired+(y+b*dt)*decay;
        s_profile_follow_rate = (s_profile_follow_rate-w*b*dt)*decay;
        s_profile_follow_rpm = value;
        s_profile_follow_tick = now;
    }
    value = s_distance_reference.speed_mm_s/s_profile_mm_per_rpm_s+s_profile_follow_rpm;
    if (!(value >= -c-0.001f && value <= (float)s_distance_max_rpm+0.001f)) return 0U;
    if (value < 0.0f) value = 0.0f;
    if (value > (float)s_distance_max_rpm) value = (float)s_distance_max_rpm;
    /* 从减速段连续接近，防止取整先变0后再补一脚。仍受原duration+2s截止约束。 */
    if (elapsed >= s_distance_profile.boundary_s[4] &&
        value < (float)APP_DISTANCE_SCURVE_APPROACH_RPM)
        value = (float)APP_DISTANCE_SCURVE_APPROACH_RPM;
    *rpm = value;
    return 1U;
}
static unsigned checks,failures;
static void ck(unsigned ok){checks++;if(!ok){failures++;if(failures<12)printf("failed check %u\n",checks);}}
static void command_checks(unsigned direction,unsigned rpm){
 ChassisMotorCommand wheel[4];uint8_t frame[7];
 ck(Chassis_TranslationProfileCommands(direction,rpm,2.0f,wheel));
 for(unsigned i=0;i<4;i++){
  ck(wheel[i].speed_raw<=1040 && wheel[i].speed_raw<=ZDT_EMM_APP_MAX_SPEED_RAW);
  ck(ZdtEmm_EncodeSpeed(wheel[i].direction,wheel[i].speed_raw,ZDT_EMM_MOTION_ACCELERATION,0,frame));
  ck((((unsigned)frame[2]<<8)|frame[3])==wheel[i].speed_raw && frame[0]==0xf6 && frame[6]==0x6b);
 }
}
static void profile(unsigned dir,float distance,float speed){
 MotionProfile plan;MotionProfileSample p,last={0};unsigned cap,deadline;float factor;
 ck(MotionRoute_Build(dir,distance,speed,&plan,&cap,&deadline));
 ck(plan.valid && cap<=102 && deadline>plan.duration_s*1000 && plan.peak_speed<=speed+0.01f);
 ck(Chassis_CalibrateDistanceMm(dir,3.14159265358979323846f*77.0f/60,&factor));
 for(float t=0;t<=plan.duration_s;t+=0.001f){
  ck(MotionProfile_Sample(&plan,t,&p));
  ck(p.position_mm>=last.position_mm-0.01f && p.position_mm<=distance+0.01f && p.speed_mm_s>=-0.01f && p.speed_mm_s<=speed+0.01f);
  ck(fabsf(p.acceleration_mm_s2)<=VEHICLE_ROUTE_ACCEL_MM_S2+0.01f && fabsf(p.jerk_mm_s3)<=VEHICLE_ROUTE_JERK_MM_S3+0.01f);
  unsigned rpm=(unsigned)(p.speed_mm_s/factor+2.0f+0.5f);if(rpm>cap)rpm=cap;
  command_checks(dir,rpm);last=p;
 }
 ck(MotionProfile_Sample(&plan,plan.duration_s+0.01f,&p)==MOTION_PROFILE_DONE);
 ck(fabsf(p.position_mm-distance)<0.001f && p.speed_mm_s==0 && p.acceleration_mm_s2==0);
 printf("PROFILE,%u,%.0f,%.0f,%.4f,%.4f,%u,%u\n",dir,distance,speed,plan.duration_s,plan.peak_speed,cap,deadline);
}
static void model(unsigned dir,float distance,float speed,float tau,unsigned delay){
 unsigned deadline;float pos=0,velocity=0,command=0,history[2500]={0},max_error=0,max_rpm=0;
 unsigned fault=0,done=0;float now;
 ck(MotionRoute_Build(dir,distance,speed,&s_distance_profile,&s_distance_max_rpm,&deadline));
 ck(Chassis_CalibrateDistanceMm(dir,3.14159265358979323846f*77.0f/60,&s_profile_mm_per_rpm_s));
 s_profile_follow_tick=1000;s_profile_follow_rpm=s_profile_follow_rate=0;
 for(unsigned k=0;k<2000;k++){
  float desired=command*s_profile_mm_per_rpm_s;
  float dv=(desired-velocity)*0.01f/(tau+0.01f);
  /* 仅假设驱动响应，不是实测。名义1000RPM/s另留余量，取4000mm/s²。 */
  if(dv>40)dv=40;if(dv< -40)dv=-40;velocity+=dv;pos+=velocity*0.01f;history[k]=pos;
  now=k*0.01f;ck(MotionProfile_Sample(&s_distance_profile,now,&s_distance_reference));
  unsigned age=delay/10;unsigned index=k>age?k-age:0;float common=index*0.01f;MotionProfileSample old;
  ck(MotionProfile_Sample(&s_distance_profile,common,&old));
  float error=fabsf(history[index]-old.position_mm);if(error>max_error)max_error=error;
  if(error>APP_DISTANCE_SCURVE_TRACK_MM){fault=1;break;}
  if(pos>=distance || (now>=s_distance_profile.duration_s && pos>=distance-APP_DISTANCE_SCURVE_ARRIVAL_MM)){done=1;break;}
  if(now>=s_distance_profile.duration_s+APP_DISTANCE_SCURVE_FINAL_WAIT_MS/1000.0f){fault=2;break;}
  float equivalent=history[index]+s_distance_reference.position_mm-old.position_mm;
  ck(Auto_ProfileFollowRpmLocked(now,equivalent,1000+k*10,&command));
  command=(float)(unsigned)(command+0.5f);if(command>s_distance_max_rpm)command=(float)s_distance_max_rpm;
  if(command>max_rpm)max_rpm=command;
 }
 if(tau<=0.05f)ck(done && !fault);
 if(tau>=0.2f && speed>=400)ck(fault && !done);
 ck(max_rpm<=s_distance_max_rpm);
 printf("MODEL,%u,%.0f,%.0f,%.3f,%u,%u,%u,%.4f,%.4f,%.4f\n",dir,distance,speed,tau,delay,done,fault,pos,max_error,max_rpm);
}
int main(void){
 ck(g_chassis_geometry.wheel_diameter_mm==77.0f);
 float ratio=60.0f/(3.14159265358979323846f*77.0f);
 ck(fabsf(ratio-0.248033f)<0.00001f);
 ck(fabsf(200*ratio-49.6066f)<0.001f && fabsf(400*ratio-99.2132f)<0.001f);
 ck(Chassis_DistanceSelfTest() && Chassis_TranslationHoldSelfTest());
 profile(0,150,200);profile(1,1050,400);profile(2,1050,400);
 MotionProfile p;unsigned cap,ms;
 ck(!MotionRoute_Build(0,150,401,&p,&cap,&ms));
 ck(!MotionRoute_Build(1,1050,NAN,&p,&cap,&ms));
 ck(!MotionRoute_Build(1,1050,0,&p,&cap,&ms));
 uint8_t frame[7];ck(!ZdtEmm_EncodeSpeed(0,3001,236,0,frame));
 for(unsigned dir=0;dir<2;dir++)for(unsigned lag=0;lag<2;lag++)for(unsigned delay=0;delay<3;delay++)
  model(dir,dir?1050.0f:150.0f,dir?400.0f:200.0f,lag?0.05f:0.02f,delay*80);
 model(1,1050,400,0.2f,80);
 printf("CHECKS,%u,%u\n",checks,failures);return failures?1:0;
}
