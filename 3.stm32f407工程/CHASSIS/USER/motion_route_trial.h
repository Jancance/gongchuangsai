#ifndef MOTION_ROUTE_TRIAL_H
#define MOTION_ROUTE_TRIAL_H
#include "app_config.h"
#include "motion_profile.h"
/* 调试路线候选；几何/比例沿用当前值，并非新的地速标定。
 * 52RPM基速+原2RPM航向纠偏；80RPM反馈阈值仅此候选使用。
 */
#define ROUTE_TRIAL_MAX_RPM 52U
#define ROUTE_TRIAL_FEEDBACK_RPM 80U
#define ROUTE_TRIAL_FOLLOW_RPM 2.0f
#define ROUTE_TRIAL_FOLLOW_KP 1.0f
#define ROUTE_TRIAL_MARGIN_MS 2000U
static __inline float MotionRoute_MaxSpeed(uint32_t direction)
{
    float cm_s;
    if(direction>3U) return 0.0f;
    cm_s=(direction==0U || direction==3U) ? APP_ROUTE_FORWARD_MAX_CM_S :
        (direction==1U ? APP_ROUTE_LEFT_MAX_CM_S : APP_ROUTE_RIGHT_MAX_CM_S);
    if(!(cm_s>0.0f && cm_s<=20.0f)) return 0.0f;
    return cm_s*10.0f;
}
static __inline uint32_t MotionRoute_Limits(uint32_t direction,float mm,float speed,
    MotionProfileLimits *limits,uint32_t *rpm)
{
    float factor,value,whole;
    uint32_t base;
    if(!APP_ROUTE_SPEED_TRIAL_ACTIVE || !limits || !rpm || direction>3U ||
        !(mm>=VEHICLE_MIN_ROUTE_CM*10.0f && mm<=VEHICLE_MAX_ROUTE_CM*10.0f) ||
        !(speed>=12.1f && speed<=MotionRoute_MaxSpeed(direction)) ||
        !Chassis_CalibrateDistanceMm(direction,
            3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor) ||
        !(factor>0.0f && factor<100.0f)) return 0U;
    value=speed/factor;
    if(!(value>0.0f && value<=50.0f)) return 0U;
    base=(uint32_t)value;whole=(float)base;
    if(whole<value) base++;
    base+=2U; /* 两RPM用于位置跟随，参考速度本身不向下量化。 */
    if(base<3U || base>ROUTE_TRIAL_MAX_RPM) return 0U;
    limits->speed=speed;
    limits->acceleration=APP_DISTANCE_SCURVE_ACCEL_MM_S2-
        factor*2.0f*ROUTE_TRIAL_FOLLOW_RPM*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
    limits->jerk=APP_DISTANCE_SCURVE_JERK_MM_S3-
        factor*6.0f*ROUTE_TRIAL_FOLLOW_RPM*APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
    if (direction==1U && APP_LEFT_TRIM_ACTIVE)
    {
        /* 连续分量的A/J预算预留；整数取整台阶仍须实物检验。 */
        limits->acceleration/=(1.0f+APP_LEFT_BACKWARD_RATIO);
        limits->jerk/=(1.0f+APP_LEFT_BACKWARD_RATIO);
    }
    *rpm=base;return 1U;
}
static __inline uint32_t MotionRoute_Deadline(float duration_s)
{
    float ms=duration_s*1000.0f;
    uint32_t rounded;
    if(!(ms>0.0f && ms<56000.0f)) return 0U;
    rounded=(uint32_t)ms;if((float)rounded<ms) rounded++;
    return rounded+APP_DISTANCE_SCURVE_FINAL_WAIT_MS+ROUTE_TRIAL_MARGIN_MS;
}
/* 只在锁外准备局部曲线；失败不能发布一部分路线或授权运动。 */
static __inline uint32_t MotionRoute_Build(uint32_t direction,float mm,float speed,
    MotionProfile *profile,uint32_t *rpm,uint32_t *run_ms)
{
    MotionProfileLimits limits;
    if(profile) profile->valid=0U;
    if(run_ms) *run_ms=0U;
    if(rpm) *rpm=0U;
    if(!profile || !run_ms || !MotionRoute_Limits(direction,mm,speed,&limits,rpm) ||
        !MotionProfile_Prepare(profile,mm,&limits)) return 0U;
    *run_ms=MotionRoute_Deadline(profile->duration_s);
    return *run_ms!=0U;
}
static __inline uint32_t MotionRoute_ShapeAllowed(uint32_t direction,uint32_t rpm,
    uint32_t run_ms,float mm,float speed)
{
    MotionProfileLimits limits;uint32_t expected;
    return MotionRoute_Limits(direction,mm,speed,&limits,&expected) && rpm==expected &&
        run_ms>APP_DISTANCE_SCURVE_FINAL_WAIT_MS+ROUTE_TRIAL_MARGIN_MS && run_ms<=60000U;
}
#endif
