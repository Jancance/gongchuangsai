#ifndef MOTION_TRIAL40_H
#define MOTION_TRIAL40_H
#include "app_config.h"
static __inline uint32_t MotionTrial40Allowed(uint32_t direction,uint32_t rpm,float mm)
{
    return APP_BASELINE40_ACTIVE &&
        (mm==APP_BASELINE40_MM || mm==1500.0f || mm==300.0f) &&
        (((direction==0U || direction==3U) && rpm==APP_BASELINE40_RPM) ||
         ((direction==1U || direction==2U) && rpm==APP_BASELINE40_LEFT_RPM));
}
static __inline float MotionDistanceAcceleration(uint32_t direction,uint32_t rpm,float mm)
{
#if APP_LEFT40_DRIFT_TRIM_ENABLE
    if (direction==1U && rpm==APP_BASELINE40_LEFT_RPM &&
        MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_ACCEL_MM_S2/(1.0f+APP_LEFT40_DRIFT_RATIO);
#endif
    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_ACCEL_MM_S2 : APP_DISTANCE_SCURVE_ACCEL_MM_S2;
}
static __inline float MotionDistanceJerk(uint32_t direction,uint32_t rpm,float mm)
{
#if APP_LEFT40_DRIFT_TRIM_ENABLE
    if (direction==1U && rpm==APP_BASELINE40_LEFT_RPM &&
        MotionTrial40Allowed(direction,rpm,mm)) return APP_BASELINE40_JERK_MM_S3/(1.0f+APP_LEFT40_DRIFT_RATIO);
#endif
    return MotionTrial40Allowed(direction,rpm,mm) ? APP_BASELINE40_JERK_MM_S3 : APP_DISTANCE_SCURVE_JERK_MM_S3;
}
#endif
