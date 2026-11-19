#ifndef MOTION_TRIAL20_H
#define MOTION_TRIAL20_H
#include <stdint.h>
#include "app_config.h"
#include "motion_route_trial.h"
/* 只放行一个完全匹配的S曲线距离任务；开关关闭时恒拒绝。 */
static __inline uint32_t MotionTrial20Allowed(uint32_t direction, uint32_t rpm, float mm)
{
    return APP_SPEED20_TRIAL_ACTIVE && direction==0U && rpm==APP_SPEED20_TRIAL_RPM &&
        mm==APP_SPEED20_TRIAL_MM;
}
#define MOTION_PROFILE_BASE_CEILING (APP_ROUTE_SPEED_TRIAL_ACTIVE ? ROUTE_TRIAL_MAX_RPM : (APP_SPEED20_TRIAL_ACTIVE ? APP_SPEED20_TRIAL_RPM : 15U))
static __inline uint32_t MotionDistanceFeedbackLimit(uint32_t direction,uint32_t rpm,float mm)
{
    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_FEEDBACK_RPM;
    return direction==0U ? APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM : APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM;
}
static __inline uint32_t MotionDistanceJumpCountsPerMs(uint32_t direction,uint32_t rpm,float mm)
{
    /* 原低速60RPM等效仍66counts/ms；高速按反馈上限+原30RPM余量换算。 */
    if (MotionTrial20Allowed(direction,rpm,mm))
        return ((APP_SPEED20_FEEDBACK_RPM+30U)*65536U+59999U)/60000U;
    return 66U;
}
/* 20cm/s需要约49RPM持续速度，另留2RPM位置修正余量；普通候选仍1RPM。 */
static __inline float MotionDistanceFollowRpm(uint32_t rpm)
{
    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 2.0f : APP_DISTANCE_SCURVE_FOLLOW_RPM;
}
static __inline float MotionDistanceFollowKp(uint32_t rpm)
{
    return APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM ? 1.0f : APP_DISTANCE_SCURVE_FOLLOW_KP;
}
static __inline float MotionDistanceReferenceSpeed(float factor,uint32_t rpm)
{
    float speed=factor*((float)rpm-MotionDistanceFollowRpm(rpm));
    if (APP_SPEED20_TRIAL_ACTIVE && rpm==APP_SPEED20_TRIAL_RPM && speed>APP_SPEED20_TRIAL_MM_S)
        speed=APP_SPEED20_TRIAL_MM_S;
    return speed;
}
#endif
