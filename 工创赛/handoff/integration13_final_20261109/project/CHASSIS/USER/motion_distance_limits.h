#ifndef MOTION_DISTANCE_LIMITS_H
#define MOTION_DISTANCE_LIMITS_H
#include <stdint.h>
#include "app_config.h"
#include "motion_trial20.h"
/* 普通10..100cm/3..15RPM及低速210cm保留；20cm/s精确例外见motion_trial20.h。 */
static __inline uint32_t MotionDistanceAllowed(uint32_t direction, uint32_t rpm, float mm)
{
    if (MotionTrial20Allowed(direction,rpm,mm)) return 1U;
    if (direction>2U || rpm<3U || rpm>15U) return 0U;
    if (mm>=100.0f && mm<=1000.0f) return 1U;
#if APP_IS_DEBUG && (APP_TEST_MODE == 3U)
    return (direction==0U && rpm==15U && mm==2100.0f) ? 1U : 0U;
#else
    return 0U;
#endif
}
static __inline uint32_t MotionDistanceRunMs(float mm)
{
    if (mm>=100.0f && mm<=1000.0f) return 30000U;
#if APP_IS_DEBUG && (APP_TEST_MODE == 3U)
    if (mm==2100.0f) return 60000U;
#endif
    return 0U;
}
static __inline uint32_t MotionDistanceRunMsFor(uint32_t direction,uint32_t rpm,float mm)
{
    if (MotionTrial20Allowed(direction,rpm,mm)) return APP_SPEED20_RUN_MS;
    return MotionDistanceRunMs(mm);
}
#endif
