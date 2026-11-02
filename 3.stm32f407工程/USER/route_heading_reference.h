#ifndef ROUTE_HEADING_REFERENCE_H
#define ROUTE_HEADING_REFERENCE_H
#include <stdint.h>
#include "heading_controller.h"
/* 独立外围候选：只建立一次参考，不在换段/失败后重新定向。单位：度。 */
#define ROUTE_HEADING_RESIDUAL_MAX_DEG 3.0f
typedef struct { uint32_t valid; float origin_deg; } RouteHeadingReference;
static __inline uint32_t RouteHeading_Init(RouteHeadingReference *r, float yaw)
{
    if (!r || r->valid || !(yaw >= -180.0f && yaw <= 180.0f)) return 0U;
    r->origin_deg = HeadingController_WrapDeg(yaw); r->valid = 1U;
    return 1U;
}
static __inline uint32_t RouteHeading_Target(const RouteHeadingReference *r,
    float planned_offset, float *target)
{
    if (!r || !r->valid || !target ||
        !(r->origin_deg >= -180.0f && r->origin_deg <= 180.0f) ||
        !(planned_offset >= -360.0f && planned_offset <= 360.0f)) return 0U;
    *target = HeadingController_WrapDeg(r->origin_deg + planned_offset);
    return 1U;
}
/* 只接受名义±90度；实际87..93度，且不得反向追角。 */
static __inline uint32_t RouteHeading_TurnDelta(float nominal, float target,
    float yaw, float *delta)
{
    float residual, actual;
    if (!delta || !(nominal == 90.0f || nominal == -90.0f) ||
        !(target >= -180.0f && target <= 180.0f) ||
        !(yaw >= -180.0f && yaw <= 180.0f)) return 0U;
    residual = HeadingController_ErrorDeg(target,
        HeadingController_WrapDeg(yaw + nominal));
    if (!(residual >= -ROUTE_HEADING_RESIDUAL_MAX_DEG &&
          residual <= ROUTE_HEADING_RESIDUAL_MAX_DEG)) return 0U;
    actual = nominal + residual;
    if (!((actual >= 87.0f && actual <= 93.0f) ||
          (actual <= -87.0f && actual >= -93.0f))) return 0U;
    *delta = actual; return 1U;
}
#endif
