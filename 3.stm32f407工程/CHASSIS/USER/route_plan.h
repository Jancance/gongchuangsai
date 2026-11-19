#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H
#include "motion_action_plan.h"
#include "chassis_kinematics.h"
#include "app_config.h"
#include "route_config.h"
#include "motion_route_trial.h"
#if APP_ROUTE_USE_CUSTOM != 0 && APP_ROUTE_USE_CUSTOM != 1
#error APP_ROUTE_USE_CUSTOM_must_be_0_or_1
#endif
#define ROUTE_OK 0U
#define ROUTE_COUNT 1U
#define ROUTE_KIND 2U
#define ROUTE_AMOUNT 3U
#define ROUTE_SPEED 4U
#define ROUTE_DURATION 5U
#define ROUTE_BUDGET 6U
#define ROUTE_GEOMETRY 7U
typedef struct { uint32_t kind; float amount, speed; } RouteRow;
typedef struct { uint32_t error, segment, budget_ms; } RouteCheck;
#define ROUTE_FORWARD MOTION_FORWARD_DISTANCE
#define ROUTE_LEFT MOTION_LEFT_DISTANCE
#define ROUTE_RIGHT MOTION_RIGHT_DISTANCE
#define ROUTE_BACKWARD MOTION_BACKWARD_DISTANCE
#define ROUTE_TURN MOTION_TURN
#define ROUTE_ROW(kind, amount, speed) {ROUTE_##kind, amount, speed},
/* 纯参数转换，不发CAN、不申请VM。全部通过后才发布动作表。
 * segment从1开始；0表示整表错误。时长检查只是必要条件，不保证能到位。
 */
static __inline uint32_t RoutePlan_Convert(const RouteRow *rows, uint32_t count,
    const ChassisGeometry *geometry, MotionAction *actions, RouteCheck *check)
{
    MotionAction staged[MOTION_ACTION_CAPACITY];
    uint32_t i, rpm, budget=5000U;
    float factor, maximum, minimum, duration, unit;
    #if APP_ROUTE_SPEED_TRIAL_ACTIVE
    MotionProfile profile;uint32_t route_ms;
    #endif
    check->error=ROUTE_OK; check->segment=0U; check->budget_ms=budget;
    if (!rows || !actions || !count || count>MOTION_ACTION_CAPACITY)
    { check->error=ROUTE_COUNT; return 0U; }
    if (!geometry || !(geometry->wheel_diameter_mm>0.0f && geometry->wheel_diameter_mm<1000.0f) ||
        !(geometry->track_width_mm>0.0f && geometry->track_width_mm<1000.0f) ||
        !(geometry->wheelbase_mm>0.0f && geometry->wheelbase_mm<1000.0f))
    { check->error=ROUTE_GEOMETRY; return 0U; }
    for (i=0U;i<count;i++) {
        const RouteRow *row=&rows[i];
        MotionAction *action=&staged[i];
        check->segment=i+1U;
        action->kind=row->kind; action->run_ms=0U;
        action->relative_deg=0.0f; action->distance_mm=0.0f; action->reference_mm_s=0.0f;
        if (row->kind==MOTION_TURN) {
            if (!((row->amount>=5.0f && row->amount<=90.0f) ||
                  (row->amount<=-5.0f && row->amount>=-90.0f)))
            { check->error=ROUTE_AMOUNT; return 0U; }
            action->relative_deg=row->amount;
            factor=6.0f*geometry->wheel_diameter_mm/(geometry->track_width_mm+geometry->wheelbase_mm);
            minimum=1.0f; maximum=5.0f;
            if(APP_TURN_BRAKE_TRIAL_ACTIVE && (count>=1U && count<=MOTION_ACTION_CAPACITY) &&
               (row->amount==90.0f || row->amount==-90.0f)) maximum=(float)APP_TURN_BRAKE_TRIAL_DPS/factor;
        } else if (row->kind>=MOTION_FORWARD_DISTANCE && row->kind<=MOTION_BACKWARD_DISTANCE) {
            #if APP_ROUTE_SPEED_TRIAL_ACTIVE
            {
                uint32_t direction=row->kind-MOTION_FORWARD_DISTANCE;
                if(!(row->amount>=VEHICLE_MIN_ROUTE_CM && row->amount<=VEHICLE_MAX_ROUTE_CM))
                { check->error=ROUTE_AMOUNT;return 0U; }
                if(!(row->speed>=1.21f && row->speed*10.0f<=MotionRoute_MaxSpeed(direction)))
                { check->error=ROUTE_SPEED;return 0U; }
                if(!MotionRoute_Build(direction,row->amount*10.0f,row->speed*10.0f,&profile,&rpm,&route_ms))
                { check->error=ROUTE_DURATION;return 0U; }
                action->distance_mm=row->amount*10.0f;action->reference_mm_s=row->speed*10.0f;
                if(row->kind==MOTION_TURN && rpm>5U &&
            (!APP_TURN_BRAKE_TRIAL_ACTIVE || rpm!=APP_TURN_BRAKE_MAX_RPM))
        { check->error=ROUTE_SPEED;return 0U; }
        action->base_rpm=rpm;action->run_ms=route_ms;
                budget+=route_ms+6000U; /* 准备、停车、段间余量，不缩减实际阶段截止。 */
                check->budget_ms=budget;
                if(budget>APP_PERIMETER_GROUP_MS) { check->error=ROUTE_BUDGET;return 0U; }
                continue;
            }
            #else
            if (!(row->amount>=10.0f && row->amount<=100.0f))
            { check->error=ROUTE_AMOUNT; return 0U; }
            action->distance_mm=row->amount*10.0f;
            factor=3.14159265358979323846f*geometry->wheel_diameter_mm/600.0f;
            minimum=3.0f; maximum=15.0f;
            #endif
        } else { check->error=ROUTE_KIND; return 0U; }
        /* 在转换无符号整数前拒绝负数、NaN、无穷和超限值。 */
        if (!(row->speed>=minimum*factor && row->speed<=maximum*factor))
        { check->error=ROUTE_SPEED; return 0U; }
        rpm=(uint32_t)(row->speed/factor);
        if (rpm<(uint32_t)minimum || rpm>(uint32_t)maximum)
        { check->error=ROUTE_SPEED; return 0U; }
        if(row->kind==MOTION_TURN && rpm>5U &&
            (!APP_TURN_BRAKE_TRIAL_ACTIVE || rpm!=APP_TURN_BRAKE_MAX_RPM))
        { check->error=ROUTE_SPEED;return 0U; }
        action->base_rpm=rpm;
        unit=(row->amount<0.0f) ? -row->amount : row->amount;
        duration=(row->kind==MOTION_TURN) ? (float)APP_HEADING_RUN_MS : 30000.0f;
        if (unit/(factor*(float)rpm)*1000.0f>duration)
        { check->error=ROUTE_DURATION; return 0U; }
        budget+=(uint32_t)duration+3500U;
        check->budget_ms=budget;
        if (budget>APP_PERIMETER_GROUP_MS) { check->error=ROUTE_BUDGET; return 0U; }
    }
    for(i=0U;i<count;i++) actions[i]=staged[i];
    check->segment=0U; return 1U;
}
#endif
