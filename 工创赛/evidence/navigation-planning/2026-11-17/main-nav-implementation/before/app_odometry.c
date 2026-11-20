#include "app_odometry.h"
#include "vehicle_tuning.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "heading_source.h"
#include "chassis_auto_test.h"
#include "power.h"
#include "safety.h"
#include "can.h"
#include <string.h>
#if APP_NAV_RESIDUAL_ACTIVE
#include "app_motion_runtime.h"
#include "app_entry.h"
#include <math.h>
#endif

#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 s_baseline40;
#if APP_NAV_RESIDUAL_ACTIVE
static AppOdometryBaseline40 s_navigation40;
static uint32_t s_navigation_left_scope;
static uint32_t AppOdometry_LeftNavigationScope(const AppOdometryInputs *inputs)
{
    const AppMotionObservation *motion=AppMotion_Live();
    const MotionAction *action=motion->actions;
    uint32_t id=g_debug_plan_id;
    if ((*inputs->runtime_route_id) || (*motion->action_count)!=1U ||
        action->run_ms || action->reference_mm_s!=0.0f || action->relative_deg!=0.0f) return 0U;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    if ((id==DEBUG_PLAN_LEFT210_40 || id==DEBUG_PLAN_LEFT150_40) &&
        action->kind==MOTION_LEFT_DISTANCE && action->base_rpm==APP_BASELINE40_LEFT_RPM &&
        AppDebugPlan40Allowed(id,1U,action->base_rpm,action->distance_mm)) return 1U;
#endif
#if APP_DIRECTION_NAV_RESIDUAL_ENABLE
    if (action->distance_mm!=2100.0f) return 0U;
    if (APP_RIGHT40_FORWARD_TRIM_ENABLE && id==DEBUG_PLAN_RIGHT210_40 &&
        action->kind==MOTION_RIGHT_DISTANCE && action->base_rpm==APP_BASELINE40_LEFT_RPM &&
        AppDebugPlan40Allowed(id,2U,action->base_rpm,action->distance_mm)) return 2U;
    if (action->base_rpm==APP_BASELINE40_RPM) {
        if (id==DEBUG_PLAN_FORWARD210_40 && action->kind==MOTION_FORWARD_DISTANCE &&
            AppDebugPlan40Allowed(id,0U,action->base_rpm,action->distance_mm)) return 3U;
        if (id==DEBUG_PLAN_BACKWARD210_40 && action->kind==MOTION_BACKWARD_DISTANCE &&
            AppDebugPlan40Allowed(id,3U,action->base_rpm,action->distance_mm)) return 4U;
    }
#endif
    return 0U;
}
#endif
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot=s_baseline40;
    taskEXIT_CRITICAL();
}
void AppOdometry_ReadNavigation40(AppOdometryBaseline40 *snapshot)
{
#if APP_NAV_RESIDUAL_ACTIVE
    taskENTER_CRITICAL();
    *snapshot=s_navigation40;
    taskEXIT_CRITICAL();
#else
    AppOdometry_ReadBaseline40(snapshot);
#endif
}
#endif

#if APP_TEST_MODE == 3U
volatile PerimeterRecord g_perimeter_records[9];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
void AppOdometry_ResetRecords(void)
{
#if APP_BASELINE40_ACTIVE
    s_baseline40.anchor_valid=0U;
#if APP_NAV_RESIDUAL_ACTIVE
    s_navigation40.anchor_valid=0U;
#endif
#endif
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
}
#endif

#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
/* CAN任务独占计算状态；TaskA只复制小型只读结果。 */
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
/* 调试Watch：先填三个目标，最后写request=1；仅VM关闭且无运动时受理。
 * result:1接受，2条件不足；CAN任务消费一次，不等待条件后自动执行。 */
#if APP_IS_DEBUG
volatile uint32_t g_odometry_pose_request=0U, g_odometry_pose_result=0U;
volatile float g_odometry_pose_x_mm=0.0f, g_odometry_pose_y_mm=0.0f;
volatile float g_odometry_pose_heading_deg=0.0f;
#endif
void AppOdometry_Init(void)
{
#if APP_BASELINE40_ACTIVE
    memset(&s_baseline40,0,sizeof(s_baseline40));
#if APP_NAV_RESIDUAL_ACTIVE
    memset(&s_navigation40,0,sizeof(s_navigation40));
    s_navigation_left_scope=0U;
#endif
#endif
    ChassisOdometry_Init(&s_odometry);
}

void AppOdometry_Step(const AppOdometryInputs *inputs)
{
    AppHeadingSource source;
    ChassisOdometrySamples samples;
#if APP_NAV_RESIDUAL_ACTIVE
    float navigation_x,navigation_y;
#endif
    uint32_t i,now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    (void)App_HeadingSourceRead(&source);
#if APP_IS_DEBUG
    if(g_odometry_pose_request==1U) {
        PowerCommandStatus power;
        float x,y,heading;
        taskENTER_CRITICAL();
        x=g_odometry_pose_x_mm; y=g_odometry_pose_y_mm;
        heading=g_odometry_pose_heading_deg; g_odometry_pose_request=0U;
        taskEXIT_CRITICAL();
        Power_ReadCommandStatus(&power);
        g_odometry_pose_result=2U;
#if APP_BASELINE40_ACTIVE
        s_baseline40.anchor_valid=0U;
#if APP_NAV_RESIDUAL_ACTIVE
        s_navigation40.anchor_valid=0U;
#endif
#endif
        if(!(*inputs->vm_request) && !power.active && !g_chassis_auto_active &&
           !g_wheel_test_active && !(*inputs->jog_preparing) && source.healthy &&
           App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS &&
           !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)
            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
            {
                g_odometry_pose_result=1U;
#if APP_NAV_RESIDUAL_ACTIVE
                s_navigation_left_scope=0U;
#endif
            }
    }
#endif
    if(!s_perimeter_odom_started && (*inputs->combined_state)==1U && source.healthy) {
        if(inputs->route_heading_reference->valid && ChassisOdometry_SetPose(&s_odometry,
            (VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:2100.0f,(VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:2100.0f,
            (VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:-90.0f,inputs->route_heading_reference->origin_deg))
        {
            s_perimeter_odom_started=1U;
#if APP_BASELINE40_ACTIVE
            s_baseline40.origin_x_mm=s_odometry.status.x_mm;
            s_baseline40.origin_y_mm=s_odometry.status.y_mm;
            s_baseline40.path_heading_deg=s_odometry.status.heading_deg;
            s_baseline40.heading_offset_deg=s_odometry.heading_offset;
            s_baseline40.reference_set_ms=now;
            s_baseline40.anchor_valid=1U;
#if APP_NAV_RESIDUAL_ACTIVE
            s_navigation_left_scope=AppOdometry_LeftNavigationScope(inputs);
#endif
#endif
        }
    }
    ChassisAuto_ReadOdometrySamples(&samples);
    if(source.healthy && App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS)
        ChassisOdometry_Yaw(&s_odometry,source.yaw_deg,source.angle_ms);
    else ChassisOdometry_Invalidate(&s_odometry);
    for(i=0U;i<4U;i++)
        if(samples.mask & (1UL<<i))
            ChassisOdometry_Wheel(&s_odometry,i,samples.sign[i],samples.raw[i],samples.ms[i]);
    if(samples.mask!=15U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)
        ChassisOdometry_Invalidate(&s_odometry);
    ChassisOdometry_Update(&s_odometry,now);
#if APP_NAV_RESIDUAL_ACTIVE
    navigation_x=s_odometry.status.x_mm;navigation_y=s_odometry.status.y_mm;
    if(s_navigation_left_scope)
    {
        float angle=s_baseline40.path_heading_deg*0.01745329251994329577f;
        float cs=cosf(angle),sn=sinf(angle);
        float left=-sn*(navigation_x-s_baseline40.origin_x_mm)+
            cs*(navigation_y-s_baseline40.origin_y_mm);
        float forward=cs*(navigation_x-s_baseline40.origin_x_mm)+
            sn*(navigation_y-s_baseline40.origin_y_mm);
        float distance=left,coefficient=APP_LEFT_NAV_RESIDUAL_COEFFICIENT;
        float correction;
        if(s_navigation_left_scope==2U) {
            distance=-left;coefficient=APP_RIGHT_NAV_RESIDUAL_COEFFICIENT;
        } else if(s_navigation_left_scope==3U) {
            distance=forward;coefficient=APP_FORWARD_NAV_RESIDUAL_COEFFICIENT;
        } else if(s_navigation_left_scope==4U) {
            distance=-forward;coefficient=APP_BACKWARD_NAV_RESIDUAL_COEFFICIENT;
        }
        correction=APP_LEFT_NAV_RESIDUAL_WEIGHT*coefficient*((distance>0.0f)?distance:0.0f);
        if(s_navigation_left_scope<=2U) {
            navigation_x+=cs*correction;navigation_y+=sn*correction;
        } else {
            navigation_x-=sn*correction;navigation_y+=cs*correction;
        }
    }
#endif
    taskENTER_CRITICAL();
#if APP_BASELINE40_ACTIVE
    s_baseline40.pose=s_odometry.status;
#endif
#if APP_NAV_RESIDUAL_ACTIVE
    s_navigation40=s_baseline40;
    s_navigation40.pose.x_mm=navigation_x;s_navigation40.pose.y_mm=navigation_y;
    g_chassis_odometry=s_navigation40.pose;
#else
    g_chassis_odometry=s_odometry.status;
#endif
    if(s_perimeter_odom_started && !g_perimeter_records[0].captured && s_odometry.status.valid &&
       (*inputs->combined_completed)==0U) {
        g_perimeter_records[0].captured=1U;g_perimeter_records[0].tick_ms=now;
        g_perimeter_records[0].pose=s_odometry.status;
    }
    if((*inputs->combined_completed)>s_perimeter_completed_seen && (*inputs->combined_completed)<=8U) {
        uint32_t slot=(*inputs->combined_completed);
        g_perimeter_records[slot].captured=1U;g_perimeter_records[slot].tick_ms=now;
        g_perimeter_records[slot].pose=s_odometry.status;
        s_perimeter_completed_seen=slot;
    }
    taskEXIT_CRITICAL();
}
#endif
