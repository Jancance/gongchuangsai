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

#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 s_baseline40;
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot=s_baseline40;
    taskEXIT_CRITICAL();
}
#endif

#if APP_TEST_MODE == 3U
volatile PerimeterRecord g_perimeter_records[9];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
void AppOdometry_ResetRecords(void)
{
#if APP_BASELINE40_ACTIVE
    s_baseline40.anchor_valid=0U;
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
#endif
    ChassisOdometry_Init(&s_odometry);
}

void AppOdometry_Step(const AppOdometryInputs *inputs)
{
    AppHeadingSource source;
    ChassisOdometrySamples samples;
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
#endif
        if(!(*inputs->vm_request) && !power.active && !g_chassis_auto_active &&
           !g_wheel_test_active && !(*inputs->jog_preparing) && source.healthy &&
           App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS &&
           !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)
            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
                g_odometry_pose_result=1U;
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
    taskENTER_CRITICAL();
    g_chassis_odometry=s_odometry.status;
#if APP_BASELINE40_ACTIVE
    s_baseline40.pose=s_odometry.status;
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
