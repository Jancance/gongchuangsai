#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H
#include "app_config.h"
#include "chassis_odometry.h"
#include "route_heading_reference.h"

#if APP_TEST_MODE == 3U
typedef struct {uint32_t captured,tick_ms;ChassisOdometryStatus pose;} PerimeterRecord;
extern volatile PerimeterRecord g_perimeter_records[9];
/* 只清路线记录及两项记录状态，不重置里程计、位姿或Watch请求。 */
void AppOdometry_ResetRecords(void);
#endif

#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
/* 只读live引用；由原CAN任务在原语句位置读取，不提前缓存共享值。 */
typedef struct {
    const volatile uint32_t *vm_request;
    const uint32_t *jog_preparing;
    const volatile uint32_t *combined_state;
    const volatile uint32_t *combined_completed;
    const RouteHeadingReference *route_heading_reference;
    const uint32_t *runtime_route_id;
} AppOdometryInputs;
#if APP_BASELINE40_ACTIVE
typedef struct {
    ChassisOdometryStatus pose;
    float origin_x_mm, origin_y_mm, path_heading_deg, heading_offset_deg;
    uint32_t anchor_valid, reference_set_ms;
} AppOdometryBaseline40;
/* CAN owner publishes one complete pose/reference snapshot; no context calls. */
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot);
void AppOdometry_ReadNavigation40(AppOdometryBaseline40 *snapshot);
#endif
void AppOdometry_Init(void);
void AppOdometry_Step(const AppOdometryInputs *inputs);
extern volatile ChassisOdometryStatus g_chassis_odometry;
#if APP_IS_DEBUG
extern volatile uint32_t g_odometry_pose_request, g_odometry_pose_result;
extern volatile float g_odometry_pose_x_mm, g_odometry_pose_y_mm, g_odometry_pose_heading_deg;
#endif
#endif
#endif
