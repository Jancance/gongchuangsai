#ifndef APP_ODOMETRY_H
#define APP_ODOMETRY_H
#include "app_config.h"
#include "chassis_odometry.h"
#include "route_heading_reference.h"

#define APP_MAP_OFF 0U
#define APP_MAP_WAIT_BASELINE 1U
#define APP_MAP_TRACKING 2U
#define APP_MAP_GAP 3U
#define APP_MAP_NO_START 4U
#define APP_MAP_NO_START_REASON 16U
#define APP_MAP_NO_BASELINE 32U
#define APP_MAP_FAULT 64U
#define APP_MAP_BAD_SOURCE 128U
#define APP_MAP_RAW_WHEEL_HWT 1U
#define APP_MAP_CALIBRATED_WHEEL_HWT 2U
#define APP_MAP_CAL_REFERENCE 1U
#define APP_MAP_CAL_EXTRAPOLATED 2U
#define APP_MAP_CAL_BOUNDARY 1U
#define APP_MAP_CAL_UNSUPPORTED 2U
#define APP_MAP_CAL_TAIL 4U
typedef struct {
    ChassisOdometryStatus pose;
    uint32_t run_id,start_id,map_version,begin_ms,now_ms,state,based;
    uint32_t source,calibration_scope,nav_applied,wheel_ms[4],angle_ms;
    float raw_x_mm,raw_y_mm,correction_x_mm,correction_y_mm;
    uint32_t action_epoch,calibration_flags,calibration_direction;
} AppMapPose;
/* TaskA submits only a genuinely new round; CAN owns the map context. */
void AppOdometry_RequestMapRun(uint32_t run_id,uint32_t start_id);
void AppOdometry_ReadMap(AppMapPose *snapshot);
uint32_t AppOdometry_MapTrackingRun(void);
/* Call only after successful motion acceptance; unsupported actions close the old scope. */
void AppOdometry_MapActionAccepted(uint32_t direction,uint32_t rpm,float distance_mm,float route_speed);

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
