#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H
#include <stdint.h>

/* 底盘几何中心；场地X/Y固定，车身前/左/逆时针为正。仅观察，无运动许可。 */
#define ODOM_HISTORY 32U
#define ODOM_WAITING 1U
#define ODOM_STALE 2U
#define ODOM_BAD_SAMPLE 4U
#define ODOM_GAP 8U
#define ODOM_WHEEL_MAX_AGE_MS 400U
#define ODOM_YAW_MAX_AGE_MS 100U
typedef struct { uint32_t ms; float value; } ChassisOdomPoint;
typedef struct {
    ChassisOdomPoint points[ODOM_HISTORY];
    uint32_t count, head;
} ChassisOdomHistory;
typedef struct {
    float x_mm, y_mm, heading_deg;
    uint32_t sample_ms, age_ms, updates, valid, reasons, continuity_lost;
    uint32_t local_origin; /* 1:首次有效观测建立的局部坐标，非比赛场地坐标。 */
} ChassisOdometryStatus;
typedef struct {
    ChassisOdomHistory wheel[4], yaw;
    int64_t raw[4];
    float wheel_mm[4], previous_mm[4], previous_yaw, heading_offset;
    uint32_t raw_mask, based, configured, last_ms, rejected;
    ChassisOdometryStatus status;
} ChassisOdometry;
/* 单一任务拥有整个context；跨任务读取必须复制status，不能并发调用以下接口。 */
void ChassisOdometry_Init(ChassisOdometry *context);
/* 明确建立新参考；调用方须先确认停车。不会发送任何电机命令。
 * raw_yaw是同一时刻HWT读数；调用后等待新的完整基准，不拼接旧历史。 */
uint32_t ChassisOdometry_SetPose(ChassisOdometry *context, float x_mm,
    float y_mm, float heading_deg, float raw_yaw_deg);
void ChassisOdometry_Wheel(ChassisOdometry *context, uint32_t wheel,
    uint32_t sign, uint32_t raw, uint32_t sample_ms);
void ChassisOdometry_Yaw(ChassisOdometry *context, float yaw_deg, uint32_t sample_ms);
void ChassisOdometry_Invalidate(ChassisOdometry *context);
void ChassisOdometry_Update(ChassisOdometry *context, uint32_t now_ms);
#endif
