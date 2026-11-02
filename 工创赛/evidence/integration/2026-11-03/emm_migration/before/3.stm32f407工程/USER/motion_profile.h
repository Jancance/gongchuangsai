#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include "chassis_kinematics.h"

/* 纯计算候选模块：不发CAN、不访问RTOS、不修改VM或运动许可。
 * 第一版仅支持静止到静止的单段标量行程；方向由上层单独给定。
 * 不支持运动中重规划、连续途经点、转弯或任意初末速度/加速度。 */
#define MOTION_PROFILE_PHASES 7U
#define MOTION_PROFILE_INVALID 0U
#define MOTION_PROFILE_RUNNING 1U
#define MOTION_PROFILE_DONE 2U

typedef struct
{
    float speed;        /* mm/s */
    float acceleration; /* mm/s^2，对称加速/减速上限 */
    float jerk;         /* mm/s^3，加速度变化率上限 */
} MotionProfileLimits;

typedef struct
{
    uint32_t valid;
    float distance_mm, duration_s, peak_speed;
    MotionProfileLimits limits;
    float boundary_s[MOTION_PROFILE_PHASES + 1U];
    float position[MOTION_PROFILE_PHASES], speed[MOTION_PROFILE_PHASES];
    float acceleration[MOTION_PROFILE_PHASES], jerk[MOTION_PROFILE_PHASES];
} MotionProfile;

typedef struct
{
    float position_mm, speed_mm_s, acceleration_mm_s2, jerk_mm_s3;
} MotionProfileSample;

/* wheel与reserve的单位分别是RPM、RPM/s、RPM/s^2。
 * reserve由上层预留给纠偏，须包含其速度、加速度和加加速度需求。
 * 数学有效不代表机械/电气允许；实车参数仍须验证。 */
typedef struct
{
    float speed_rpm, acceleration_rpm_s, jerk_rpm_s2;
} MotionProfileWheelLimits;

/* 按固定车体平移方向统一时间缩放：v乘scale、a乘scale^2、j乘scale^3。
 * 不独立削顶车轮，不调整公共轮径，不量化RPM。
 * direction_x/y为任意非零方向向量；仅适用于固定航向的直线平移。 */
uint32_t MotionProfile_LimitTranslation(const ChassisGeometry *geometry,
    float direction_x, float direction_y, const MotionProfileLimits *requested,
    const MotionProfileWheelLimits *wheel, const MotionProfileWheelLimits *reserve,
    MotionProfileLimits *limited, float *time_scale);

/* 构建有界七段S曲线；短行程自动退化，允许零长度。
 * prepare可覆盖旧plan，仅可对未执行的对象调用；上层不得替换运行中的计划。
 * 数值边界不是实车安全上限。失败清valid，有限计算，不申请内存。 */
uint32_t MotionProfile_Prepare(MotionProfile *plan, float distance_mm,
    const MotionProfileLimits *limits);
/* elapsed_s是自本段开始的实际累计时间，不靠调用次数推算。
 * 终点输出精确目标和零速度/零加速度；无效输入输出清零并返回INVALID。
 * DONE仅表示参考轨迹结束，不表示实车到位或停稳；不能据此关VM/签收运动。 */
uint32_t MotionProfile_Sample(const MotionProfile *plan, float elapsed_s,
    MotionProfileSample *sample);

#endif
