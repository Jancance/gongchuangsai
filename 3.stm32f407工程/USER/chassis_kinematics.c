#include "chassis_kinematics.h"
#include <float.h>
#include <stddef.h>

#define CHASSIS_PI 3.14159265358979323846f

const ChassisGeometry g_chassis_geometry = {77.0f, 210.0f, 210.0f};
const uint8_t g_chassis_forward_direction[CHASSIS_WHEEL_COUNT] = {0U, 1U, 0U, 1U};

static uint32_t Chassis_Finite(float value)
{
    /* 比较会拒绝NaN和正负无穷，不依赖不同工具链的isfinite实现 */
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}

static float Chassis_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t Chassis_GeometryFactors(const ChassisGeometry *g,
                                       float *k, float *mm_s_per_rpm)
{
    if ((g == NULL) || !Chassis_Finite(g->wheel_diameter_mm) ||
        !Chassis_Finite(g->track_width_mm) || !Chassis_Finite(g->wheelbase_mm) ||
        (g->wheel_diameter_mm <= 0.0f) || (g->track_width_mm <= 0.0f) ||
        (g->wheelbase_mm <= 0.0f)) return 0U;
    *k = 0.5f * g->track_width_mm + 0.5f * g->wheelbase_mm;
    *mm_s_per_rpm = (g->wheel_diameter_mm / 60.0f) * CHASSIS_PI;
    return (Chassis_Finite(*k) && (*k > 0.0f) &&
            Chassis_Finite(*mm_s_per_rpm) && (*mm_s_per_rpm > 0.0f)) ? 1U : 0U;
}

uint32_t Chassis_Inverse(const ChassisGeometry *geometry,
    const ChassisBodyVelocity *body, float max_wheel_rpm,
    ChassisWheelSpeeds *wheels, float *scale)
{
    float k, mm_s_per_rpm, turn, peak = 0.0f, factor = 1.0f;
    float rpm[CHASSIS_WHEEL_COUNT];
    uint32_t i;
    if (scale != NULL) *scale = 0.0f;
    if (wheels == NULL) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) wheels->rpm[i] = 0.0f;
    if ((body == NULL) || !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm) ||
        !Chassis_Finite(body->vx_mm_s) || !Chassis_Finite(body->vy_mm_s) ||
        !Chassis_Finite(body->wz_rad_s) || !Chassis_Finite(max_wheel_rpm) ||
        (max_wheel_rpm <= 0.0f)) return 0U;
    turn = k * body->wz_rad_s;
    if (!Chassis_Finite(turn)) return 0U;
    /* 轮缘线速按当前X形和车体前/左/逆时针坐标合同计算 */
    rpm[CHASSIS_LEFT_REAR] = (body->vx_mm_s + body->vy_mm_s - turn) / mm_s_per_rpm;
    rpm[CHASSIS_RIGHT_REAR] = (body->vx_mm_s - body->vy_mm_s + turn) / mm_s_per_rpm;
    rpm[CHASSIS_LEFT_FRONT] = (body->vx_mm_s - body->vy_mm_s - turn) / mm_s_per_rpm;
    rpm[CHASSIS_RIGHT_FRONT] = (body->vx_mm_s + body->vy_mm_s + turn) / mm_s_per_rpm;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!Chassis_Finite(rpm[i])) return 0U;
        if (Chassis_Abs(rpm[i]) > peak) peak = Chassis_Abs(rpm[i]);
    }
    if (peak > max_wheel_rpm) factor = max_wheel_rpm / peak;
    if (!Chassis_Finite(factor) || (factor <= 0.0f)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) wheels->rpm[i] = rpm[i] * factor;
    if (scale != NULL) *scale = factor;
    return 1U;
}

uint32_t Chassis_Forward(const ChassisGeometry *geometry,
    const ChassisWheelSpeeds *wheels, ChassisBodyVelocity *body)
{
    float k, mm_s_per_rpm, lr, rr, lf, rf;
    ChassisBodyVelocity result;
    uint32_t i;
    if (body == NULL) return 0U;
    body->vx_mm_s = body->vy_mm_s = body->wz_rad_s = 0.0f;
    if ((wheels == NULL) || !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm))
        return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        if (!Chassis_Finite(wheels->rpm[i])) return 0U;
    /* 先取四分之一，减少求和中间量溢出的机会 */
    lr = 0.25f * wheels->rpm[CHASSIS_LEFT_REAR] * mm_s_per_rpm;
    rr = 0.25f * wheels->rpm[CHASSIS_RIGHT_REAR] * mm_s_per_rpm;
    lf = 0.25f * wheels->rpm[CHASSIS_LEFT_FRONT] * mm_s_per_rpm;
    rf = 0.25f * wheels->rpm[CHASSIS_RIGHT_FRONT] * mm_s_per_rpm;
    result.vx_mm_s = lr + rr + lf + rf;
    result.vy_mm_s = lr - rr - lf + rf;
    result.wz_rad_s = (-lr + rr - lf + rf) / k;
    if (!Chassis_Finite(result.vx_mm_s) || !Chassis_Finite(result.vy_mm_s) ||
        !Chassis_Finite(result.wz_rad_s)) return 0U;
    *body = result;
    return 1U;
}

/* 计算逆解时的舍入误差容差，不作为低速死区或运动参数 */
#define CHASSIS_INTEGER_RPM_TOLERANCE 0.0001f

static void Chassis_ClearMotorCommands(ChassisMotorCommand *commands)
{
    uint32_t i;
    if (commands == NULL) return;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        commands[i].address = (uint8_t)(i + 1U);
        commands[i].direction = g_chassis_forward_direction[i];
        commands[i].speed_raw = 0U;
    }
}

uint32_t Chassis_EncodeEmm(const ChassisWheelSpeeds *logical,
    uint16_t max_raw, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisMotorCommand result[CHASSIS_WHEEL_COUNT];
    uint32_t i;
    float magnitude;
    uint16_t raw;
    Chassis_ClearMotorCommands(commands);
    if ((commands == NULL) || (logical == NULL) || (max_raw == 0U) ||
        (max_raw > 3000U)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        if (!Chassis_Finite(logical->rpm[i])) return 0U;
        magnitude = Chassis_Abs(logical->rpm[i]);
        if (magnitude > (float)max_raw + CHASSIS_INTEGER_RPM_TOLERANCE) return 0U;
        /* 已先限到协议范围，转换不会溢出；拒绝不能表示的小数速度 */
        raw = (uint16_t)(magnitude + 0.5f);
        if ((raw > max_raw) ||
            (Chassis_Abs(magnitude - (float)raw) > CHASSIS_INTEGER_RPM_TOLERANCE))
            return 0U;
        result[i].address = (uint8_t)(i + 1U);
        result[i].speed_raw = raw;
        result[i].direction = g_chassis_forward_direction[i];
        if ((raw != 0U) && (logical->rpm[i] < 0.0f)) result[i].direction ^= 1U;
    }
    /* 四台全部有效后一次提交；后面节点失败不残留前面节点的新目标 */
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) commands[i] = result[i];
    return 1U;
}

uint32_t Chassis_EmmFeedbackToLogical(const int32_t driver_rpm[CHASSIS_WHEEL_COUNT],
    uint32_t valid_mask, ChassisWheelSpeeds *logical)
{
    uint32_t i;
    float rpm[CHASSIS_WHEEL_COUNT];
    if (logical == NULL) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) logical->rpm[i] = 0.0f;
    if ((driver_rpm == NULL) || (valid_mask != 0x0FU)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        /* ReadSpeed来自16位幅值，不接受其他来源溢出值 */
        if ((driver_rpm[i] < -65535) || (driver_rpm[i] > 65535)) return 0U;
        rpm[i] = (float)driver_rpm[i];
        if (g_chassis_forward_direction[i] == 1U) rpm[i] = -rpm[i];
    }
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) logical->rpm[i] = rpm[i];
    return 1U;
}

uint32_t Chassis_BasicTestCommands(const ChassisGeometry *geometry, uint32_t step,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisBodyVelocity body = {0.0f, 0.0f, 0.0f};
    ChassisWheelSpeeds logical;
    float k, mm_s_per_rpm;
    Chassis_ClearMotorCommands(commands);
    if ((commands == NULL) || (step > 2U) ||
        !Chassis_GeometryFactors(geometry, &k, &mm_s_per_rpm)) return 0U;
    if (step == 0U) body.vx_mm_s = mm_s_per_rpm;
    else if (step == 1U) body.vy_mm_s = mm_s_per_rpm;
    else body.wz_rad_s = mm_s_per_rpm / k;
    if (!Chassis_Inverse(geometry, &body, 1.0f, &logical, NULL)) return 0U;
    return Chassis_EncodeEmm(&logical, 1U, commands);
}

/* 纯指令转换：正值逆时针、负值顺时针，不发送CAN。 */
uint32_t Chassis_HeadingCommands(int32_t command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    if (commands == 0) return 0U;
    /* 先检查有符号范围再取反，避免最小负整数溢出。 */
    if ((command_rpm < -5) || (command_rpm > 5))
    {
        (void)Chassis_EncodeEmm(&logical, 5U, commands);
        return 0U;
    }
    /* 轮序：左后、右后、左前、右前；逆时针时左轮后退。 */
    logical.rpm[CHASSIS_LEFT_REAR] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_REAR] = (float)command_rpm;
    logical.rpm[CHASSIS_LEFT_FRONT] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_FRONT] = (float)command_rpm;
    return Chassis_EncodeEmm(&logical, 5U, commands);
}

