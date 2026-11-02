#include "chassis_kinematics.h"
#include "app_config.h"
#include "motion_trial20.h"
#include <float.h>
#include <stddef.h>

#define CHASSIS_PI 3.14159265358979323846f

/* 字段顺序：轮径、左右轮心距、前后轮心距；按用户2026-10-24确认。 */
const ChassisGeometry g_chassis_geometry = {77.0f, 212.0f, 206.0f};
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

uint32_t Chassis_EncodeX(const ChassisWheelSpeeds *logical,
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
        raw = (uint16_t)(magnitude*10.0f + 0.5f);
        if ((raw > (uint32_t)max_raw*10U) || raw>3000U ||
            (Chassis_Abs(magnitude - (float)raw*0.1f) > 0.0501f))
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

uint32_t Chassis_DriverFeedbackToLogical(const float driver_rpm[CHASSIS_WHEEL_COUNT],
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
        if (!(driver_rpm[i]>=-6553.5f && driver_rpm[i]<=6553.5f)) return 0U;
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
    return Chassis_EncodeX(&logical, 1U, commands);
}

/* 纯指令转换：正值逆时针、负值顺时针，不发送CAN。 */
uint32_t Chassis_HeadingCommands(float command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    if (commands == 0) return 0U;
    /* 先检查有符号范围再取反，避免最小负整数溢出。 */
    if (!(command_rpm >= -5.0f && command_rpm <= 5.0f))
    {
        (void)Chassis_EncodeX(&logical, 5U, commands);
        return 0U;
    }
    /* 轮序：左后、右后、左前、右前；逆时针时左轮后退。 */
    logical.rpm[CHASSIS_LEFT_REAR] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_REAR] = (float)command_rpm;
    logical.rpm[CHASSIS_LEFT_FRONT] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_FRONT] = (float)command_rpm;
    return Chassis_EncodeX(&logical, 5U, commands);
}

uint32_t Chassis_TurnBrakeCommands(float command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    uint32_t cap=APP_TURN_BRAKE_TRIAL_ACTIVE ? APP_TURN_BRAKE_MAX_RPM : 13U;
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    if (commands == 0) return 0U;
    /* 先检查有符号范围再取反，避免最小负整数溢出。 */
    if (!(command_rpm >= -(float)cap && command_rpm <= (float)cap))
    {
        (void)Chassis_EncodeX(&logical, cap, commands);
        return 0U;
    }
    /* 轮序：左后、右后、左前、右前；逆时针时左轮后退。 */
    logical.rpm[CHASSIS_LEFT_REAR] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_REAR] = (float)command_rpm;
    logical.rpm[CHASSIS_LEFT_FRONT] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_FRONT] = (float)command_rpm;
    return Chassis_EncodeX(&logical, cap, commands);
}


/* 同时给出前进和受限旋转分量；不反转任何车轮。 */
static uint32_t Chassis_TranslationCommandsAtSpeed(uint32_t mode, uint32_t base_rpm,
    float correction_rpm, uint32_t minimum_base, uint32_t maximum_base,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    float base, lateral;
    uint32_t limit;
    Chassis_ClearMotorCommands(commands);
    if ((commands == NULL) || (mode > 3U) || (base_rpm < minimum_base) || (base_rpm > maximum_base) ||
        !(correction_rpm >= -2.0f && correction_rpm <= 2.0f)) return 0U;
    base = (float)base_rpm;
    limit = base_rpm + 2U;
    if (mode == 0U || mode == 3U)
    {
        if(mode==3U) base=-base;
        logical.rpm[0] = base; logical.rpm[1] = base;
        logical.rpm[2] = base; logical.rpm[3] = base;
    }
    else
    {
        lateral = (mode == 1U) ? base : -base;
        logical.rpm[0] = lateral; logical.rpm[1] = -lateral;
        logical.rpm[2] = -lateral; logical.rpm[3] = lateral;
    }
    /* 与既有逆运动学一致：正修正为车体逆时针，平移方向不反转。 */
    logical.rpm[0] -= (float)correction_rpm;
    logical.rpm[1] += (float)correction_rpm;
    logical.rpm[2] -= (float)correction_rpm;
    logical.rpm[3] += (float)correction_rpm;
    return Chassis_EncodeX(&logical, limit, commands);
}

uint32_t Chassis_TranslationHoldCommandsAtSpeed(uint32_t mode, uint32_t base_rpm,
    float correction_rpm, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{ return Chassis_TranslationCommandsAtSpeed(mode,base_rpm,correction_rpm,3U,15U,commands); }

uint32_t Chassis_ProfileCorrectionRpm(uint32_t base_rpm, float requested, float *effective)
{
    float limit;
    if (!effective) return 0U;
    *effective = 0;
    if (base_rpm > MOTION_PROFILE_BASE_CEILING || !(requested >= -2.0f && requested <= 2.0f)) return 0U;
    limit = (base_rpm < 2U) ? (int32_t)base_rpm : 2;
    *effective = (requested > limit) ? limit : ((requested < -limit) ? -limit : requested);
    return 1U;
}
uint32_t Chassis_TranslationProfileCommands(uint32_t mode, uint32_t base_rpm,
    float correction_rpm, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    float effective;
    Chassis_ClearMotorCommands(commands);
    if (!Chassis_ProfileCorrectionRpm(base_rpm,correction_rpm,&effective)) return 0U;
    return Chassis_TranslationCommandsAtSpeed(mode,base_rpm,effective,0U,MOTION_PROFILE_BASE_CEILING,commands);
}

uint32_t Chassis_LeftTrimCommands(uint32_t base_rpm, float correction_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelSpeeds logical;
    uint32_t trim, room, i;
    float effective;
    Chassis_ClearMotorCommands(commands);
    if (!APP_LEFT_TRIM_ACTIVE || !commands || base_rpm>52U ||
        !(APP_LEFT_BACKWARD_RATIO>=0.0f && APP_LEFT_BACKWARD_RATIO<=0.02f) ||
        !Chassis_ProfileCorrectionRpm(base_rpm,correction_rpm,&effective)) return 0U;
    trim=(uint32_t)((float)base_rpm*APP_LEFT_BACKWARD_RATIO+0.5f);
    room=54U-base_rpm-trim;
    if (room>base_rpm-trim) room=base_rpm-trim;
    if (room>2U) room=2U;
    if (effective>(int32_t)room) effective=(int32_t)room;
    if (effective<-(int32_t)room) effective=-(int32_t)room;
    logical.rpm[0]=(float)base_rpm-(float)trim-(float)effective;
    logical.rpm[1]=-(float)base_rpm-(float)trim+(float)effective;
    logical.rpm[2]=-(float)base_rpm-(float)trim-(float)effective;
    logical.rpm[3]=(float)base_rpm-(float)trim+(float)effective;
    for (i=0U;i<4U;i++)
        if (Chassis_Abs(logical.rpm[i])>54.0f) return 0U;
    return Chassis_EncodeX(&logical,54U,commands);
}

uint32_t Chassis_TranslationHoldCommands(uint32_t mode, float correction_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    return Chassis_TranslationHoldCommandsAtSpeed(mode,
        (mode == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM,
        correction_rpm, commands);
}

uint32_t Chassis_ForwardHoldCommands(float correction_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    return Chassis_TranslationHoldCommands(0U, correction_rpm, commands);
}

/* 启动时纯数学自检：三方向/五级修正与已有逆解交叉核对，不发CAN。 */
uint32_t Chassis_TranslationHoldSelfTest(void)
{
    ChassisMotorCommand actual[4], expected[4];
    ChassisWheelSpeeds wheels;
    ChassisBodyVelocity body;
    uint32_t mode, i, limit;
    int32_t correction;
    float k, per_rpm, base, scale;
    if (!Chassis_GeometryFactors(&g_chassis_geometry, &k, &per_rpm)) return 0U;
    for (mode = 0U; mode < 4U; mode++)
    {
        base = (float)((mode == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM);
        limit = (mode == 0U) ? APP_FORWARD_HOLD_MAX_RPM : APP_LATERAL_HOLD_MAX_RPM;
        for (correction = -2; correction <= 2; correction++)
        {
            body.vx_mm_s = (mode == 0U || mode == 3U) ? (mode==3U ? -base : base)*per_rpm : 0.0f;
            body.vy_mm_s = (mode == 0U || mode == 3U) ? 0.0f : ((mode == 1U) ? base : -base) * per_rpm;
            body.wz_rad_s = (float)correction * per_rpm / k;
            if (!Chassis_Inverse(&g_chassis_geometry, &body, (float)limit, &wheels, &scale) ||
                !Chassis_EncodeX(&wheels, limit, expected) ||
                !Chassis_TranslationHoldCommands(mode, correction, actual)) return 0U;
            for (i = 0U; i < 4U; i++)
                if ((actual[i].address != expected[i].address) ||
                    (actual[i].speed_raw != expected[i].speed_raw) ||
                    (actual[i].direction != expected[i].direction) ||
                    (actual[i].speed_raw == 0U)) return 0U;
            if (mode == 0U)
            {
                if (!Chassis_ForwardHoldCommands(correction, expected)) return 0U;
                for (i = 0U; i < 4U; i++)
                    if ((actual[i].speed_raw != expected[i].speed_raw) ||
                        (actual[i].direction != expected[i].direction)) return 0U;
            }
        }
    }
    if (Chassis_TranslationHoldCommands(4U, 0, actual) != 0U) return 0U;
    for (i = 0U; i < 4U; i++) if (actual[i].speed_raw != 0U) return 0U;
    if (Chassis_TranslationHoldCommands(1U, 3, actual) != 0U) return 0U;
    for (i = 0U; i < 4U; i++) if (actual[i].speed_raw != 0U) return 0U;
    if (Chassis_TranslationHoldCommands(2U, 0, NULL) != 0U) return 0U;
    return 1U;
}

uint32_t Chassis_TranslationPositionMm(uint32_t mode,
    const uint32_t start_sign[4], const uint32_t start_raw[4],
    const uint32_t sign[4], const uint32_t raw[4], float *mean_mm, float *spread_mm)
{
    static const int8_t projection[4][4] = {{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1},{-1,-1,-1,-1}};
    uint32_t i;
    float sum = 0.0f, low = 0.0f, high = 0.0f;
    if (!start_sign || !start_raw || !sign || !raw || !mean_mm || !spread_mm) return 0U;
    *mean_mm = *spread_mm = 0.0f;
    if (mode > 3U) return 0U;
    for (i = 0U; i < 4U; i++)
    {
        int64_t first, current;
        float mm;
        if (start_sign[i] > 1U || sign[i] > 1U) return 0U;
        first = (int64_t)start_raw[i]; current = (int64_t)raw[i];
        if (start_sign[i]) first = -first;
        if (sign[i]) current = -current;
        mm = (float)(current - first) * CHASSIS_PI * g_chassis_geometry.wheel_diameter_mm / 65536.0f;
        if (g_chassis_forward_direction[i]) mm = -mm;
        mm *= (float)projection[mode][i];
        if (i == 0U || mm < low) low = mm;
        if (i == 0U || mm > high) high = mm;
        sum += mm;
    }
    *mean_mm = sum * 0.25f; *spread_mm = high - low;
    return 1U;
}
uint32_t Chassis_ForwardPositionMm(const uint32_t start_sign[4], const uint32_t start_raw[4],
    const uint32_t sign[4], const uint32_t raw[4], float *mean_mm, float *spread_mm)
{ return Chassis_TranslationPositionMm(0U,start_sign,start_raw,sign,raw,mean_mm,spread_mm); }
uint32_t Chassis_CalibrateDistanceMm(uint32_t mode, float encoder_mm, float *progress_mm)
{
    float result;
    if (progress_mm == NULL) return 0U;
    *progress_mm = 0.0f;
    if (mode > 3U || !Chassis_Finite(encoder_mm)) return 0U;
    result = encoder_mm;
    if (mode == 0U || mode == 3U) result *= CHASSIS_FORWARD_DISTANCE_SCALE;
    if (mode == 1U && APP_LEFT_TRIM_ACTIVE) result *= APP_LEFT_DISTANCE_SCALE;
    if (!Chassis_Finite(result)) return 0U;
    *progress_mm = result;
    return 1U;
}

uint32_t Chassis_DistanceRpm(float remaining_mm, uint32_t maximum_rpm)
{
    uint32_t rpm;
    if (!Chassis_Finite(remaining_mm) || remaining_mm <= 0.0f ||
        maximum_rpm < 3U || maximum_rpm > 15U) return 0U;
    rpm = (remaining_mm <= 40.0f) ? 3U : ((remaining_mm <= 120.0f) ? 8U : maximum_rpm);
    return (rpm < maximum_rpm) ? rpm : maximum_rpm;
}
uint32_t Chassis_DistanceSelfTest(void)
{
    uint32_t zero[4] = {0}, sign[4] = {0,1,0,1};
    uint32_t raw[4] = {65536U,65536U,65536U,65536U};
    float mean, spread;
    if (!Chassis_ForwardPositionMm(zero,zero,sign,raw,&mean,&spread) ||
        Chassis_Abs(mean - 241.90263f) > 0.01f || spread > 0.01f) return 0U;
    sign[0]=0U; sign[1]=0U; sign[2]=1U; sign[3]=1U;
    if (!Chassis_TranslationPositionMm(1U,zero,zero,sign,raw,&mean,&spread) ||
        Chassis_Abs(mean - 241.90263f) > 0.01f || spread > 0.01f) return 0U;
    if (!Chassis_TranslationPositionMm(2U,zero,zero,sign,raw,&mean,&spread) || mean >= 0.0f) return 0U;
    sign[0]=1U; sign[1]=1U; sign[2]=0U; sign[3]=0U;
    if (!Chassis_TranslationPositionMm(2U,zero,zero,sign,raw,&mean,&spread) ||
        Chassis_Abs(mean - 241.90263f) > 0.01f || spread > 0.01f) return 0U;
    if (Chassis_TranslationPositionMm(4U,zero,zero,sign,raw,&mean,&spread)) return 0U;
    sign[0]=1U;sign[1]=0U;sign[2]=1U;sign[3]=0U;
    if(!Chassis_TranslationPositionMm(3U,zero,zero,sign,raw,&mean,&spread) ||
       Chassis_Abs(mean-241.90263f)>0.01f || spread>0.01f)return 0U;
    sign[0]=0U; sign[1]=1U; sign[2]=0U; sign[3]=1U;
    if (!Chassis_CalibrateDistanceMm(0U,300.0f,&mean) ||
        Chassis_Abs(mean - 306.54762f) > 0.01f ||
        !Chassis_CalibrateDistanceMm(1U,300.0f,&mean) || Chassis_Abs(mean - (APP_LEFT_TRIM_ACTIVE ? 300.0f*APP_LEFT_DISTANCE_SCALE : 300.0f)) > 0.01f ||
        !Chassis_CalibrateDistanceMm(2U,300.0f,&mean) || mean != 300.0f ||
        !Chassis_CalibrateDistanceMm(3U,300.0f,&mean) ||
        Chassis_Abs(mean-300.0f*CHASSIS_FORWARD_DISTANCE_SCALE)>0.01f ||
        Chassis_CalibrateDistanceMm(4U,300.0f,&mean) ||
        Chassis_CalibrateDistanceMm(0U,300.0f,NULL)) return 0U;
    if (Chassis_DistanceRpm(121.0f,15U) != 15U || Chassis_DistanceRpm(120.0f,15U) != 8U ||
        Chassis_DistanceRpm(40.0f,15U) != 3U || Chassis_DistanceRpm(0.0f,15U) != 0U ||
        Chassis_DistanceRpm(120.0f,5U) != 5U) return 0U;
    sign[0] = 2U;
    if (Chassis_ForwardPositionMm(zero,zero,sign,raw,&mean,&spread)) return 0U;
    sign[0] = 0U; raw[0] = 0xFFFFFFFFU;
    if (!Chassis_ForwardPositionMm(zero,zero,sign,raw,&mean,&spread) || mean <= 3000000.0f) return 0U;
    return 1U;
}
