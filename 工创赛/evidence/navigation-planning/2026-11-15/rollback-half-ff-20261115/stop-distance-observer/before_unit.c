
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
typedef uint32_t TickType_t;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
static uint32_t clock_ms,lock_depth,test_enter_advance_ms;
#define taskENTER_CRITICAL() (++lock_depth,clock_ms+=test_enter_advance_ms,test_enter_advance_ms=0U)
#define taskEXIT_CRITICAL() (--lock_depth)
#include "chassis_auto_test.h"
#include "app_odometry.h"
#include "chassis_direction_guard.h"
#include "motion_trial20.h"
#include "zdt_emm_config.h"
typedef struct {uint8_t Data[8];} CanRxMsg;
static TickType_t xTaskGetTickCount(void){return clock_ms;}

#include "app_entry.h"
#include "route_plan.h"
#include "app_motion_runtime.h"
#include "heading_source.h"
#include "motion_control.h"
#include "motion_distance_limits.h"
static uint32_t __get_BASEPRI(void){return 0U;}
static uint32_t __get_PRIMASK(void){return 0U;}







#define CHASSIS_PI 3.14159265358979323846f

/* 字段顺序：轮径、左右轮心距、前后轮心距；按用户2026-10-24确认。 */
const ChassisGeometry g_chassis_geometry = {VEHICLE_WHEEL_DIAMETER_MM, VEHICLE_TRACK_WIDTH_MM, VEHICLE_WHEELBASE_MM};
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
    return Chassis_EncodeEmm(&logical, 1U, commands);
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

uint32_t Chassis_TurnBrakeCommands(float command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    uint32_t cap=APP_TURN_BRAKE_TRIAL_ACTIVE ? APP_TURN_BRAKE_MAX_RPM : 13U;
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    if (commands == 0) return 0U;
    /* 先检查有符号范围再取反，避免最小负整数溢出。 */
    if (!(command_rpm >= -(float)cap && command_rpm <= (float)cap))
    {
        (void)Chassis_EncodeEmm(&logical, cap, commands);
        return 0U;
    }
    /* 轮序：左后、右后、左前、右前；逆时针时左轮后退。 */
    logical.rpm[CHASSIS_LEFT_REAR] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_REAR] = (float)command_rpm;
    logical.rpm[CHASSIS_LEFT_FRONT] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_FRONT] = (float)command_rpm;
    return Chassis_EncodeEmm(&logical, cap, commands);
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
    return Chassis_EncodeEmm(&logical, limit, commands);
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

#if APP_BASELINE40_ACTIVE
uint32_t Chassis_Baseline40Budget(uint32_t base_rpm, float heading_rpm,
    int32_t *heading_deci, int32_t *lateral_limit_deci, float *mm_s_per_deci)
{
    float effective, factor=CHASSIS_PI*g_chassis_geometry.wheel_diameter_mm/600.0f;
    int32_t c, room, limit;
    if (!heading_deci || !lateral_limit_deci || !mm_s_per_deci || base_rpm>100U ||
        !Chassis_ProfileCorrectionRpm(base_rpm,heading_rpm,&effective) ||
        !Chassis_Finite(factor) || !(factor>0.0f)) return 0U;
    c=(int32_t)(Chassis_Abs(effective)*10.0f+0.5f);
    if(effective<0.0f) c=-c;
    room=(c<0)?-c:c;
    limit=(int32_t)(12.1f/factor);
    if(limit>30) limit=30;
    if(limit>(int32_t)(base_rpm*10U)-room) limit=(int32_t)(base_rpm*10U)-room;
    if(limit>50-room) limit=50-room;
    if(limit<0) limit=0;
    *heading_deci=c; *lateral_limit_deci=limit; *mm_s_per_deci=factor;
    return 1U;
}
uint32_t Chassis_Baseline40Commands(uint32_t base_rpm, int32_t heading_deci,
    int32_t lateral_deci, ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    int32_t c=heading_deci, l=lateral_deci, b, raw[4], ca, la;
    uint32_t i;
    if(!commands || base_rpm>100U || c< -20 || c>20 || l< -30 || l>30) return 0U;
    b=(int32_t)(10U*base_rpm); ca=(c<0)?-c:c; la=(l<0)?-l:l;
    if(ca+la>b || ca+la>50) return 0U;
    raw[0]=b+l-c; raw[1]=b-l+c; raw[2]=b-l-c; raw[3]=b+l+c;
    for(i=0U;i<4U;i++)
        if(raw[i]<0 || raw[i]>b+50 || raw[i]>3000) return 0U;
    for(i=0U;i<4U;i++) {
        commands[i].address=(uint8_t)(i+1U);
        commands[i].direction=g_chassis_forward_direction[i];
        commands[i].speed_raw=(uint16_t)raw[i];
    }
    return 1U;
}
#endif

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
    return Chassis_EncodeEmm(&logical,54U,commands);
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
                !Chassis_EncodeEmm(&wheels, limit, expected) ||
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

#include "motion_profile.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

static uint32_t Profile_Finite(float value)
{ return value >= -FLT_MAX && value <= FLT_MAX; }
static float Profile_Min(float a, float b) { return a < b ? a : b; }
static float Profile_Abs(float a) { return a < 0.0f ? -a : a; }
static uint32_t Profile_LimitsValid(const MotionProfileLimits *limits)
{
    return limits && Profile_Finite(limits->speed) &&
        Profile_Finite(limits->acceleration) && Profile_Finite(limits->jerk) &&
        limits->speed >= 0.001f && limits->speed <= 10000.0f &&
        limits->acceleration >= 0.001f && limits->acceleration <= 100000.0f &&
        limits->jerk >= 0.001f && limits->jerk <= 1000000.0f;
}

uint32_t MotionProfile_LimitTranslation(const ChassisGeometry *geometry,
    float direction_x, float direction_y, const MotionProfileLimits *requested,
    const MotionProfileWheelLimits *wheel, const MotionProfileWheelLimits *reserve,
    MotionProfileLimits *limited, float *time_scale)
{
    MotionProfileLimits result;
    ChassisBodyVelocity body;
    ChassisWheelSpeeds coefficients;
    float norm, scale = 1.0f, peak = 0.0f, unused, speed, acceleration, jerk;
    uint32_t i;
    if (time_scale) *time_scale = 0.0f;
    if (!limited) return 0U;
    /* 输入输出不得重叠；失败输出保持0，避免沿用先前有效参数。 */
    if (limited == requested) return 0U;
    limited->speed = limited->acceleration = limited->jerk = 0.0f;
    if (!time_scale || !Profile_LimitsValid(requested) || !wheel || !reserve ||
        !Profile_Finite(direction_x) || !Profile_Finite(direction_y) ||
        !Profile_Finite(wheel->speed_rpm) || !Profile_Finite(wheel->acceleration_rpm_s) ||
        !Profile_Finite(wheel->jerk_rpm_s2) || !Profile_Finite(reserve->speed_rpm) ||
        !Profile_Finite(reserve->acceleration_rpm_s) || !Profile_Finite(reserve->jerk_rpm_s2) ||
        reserve->speed_rpm < 0.0f || reserve->acceleration_rpm_s < 0.0f || reserve->jerk_rpm_s2 < 0.0f)
        return 0U;
    norm = sqrtf(direction_x * direction_x + direction_y * direction_y);
    if (!Profile_Finite(norm) || norm < 0.000001f) return 0U;
    speed = wheel->speed_rpm - reserve->speed_rpm;
    acceleration = wheel->acceleration_rpm_s - reserve->acceleration_rpm_s;
    jerk = wheel->jerk_rpm_s2 - reserve->jerk_rpm_s2;
    if (speed <= 0.0f || acceleration <= 0.0f || jerk <= 0.0f) return 0U;
    body.vx_mm_s = direction_x / norm;
    body.vy_mm_s = direction_y / norm;
    body.wz_rad_s = 0.0f;
    if (!Chassis_Inverse(geometry, &body, FLT_MAX, &coefficients, &unused)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        if (Profile_Abs(coefficients.rpm[i]) > peak) peak = Profile_Abs(coefficients.rpm[i]);
    if (!Profile_Finite(peak) || peak <= 0.0f) return 0U;
    scale = Profile_Min(scale, speed / (peak * requested->speed));
    scale = Profile_Min(scale, sqrtf(acceleration / (peak * requested->acceleration)));
    scale = Profile_Min(scale, powf(jerk / (peak * requested->jerk), 1.0f / 3.0f));
    if (!Profile_Finite(scale) || scale <= 0.0f) return 0U;
    result.speed = requested->speed * scale;
    result.acceleration = requested->acceleration * scale * scale;
    result.jerk = requested->jerk * scale * scale * scale;
    if (!Profile_LimitsValid(&result)) return 0U;
    *limited = result; *time_scale = scale;
    return 1U;
}

uint32_t MotionProfile_Prepare(MotionProfile *plan, float distance_mm,
    const MotionProfileLimits *limits)
{
    static const float signs[MOTION_PROFILE_PHASES] = {1,0,-1,0,-1,0,1};
    float tj, ta, tv, peak, minimum_distance, phase_time[MOTION_PROFILE_PHASES];
    double position = 0.0, speed = 0.0, acceleration = 0.0, time = 0.0;
    uint32_t i;
    if (!plan) return 0U;
    plan->valid = 0U;
    if (!Profile_LimitsValid(limits) || !Profile_Finite(distance_mm) ||
        distance_mm < 0.0f || distance_mm > 1000000.0f ||
        (distance_mm > 0.0f && distance_mm < 0.01f)) return 0U;
    plan->distance_mm = distance_mm; plan->limits = *limits;
    plan->duration_s = plan->peak_speed = 0.0f;
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
    {
        plan->boundary_s[i] = plan->position[i] = plan->speed[i] = 0.0f;
        plan->acceleration[i] = plan->jerk[i] = 0.0f;
    }
    plan->boundary_s[MOTION_PROFILE_PHASES] = 0.0f;
    if (distance_mm == 0.0f) { plan->valid = 1U; return 1U; }
    peak = limits->speed;
    tj = Profile_Min(limits->acceleration / limits->jerk, sqrtf(peak / limits->jerk));
    ta = peak / (limits->jerk * tj) - tj;
    if (ta < 0.0f) ta = 0.0f;
    minimum_distance = peak * (2.0f * tj + ta);
    tv = 0.0f;
    if (distance_mm >= minimum_distance)
        tv = (distance_mm - minimum_distance) / peak;
    else
    {
        /* 加速与减速对称；短段可能去掉匀速、匀加速和匀减速阶段。 */
        tj = limits->acceleration / limits->jerk;
        if (distance_mm >= 2.0f * limits->acceleration * tj * tj)
            ta = 0.5f * (sqrtf(tj * tj + 4.0f * distance_mm / limits->acceleration) - 3.0f * tj);
        else
        {
            tj = powf(distance_mm / (2.0f * limits->jerk), 1.0f / 3.0f);
            ta = 0.0f;
        }
        peak = limits->jerk * tj * (tj + ta);
    }
    if (!Profile_Finite(tj) || !Profile_Finite(ta) || !Profile_Finite(tv) ||
        tj <= 0.0f || ta < 0.0f || tv < 0.0f || !Profile_Finite(peak) ||
        peak > limits->speed * 1.00001f) return 0U;
    phase_time[0] = phase_time[2] = phase_time[4] = phase_time[6] = tj;
    phase_time[1] = phase_time[5] = ta; phase_time[3] = tv;
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
    {
        double dt = phase_time[i], j = signs[i] * limits->jerk;
        plan->boundary_s[i] = (float)time;
        plan->position[i] = (float)position; plan->speed[i] = (float)speed;
        plan->acceleration[i] = (float)acceleration; plan->jerk[i] = (float)j;
        position += speed * dt + 0.5 * acceleration * dt * dt + j * dt * dt * dt / 6.0;
        speed += acceleration * dt + 0.5 * j * dt * dt;
        acceleration += j * dt; time += dt;
    }
    plan->duration_s = plan->boundary_s[MOTION_PROFILE_PHASES] = (float)time;
    if (!Profile_Finite(plan->duration_s) || plan->duration_s <= 0.0f ||
        plan->duration_s > 3600.0f || Profile_Abs((float)position-distance_mm) >
        0.002f + distance_mm * 0.00001f) return 0U;
    plan->peak_speed = peak; plan->valid = 1U;
    return 1U;
}

uint32_t MotionProfile_Sample(const MotionProfile *plan, float elapsed_s,
    MotionProfileSample *sample)
{
    uint32_t i;
    float dt, position, speed, acceleration;
    if (!sample) return MOTION_PROFILE_INVALID;
    sample->position_mm = sample->speed_mm_s = sample->acceleration_mm_s2 = sample->jerk_mm_s3 = 0.0f;
    if (!plan || plan->valid != 1U || !Profile_Finite(elapsed_s) || elapsed_s < 0.0f ||
        !Profile_Finite(plan->duration_s) || plan->duration_s < 0.0f ||
        !Profile_Finite(plan->distance_mm) || plan->distance_mm < 0.0f)
        return MOTION_PROFILE_INVALID;
    if (elapsed_s >= plan->duration_s)
    { sample->position_mm = plan->distance_mm; return MOTION_PROFILE_DONE; }
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
        if (elapsed_s < plan->boundary_s[i+1U]) break;
    if (i >= MOTION_PROFILE_PHASES) return MOTION_PROFILE_INVALID;
    dt = elapsed_s - plan->boundary_s[i];
    position = plan->position[i] + plan->speed[i] * dt +
        0.5f * plan->acceleration[i] * dt * dt + plan->jerk[i] * dt * dt * dt / 6.0f;
    speed = plan->speed[i] + plan->acceleration[i] * dt + 0.5f * plan->jerk[i] * dt * dt;
    acceleration = plan->acceleration[i] + plan->jerk[i] * dt;
    if (!Profile_Finite(position) || !Profile_Finite(speed) || !Profile_Finite(acceleration) ||
        !Profile_Finite(plan->jerk[i])) return MOTION_PROFILE_INVALID;
    sample->position_mm = position; sample->speed_mm_s = speed;
    sample->acceleration_mm_s2 = acceleration; sample->jerk_mm_s3 = plan->jerk[i];
    return MOTION_PROFILE_RUNNING;
}

#include "heading_controller.h"
#include <float.h>
#include <stddef.h>

#define HEADING_CONTROLLER_MAX_RPM 10U
#define HEADING_CONTROLLER_INPUT_LIMIT_DEG 3600.0f

const HeadingControllerConfig g_heading_controller_default_config =
{
    2.0f, 8.0f, 25.0f, 3.0f,
    1U, 3U, 5U, 700U, 20000U
};

static uint32_t Heading_Finite(float value)
{
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}

static float Heading_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

float HeadingController_WrapDeg(float angle_deg)
{
    /* HWT正常范围为±180度；拒绝异常大值，避免归一化循环无界执行。 */
    if (!Heading_Finite(angle_deg) ||
        (Heading_Abs(angle_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0.0f;
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float HeadingController_ErrorDeg(float target_deg, float current_deg)
{
    if (!Heading_Finite(target_deg) || !Heading_Finite(current_deg) ||
        (Heading_Abs(target_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(current_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0.0f;
    /* 先分别归一化，避免相减后的大角度被误判为零误差 */
    target_deg = HeadingController_WrapDeg(target_deg);
    current_deg = HeadingController_WrapDeg(current_deg);
    return HeadingController_WrapDeg(target_deg - current_deg);
}

uint32_t HeadingController_ConfigValid(const HeadingControllerConfig *config)
{
    if ((config == NULL) || !Heading_Finite(config->tolerance_deg) ||
        !Heading_Finite(config->medium_error_deg) ||
        !Heading_Finite(config->coarse_error_deg) ||
        !Heading_Finite(config->settle_rate_dps)) return 0U;
    if ((config->tolerance_deg <= 0.0f) ||
        (config->medium_error_deg <= config->tolerance_deg) ||
        (config->coarse_error_deg <= config->medium_error_deg) ||
        (config->coarse_error_deg > 180.0f) ||
        (config->settle_rate_dps < 0.0f)) return 0U;
    if ((config->fine_rpm == 0U) ||
        (config->medium_rpm < config->fine_rpm) ||
        (config->coarse_rpm < config->medium_rpm) ||
        (config->coarse_rpm > HEADING_CONTROLLER_MAX_RPM)) return 0U;
    if ((config->settle_ms == 0U) || (config->timeout_ms <= config->settle_ms))
        return 0U;
    return 1U;
}

static void Heading_Output(const HeadingController *controller,
                           HeadingControllerOutput *output)
{
    if (output == NULL) return;
    output->state = controller->state;
    output->cause = controller->cause;
    output->target_deg = controller->target_deg;
    output->error_deg = controller->error_deg;
    output->command_rpm = controller->command_rpm;
    output->stop_required = ((controller->state == HEADING_CTRL_SETTLING) ||
        (controller->state == HEADING_CTRL_DONE) ||
        (controller->state == HEADING_CTRL_FAULT) ||
        (controller->state == HEADING_CTRL_CANCELLED)) ? 1U : 0U;
}

static void Heading_Fault(HeadingController *controller, uint32_t cause,
                          HeadingControllerOutput *output)
{
    controller->state = HEADING_CTRL_FAULT;
    controller->cause = cause;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    Heading_Output(controller, output);
}

uint32_t HeadingController_Start(HeadingController *controller,
    const HeadingControllerConfig *config, float target_deg,
    float current_deg, uint32_t now_tick)
{
    if (controller == NULL) return 0U;
    controller->state = HEADING_CTRL_FAULT;
    controller->cause = HEADING_CAUSE_INVALID_CONFIG;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    controller->target_deg = 0.0f;
    controller->error_deg = 0.0f;
    controller->start_tick = now_tick;
    controller->settle_tick = now_tick;
    if (!HeadingController_ConfigValid(config) ||
        !Heading_Finite(target_deg) || !Heading_Finite(current_deg) ||
        (Heading_Abs(target_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(current_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0U;
    controller->config = *config;
    controller->target_deg = HeadingController_WrapDeg(target_deg);
    controller->error_deg = HeadingController_ErrorDeg(
        controller->target_deg, current_deg);
    controller->state = HEADING_CTRL_TURNING;
    controller->cause = HEADING_CAUSE_NONE;
    return 1U;
}

uint32_t HeadingController_Step(HeadingController *controller,
    uint32_t now_tick, float yaw_deg, float yaw_rate_dps,
    uint32_t sensor_valid, HeadingControllerOutput *output)
{
    float magnitude;
    uint16_t rpm;
    if ((controller == NULL) || (output == NULL)) return 0U;
    if ((controller->state == HEADING_CTRL_DONE) ||
        (controller->state == HEADING_CTRL_FAULT) ||
        (controller->state == HEADING_CTRL_CANCELLED) ||
        (controller->state == HEADING_CTRL_IDLE))
    {
        Heading_Output(controller, output);
        return 1U;
    }
    if ((sensor_valid != 1U) || !Heading_Finite(yaw_deg) ||
        !Heading_Finite(yaw_rate_dps) ||
        (Heading_Abs(yaw_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(yaw_rate_dps) > HEADING_CONTROLLER_INPUT_LIMIT_DEG))
    {
        Heading_Fault(controller, HEADING_CAUSE_SENSOR_INVALID, output);
        return 1U;
    }
    if ((uint32_t)(now_tick - controller->start_tick) >=
        controller->config.timeout_ms)
    {
        Heading_Fault(controller, HEADING_CAUSE_TIMEOUT, output);
        return 1U;
    }
    controller->error_deg = HeadingController_ErrorDeg(
        controller->target_deg, yaw_deg);
    magnitude = Heading_Abs(controller->error_deg);
    if (magnitude <= controller->config.tolerance_deg)
    {
        controller->state = HEADING_CTRL_SETTLING;
        controller->command_rpm = 0;
        if (Heading_Abs(yaw_rate_dps) > controller->config.settle_rate_dps)
        {
            controller->settle_active = 0U;
        }
        else
        {
            if (controller->settle_active == 0U)
            {
                controller->settle_active = 1U;
                controller->settle_tick = now_tick;
            }
            if ((uint32_t)(now_tick - controller->settle_tick) >=
                controller->config.settle_ms)
            {
                controller->state = HEADING_CTRL_DONE;
            }
        }
        Heading_Output(controller, output);
        return 1U;
    }
    controller->settle_active = 0U;
    controller->state = HEADING_CTRL_TURNING;
    if (magnitude > controller->config.coarse_error_deg)
        rpm = controller->config.coarse_rpm;
    else if (magnitude > controller->config.medium_error_deg)
        rpm = controller->config.medium_rpm;
    else
        rpm = controller->config.fine_rpm;
    controller->command_rpm = (controller->error_deg > 0.0f) ?
        (int32_t)rpm : -(int32_t)rpm;
    Heading_Output(controller, output);
    return 1U;
}

void HeadingController_Cancel(HeadingController *controller,
    HeadingControllerOutput *output)
{
    if (controller == NULL) return;
    controller->state = HEADING_CTRL_CANCELLED;
    controller->cause = HEADING_CAUSE_CANCELLED;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    Heading_Output(controller, output);
}

/* 原始角速度与HWT yaw同向，阻尼项取负；只计算，不操作CAN/VM。 */
uint32_t HeadingController_HoldCorrection(float target_deg, float yaw_deg,
    float rate_dps, float *correction_rpm, float *error_deg)
{
    float error, command;
    if ((correction_rpm == NULL) || (error_deg == NULL)) return 0U;
    *correction_rpm = 0.0f;
    *error_deg = 0.0f;
    if (!Heading_Finite(target_deg) || !Heading_Finite(yaw_deg) ||
        !Heading_Finite(rate_dps) || (Heading_Abs(rate_dps) > 2000.0f) ||
        (Heading_Abs(target_deg) > 180.0f) ||
        (Heading_Abs(yaw_deg) > 180.0f)) return 0U;
    error = HeadingController_ErrorDeg(target_deg, yaw_deg);
    *error_deg = error;
    if (Heading_Abs(error) > 10.0f) return 0U;
    command = 1.0f * error - 0.25f * rate_dps;
    if ((Heading_Abs(error) <= 0.3f) && (Heading_Abs(rate_dps) <= 0.3f)) command = 0.0f;
    if (command > 2.0f) command = 2.0f;
    if (command < -2.0f) command = -2.0f;
    *correction_rpm = command;
    return 1U;
}

/* 仅新鲜源帧调用；80ms角速度低通、10RPM/s限斜率、整数命令迟滞。 */
uint32_t HeadingController_HoldStep(HeadingHoldController *hold,
    float target_deg, float yaw_deg, float rate_dps, uint32_t now_ms,
    float *correction_rpm, float *error_deg)
{
    uint32_t elapsed;
    float desired, filtered, step, difference;
    if ((hold == NULL) || (correction_rpm == NULL) || (error_deg == NULL)) return 0U;
    *correction_rpm = 0;
    if (HeadingController_HoldCorrection(target_deg, yaw_deg, rate_dps,
        &desired, error_deg) != 1U) return 0U;
    elapsed = (hold->initialized != 0U) ? now_ms - hold->tick : 20U;
    if (elapsed >= 100U) return 0U;
    if ((hold->initialized != 0U) && (elapsed == 0U))
    { *correction_rpm = (float)hold->quantized_deci_rpm*0.1f; return 1U; }
    filtered = (hold->initialized != 0U) ? hold->filtered_rate_dps +
        ((float)elapsed / (80.0f + (float)elapsed)) *
        (rate_dps - hold->filtered_rate_dps) : rate_dps;
    if (HeadingController_HoldCorrection(target_deg, yaw_deg, filtered,
        &desired, error_deg) != 1U) return 0U;
    step = 0.01f * (float)elapsed;
    difference = desired - hold->command_rpm;
    if (difference > step) difference = step;
    if (difference < -step) difference = -step;
    hold->command_rpm += difference;
    /* 0.1RPM resolution, proportional hysteresis; bounded updates preserve slew. */
    {
        uint32_t quantize_step;
        for(quantize_step=0U;quantize_step<40U;quantize_step++) {
            if(hold->command_rpm*10.0f-(float)hold->quantized_deci_rpm>0.65f)
                hold->quantized_deci_rpm++;
            else if(hold->command_rpm*10.0f-(float)hold->quantized_deci_rpm < -0.65f)
                hold->quantized_deci_rpm--;
            else break;
        }
        if(hold->quantized_deci_rpm>20)hold->quantized_deci_rpm=20;
        if(hold->quantized_deci_rpm < -20)hold->quantized_deci_rpm=-20;
    }
    hold->filtered_rate_dps = filtered;
    hold->tick = now_ms;
    hold->initialized = 1U;
    *correction_rpm = (float)hold->quantized_deci_rpm*0.1f;
    return 1U;
}

volatile uint32_t g_chassis_auto_request = 0U;
volatile uint32_t g_chassis_auto_dry_run = 1U;
volatile uint32_t g_chassis_auto_mode = 1U;
volatile uint32_t g_chassis_auto_state = CHASSIS_AUTO_IDLE;
volatile uint32_t g_chassis_auto_segment = 0U;
volatile uint32_t g_chassis_auto_active = 0U;
volatile uint32_t g_chassis_auto_cut_vm = 0U;
volatile uint32_t g_chassis_auto_session = 0U;
volatile uint32_t g_chassis_auto_result = 0U;
volatile uint32_t g_chassis_auto_cause = 0U;
volatile uint32_t g_chassis_auto_io_result = 0U;
volatile uint32_t g_chassis_auto_last_address = 0U;
volatile uint32_t g_chassis_auto_start_mask[3] = {0U};
volatile uint32_t g_chassis_auto_stop_mask[3] = {0U};
volatile float g_chassis_auto_driver_rpm[4] = {0};

#if APP_POSITION_DIAGNOSTIC
static ChassisPositionSnapshot s_position;
static ChassisOdometrySamples s_odometry_samples;
/* 仅在原锁内记录整数，不在串口中断屏蔽期间进行位置计算。 */
static void Auto_OdometrySample(uint32_t wheel,uint32_t result,uint32_t sign,uint32_t raw,uint32_t ms)
{
    if(result==1U) {
        s_odometry_samples.sign[wheel]=sign; s_odometry_samples.raw[wheel]=raw;
        s_odometry_samples.ms[wheel]=ms; s_odometry_samples.mask|=1UL<<wheel;
    } else s_odometry_samples.mask &= ~(1UL<<wheel);
}
static TickType_t s_position_tick[4];
static uint32_t s_position_valid, s_position_due;
static ChassisPositionEndpoints s_endpoints[CHASSIS_POSITION_ENDPOINT_CAPACITY];
static uint32_t s_endpoint_count, s_endpoint_index;
/* 有界历史仅供同一时刻的位置比较，不外推，不延长400ms时效。 */
#define AUTO_POSITION_HISTORY 16U
typedef struct { uint32_t ms; float mm; } AutoPositionPoint;
static AutoPositionPoint s_position_history[4][AUTO_POSITION_HISTORY];
static uint32_t s_position_history_count[4], s_position_history_head[4];
static uint32_t s_alignment_seen, s_alignment_ms;
static uint32_t s_left_trim_tick, s_left_trim_valid, s_left_trim_rpm;
static float s_left_trim_mm, s_position_trim_mm[4];
static void Auto_LeftTrimAdvance(uint32_t now_ms)
{
    if (s_left_trim_valid)
        s_left_trim_mm-=(float)(now_ms-s_left_trim_tick)*0.001f*(float)s_left_trim_rpm*
            (3.14159265358979323846f*77.0f/60.0f);
    s_left_trim_tick=now_ms; s_left_trim_valid=1U;
}
static void Auto_PositionRemember(uint32_t wheel, uint32_t ms, float mm)
{
    uint32_t n = s_position_history_head[wheel];
    s_position_history[wheel][n].ms = ms;
    s_position_history[wheel][n].mm = mm;
    s_position_history_head[wheel] = (n+1U)%AUTO_POSITION_HISTORY;
    if (s_position_history_count[wheel] < AUTO_POSITION_HISTORY)
        s_position_history_count[wheel]++;
}
static uint32_t Auto_PositionAt(uint32_t wheel, uint32_t ms, float *mm)
{
    uint32_t j, count = s_position_history_count[wheel];
    AutoPositionPoint upper = {0U,0.0f};
    uint32_t have_upper = 0U;
    for (j=0U; j<count; j++)
    {
        AutoPositionPoint point = s_position_history[wheel]
            [(s_position_history_head[wheel]+AUTO_POSITION_HISTORY-1U-j)%AUTO_POSITION_HISTORY];
        int32_t offset = (int32_t)(point.ms-ms);
        if (!offset) { *mm=point.mm; return 1U; }
        if (offset>0) { upper=point; have_upper=1U; }
        else
        {
            uint32_t span=upper.ms-point.ms;
            if (!have_upper || !span || span>=400U) return 0U;
            *mm=point.mm+(upper.mm-point.mm)*(float)(ms-point.ms)/(float)span;
            return 1U;
        }
    }
    return 0U;
}
static void Auto_PositionReset(uint32_t session)
{
    ChassisPositionSnapshot empty = {0};
    uint32_t i;
    s_position = empty; s_position.session = session;
    s_alignment_seen = s_alignment_ms = 0U;
    s_left_trim_tick=s_left_trim_valid=s_left_trim_rpm=0U;s_left_trim_mm=0.0f;
    for(i=0U;i<4U;i++) s_position_trim_mm[i]=0.0f;
    for (i=0U;i<4U;i++) s_position_history_count[i]=s_position_history_head[i]=0U;
    if (session == 0U) s_endpoint_count = 0U;
    s_endpoint_index = CHASSIS_POSITION_ENDPOINT_CAPACITY;
    if (session && s_endpoint_count < CHASSIS_POSITION_ENDPOINT_CAPACITY)
    {
        ChassisPositionEndpoints endpoint = {0};
        endpoint.session = session;
        endpoint.translation_mode = 4U; /* 3表示非平移/尚未指定。 */
        s_endpoint_index = s_endpoint_count++;
        s_endpoints[s_endpoint_index] = endpoint;
    }
    s_position_valid = s_position_due = 0U;
    for (i = 0U; i < 4U; i++) s_position_tick[i] = 0U;
}
#endif
static uint32_t s_vm_requested, s_used, s_mode;
static uint32_t s_motor_index, s_monitor_cursor, s_monitor_between;
static uint32_t s_status_valid, s_speed_valid, s_run_started, s_stop_due;
static uint32_t s_status[4];
static TickType_t s_status_tick[4], s_speed_tick[4];
static TickType_t s_total_tick, s_phase_tick, s_run_tick;
static ChassisMotorCommand s_commands[3][4];
static ChassisDirectionGuard s_direction_guard[4];
/* 正常停车后结束本次序列，记录首次请求时刻。 */
static uint32_t s_finish_requested = 0U;
static TickType_t s_finish_tick = 0U;
/* 仅BeginHeading会话使用，旧三段测试不走动态更新。 */
static uint32_t s_heading_session = 0U;
static uint32_t s_turn_brake_trial;
static uint32_t s_forward_hold = 0U;
static uint32_t s_translation_mode = 0U;
static uint32_t s_translation_base_rpm, s_translation_run_ms;
static uint32_t s_route_trial;
static uint32_t s_distance_mode, s_distance_reached, s_distance_fault_reason;
static uint32_t s_translation_applied_base;
static float s_distance_target_mm, s_distance_progress_mm;
#if APP_DISTANCE_SCURVE_ACTIVE
static MotionProfile s_distance_profile;
static MotionProfileSample s_distance_reference;
static uint32_t s_profile_active, s_profile_done;
static float s_profile_mm_per_rpm_s;
static float s_profile_follow_rpm, s_profile_follow_rate;
static TickType_t s_profile_follow_tick;

#endif
#if APP_POSITION_DIAGNOSTIC
static uint32_t s_distance_max_rpm;
static uint32_t s_distance_seen_ms[4], s_distance_seen_raw[4], s_distance_seen_sign[4];
#endif
static float s_heading_initial_rpm = 0;
static float s_heading_pending_rpm = 0, s_heading_applied_rpm = 0;
static TickType_t s_heading_update_tick = 0U;
/* 最近一次合法轮速提交时间，与实际四轮更新起点分开。 */
static TickType_t s_heading_command_tick = 0U;
static ChassisHeadingRequest s_heading_request = {0, 1U, 0U};
static TickType_t s_heading_request_tick = 0U;
static uint32_t s_heading_request_valid = 0U;
#if APP_TEST_MODE == 3U
static uint32_t s_sequence_hold, s_sequence_continue;
#define AUTO_SEQUENCE_HOLD() (s_sequence_hold)
#else
#define AUTO_SEQUENCE_HOLD() (0U)
#endif

#if APP_BASELINE40_ACTIVE
static uint32_t s_lateral_enabled, s_lateral_tick_valid;
static uint32_t s_lateral_tick_ms, s_lateral_begin_ms;
static float s_lateral_vy;
static int32_t s_lateral_applied_deci;
static void Auto_LateralReset(void)
{
    s_lateral_enabled=s_lateral_tick_valid=0U;
    s_lateral_tick_ms=s_lateral_begin_ms=0U;
    s_lateral_vy=0.0f; s_lateral_applied_deci=0;
}
static float Auto_LateralClamp(float value,float limit)
{ return value>limit ? limit : (value< -limit ? -limit : value); }
/* Trigonometry is outside the state lock; current age/session is checked again inside. */
static float Auto_LateralTarget(const AppOdometryBaseline40 *od,uint32_t begin_ms,uint32_t now_ms,uint32_t *valid)
{
    float angle,delta,e,value;
    *valid=0U;
    if(!od->anchor_valid || (int32_t)(od->reference_set_ms-begin_ms)<0 ||
       !od->pose.valid || od->pose.reasons || od->pose.continuity_lost || od->pose.local_origin ||
       now_ms-od->pose.sample_ms>=ODOM_WHEEL_MAX_AGE_MS ||
       !(od->origin_x_mm>=-FLT_MAX && od->origin_x_mm<=FLT_MAX) ||
       !(od->origin_y_mm>=-FLT_MAX && od->origin_y_mm<=FLT_MAX) ||
       !(od->heading_offset_deg>=-FLT_MAX && od->heading_offset_deg<=FLT_MAX) ||
       !(od->path_heading_deg>=-180.0f && od->path_heading_deg<=180.0f) ||
       !(od->pose.heading_deg>=-180.0f && od->pose.heading_deg<=180.0f) ||
       !(od->pose.x_mm>=-FLT_MAX && od->pose.x_mm<=FLT_MAX) ||
       !(od->pose.y_mm>=-FLT_MAX && od->pose.y_mm<=FLT_MAX)) return 0.0f;
    angle=od->path_heading_deg*0.01745329251994329577f;
    delta=od->pose.heading_deg-od->path_heading_deg;
    if(delta>180.0f) delta-=360.0f;
    if(delta< -180.0f) delta+=360.0f;
    e=-sinf(angle)*(od->pose.x_mm-od->origin_x_mm)+cosf(angle)*(od->pose.y_mm-od->origin_y_mm);
    value=-0.75f*e*cosf(delta*0.01745329251994329577f);
    if(!(value>=-FLT_MAX && value<=FLT_MAX)) return 0.0f;
    *valid=1U;
    return value;
}
static uint32_t Auto_LateralHeadingFresh(const AppHeadingSource *heading,uint32_t ready,uint32_t now_ms)
{
    return ready==1U && heading->healthy==1U && now_ms-heading->sample_ms<100U &&
        App_HeadingFrameAge(heading,now_ms,1U)<100U &&
        App_HeadingFrameAge(heading,now_ms,2U)<100U &&
        heading->yaw_deg>=-180.0f && heading->yaw_deg<=180.0f &&
        heading->rate_dps>=-2000.0f && heading->rate_dps<=2000.0f;
}
static float Auto_LateralFeedforwardPerRpm(const AppOdometryBaseline40 *od,
    const AppHeadingSource *heading,uint32_t ready,uint32_t now_ms)
{
    float delta,value;
    if(!Auto_LateralHeadingFresh(heading,ready,now_ms)) return 0.0f;
    delta=heading->yaw_deg+od->heading_offset_deg-od->path_heading_deg;
    if(!(delta>=-FLT_MAX && delta<=FLT_MAX)) return 0.0f;
    delta=fmodf(delta,360.0f);
    if(delta>180.0f) delta-=360.0f;
    if(delta< -180.0f) delta+=360.0f;
    if(delta< -10.0f || delta>10.0f) return 0.0f;
    value=-0.5f*3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f*
        tanf(delta*0.01745329251994329577f);
    return (value>=-FLT_MAX && value<=FLT_MAX) ? value:0.0f;
}
static uint32_t Auto_LateralStepLocked(uint32_t now_ms,float target,float heading,
    int32_t *heading_deci,int32_t *lateral_deci)
{
    int32_t limit,q;
    float factor,budget,step;
    uint32_t elapsed=s_lateral_tick_valid ? now_ms-s_lateral_tick_ms : 0U;
    if(!Chassis_Baseline40Budget(s_translation_base_rpm,heading,heading_deci,&limit,&factor)) return 0U;
    s_lateral_tick_ms=now_ms; s_lateral_tick_valid=1U;
    if(elapsed>100U) elapsed=100U;
    budget=(float)limit*factor;
    s_lateral_vy=Auto_LateralClamp(s_lateral_vy,budget);
    target=Auto_LateralClamp(target,budget);
    step=20.0f*(float)elapsed*0.001f;
    s_lateral_vy+=Auto_LateralClamp(target-s_lateral_vy,step);
    s_lateral_vy=Auto_LateralClamp(s_lateral_vy,budget);
    q=(int32_t)(fabsf(s_lateral_vy)/factor);
    if(q>limit) q=limit;
    *lateral_deci=s_lateral_vy<0.0f ? -q:q;
    return 1U;
}
#endif



static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count;
static AppHeadingSource test_heading;
static uint32_t test_heading_override,test_heading_ready=1U,test_read_advance_ms,test_read_publish_now;
static uint32_t test_check_base_valid,test_check_base,test_check_session;
static uint32_t target_valid;
static float test_entry_origin;
#define MOTION_ACTION_COUNT s_motion_action_count
volatile uint32_t g_debug_plan_id=DEBUG_PLAN_DEFAULT;
volatile RouteCheck g_route_check;
static uint32_t s_combined_keep_vm,s_combined_powered_segment,s_combined_base_rpm,s_combined_run_ms;
static uint32_t s_motion_session_open,s_motion_has_action,s_motion_list_submit,s_combined_turn,s_combined_mode;
static uint32_t s_combined_begin_ms,s_combined_stage_ms;
static volatile uint32_t s_combined_end_ms;
static float s_combined_reference_mm_s,s_combined_distance_mm,s_route_heading_target;
static uint32_t s_route_heading_turn_active;
static RouteHeadingReference s_route_heading_reference;
static float s_perimeter_targets[9][3];
volatile float g_route_heading_targets[8],g_route_heading_start_yaw[8];
volatile uint32_t g_vm_test_request,g_motion_fault_diag_valid,g_heading_exec_request,g_stage1_stop_requested;
volatile ForwardHoldDebug g_forward_hold;
volatile uint32_t g_translation_hold_selftest=1U;
volatile ForwardHoldTraceSample g_forward_hold_trace[96];
volatile uint32_t g_forward_hold_trace_count;
static uint32_t s_hold_wait_ms,s_hold_begin_ms,s_hold_run_ms,s_hold_stop_ms,s_hold_sequence,s_hold_new_ms,s_hold_trace_ms;
static float s_hold_target;
static HeadingHoldController s_hold_controller;
static const AppTranslationRuntimeInputs s_translation_runtime_inputs={&s_combined_mode,&s_combined_keep_vm,&s_combined_powered_segment,&s_combined_base_rpm,&s_combined_run_ms,&s_combined_distance_mm,&s_combined_reference_mm_s,&s_route_heading_target,&s_route_heading_reference};

static const AppMotionObservation s_motion_observation = {
&s_motion_action_count, &s_motion_session_open, &s_motion_has_action, &s_combined_turn, &s_combined_mode, &s_combined_base_rpm, &s_combined_distance_mm, &s_combined_reference_mm_s, &s_combined_begin_ms, &s_combined_end_ms, &s_combined_stage_ms, s_motion_actions, (const float (*)[3])s_perimeter_targets
};
const AppMotionObservation *AppMotion_Live(void) { return &s_motion_observation; }


static AppOdometryBaseline40 input_od;
static uint32_t sends,stops,monitors,generations,last_source,cut_cause;
static uint32_t sent_raw[4],sent_address[4];
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *p){*p=input_od;}
static void Auto_CheckLocked(TickType_t now){(void)now;
 if(test_check_base_valid){s_translation_base_rpm=test_check_base;test_check_base_valid=0U;}
 if(test_check_session){g_chassis_auto_session++;test_check_session=0U;}
}
static void Auto_CutLocked(uint32_t cause){cut_cause=cause;g_chassis_auto_active=0U;Auto_LateralReset();}
static void Auto_CutDetailLocked(uint32_t a,uint32_t b,uint32_t c,uint32_t d){Auto_CutLocked(a);}
static void Auto_BatchEndLocked(uint32_t a,uint32_t b){}
static void Auto_BatchChangedLocked(uint32_t tick,uint32_t source){generations++;last_source=source;}
static void Auto_BatchSendLocked(uint32_t i,uint32_t tick){}
static void Auto_BatchReplyLocked(uint32_t s,uint32_t i,uint32_t r,uint32_t tick){}
static void Auto_Monitor(uint32_t session){monitors++;}
static uint32_t Auto_AllStatusSinceLocked(uint32_t now){return 0U;}
static uint32_t Auto_AllSpeedSinceLocked(uint32_t now){return 0U;}
static uint32_t Auto_ZeroSpeedLocked(void){return 1U;}
static uint32_t Auto_EndpointStep(uint32_t session,uint32_t end){return 0U;}
#define AUTO_SEQUENCE_HOLD() (s_sequence_hold)
static uint32_t CAN1_SetMotorEnable(uint8_t address,uint32_t on,CanRxMsg *r){return 1U;}
static uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t dir,uint16_t raw,uint8_t acc,CanRxMsg *r){
 if(sends<4){sent_address[sends]=address;sent_raw[sends]=raw;}sends++;return 1U;}
static uint32_t CAN1_StopMotor(uint8_t address,CanRxMsg *r){stops++;return 1U;}

volatile ChassisAutoFaultDiagnostic g_chassis_auto_fault_diagnostic;
static uint32_t s_fault_diagnostic_valid;
static uint32_t Auto_BaseHealthy(void){return 1U;}

static uint32_t App_MotionAccept(void){return 1U;}
void HWT101_DiagnosticArm(void){}
uint32_t HWT101_TimingBegin(void){return 0U;}
void HWT101_TimingEnd(uint32_t a,uint32_t b){}
static uint32_t App_PerimeterTurnStart(uint32_t a,float b,uint32_t c){return 0U;}
uint32_t App_HeadingSourceRead(AppHeadingSource *h){
 if(test_heading_override){*h=test_heading;clock_ms+=test_read_advance_ms;test_read_advance_ms=0U;if(test_read_publish_now){h->sample_ms=h->angle_ms=h->rate_ms=clock_ms;test_read_publish_now=0U;}return test_heading_ready;}
 memset(h,0,sizeof(*h));h->healthy=1;h->valid_mask=3;h->yaw_deg=test_entry_origin;
 h->sample_ms=h->angle_ms=h->rate_ms=clock_ms;return 1U;}

void App_HeadingSourceObserve(void){}
uint32_t App_HeadingRuntimeSource(uint32_t now){return 1U;}
uint32_t App_HeadingRuntimeExclusive(void){return 1U;}
uint32_t App_HeadingPowerReady(uint32_t now){return 1U;}
static uint32_t App_CanJogReady(void){return 1U;}
void App_ForwardHoldSafety(const AppTranslationRuntimeInputs *p){}
void App_ForwardHoldTrip(uint32_t cause){g_forward_hold.trip=cause;g_forward_hold.phase=HR_FAULT;}
uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p,const AppTranslationRuntimeInputs *i){return 1U;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){memset(s,0,sizeof(*s));s->session=g_chassis_auto_session;s->state=g_chassis_auto_state;s->active=g_chassis_auto_active;return 1U;}
uint32_t ChassisAuto_RequestStop(void){s_stop_due=1U;return 1U;}
uint32_t ChassisAuto_UpdateForwardHold(float c){s_heading_pending_rpm=c;return 1U;}

static uint32_t App_MoveSubmit(uint32_t,uint32_t,uint32_t,float,float);

static uint32_t Auto_ProfileFollowRpmLocked(float elapsed, float progress, TickType_t now, float *rpm)
{
    float c = s_route_trial ? ROUTE_TRIAL_FOLLOW_RPM : MotionDistanceFollowRpm(s_distance_max_rpm), w = APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
    float desired = (s_route_trial ? ROUTE_TRIAL_FOLLOW_KP : MotionDistanceFollowKp(s_distance_max_rpm)) *
        (s_distance_reference.position_mm-progress) / s_profile_mm_per_rpm_s;
    TickType_t ticks = (TickType_t)(now-s_profile_follow_tick);
    float value;
    if (!(desired >= -1.0e6f && desired <= 1.0e6f) ||
        !(s_profile_follow_rpm >= -c-0.001f && s_profile_follow_rpm <= c+0.001f) ||
        !(s_profile_follow_rate >= -2.0f*c*w-0.001f && s_profile_follow_rate <= 2.0f*c*w+0.001f)) return 0U;
    if (desired > c) desired = c;
    if (desired < -c) desired = -c;
    if (ticks >= pdMS_TO_TICKS(10U))
    {
        float dt = (float)ticks * portTICK_PERIOD_MS / 1000.0f;
        float y, b, decay;
        /* 调度迟到时不回放多步；较长缺口仍由原源/位置保护处理。 */
        if (dt > 0.1f) dt = 0.1f;
        y = s_profile_follow_rpm-desired;
        b = s_profile_follow_rate+w*y;
        decay = expf(-w*dt);
        value = desired+(y+b*dt)*decay;
        s_profile_follow_rate = (s_profile_follow_rate-w*b*dt)*decay;
        s_profile_follow_rpm = value;
        s_profile_follow_tick = now;
    }
    value = s_distance_reference.speed_mm_s/s_profile_mm_per_rpm_s+s_profile_follow_rpm;
    if (!(value >= -c-0.001f && value <= (float)s_distance_max_rpm+0.001f)) return 0U;
    if (value < 0.0f) value = 0.0f;
    if (value > (float)s_distance_max_rpm) value = (float)s_distance_max_rpm;
    /* 从减速段连续接近，防止取整先变0后再补一脚。仍受原duration+2s截止约束。 */
    if (elapsed >= s_distance_profile.boundary_s[4] &&
        value < (float)APP_DISTANCE_SCURVE_APPROACH_RPM)
        value = (float)APP_DISTANCE_SCURVE_APPROACH_RPM;
    *rpm = value;
    return 1U;
}
static void Auto_DistanceFaultLocked(uint32_t reason)
{ s_distance_fault_reason = reason; Auto_CutDetailLocked(3U,8U,0U,0U); }
static void Auto_DistanceCheckLocked(TickType_t now)
{
    uint32_t i, rpm;
    float mean;
    uint32_t common_ms=0U, oldest_age=0U;
    ChassisPositionEndpoints *base;
    if (!s_distance_mode || s_finish_requested || s_stop_due ||
        (g_chassis_auto_state != CHASSIS_AUTO_RUN
#if APP_DISTANCE_SCURVE_ACTIVE
         && !(s_profile_active && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START)
#endif
        )) return;
    if (s_endpoint_index >= s_endpoint_count) { Auto_DistanceFaultLocked(1U); return; }
    base = &s_endpoints[s_endpoint_index];
    if (base->start_mask != 15U) { Auto_DistanceFaultLocked(1U); return; }
    if (s_position_valid != 15U)
    {
        if ((TickType_t)(now - s_run_tick) >= pdMS_TO_TICKS(500U)) Auto_DistanceFaultLocked(2U);
        return;
    }
    for (i = 0U; i < 4U; i++)
    {
        uint32_t ms = (uint32_t)s_position_tick[i] * portTICK_PERIOD_MS;
        if ((TickType_t)(now - s_position_tick[i]) >= pdMS_TO_TICKS(400U))
        { Auto_DistanceFaultLocked(3U); return; }
        if (!s_position_history_count[i])
            Auto_PositionRemember(i,base->start_ms[i],0.0f);
        if (ms != s_distance_seen_ms[i])
        {
            int64_t old = (int64_t)s_distance_seen_raw[i], current = (int64_t)s_position.raw[i];
            uint32_t dt = ms - s_distance_seen_ms[i];
            int64_t change;
            if (s_distance_seen_sign[i]) old = -old;
            if (s_position.sign[i]) current = -current;
            change = current - old; if (change < 0) change = -change;
            /* 60RPM等效上限加量化余量：仅跳变诊断，原30RPM保护不变。 */
            if (dt > 2000U || change > (int64_t)dt * (s_route_trial ? ((ROUTE_TRIAL_FEEDBACK_RPM+30U)*65536U+59999U)/60000U :
                    MotionDistanceJumpCountsPerMs(s_translation_mode,s_distance_max_rpm,s_distance_target_mm)) + 512)
            { Auto_DistanceFaultLocked(4U); return; }
            {
                static const int8_t projection[4][4]={{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1},{-1,-1,-1,-1}};
                int64_t start=(int64_t)base->start_raw[i];
                float mm;
                if (s_translation_mode>3U || base->start_sign[i]>1U || s_position.sign[i]>1U)
                { Auto_DistanceFaultLocked(5U); return; }
                if (base->start_sign[i]) start=-start;
                mm=(float)(current-start)*3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/65536.0f;
                if (g_chassis_forward_direction[i]) mm=-mm;
                mm*=projection[s_translation_mode][i];
                if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE)
                    mm-=((i==0U || i==3U)?1.0f:-1.0f)*s_position_trim_mm[i];
                Auto_PositionRemember(i,ms,mm);
            }
            s_distance_seen_ms[i] = ms; s_distance_seen_raw[i] = s_position.raw[i];
            s_distance_seen_sign[i] = s_position.sign[i];
        }
    }
    /* 最旧的最新样本是共同时间；其它轮只在历史包围区间内插值。 */
    for (i=0U;i<4U;i++)
    {
        uint32_t age=(uint32_t)(now-s_position_tick[i])*portTICK_PERIOD_MS;
        if (!i || age>oldest_age) { oldest_age=age; common_ms=(uint32_t)s_position_tick[i]*portTICK_PERIOD_MS; }
    }
    mean=0.0f;
    for (i=0U;i<4U;i++)
    {
        float mm;
        if (!Auto_PositionAt(i,common_ms,&mm))
        {
            /* 数据持续更新但历史不相交也不能无限等待。 */
            uint32_t since=s_alignment_seen ? (uint32_t)now*portTICK_PERIOD_MS-s_alignment_ms :
                (uint32_t)(now-s_run_tick)*portTICK_PERIOD_MS;
            if (since>=400U) Auto_DistanceFaultLocked(3U);
            return;
        }
        mean+=mm*0.25f;
    }
    if (!Chassis_CalibrateDistanceMm(s_translation_mode,mean,&mean) ||
        mean < -5.0f || mean < s_distance_progress_mm-5.0f ||
        mean > s_distance_target_mm+50.0f)
    { Auto_DistanceFaultLocked(5U); return; }
    s_alignment_seen=1U; s_alignment_ms=common_ms;
    s_distance_progress_mm = mean;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (s_profile_active)
    {
        uint32_t result;
        float elapsed = (float)(TickType_t)(now-s_run_tick) * portTICK_PERIOD_MS / 1000.0f;
        float ref_rpm;
        result = MotionProfile_Sample(&s_distance_profile,elapsed,&s_distance_reference);
        if (!result) { Auto_DistanceFaultLocked(8U); return; }
        s_profile_done = (result == MOTION_PROFILE_DONE);
        /* 到实际编码目标优先停；参考结束不能冒充实际到距。 */
        if (mean >= s_distance_target_mm ||
            (s_profile_done && mean >= s_distance_target_mm-APP_DISTANCE_SCURVE_ARRIVAL_MM))
        {
            s_distance_reached = 1U; s_finish_requested = s_stop_due = 1U;
            s_finish_tick = now; return;
        }
        {
            MotionProfileSample observed_reference;
            float observed_elapsed=(float)(uint32_t)(common_ms-(uint32_t)s_run_tick*portTICK_PERIOD_MS)/1000.0f;
            if ((int32_t)(common_ms-(uint32_t)s_run_tick*portTICK_PERIOD_MS)<0) observed_elapsed=0.0f;
            if (!MotionProfile_Sample(&s_distance_profile,observed_elapsed,&observed_reference))
            { Auto_DistanceFaultLocked(8U); return; }
            if (mean-observed_reference.position_mm > APP_DISTANCE_SCURVE_TRACK_MM ||
                observed_reference.position_mm-mean > APP_DISTANCE_SCURVE_TRACK_MM)
            { Auto_DistanceFaultLocked(7U); return; }
            /* 跟随修正使用同一时刻的误差，速度前馈仍按当前时刻推进。 */
            float current_reference=s_distance_reference.position_mm;
            float equivalent_progress=mean+current_reference-observed_reference.position_mm;
            if (!Auto_ProfileFollowRpmLocked(elapsed,equivalent_progress,now,&ref_rpm))
            { Auto_DistanceFaultLocked(8U); return; }
        }
        if (s_profile_done && elapsed >= s_distance_profile.duration_s +
                (float)APP_DISTANCE_SCURVE_FINAL_WAIT_MS/1000.0f)
        { Auto_DistanceFaultLocked(9U); return; }
        if (!(ref_rpm >= -0.001f && ref_rpm <= (float)s_distance_max_rpm+0.001f))
        { Auto_DistanceFaultLocked(8U); return; }
        rpm = (ref_rpm <= 0.0f) ? 0U : (uint32_t)(ref_rpm+0.5f);
        if (rpm > s_distance_max_rpm) rpm = s_distance_max_rpm;
        if (rpm != s_translation_base_rpm) Auto_BatchChangedLocked(now,3U);
        s_translation_base_rpm = rpm;
        return; /* 末端有限接近；到距、源失效、跟随误差及原截止仍优先。 */
    }
#endif
    rpm = Chassis_DistanceRpm(s_distance_target_mm - mean, s_distance_max_rpm);
    if (!rpm)
    {
        s_distance_reached = 1U;
        s_finish_requested = s_stop_due = 1U;
        s_finish_tick = now;
    }
    else
    {
        if (rpm != s_translation_base_rpm) Auto_BatchChangedLocked(now, 3U);
        s_translation_base_rpm = rpm;
    }
}
static void Auto_EnterLocked(uint32_t state, TickType_t now)
{
    if (state == CHASSIS_AUTO_STOP) {
        Auto_BatchEndLocked(3U, now);
#if APP_BASELINE40_ACTIVE
        Auto_LateralReset();
#endif
    }
    g_chassis_auto_state = state;
    s_phase_tick = now;
    /* 普通首次批次没有前端发布时间，以选择START时刻为生成边界。 */
    if (state == CHASSIS_AUTO_START && !s_heading_session)
        Auto_BatchChangedLocked(now, 0U);
    s_motor_index = 0U;
    s_monitor_between = 0U;
}
static uint32_t Auto_PrepareBasicCommands(ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT])
{
    uint32_t i, j;
    for (i = 0U; i < 3U; i++)
    {
        if (!Chassis_BasicTestCommands(&g_chassis_geometry, i, commands[i])) return 0U;
        for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++)
            commands[i][j].speed_raw = (uint16_t)(10U*CHASSIS_AUTO_TARGET_RPM);
#if APP_HWT_TURN_TEST && APP_HWT_TURN_CLOCKWISE
        if (i == 2U)
            for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++) commands[i][j].direction ^= 1U;
#endif
    }
    return 1U;
}
static uint32_t Auto_BeginPreparedBasicLocked(uint32_t dry_run,
    const ChassisMotorCommand *commands)
{
    uint32_t i, j, accepted = 0U;
    if ((dry_run <= 1U) && s_vm_requested && !s_used && !g_chassis_auto_active &&
        !g_chassis_auto_cut_vm && Auto_BaseHealthy())
    {
        for (i = 0U; i < 3U; i++)
            for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++) s_commands[i][j] = commands[i * CHASSIS_WHEEL_COUNT + j];
        ChassisAutoFaultDiagnostic empty_fault = {0};
        g_chassis_auto_fault_diagnostic = empty_fault; s_fault_diagnostic_valid = 0U;
#if APP_BASELINE40_ACTIVE
        Auto_LateralReset();
#endif
        s_used = 1U; s_mode = dry_run; g_chassis_auto_mode = dry_run;
        g_chassis_auto_session++;
        for(i=0U;i<4U;i++)DirectionGuard_Reset(&s_direction_guard[i]);
        Auto_BatchChangedLocked(xTaskGetTickCount(), 0U);
#if APP_POSITION_DIAGNOSTIC
        Auto_PositionReset(g_chassis_auto_session);
#endif
        g_chassis_auto_segment = CHASSIS_AUTO_FIRST_SEGMENT;
        g_chassis_auto_result = g_chassis_auto_cause = 0U;
        s_total_tick = xTaskGetTickCount();
        s_finish_requested = 0U;
        s_finish_tick = 0U;
        s_heading_session = 0U;
    s_turn_brake_trial = 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
        s_profile_active = s_profile_done = 0U;
        s_profile_follow_rpm = s_profile_follow_rate = 0.0f; s_profile_follow_tick = 0U;
#endif
        s_forward_hold = 0U;s_route_trial=0U; s_distance_mode = s_distance_reached = s_distance_fault_reason = 0U;
        s_translation_mode = 0U;
        s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
        s_heading_update_tick = 0U;
        s_heading_command_tick = 0U;
        s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
        s_monitor_cursor = 0U;
        for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
        g_chassis_auto_active = 1U;
#if APP_TEST_MODE == 3U
        /* 后续段保持使能，只重新核对零速/状态，不重复失能和使能。 */
        Auto_EnterLocked(s_sequence_continue ? CHASSIS_AUTO_VERIFY_ON :
                         CHASSIS_AUTO_DISABLE, s_total_tick);
        s_sequence_continue = 0U;
#else
        Auto_EnterLocked(CHASSIS_AUTO_DISABLE, s_total_tick);
#endif
        accepted = 1U;
    }
    if (!accepted) g_chassis_auto_result = 9U;
    return accepted;
}
static uint32_t Auto_PrepareTranslationEx(uint32_t motion_mode, uint32_t base_rpm,
    uint32_t run_ms, float distance_mm,float route_speed,ChassisAutoPreparedTranslation *prepared)
{
    if (!prepared) return 0U;
    prepared->valid = 0U;prepared->route_reference_mm_s=0.0f;
    if(route_speed!=0.0f && !MotionRoute_ShapeAllowed(motion_mode,base_rpm,run_ms,distance_mm,route_speed)) return 0U;
    if (__get_BASEPRI() || __get_PRIMASK() || motion_mode > 3U ||
        base_rpm < 3U || (base_rpm > 15U && route_speed==0.0f && !MotionTrial20Allowed(motion_mode,base_rpm,distance_mm) && !MotionTrial40Allowed(motion_mode,base_rpm,distance_mm))) return 0U;
    if (distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if ((!route_speed && (!MotionDistanceAllowed(motion_mode,base_rpm,distance_mm) ||
                run_ms != MotionDistanceRunMsFor(motion_mode,base_rpm,distance_mm))) ||
            !Chassis_DistanceSelfTest()) return 0U;
#else
        return 0U;
#endif
    }
    else if (!run_ms || run_ms > 10000U) return 0U;
    if (!Auto_PrepareBasicCommands(prepared->commands)) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (distance_mm>0.0f)
    { if (!Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U; }
    else
#endif
    if (!Chassis_TranslationHoldCommandsAtSpeed(motion_mode,base_rpm,0,prepared->commands[2])) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    prepared->profile.valid = 0U;
    if (distance_mm > 0.0f)
    {
        MotionProfileLimits limits;
        float factor;
        if (!Chassis_CalibrateDistanceMm(motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor)) return 0U;
        limits.speed = MotionDistanceReferenceSpeed(factor,base_rpm);
        limits.acceleration = MotionDistanceAcceleration(motion_mode,base_rpm,distance_mm) -
            factor*2.0f*MotionDistanceFollowRpm(base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        limits.jerk = MotionDistanceJerk(motion_mode,base_rpm,distance_mm) -
            factor*6.0f*MotionDistanceFollowRpm(base_rpm)*
                APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        if(route_speed!=0.0f) {
            uint32_t expected_rpm,expected_ms;
            if(!MotionRoute_Build(motion_mode,distance_mm,route_speed,&prepared->profile,&expected_rpm,&expected_ms) ||
                expected_rpm!=base_rpm || expected_ms!=run_ms) return 0U;
        } else if(!MotionProfile_Prepare(&prepared->profile,distance_mm,&limits)) return 0U;
        if (
            prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS >= (float)run_ms ||
            !Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U;
    }
#endif
    prepared->motion_mode = motion_mode; prepared->base_rpm = base_rpm;
    prepared->run_ms = run_ms; prepared->distance_mm = distance_mm;
    prepared->route_reference_mm_s=route_speed;
    prepared->valid = 1U;
    return 1U;
}
uint32_t ChassisAuto_PrepareTranslation(uint32_t mode,uint32_t rpm,uint32_t run_ms,
    float mm,ChassisAutoPreparedTranslation *prepared)
{ return Auto_PrepareTranslationEx(mode,rpm,run_ms,mm,0.0f,prepared); }
uint32_t ChassisAuto_PrepareRouteTranslation(uint32_t mode,uint32_t rpm,uint32_t run_ms,
    float mm,float speed,ChassisAutoPreparedTranslation *prepared)
{
    if(!(speed>0.0f)) { if(prepared) prepared->valid=0U;return 0U; }
    return Auto_PrepareTranslationEx(mode,rpm,run_ms,mm,speed,prepared);
}
uint32_t ChassisAuto_BeginPreparedTranslation(const ChassisAutoPreparedTranslation *prepared,
    uint32_t expected_session)
{
    uint32_t accepted = 0U;
    if(prepared && prepared->route_reference_mm_s!=0.0f &&
        !MotionRoute_ShapeAllowed(prepared->motion_mode,prepared->base_rpm,prepared->run_ms,
            prepared->distance_mm,prepared->route_reference_mm_s)) return 0U;
    if (!prepared || prepared->valid != 1U || prepared->motion_mode > 3U ||
        prepared->base_rpm < 3U || (prepared->base_rpm > 15U && prepared->route_reference_mm_s==0.0f &&
            !MotionTrial20Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&
            !MotionTrial40Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
    if (prepared->distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if (!prepared->route_reference_mm_s &&
            (!MotionDistanceAllowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) ||
             prepared->run_ms != MotionDistanceRunMsFor(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
#else
        return 0U;
#endif
    }
    else if (!prepared->run_ms || prepared->run_ms > 10000U) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (prepared->distance_mm > 0.0f)
    {
        float factor;
        MotionProfileLimits route_limits;uint32_t route_rpm;
        if(prepared->route_reference_mm_s!=0.0f) {
            if(!MotionRoute_Limits(prepared->motion_mode,prepared->distance_mm,prepared->route_reference_mm_s,&route_limits,&route_rpm) ||
                prepared->base_rpm!=route_rpm || prepared->run_ms!=MotionRoute_Deadline(prepared->profile.duration_s) ||
                prepared->profile.limits.speed!=route_limits.speed ||
                prepared->profile.limits.acceleration!=route_limits.acceleration ||
                prepared->profile.limits.jerk!=route_limits.jerk) return 0U;
        }
        if (!Chassis_CalibrateDistanceMm(prepared->motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor) ||
            (!prepared->route_reference_mm_s && (prepared->profile.limits.speed != MotionDistanceReferenceSpeed(factor,prepared->base_rpm) ||
            prepared->profile.limits.acceleration != MotionDistanceAcceleration(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -
                factor*2.0f*MotionDistanceFollowRpm(prepared->base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA ||
            prepared->profile.limits.jerk != MotionDistanceJerk(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -
                factor*6.0f*MotionDistanceFollowRpm(prepared->base_rpm)*
                    APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA)) ||
            prepared->profile.boundary_s[0] != 0.0f || prepared->profile.position[0] != 0.0f ||
            prepared->profile.speed[0] != 0.0f || prepared->profile.acceleration[0] != 0.0f)
            return 0U;
    }
    if (prepared->distance_mm > 0.0f &&
        (prepared->profile.valid != 1U || prepared->profile.distance_mm != prepared->distance_mm ||
         !(prepared->profile.duration_s > 0.0f) ||
         !(prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS < (float)prepared->run_ms))) return 0U;
#endif
    taskENTER_CRITICAL();
    if (g_chassis_auto_session == expected_session && g_chassis_auto_state == CHASSIS_AUTO_IDLE &&
        !g_chassis_auto_active && !g_chassis_auto_cut_vm)
    {
#if APP_POSITION_DIAGNOSTIC
        if (prepared->distance_mm > 0.0f && s_endpoint_count >= CHASSIS_POSITION_ENDPOINT_CAPACITY)
        { taskEXIT_CRITICAL(); return 0U; }
#endif
        accepted = Auto_BeginPreparedBasicLocked(0U, &prepared->commands[0][0]);
        if (accepted)
        {
            g_chassis_auto_segment = 2U;
            s_heading_session = s_forward_hold = 1U;
            s_translation_mode = prepared->motion_mode;
            s_translation_base_rpm = s_translation_applied_base = prepared->base_rpm;
            s_translation_run_ms = prepared->run_ms;s_route_trial=prepared->route_reference_mm_s!=0.0f;
#if APP_BASELINE40_ACTIVE
            s_lateral_enabled=g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 &&
                MotionTrial40Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&
                prepared->route_reference_mm_s==0.0f && prepared->profile.valid==1U &&
                prepared->run_ms==APP_BASELINE40_RUN_MS && (*AppMotion_Live()->action_count)==1U;
            s_lateral_begin_ms=(*AppMotion_Live()->begin_ms)*portTICK_PERIOD_MS;
#endif
            s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
            s_heading_update_tick = s_heading_command_tick = xTaskGetTickCount();
#if APP_POSITION_DIAGNOSTIC
            if (s_endpoint_index < s_endpoint_count)
                s_endpoints[s_endpoint_index].translation_mode = prepared->motion_mode;
            if (prepared->distance_mm > 0.0f)
            {
                s_distance_mode = 1U;
                s_distance_max_rpm = prepared->base_rpm;
                s_distance_target_mm = prepared->distance_mm;
                s_distance_progress_mm = 0.0f;
                s_endpoints[s_endpoint_index].distance_mode = 1U;
#if APP_DISTANCE_SCURVE_ACTIVE
                s_distance_profile = prepared->profile;
                s_profile_active = 1U; s_profile_done = 0U;
                (void)Chassis_CalibrateDistanceMm(prepared->motion_mode,
                    3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&s_profile_mm_per_rpm_s);
                s_distance_reference = (MotionProfileSample){0};
                s_profile_follow_rpm = s_profile_follow_rate = 0.0f; s_profile_follow_tick = 0U;
                s_translation_base_rpm = s_translation_applied_base = 0U;
                /* 初批必须是零速，避免错误准备向量绕过曲线起步。 */
                for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; wheel++)
                {
                    s_commands[2][wheel].address = (uint8_t)(wheel+1U);
                    s_commands[2][wheel].direction = g_chassis_forward_direction[wheel];
                    s_commands[2][wheel].speed_raw = 0U;
                }
#endif
            }
#endif
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}
void ChassisAuto_CanStep(void)
{
    uint32_t state, session, index, result, expected, i;
    float effective_heading;
    TickType_t now;
    CanRxMsg reply = {0};
#if APP_BASELINE40_ACTIVE
    AppOdometryBaseline40 lateral_od;
    AppHeadingSource lateral_heading={0};
    uint32_t lateral_session=g_chassis_auto_session, lateral_begin=s_lateral_begin_ms;
    uint32_t lateral_valid,lateral_heading_ready,lateral_now_ms;
    float lateral_target,lateral_ff_per_rpm=0.0f;
    int32_t lateral_deci=s_lateral_applied_deci, heading_deci=0;
    AppOdometry_ReadBaseline40(&lateral_od);
    lateral_heading_ready=App_HeadingSourceRead(&lateral_heading);
    lateral_now_ms=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    lateral_target=Auto_LateralTarget(&lateral_od,lateral_begin,lateral_now_ms,&lateral_valid);
    if(lateral_valid) lateral_ff_per_rpm=Auto_LateralFeedforwardPerRpm(&lateral_od,
        &lateral_heading,lateral_heading_ready,lateral_now_ms);
#endif
    taskENTER_CRITICAL();
    now = xTaskGetTickCount(); Auto_CheckLocked(now);
    if (!g_chassis_auto_active) {
#if APP_BASELINE40_ACTIVE
        Auto_LateralReset();
#endif
        taskEXIT_CRITICAL(); return;
    }
    state = g_chassis_auto_state; session = g_chassis_auto_session;
#if APP_BASELINE40_ACTIVE
    if(s_stop_due || s_finish_requested || state==CHASSIS_AUTO_STOP) {
        s_lateral_vy=0.0f; s_lateral_tick_valid=0U; s_lateral_applied_deci=0;
    }
    if(s_lateral_enabled && !s_stop_due && !s_finish_requested &&
       (state==CHASSIS_AUTO_RUN || (state==CHASSIS_AUTO_START &&
        s_motor_index==0U && s_monitor_between==0U))) {
        if(lateral_session!=session || lateral_begin!=s_lateral_begin_ms ||
           (int32_t)(lateral_od.reference_set_ms-s_lateral_begin_ms)<0 ||
           (uint32_t)now*portTICK_PERIOD_MS-lateral_od.pose.sample_ms>=ODOM_WHEEL_MAX_AGE_MS)
            lateral_valid=0U;
        lateral_now_ms=(uint32_t)now*portTICK_PERIOD_MS;
        if(!Auto_LateralHeadingFresh(&lateral_heading,lateral_heading_ready,lateral_now_ms))
            lateral_ff_per_rpm=0.0f;
        if(lateral_valid) {
            lateral_target+=(float)s_translation_base_rpm*lateral_ff_per_rpm;
            lateral_target=(lateral_target>=-FLT_MAX && lateral_target<=FLT_MAX) ?
                Auto_LateralClamp(lateral_target,12.1f):0.0f;
        } else lateral_target=0.0f;
        if(!Auto_LateralStepLocked((uint32_t)now*portTICK_PERIOD_MS,lateral_target,
                s_heading_pending_rpm,&heading_deci,&lateral_deci))
        { Auto_CutLocked(8U); taskEXIT_CRITICAL(); return; }
    }
#endif
    effective_heading = s_heading_pending_rpm;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (s_profile_active && !Chassis_ProfileCorrectionRpm(s_translation_base_rpm,
            s_heading_pending_rpm,&effective_heading))
    { Auto_CutLocked(8U); taskEXIT_CRITICAL(); return; }
#endif
#if APP_BASELINE40_ACTIVE
    if(s_lateral_enabled && (state==CHASSIS_AUTO_RUN || (state==CHASSIS_AUTO_START &&
        s_motor_index==0U && s_monitor_between==0U))) effective_heading=(float)heading_deci*0.1f;
#endif
/* 仅在整组边界接纳新目标，避免一组四轮混入两个不同轮速。 */
    if ((s_heading_session == 1U) && (s_stop_due == 0U) &&
        (s_finish_requested == 0U) &&
        ((state == CHASSIS_AUTO_RUN) ||
         ((state == CHASSIS_AUTO_START) && (s_motor_index == 0U) &&
          (s_monitor_between == 0U))) &&
        ((effective_heading != s_heading_applied_rpm) ||
         (s_distance_mode && s_translation_base_rpm != s_translation_applied_base)
#if APP_BASELINE40_ACTIVE
         || (s_lateral_enabled && lateral_deci!=s_lateral_applied_deci)
#endif
         ))
    {
#if APP_BASELINE40_ACTIVE
        if(s_lateral_enabled) {
            if(!Chassis_Baseline40Commands(s_translation_base_rpm,heading_deci,lateral_deci,s_commands[2]))
            { Auto_CutLocked(8U); taskEXIT_CRITICAL(); return; }
            if(lateral_deci!=s_lateral_applied_deci) Auto_BatchChangedLocked(now,3U);
            s_lateral_applied_deci=lateral_deci;
        } else
#endif
        if (((s_forward_hold == 1U) ?
#if APP_DISTANCE_SCURVE_ACTIVE
            (s_profile_active ? ((s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE) ?
                Chassis_LeftTrimCommands(s_translation_base_rpm,effective_heading,s_commands[2]) :
                Chassis_TranslationProfileCommands(s_translation_mode,
                s_translation_base_rpm,effective_heading,s_commands[2])) :
#endif
            Chassis_TranslationHoldCommandsAtSpeed(s_translation_mode, s_translation_base_rpm, effective_heading, s_commands[2])
#if APP_DISTANCE_SCURVE_ACTIVE
            )
#endif
            : (s_turn_brake_trial ? Chassis_TurnBrakeCommands(effective_heading, s_commands[2]) :
                Chassis_HeadingCommands(effective_heading, s_commands[2]))) != 1U)
        {
            Auto_CutLocked(8U);
            taskEXIT_CRITICAL();
            return;
        }
        s_heading_applied_rpm = effective_heading;
        s_translation_applied_base = s_translation_base_rpm;
        s_heading_update_tick = now;
        g_chassis_auto_start_mask[2] = 0U;
        Auto_EnterLocked(CHASSIS_AUTO_START, now);
        state = CHASSIS_AUTO_START;
    }

    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)
    { Auto_EnterLocked(CHASSIS_AUTO_STOP, now); state = CHASSIS_AUTO_STOP; }
    if ((state == CHASSIS_AUTO_DISABLE) || (state == CHASSIS_AUTO_ENABLE) ||
        (state == CHASSIS_AUTO_START))
    {
        if (s_monitor_between)
        { s_monitor_between = 0U; taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    }
    if ((state == CHASSIS_AUTO_VERIFY_OFF) || (state == CHASSIS_AUTO_VERIFY_ON))
    {
        expected = (state == CHASSIS_AUTO_VERIFY_ON) ? 1U : 0U;
        if (Auto_AllStatusSinceLocked(now) &&
            ((state == CHASSIS_AUTO_VERIFY_OFF) || Auto_AllSpeedSinceLocked(now)))
        {
            for (i = 0U; i < 4U; i++) if ((s_status[i] & 0x0DU) != expected) break;
            if (i != 4U) Auto_CutLocked(1U);
            else if ((state == CHASSIS_AUTO_VERIFY_ON) && !Auto_ZeroSpeedLocked())
                Auto_CutLocked(7U);
            else if (state == CHASSIS_AUTO_VERIFY_OFF) Auto_EnterLocked(CHASSIS_AUTO_ENABLE, now);
            else
            {
#if APP_POSITION_DIAGNOSTIC
                taskEXIT_CRITICAL();
                if (Auto_EndpointStep(session, 0U)) return;
                taskENTER_CRITICAL();
                now = xTaskGetTickCount(); Auto_CheckLocked(now);
                if (!g_chassis_auto_active || g_chassis_auto_session != session ||
                    g_chassis_auto_state != CHASSIS_AUTO_VERIFY_ON)
                { taskEXIT_CRITICAL(); return; }
#endif
#if APP_POSITION_DIAGNOSTIC
                if (s_distance_mode)
                {
                    if (s_endpoint_index >= s_endpoint_count || s_endpoints[s_endpoint_index].start_mask != 15U)
                    { Auto_CutDetailLocked(3U,7U,0U,0U); taskEXIT_CRITICAL(); return; }
                    for (i = 0U; i < 4U; i++)
                    {
                        s_distance_seen_ms[i] = s_endpoints[s_endpoint_index].start_ms[i];
                        s_distance_seen_raw[i] = s_endpoints[s_endpoint_index].start_raw[i];
                        s_distance_seen_sign[i] = s_endpoints[s_endpoint_index].start_sign[i];
                        s_position.sign[i] = s_distance_seen_sign[i];
                        s_position.raw[i] = s_distance_seen_raw[i]; s_position.result[i] = 1U;
                        s_position_tick[i] = (TickType_t)(s_distance_seen_ms[i] / portTICK_PERIOD_MS);
                    }
                    s_position_valid = 15U; s_position.sequence += 4U;
                }
#endif
                s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_SETTLE)
    {
        if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&
            ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS)))
        {
            if ((s_finish_requested != 0U) || (g_chassis_auto_segment == 2U))
            {
#if APP_POSITION_DIAGNOSTIC
                taskEXIT_CRITICAL();
                if (Auto_EndpointStep(session, 1U)) return;
                taskENTER_CRITICAL();
                now = xTaskGetTickCount(); Auto_CheckLocked(now);
                if (!g_chassis_auto_active || g_chassis_auto_session != session ||
                    g_chassis_auto_state != CHASSIS_AUTO_SETTLE)
                { taskEXIT_CRITICAL(); return; }
                if (s_endpoint_index < s_endpoint_count && s_heading_session)
                    s_endpoints[s_endpoint_index].outcome = 1U;
#endif
                g_chassis_auto_state = CHASSIS_AUTO_DONE;
                g_chassis_auto_result = 1U; g_chassis_auto_cause = 0U;
                g_chassis_auto_active = 0U;
                g_chassis_auto_cut_vm = AUTO_SEQUENCE_HOLD() ? 0U : 1U;
            }
            else
            {
                g_chassis_auto_segment++; s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_RUN)
    { taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    index = s_motor_index;
    if (index >= 4U) { Auto_CutLocked(6U); taskEXIT_CRITICAL(); return; }
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    {
        s_run_tick = now;
#if APP_DISTANCE_SCURVE_ACTIVE
        s_profile_follow_tick = now;
#endif
        s_run_started = 1U;
        if (s_heading_session == 1U) s_heading_update_tick = now;
    }
    if (state == CHASSIS_AUTO_START) Auto_BatchSendLocked(index, xTaskGetTickCount());
    taskEXIT_CRITICAL();

    if (state == CHASSIS_AUTO_DISABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 0U, &reply);
    else if (state == CHASSIS_AUTO_ENABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 1U, &reply);
    else if (state == CHASSIS_AUTO_START)
    {
        ChassisMotorCommand command = s_commands[g_chassis_auto_segment][index];
        result = (s_mode == 1U) ? 1U : CAN1_SetEmmSpeedRaw(command.address,
            command.direction, command.speed_raw, ZDT_EMM_MOTION_ACCELERATION, &reply); /* Emm加速度档位，外层轨迹保持。 */
    }
    else if (state == CHASSIS_AUTO_STOP)
        result = (s_mode == 1U) ? 1U : CAN1_StopMotor((uint8_t)(index + 1U), &reply);
    else return;

    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active &&
        (g_chassis_auto_state == state))
    {
        g_chassis_auto_last_address = index + 1U; g_chassis_auto_io_result = result;
        if (state == CHASSIS_AUTO_START)
            Auto_BatchReplyLocked(session, index, result, xTaskGetTickCount());
        if (result != 1U)
            Auto_CutLocked((state == CHASSIS_AUTO_START) ? 4U :
                           ((state == CHASSIS_AUTO_STOP) ? 5U : 1U));
        else
        {
            if (state == CHASSIS_AUTO_START) {
                ChassisMotorCommand command=s_commands[g_chassis_auto_segment][index];
                int32_t target=command.direction ? -(int32_t)command.speed_raw : (int32_t)command.speed_raw;
                DirectionGuard_Ack(&s_direction_guard[index],target,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                g_chassis_auto_start_mask[g_chassis_auto_segment] |= 1UL << index;
            }
            if (state == CHASSIS_AUTO_STOP) {
                DirectionGuard_Ack(&s_direction_guard[index],0,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                g_chassis_auto_stop_mask[g_chassis_auto_segment] |= 1UL << index;
            }
            s_motor_index++;
            if (s_motor_index == 4U)
            {
#if APP_POSITION_DIAGNOSTIC
                if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE &&
                    (state==CHASSIS_AUTO_START || state==CHASSIS_AUTO_STOP))
                {
                    Auto_LeftTrimAdvance((uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                    s_left_trim_rpm=state==CHASSIS_AUTO_STOP ? 0U :
                        (uint32_t)((float)s_translation_applied_base*APP_LEFT_BACKWARD_RATIO+0.5f);
                }
#endif
                if (state == CHASSIS_AUTO_DISABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_OFF, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_ENABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_ON, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_START) Auto_EnterLocked(CHASSIS_AUTO_RUN, xTaskGetTickCount());
                else Auto_EnterLocked(CHASSIS_AUTO_SETTLE, xTaskGetTickCount());
            }
            else if (state != CHASSIS_AUTO_STOP) s_monitor_between = 1U;
        }
    }
    taskEXIT_CRITICAL();
}
void ChassisAuto_SafetyUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (!s_vm_requested)
    {
        Auto_BatchEndLocked(3U, xTaskGetTickCount());
#if APP_BASELINE40_ACTIVE
        Auto_LateralReset();
#endif
#if APP_POSITION_DIAGNOSTIC
        s_position_valid = s_position_due = 0U;
#endif
        if (g_chassis_auto_active)
        {
#if APP_POSITION_DIAGNOSTIC
            if (s_endpoint_index < s_endpoint_count) s_endpoints[s_endpoint_index].outcome = 2U;
#endif
            g_chassis_auto_state = CHASSIS_AUTO_CANCELLED; g_chassis_auto_result = 3U;
        }
        g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
        s_used = 0U;
#if APP_TEST_MODE == 3U
        s_sequence_hold = s_sequence_continue = 0U;
#endif
    }
    else Auto_CheckLocked(xTaskGetTickCount());
    taskEXIT_CRITICAL();
}
uint32_t AppDebugPlan_Load(uint32_t id, MotionAction *actions, uint32_t *count)
{
#if !APP_ROUTE_USE_CUSTOM || (APP_TEST_MODE != 3U)
    static const MotionAction defaults[] = APP_MOTION_ACTIONS;
#endif
    static const MotionAction route[] = {
        {MOTION_LEFT_DISTANCE,15U,0U,0.0f,300.0f},
        {MOTION_FORWARD_DISTANCE,15U,0U,0.0f,300.0f},
        {MOTION_RIGHT_DISTANCE,15U,0U,0.0f,300.0f}
    };
    uint32_t i,n;
    if (!APP_IS_DEBUG || !actions || !count || id > DEBUG_PLAN_MAX || id == DEBUG_PLAN_FORWARD237) return 0U;
    if (APP_BASELINE40_ACTIVE && id!=DEBUG_PLAN_FORWARD210_40) return 0U;
    if (id == DEBUG_PLAN_FORWARD210_40) {
        MotionAction trial={MOTION_FORWARD_DISTANCE,APP_BASELINE40_RPM,0U,0.0f,APP_BASELINE40_MM};
        if (!MotionTrial40Allowed(0U,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U;actions[0]=trial;
    } else if (id == DEBUG_PLAN_DEFAULT) {
#if VEHICLE_STAGE1_ENABLE
        FieldStage1Plan p;RouteRow rows[2];RouteCheck check;
        if(!FieldMap_Stage1(&p)) return 0U;
        rows[0].kind=MOTION_FORWARD_DISTANCE;rows[0].amount=p.forward_cm;rows[0].speed=p.entry_speed_cm_s;
        rows[1].kind=MOTION_LEFT_DISTANCE;rows[1].amount=p.left_cm;rows[1].speed=p.speed_cm_s;
        if(!RoutePlan_Convert(rows,2U,&g_chassis_geometry,actions,&check))
        {g_route_check=check;return 0U;}
        g_route_check=check;*count=2U;return 1U;
#else

#if APP_ROUTE_USE_CUSTOM && (APP_TEST_MODE == 3U)
        static const RouteRow configured[] = { APP_ROUTE_ROWS(ROUTE_ROW) };
        RouteCheck check;
        n=sizeof(configured)/sizeof(configured[0]);
        if (!RoutePlan_Convert(configured,n,&g_chassis_geometry,actions,&check))
        { g_route_check=check; return 0U; }
        g_route_check=check; *count=n; return 1U;
#else
        n = sizeof(defaults)/sizeof(defaults[0]);
        if (!n || n > MOTION_ACTION_CAPACITY) return 0U;
        for (i=0U;i<n;i++) actions[i]=defaults[i];
#endif
#endif
    } else if (id == DEBUG_PLAN_ROUTE_A) {
        n=3U; for(i=0U;i<n;i++) actions[i]=route[i];
    } else if (id == DEBUG_PLAN_FORWARD210_20) {
        MotionAction trial={MOTION_FORWARD_DISTANCE,APP_SPEED20_TRIAL_RPM,0U,0.0f,APP_SPEED20_TRIAL_MM};
        if (!MotionTrial20Allowed(0U,trial.base_rpm,trial.distance_mm)) return 0U;
        n=1U; actions[0]=trial;
    } else if (id == DEBUG_PLAN_FORWARD210) {
        MotionAction long_forward={MOTION_FORWARD_DISTANCE,15U,0U,0.0f,2100.0f};
        n=1U; actions[0]=long_forward;
    } else {
        n=1U; actions[0]=route[(id==DEBUG_PLAN_LEFT30)?0U:((id==DEBUG_PLAN_FORWARD30)?1U:2U)];
    }
    *count=n; return 1U;
}
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)
    {
        const MotionAction *action = &s_motion_actions[i];
        if(action->reference_mm_s!=0.0f) {
            if(g_debug_plan_id!=DEBUG_PLAN_DEFAULT || !APP_ROUTE_SPEED_TRIAL_ACTIVE ||
                action->kind<MOTION_FORWARD_DISTANCE || action->kind>MOTION_BACKWARD_DISTANCE ||
                action->relative_deg!=0.0f || !MotionRoute_ShapeAllowed(action->kind-MOTION_FORWARD_DISTANCE,
                    action->base_rpm,action->run_ms,action->distance_mm,action->reference_mm_s)) return 0U;
            budget+=action->run_ms+6000U;continue;
        }
        if (action->kind <= MOTION_RIGHT)
        {
            if ((action->base_rpm < 3U) || (action->base_rpm > 15U) ||
                (action->run_ms == 0U) || (action->run_ms > 8000U) ||
                (action->relative_deg != 0.0f) || action->distance_mm != 0.0f) return 0U;
            budget += action->run_ms + 3500U;
        }
        else if (action->kind == MOTION_TURN)
        {
            if ((action->base_rpm > 5U && !(APP_TURN_BRAKE_TRIAL_ACTIVE &&
                (MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY) && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
                action->base_rpm==APP_TURN_BRAKE_MAX_RPM &&
                (action->relative_deg==90.0f || action->relative_deg==-90.0f))) || action->run_ms || action->distance_mm != 0.0f ||
                !(((action->relative_deg >= 5.0f) && (action->relative_deg <= 90.0f)) ||
                  ((action->relative_deg <= -5.0f) && (action->relative_deg >= -90.0f)))) return 0U;
            budget += APP_HEADING_RUN_MS + 3500U;
        }
        else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        {
            if (!APP_POSITION_DIAGNOSTIC || !Chassis_DistanceSelfTest() ||
                action->base_rpm < 3U || action->run_ms ||
                action->relative_deg != 0.0f ||
                !MotionDistanceAllowed(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)) return 0U;
            if (action->base_rpm>15U &&
                (MOTION_ACTION_COUNT!=1U || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&
                 !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MotionTrial40Allowed(0U,action->base_rpm,action->distance_mm))))) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= APP_PERIMETER_GROUP_MS) ? 1U : 0U;
}
static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms, float distance_mm,float route_speed)
{
    ForwardHoldDebug empty = {0};
    uint32_t accepted = 0U;
    if(APP_BASELINE40_ACTIVE && !MotionTrial40Allowed(direction,rpm,distance_mm)) return 0U;
    if(route_speed!=0.0f) {
        if(!APP_POSITION_DIAGNOSTIC || !s_motion_list_submit || g_debug_plan_id!=DEBUG_PLAN_DEFAULT ||
            !MotionRoute_ShapeAllowed(direction,rpm,run_ms,distance_mm,route_speed) ||
            !Chassis_DistanceSelfTest()) return 0U;
    }
    else if (distance_mm!=0.0f)
    {
        if (!APP_POSITION_DIAGNOSTIC || !MotionDistanceAllowed(direction,rpm,distance_mm) ||
            run_ms!=MotionDistanceRunMsFor(direction,rpm,distance_mm) || !Chassis_DistanceSelfTest()) return 0U;
        if (rpm>15U && (!s_motion_list_submit || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&
            !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MotionTrial40Allowed(direction,rpm,distance_mm))))) return 0U;
    }
    else if (direction > MOVE_RIGHT || rpm < MOTION_MOVE_MIN_RPM ||
        rpm > MOTION_MOVE_MAX_RPM || !run_ms || run_ms > MOTION_MOVE_MAX_MS) return 0U;
    taskENTER_CRITICAL();
    if (!g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())
    {
        s_combined_turn = 0U; s_combined_mode = direction;
        s_combined_base_rpm = rpm; s_combined_run_ms = run_ms; s_combined_distance_mm = distance_mm;
        s_combined_reference_mm_s=route_speed;
        empty.motion_mode = direction; g_forward_hold = empty;
        HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;
        s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ return App_MoveSubmit(direction,rpm,run_ms,0.0f,0.0f); }
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm)
{
    float mm=distance_cm*10.0f;
    return App_MoveSubmit(direction,max_rpm,MotionDistanceRunMsFor(direction,max_rpm,mm),mm,0.0f);
}
static uint32_t App_CombinedLoadAction(uint32_t index)
{
    AppHeadingSource heading;
    const MotionAction *action;
    uint32_t accepted, load_cycle, i;
    float planned_offset=0.0f;
    if (index >= MOTION_ACTION_COUNT || !MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY ||
        (g_debug_plan_id != DEBUG_PLAN_DEFAULT &&
         !(g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 && MOTION_ACTION_COUNT==1U && index==0U &&
           s_motion_actions[0].kind==MOTION_FORWARD_DISTANCE &&
           MotionTrial40Allowed(0U,s_motion_actions[0].base_rpm,s_motion_actions[0].distance_mm)))) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset=HeadingController_WrapDeg(planned_offset+s_motion_actions[i].relative_deg);
    if(!RouteHeading_Target(&s_route_heading_reference,planned_offset,&s_route_heading_target)) return 0U;
    action = &s_motion_actions[index];
    s_route_heading_turn_active = action->kind == MOTION_TURN;
    g_route_heading_targets[index]=s_route_heading_target;
    (void)App_HeadingSourceRead(&heading);
    g_route_heading_start_yaw[index]=heading.yaw_deg;
    s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
    else if (action->kind == MOTION_TURN)
        accepted = App_PerimeterTurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm ? action->base_rpm : 5U);
    else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        accepted = MoveDistanceStart(action->kind - MOTION_FORWARD_DISTANCE, action->base_rpm, action->distance_mm / 10.0f);
    else accepted = MoveStart(action->kind, action->base_rpm, action->run_ms);
    s_motion_list_submit = 0U;
    HWT101_TimingEnd(2U, load_cycle);
    return accepted;
}
void App_ForwardHoldStep(const AppTranslationRuntimeInputs *inputs)
{
    AppHeadingSource heading;
    ChassisAutoStatus status;
    uint32_t now, request, delta, accepted, critical_cycle, begin_cycle;
#if APP_DISTANCE_SCURVE_ACTIVE
    /* 唯一CAN任务准备，避免新增轨迹对象占用CAN任务栈；不供其他上下文复用。 */
    static ChassisAutoPreparedTranslation prepared;
#else
    ChassisAutoPreparedTranslation prepared;
#endif
    uint32_t prepared_ok, expected_session, mode, base_rpm, run_ms;
    float route_speed;
    float distance_mm;
    float correction = 0;
    float error = 0.0f, desired = 0.0f;
    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    critical_cycle = HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    request = g_forward_hold.request;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_forward_hold.phase >= HR_DONE)
    { g_vm_test_request = (g_forward_hold.phase == HR_DONE) ? (*inputs->keep_vm) : 0U; HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
    if ((request == 2U) && (g_forward_hold.phase != HR_IDLE)) App_ForwardHoldTrip(1U);
    if ((request == 1U) && (g_forward_hold.phase == HR_IDLE))
    {
#if APP_SERIAL_MOTION_CONTROL && (APP_TEST_MODE != 3U)
        if (s_serial_start_permit != 1U)
        { App_ForwardHoldTrip(3U); HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
#endif
        if ((g_translation_hold_selftest != 1U) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != (*inputs->powered_segment)) || (status.active != 0U) ||
            (status.result != 0U) || (status.cut_vm != 0U) ||
            !(heading.rate_dps >= -3.0f && heading.rate_dps <= 3.0f))
            App_ForwardHoldTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
#endif
            s_hold_wait_ms = now;
            g_forward_hold.phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    App_ForwardHoldSafety(inputs);
    if ((g_forward_hold.phase == HR_WAIT) && (App_CanJogReady() == 1U) &&
        (App_HeadingPowerReady(now) == 1U))
    {
        /* 只携带参数和旧会话号出锁，不携带过期的运动许可。 */
        mode = (*inputs->mode); base_rpm = (*inputs->base_rpm);
        run_ms = (*inputs->run_ms); distance_mm = (*inputs->distance_mm);
#if APP_TEST_MODE == 3U
        route_speed=(*inputs->reference_mm_s);
#else
        route_speed=0.0f;
#endif
        (void)ChassisAuto_ReadStatus(&status); expected_session = status.session;
        HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
        prepared_ok = route_speed!=0.0f ?
            ChassisAuto_PrepareRouteTranslation(mode,base_rpm,run_ms,distance_mm,route_speed,&prepared) :
            ChassisAuto_PrepareTranslation(mode,base_rpm,run_ms,distance_mm,&prepared);
        taskENTER_CRITICAL();
        (void)App_HeadingSourceRead(&heading);
        critical_cycle = HWT101_TimingBegin();
        now = (uint32_t)xTaskGetTickCount();
        App_HeadingSourceObserve();
        App_ForwardHoldSafety(inputs);
        if (g_forward_hold.phase != HR_WAIT || g_forward_hold.trip)
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        (void)ChassisAuto_ReadStatus(&status);
        if (!prepared_ok || status.session != expected_session ||
            mode != (*inputs->mode) || base_rpm != (*inputs->base_rpm) ||
            run_ms != (*inputs->run_ms) || distance_mm != (*inputs->distance_mm)
#if APP_TEST_MODE == 3U
            || route_speed!=(*inputs->reference_mm_s)
#endif
            )
        {
            App_ForwardHoldTrip(5U);
            HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return;
        }
        /* 源/故障/取消已重查；准备条件变化则继续有界等待，不能用旧许可提交。 */
        if (!App_CanJogReady() || !App_HeadingPowerReady(now))
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        /* 提交运动前验证固定参考，异常目标不获得电机启动许可。 */
        if (!inputs->route_heading_reference->valid ||
            !HeadingController_HoldCorrection((*inputs->route_target),heading.yaw_deg,
                heading.rate_dps,&desired,&error))
        {
            App_ForwardHoldTrip(6U);
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();return;
        }
        begin_cycle = HWT101_TimingBegin();
        accepted = ChassisAuto_BeginPreparedTranslation(&prepared,expected_session);
        HWT101_TimingEnd(1U, begin_cycle);
        if (accepted != 1U) App_ForwardHoldTrip(5U);
        else
        {
            (void)ChassisAuto_ReadStatus(&status);
            g_forward_hold.backend_session = status.session;
            s_hold_target = (*inputs->route_target);
            g_forward_hold.start_deg = heading.yaw_deg;
            g_forward_hold.target_deg = s_hold_target;
            s_hold_sequence = heading.sequence;
            s_hold_begin_ms = s_hold_new_ms = now;
            s_hold_controller = (HeadingHoldController){0};
            g_forward_hold.phase = HR_RUN;
        }
    }
    if (g_forward_hold.phase == HR_RUN)
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_ForwardHoldOwns(&status,inputs)) App_ForwardHoldTrip(5U);
        else if (HeadingController_HoldCorrection(s_hold_target, heading.yaw_deg,
            heading.rate_dps, &desired, &error) != 1U) App_ForwardHoldTrip(6U);
        else
        {
            g_forward_hold.error_deg = error;
            /* 有效任务码提前结束本段；沿用唯一CAN任务有序停车及零速核验。 */
            if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
                !g_forward_hold.stopping && status.active && status.state>=CHASSIS_AUTO_START &&
                status.state<=CHASSIS_AUTO_SETTLE)
            {
                if (ChassisAuto_RequestStop()!=1U) App_ForwardHoldTrip(8U);
                else {g_forward_hold.stopping=1U;g_forward_hold.correction_rpm=0;s_hold_stop_ms=now;}
            }
            if (status.distance_mode && status.distance_reached && !g_forward_hold.stopping)
            {
                g_forward_hold.stopping = 1U; g_forward_hold.correction_rpm = 0;
                s_hold_stop_ms = now;
            }
            if ((g_forward_hold.run_started == 0U) && (status.state == CHASSIS_AUTO_RUN) &&
                (g_chassis_auto_start_mask[2] == 0x0FU))
            {
                s_hold_run_ms = now;
                g_forward_hold.run_started = 1U;
            }
            if ((g_forward_hold.run_started != 0U) && (g_forward_hold.stopping == 0U))
                g_forward_hold.elapsed_ms = now - s_hold_run_ms;
            if ((g_forward_hold.stopping == 0U) && g_forward_hold.run_started &&
                (g_forward_hold.elapsed_ms >= (*inputs->run_ms)))
            {
                if ((*inputs->distance_mm) > 0.0f) App_ForwardHoldTrip(2U);
                else if (ChassisAuto_RequestStop() != 1U) App_ForwardHoldTrip(8U);
                else
                {
                    g_forward_hold.stopping = 1U;
                    g_forward_hold.correction_rpm = 0;
                    s_hold_stop_ms = now;
                }
            }
            delta = heading.sequence - s_hold_sequence;
            if (delta > 0x7FFFFFFFU) App_ForwardHoldTrip(2U);
            else if (delta != 0U)
            {
                if (now - s_hold_new_ms > g_forward_hold.max_sample_gap_ms)
                    g_forward_hold.max_sample_gap_ms = now - s_hold_new_ms;
                s_hold_sequence = heading.sequence;
                s_hold_new_ms = now;
                g_forward_hold.sample_count++;
                g_forward_hold.rate_dps = heading.rate_dps;
                if (((error < 0.0f) ? -error : error) > g_forward_hold.peak_error_deg)
                    g_forward_hold.peak_error_deg = ((error < 0.0f) ? -error : error);
                if (((heading.rate_dps < 0.0f) ? -heading.rate_dps : heading.rate_dps) > g_forward_hold.peak_rate_dps)
                    g_forward_hold.peak_rate_dps = ((heading.rate_dps < 0.0f) ? -heading.rate_dps : heading.rate_dps);
                if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 0U))
                {
                    if (HeadingController_HoldStep(&s_hold_controller, s_hold_target,
                        heading.yaw_deg, heading.rate_dps, now,
                        &correction, &error) != 1U) App_ForwardHoldTrip(2U);
                    else if (ChassisAuto_UpdateForwardHold(correction) != 1U) App_ForwardHoldTrip(8U);
                    else
                    {
                        g_forward_hold.correction_rpm = correction;
                        if (correction != 0)
                        {
                            float magnitude = (correction < 0.0f) ? -correction : correction;
                            g_forward_hold.correction_nonzero_samples++;
                            if (magnitude > g_forward_hold.correction_peak_abs_rpm)
                                g_forward_hold.correction_peak_abs_rpm = magnitude;
                            g_forward_hold.correction_last_nonzero_rpm = correction;
                        }
                        if ((g_forward_hold.run_started == 1U) &&
                            (g_forward_hold_trace_count < 96U) &&
                            ((g_forward_hold_trace_count == 0U) ||
                             ((uint32_t)(now - s_hold_trace_ms) >= 100U)))
                        {
                            uint32_t index = g_forward_hold_trace_count;
                            g_forward_hold_trace[index].elapsed_ms = g_forward_hold.elapsed_ms;
                            g_forward_hold_trace[index].error_deg = error;
                            g_forward_hold_trace[index].rate_dps = heading.rate_dps;
                            g_forward_hold_trace[index].correction_rpm = correction;
                            s_hold_trace_ms = now;
                            g_forward_hold_trace_count = index + 1U;
                        }
                    }
                }
            }
            if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 1U) &&
                (status.active == 0U) && (status.result == 1U) &&
                (status.rotation_stop_mask == 0x0FU))
            {
                g_forward_hold.final_deg = heading.yaw_deg;
                g_forward_hold.final_valid = App_HeadingRuntimeSource(now);
                g_forward_hold.correction_rpm = 0;
                g_forward_hold.phase = HR_DONE;
                g_vm_test_request = (*inputs->keep_vm);
            }
        }
    }
    HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
    if ((g_forward_hold.phase == HR_RUN) && (g_forward_hold.trip == 0U)) ChassisAuto_CanStep();
}

static uint32_t entry_start(void){
 uint32_t n=0;
 Auto_LateralReset();g_chassis_auto_active=g_chassis_auto_cut_vm=0U;g_chassis_auto_state=CHASSIS_AUTO_IDLE;g_chassis_auto_session=17U;
 s_used=0U;s_vm_requested=1U;s_endpoint_count=0U;s_endpoint_index=0U;
 g_debug_plan_id=DEBUG_PLAN_FORWARD210_40;g_forward_hold=(ForwardHoldDebug){0};
 s_motion_has_action=0U;s_combined_powered_segment=s_combined_keep_vm=g_vm_test_request=1U;s_combined_begin_ms=100U;
 s_route_heading_reference.valid=1U;s_route_heading_reference.origin_deg=test_entry_origin;
 clock_ms=1000U;
 if(!AppDebugPlan_Load(g_debug_plan_id,s_motion_actions,&n)) return 0U;s_motion_action_count=n;
 if(n!=1 || s_motion_actions[0].reference_mm_s!=0.0f || !App_MotionPlanValid())return 0U;
 if(!App_CombinedLoadAction(0U) || s_combined_reference_mm_s!=0.0f)return 0U;
 App_ForwardHoldStep(&s_translation_runtime_inputs);
 return g_forward_hold.trip==0U && g_forward_hold.phase==HR_RUN && g_chassis_auto_active && s_route_trial==0U;
}


volatile uint32_t entry_reference_milli,entry_enabled,entry_peak_l,entry_F,entry_T,entry_N,entry_wheels[4];
static uint32_t observe_entry(void){
 uint32_t k;int32_t q0,q1,q2,q3,lat;
 if(!entry_start())return 0U;
 entry_reference_milli=(uint32_t)(1000*s_motion_actions[0].reference_mm_s);entry_enabled=s_lateral_enabled;
 s_stop_due=s_finish_requested=s_monitor_between=s_motor_index=s_run_started=0U;
 g_chassis_auto_state=CHASSIS_AUTO_RUN;s_translation_base_rpm=s_translation_applied_base=100U;
 s_heading_pending_rpm=s_heading_applied_rpm=0;
 Chassis_TranslationProfileCommands(0U,100U,0,s_commands[2]);
 memset(&input_od,0,sizeof(input_od));input_od.anchor_valid=input_od.pose.valid=1U;input_od.reference_set_ms=100U;
 input_od.origin_x_mm=input_od.origin_y_mm=2100;input_od.path_heading_deg=-90;
 input_od.pose.x_mm=2080;input_od.pose.y_mm=1000;input_od.pose.heading_deg=-90;
 for(k=0;k<70;k++){
  clock_ms=1000U+k*20U;input_od.pose.sample_ms=clock_ms;ChassisAuto_CanStep();
 }
 q0=s_commands[2][0].speed_raw;q1=s_commands[2][1].speed_raw;q2=s_commands[2][2].speed_raw;q3=s_commands[2][3].speed_raw;
 lat=(q0-q1-q2+q3)/4;entry_peak_l=(uint32_t)lat;
 entry_F=(q0+q1+q2+q3)/4;entry_T=(-q0+q1-q2+q3)/4;entry_N=(q0+q1-q2-q3)/4;
 for(k=0;k<4;k++)entry_wheels[k]=s_commands[2][k].speed_raw;
 return entry_F==1000 && entry_T==0 && entry_N==0;
}


volatile uint32_t assertions,cases;
#define CHECK(x) do {assertions++;if(!(x))return __LINE__;}while(0)
static void setup(void){
 test_heading_override=test_read_advance_ms=test_read_publish_now=test_enter_advance_ms=test_check_base_valid=test_check_session=0U;
 entry_start();g_chassis_auto_active=1U;g_chassis_auto_session=17U;
 g_chassis_auto_segment=2U;g_chassis_auto_state=CHASSIS_AUTO_RUN;
 s_mode=0U;s_heading_session=s_forward_hold=s_distance_mode=s_profile_active=1U;
 s_translation_mode=0U;s_translation_base_rpm=s_translation_applied_base=100U;
 s_stop_due=s_finish_requested=s_monitor_between=s_motor_index=s_run_started=0U;
 s_heading_pending_rpm=s_heading_applied_rpm=0.0f;
 /* enabled and begin_ms originate in real BeginPreparedTranslation */
 memset(&input_od,0,sizeof(input_od));input_od.anchor_valid=1U;input_od.reference_set_ms=100U;
 input_od.origin_x_mm=input_od.origin_y_mm=2100.0f;input_od.path_heading_deg=-90.0f;
 input_od.pose.x_mm=2067.0f;input_od.pose.y_mm=1000.0f;input_od.pose.heading_deg=-90.0f;
 input_od.pose.valid=1U;input_od.pose.sample_ms=1000U;clock_ms=1000U;
 sends=stops=monitors=generations=cut_cause=0U;lock_depth=0U;
 Chassis_Baseline40Commands(100U,0,0,s_commands[2]);
}
static void ff_setup(float delta,float right,float origin){
 test_entry_origin=origin;test_heading_override=0U;setup();
 input_od.pose.x_mm=input_od.origin_x_mm-right;input_od.pose.y_mm=input_od.origin_y_mm;
 input_od.path_heading_deg=input_od.pose.heading_deg=-90.0f;input_od.heading_offset_deg=-90.0f-origin;
 memset(&test_heading,0,sizeof(test_heading));test_heading.healthy=1U;test_heading.valid_mask=3U;
 test_heading.yaw_deg=origin+delta;if(test_heading.yaw_deg>180)test_heading.yaw_deg-=360;
 if(test_heading.yaw_deg< -180)test_heading.yaw_deg+=360;
 test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;
 test_heading_ready=test_heading_override=1U;s_lateral_tick_valid=1U;s_lateral_tick_ms=clock_ms-100U;s_lateral_vy=0;
}
static int32_t ff_L(void){return ((int32_t)s_commands[2][0].speed_raw-(int32_t)s_commands[2][1].speed_raw-(int32_t)s_commands[2][2].speed_raw+(int32_t)s_commands[2][3].speed_raw)/4;}
static void ff_steady(void){uint32_t k;for(k=0;k<70;k++){clock_ms=1000U+20U*k;input_od.pose.sample_ms=clock_ms;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;ChassisAuto_CanStep();}}
volatile uint32_t ff_positive_L,ff_negative_abs_L,ff_cross_L,ff_half_F_L,ff_HWT_fallback_L,ff_ODinvalid_L,ff_unified_clamp_L;

/* Fixture only: raw reference/common-mean inputs, not a fitted vehicle plant. */
volatile uint32_t distance_control_trace[32][12];
static void distance_inputs(uint32_t now,float mean)
{
    uint32_t w;float wheel_mm=mean/CHASSIS_FORWARD_DISTANCE_SCALE;
    clock_ms=now;s_position_valid=15U;s_endpoint_index=0U;s_endpoint_count=1U;
    s_endpoints[0].start_mask=15U;
    for(w=0;w<4;w++) {
        s_position_tick[w]=now;s_distance_seen_ms[w]=now;
        s_position_history_count[w]=2U;s_position_history_head[w]=2U;
        s_position_history[w][0].ms=now-100U;s_position_history[w][0].mm=wheel_mm;
        s_position_history[w][1].ms=now;s_position_history[w][1].mm=wheel_mm;
    }
}
static uint32_t observe_distance_control(void)
{
    uint32_t k;MotionProfileSample sample;
    CHECK(entry_start());s_run_tick=1000U;s_profile_follow_tick=1000U;
    s_run_started=1U;g_chassis_auto_state=CHASSIS_AUTO_RUN;
    s_stop_due=s_finish_requested=0U;
    for(k=0;k<32;k++) {
        float elapsed=.25f*k,mean;uint32_t now=1000U+250U*k;
        CHECK(MotionProfile_Sample(&s_distance_profile,elapsed,&sample));
        mean=sample.position_mm+(k%3U==0U ? 1.0f : -1.0f);
        if(mean<0)mean=0;distance_inputs(now,mean);
        Auto_DistanceCheckLocked(now);
        distance_control_trace[k][0]=s_translation_base_rpm;
        distance_control_trace[k][1]=s_translation_applied_base;
        distance_control_trace[k][2]=s_distance_reached;
        distance_control_trace[k][3]=s_stop_due;
        distance_control_trace[k][4]=s_finish_requested;
        distance_control_trace[k][5]=s_distance_fault_reason;
        memcpy((void*)&distance_control_trace[k][6],&s_distance_progress_mm,4);
        memcpy((void*)&distance_control_trace[k][7],&s_distance_reference.position_mm,4);
        memcpy((void*)&distance_control_trace[k][8],&s_distance_reference.speed_mm_s,4);
        memcpy((void*)&distance_control_trace[k][9],&s_profile_follow_rpm,4);
        memcpy((void*)&distance_control_trace[k][10],&s_profile_follow_rate,4);
        distance_control_trace[k][11]=s_profile_follow_tick;
        CHECK(!s_distance_fault_reason && g_chassis_auto_active);
    }
    cases++;return 0U;
}

#ifdef CHASSIS_TAIL_PROFILE_ACTIVE
volatile uint8_t snapshot_observation_blob[176];
static uint32_t check_tail_snapshots(void)
{
    ChassisAutoTailSnapshots a,saved;ChassisAutoPreparedTranslation p,bad;
    uint32_t last_follow_tick,old_freezes;float last_follow,last_rate;
    CHECK(sizeof(a)==176U && sizeof(a.slot[0])==48U);
    CHECK((uint32_t)((char*)&a.slot-(char*)&a)==32U);
    CHECK((uint32_t)((char*)&a.slot[0].common_mean_mm-(char*)&a.slot[0])==16U);
    CHECK((uint32_t)((char*)&a.slot[0].pending_base_rpm-(char*)&a.slot[0])==36U);
    CHECK((uint32_t)((char*)&a.slot[0].flags-(char*)&a.slot[0])==44U);
    CHECK(entry_start());ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.session==g_chassis_auto_session && a.format_version==1U && !a.valid_mask);
    s_run_tick=1000U;s_profile_follow_tick=8200U;s_run_started=1U;
    s_profile_follow_rpm=-1.0f;s_profile_follow_rate=.1f;
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_stop_due=s_finish_requested=0U;
    s_distance_progress_mm=1975.0f;distance_inputs(8300U,1985.0f);
    Auto_DistanceCheckLocked(8300U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==1U && a.entry_count==1U && a.slot[0].now_ms==8300U);
    CHECK(a.slot[0].follow_tick_ms==8200U && a.slot[0].follow_rpm==-1.0f);
    CHECK(a.slot[0].common_ms==8300U && a.slot[0].common_mean_mm>1984.9f);
    saved=a;distance_inputs(8310U,1986.0f);Auto_DistanceCheckLocked(8310U);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots.slot[0],&saved.slot[0],48)==0);
    /* A genuine accepted command freeze; ACK completion is not claimed. */
    input_od.pose.valid=0U;s_lateral_vy=0;s_lateral_applied_deci=0U;
    s_translation_applied_base=99U;s_heading_applied_rpm=s_heading_pending_rpm=0;
    s_monitor_between=s_motor_index=0;g_chassis_auto_state=CHASSIS_AUTO_RUN;
    clock_ms=8311U;ChassisAuto_CanStep();ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==3U && a.freeze_count==1U);
    CHECK(a.slot[1].now_ms==8311U && a.slot[1].applied_base_rpm==a.slot[1].pending_base_rpm);
    CHECK(g_chassis_auto_state==CHASSIS_AUTO_START && s_motor_index==1U);
    old_freezes=a.freeze_count;clock_ms++;ChassisAuto_CanStep();
    CHECK(g_chassis_auto_tail_snapshots.freeze_count==old_freezes);
    /* Unchanged requested/applied batch cannot generate a freeze record. */
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_motor_index=s_monitor_between=0U;
    clock_ms++;ChassisAuto_CanStep();CHECK(g_chassis_auto_tail_snapshots.freeze_count==old_freezes);
    last_follow=s_profile_follow_rpm;last_rate=s_profile_follow_rate;last_follow_tick=s_profile_follow_tick;
    distance_inputs(9050U,2104.0f);Auto_DistanceCheckLocked(9050U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.valid_mask==7U && a.arrival_count==1U && a.slot[2].now_ms==9050U);
    CHECK(a.slot[2].reference_rpm>14.0f && a.slot[2].reference_rpm<16.0f);
    CHECK(a.slot[2].common_mean_mm>2103.9f && a.slot[2].run_tick_ms==1000U);
    CHECK(a.slot[2].follow_rpm==last_follow && a.slot[2].follow_rate_rpm_s==last_rate);
    CHECK(a.slot[2].follow_tick_ms==last_follow_tick);
    CHECK((a.slot[2].flags&(8U|16U|32U))==0U);
    CHECK((a.slot[2].flags&(1U|4U|64U|128U))==(1U|4U|64U|128U));
    CHECK(s_distance_reached && s_stop_due && s_finish_requested && s_finish_tick==9050U);
    memcpy((void*)snapshot_observation_blob,&a,176U);saved=a;
    ChassisAuto_CanStep();ChassisAuto_ReadTailSnapshots(&a);CHECK(memcmp(&a,&saved,176)==0);
    ChassisAuto_SafetyUpdate(0U);ChassisAuto_ReadTailSnapshots(&a);CHECK(memcmp(&a,&saved,176)==0);
    ChassisAuto_ReadTailSnapshots(0);CHECK(!lock_depth);
    CHECK(ChassisAuto_PrepareTranslation(0U,100U,APP_BASELINE40_RUN_MS,2100.0f,&p));
    bad=p;bad.profile.valid=0U;
    CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    CHECK(!ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session+1U));
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    /* Accepted nonspecial action retains previous snapshot with its own session. */
    g_chassis_auto_active=g_chassis_auto_cut_vm=s_used=0U;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_vm_requested=1U;
    g_debug_plan_id=DEBUG_PLAN_DEFAULT;s_motion_action_count=1U;s_endpoint_count=0U;
    CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));CHECK(!s_lateral_enabled);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    Auto_TailSnapshotRecordLocked(2U,clock_ms);CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    CHECK(entry_start());ChassisAuto_ReadTailSnapshots(&a);CHECK(a.valid_mask==0 && a.session==g_chassis_auto_session);
    /* No cross-session capture; exact C unsigned wrap for common freshness. */
    s_profile_active=s_run_started=g_chassis_auto_active=s_lateral_enabled=1U;
    s_alignment_seen=1U;s_alignment_ms=0xFFFFFFF0U;s_run_tick=0xFFFFF000U;s_profile_follow_tick=0xFFFFFFFAU;
    s_distance_progress_mm=1990;s_distance_reference.speed_mm_s=80;s_profile_mm_per_rpm_s=4;
    Auto_TailSnapshotRecordLocked(0U,5U);ChassisAuto_ReadTailSnapshots(&a);
    CHECK(a.slot[0].now_ms==5U && a.slot[0].common_ms==0xFFFFFFF0U);
    CHECK((a.slot[0].flags&128U)!=0U && a.slot[0].reference_rpm==20.0f);
    saved=a;g_chassis_auto_session++;Auto_TailSnapshotRecordLocked(1U,6U);
    CHECK(memcmp((const void*)&g_chassis_auto_tail_snapshots,&saved,176)==0);
    cases+=18U;return 0U;
}
#endif

__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 ChassisMotorCommand cmd[4],saved[4];int32_t c,l,limit,b,h,sign,i;float factor,first;
 uint32_t saved_tick,saved_sends,k,j;AppOdometryBaseline40 od;

 CHECK(observe_entry());CHECK(entry_enabled==1 && entry_peak_l>0);CHECK(entry_F==1000 && entry_T==0 && entry_N==0);
 CHECK(entry_start());CHECK(s_lateral_enabled==1U);CHECK(s_motion_actions[0].reference_mm_s==0.0f);
 CHECK(AppMotion_Live()->action_count==&s_motion_action_count);CHECK(g_translation_hold_selftest==1U);
 {ChassisAutoPreparedTranslation p;uint32_t j;
 CHECK(ChassisAuto_PrepareTranslation(0,100,APP_BASELINE40_RUN_MS,2100,&p));CHECK(p.route_reference_mm_s==0.0f);
 for(j=0;j<7;j++){
  ChassisAutoPreparedTranslation bad=p;
  g_chassis_auto_active=0;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_used=0;
  Auto_LateralReset();
  if(j==0)bad.motion_mode=1;if(j==1)bad.base_rpm=99;if(j==2)bad.distance_mm=2099;
  if(j==3)bad.run_ms--;if(j==4)bad.profile.valid=0;if(j==5)bad.route_reference_mm_s=400;
  if(j==6)bad.profile.limits.speed++;
  CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));CHECK(!s_lateral_enabled);cases++;
 }
 for(j=0;j<2;j++){
  g_chassis_auto_active=0;g_chassis_auto_state=CHASSIS_AUTO_IDLE;s_used=0;Auto_LateralReset();
  g_debug_plan_id=j?DEBUG_PLAN_FORWARD210_40:DEBUG_PLAN_DEFAULT;s_motion_action_count=j?2U:1U;
  CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));CHECK(!s_lateral_enabled);cases++;
 }
 }

 {static const float errors[5]={4.0f,8.0f,10.0f,12.0f,16.0f};
  static const float expected[5]={3.0f,6.0f,7.5f,9.0f,12.0f};
  for(j=0;j<5;j++)for(sign=-1;sign<=1;sign+=2){
   setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;
   od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm+sign*errors[j];
   first=Auto_LateralTarget(&od,100U,1001U,&target_valid);
   CHECK(fabsf(first+sign*expected[j])<0.001f);
   CHECK(first*sign<0.0f);CHECK(fabsf(first)<=12.1f);
   CHECK(Auto_LateralTarget(&od,101U,1001U,&target_valid)==0.0f);
   CHECK(Auto_LateralTarget(&od,100U,1400U,&target_valid)==0.0f);cases++;
  }
  setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;
  od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm;
  CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);cases++;
 }

 /* Capacity-specific native C: every base 0..100, heading priority and integer remix. */
 for(b=0;b<=100;b++)for(h=-20;h<=20;h+=10)for(sign=-1;sign<=1;sign+=2){
  int32_t expected,room;
  CHECK(Chassis_Baseline40Budget((uint32_t)b,h*.1f,&c,&limit,&factor));
  expected=(int32_t)(12.1f/factor);if(expected>30)expected=30;
  room=10*b-(c<0?-c:c);if(expected>room)expected=room;
  room=50-(c<0?-c:c);if(expected>room)expected=room;if(expected<0)expected=0;
  CHECK(limit==expected);l=sign*limit;
  CHECK(Chassis_Baseline40Commands((uint32_t)b,c,l,cmd));
  CHECK((int32_t)(cmd[0].speed_raw+cmd[1].speed_raw+cmd[2].speed_raw+cmd[3].speed_raw)==40*b);
  CHECK((int32_t)(-cmd[0].speed_raw+cmd[1].speed_raw-cmd[2].speed_raw+cmd[3].speed_raw)==4*c);
  CHECK((int32_t)(cmd[0].speed_raw+cmd[1].speed_raw-cmd[2].speed_raw-cmd[3].speed_raw)==0);
  for(i=0;i<4;i++){CHECK(cmd[i].speed_raw<=10*(b+5));CHECK(cmd[i].address==i+1 && cmd[i].direction==g_chassis_forward_direction[i]);}
  cases++;
 }
 for(h=-20;h<=20;h+=20)for(l=-30;l<=30;l++){
  CHECK(Chassis_Baseline40Commands(100U,h,l,cmd));
  CHECK((int32_t)(cmd[0].speed_raw-cmd[1].speed_raw-cmd[2].speed_raw+cmd[3].speed_raw)==4*l);cases++;
 }
 memset(cmd,0xA5,sizeof(cmd));memcpy(saved,cmd,sizeof(saved));
 CHECK(!Chassis_Baseline40Commands(100U,0,31,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,0,-31,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,21,30,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Commands(100U,-21,-30,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 setup();od=input_od;od.path_heading_deg=od.pose.heading_deg=0.0f;od.pose.x_mm=od.origin_x_mm;
 od.pose.y_mm=od.origin_y_mm+20.0f;CHECK(fabsf(Auto_LateralTarget(&od,100U,1001U,&target_valid)+15.0f)<.0001f);
 od.pose.y_mm=od.origin_y_mm-20.0f;CHECK(fabsf(Auto_LateralTarget(&od,100U,1001U,&target_valid)-15.0f)<.0001f);
 for(sign=-1;sign<=1;sign+=2){
  setup();s_translation_base_rpm=100U;s_lateral_vy=sign*12.1f;s_lateral_tick_valid=1U;s_lateral_tick_ms=1000U;
  CHECK(Auto_LateralStepLocked(1100U,sign*12.1f,2.0f,&c,&l));CHECK(c==20 && l==sign*30);
  CHECK(Chassis_Baseline40Commands(100U,c,l,cmd));
  s_translation_base_rpm=2U;CHECK(Auto_LateralStepLocked(1101U,sign*12.1f,2.0f,&c,&l));CHECK(c==20 && l==0 && s_lateral_vy==0);
  s_translation_base_rpm=0U;CHECK(Auto_LateralStepLocked(1102U,sign*12.1f,0,&c,&l));CHECK(c==0 && l==0 && s_lateral_vy==0);cases++;
 }
 setup();s_lateral_vy=0;s_lateral_tick_valid=1U;s_lateral_tick_ms=1000U;
 CHECK(Auto_LateralStepLocked(1100U,12.1f,0,&c,&l));CHECK(s_lateral_vy==2.0f && l==4);cases++;

 /* Real PLAN8 Begin enables lateral; zero common error isolates actual FF. */
 ff_setup(-1,0,0);test_read_advance_ms=1;test_read_publish_now=1;ChassisAuto_CanStep();CHECK(clock_ms==1001 && ff_L()==4);cases++;
 ff_setup(-1.0f,0,0);ff_steady();CHECK(s_lateral_enabled==1U);CHECK(ff_L()==8);ff_positive_L=ff_L();
 CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw+s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==4000);
 CHECK((int32_t)(-s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==0);cases++;
 ff_setup(1.0f,0,0);ff_steady();CHECK(ff_L()==-8);ff_negative_abs_L=-ff_L();cases++;
 ff_setup(1.0f,0,179.5f);ff_steady();CHECK(ff_L()==-8);ff_cross_L=-ff_L();cases++;
 ff_setup(-1.0f,0,-179.5f);ff_steady();CHECK(ff_L()==8);cases++;
 /* Actual locked F changes after lock-free trigonometry: target uses F50. */
 ff_setup(-.5f,0,0);test_check_base_valid=1;test_check_base=50;ChassisAuto_CanStep();
 CHECK(s_translation_base_rpm==50 && ff_L()==2);ff_half_F_L=ff_L();CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw+s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==2000);cases++;
 ff_setup(-1,0,0);s_translation_base_rpm=0;ChassisAuto_CanStep();CHECK(s_lateral_vy==0 && s_lateral_applied_deci==0);cases++;
 /* Raw P22.5 plus opposite FF must stay positive: separate P clamp would flip it. */
 ff_setup(4.0f,30.0f,0);ChassisAuto_CanStep();CHECK(ff_L()==4 && s_lateral_vy==2.0f);ff_unified_clamp_L=ff_L();cases++;
 for(j=0;j<8;j++){
  ff_setup(-1,4,0);s_lateral_vy=3.0f;
  if(j==0)test_heading_ready=0;if(j==1)test_heading.healthy=0;if(j==2)test_heading.sample_ms=900;
  if(j==3)test_heading.angle_ms=900;if(j==4)test_heading.rate_ms=900;if(j==5)test_heading.valid_mask=2;
  if(j==6)test_heading.yaw_deg=11;if(j==7)test_heading.rate_dps=3000;
  ChassisAuto_CanStep();CHECK(s_lateral_vy==3.0f && ff_L()==7);CHECK(g_chassis_auto_active && !cut_cause);ff_HWT_fallback_L=ff_L();cases++;
 }
 for(j=0;j<3;j++){
  ff_setup(-1,0,0);test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=901;
  if(j==1){test_heading.sample_ms=1000;test_heading.angle_ms=901;test_heading.rate_ms=1000;}
  if(j==2){test_heading.sample_ms=1000;test_heading.angle_ms=1000;test_heading.rate_ms=901;}
  ChassisAuto_CanStep();CHECK(ff_L()==4);cases++;
 }
 /* Source valid outside, age100 at lock entry: FF0, P still3. */
 ff_setup(-1,4,0);clock_ms=999;input_od.pose.sample_ms=999;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=900;s_lateral_tick_ms=899;s_lateral_vy=3;test_enter_advance_ms=1;
 ChassisAuto_CanStep();CHECK(clock_ms==1000 && s_lateral_vy==3 && ff_L()==7);cases++;
 for(j=0;j<9;j++){
  ff_setup(-1,4,0);s_lateral_vy=3;
  if(j==0)input_od.anchor_valid=0;if(j==1)input_od.pose.valid=0;if(j==2)input_od.pose.reasons=1;
  if(j==3)input_od.pose.continuity_lost=1;if(j==4)input_od.pose.local_origin=1;
  if(j==5)input_od.reference_set_ms=99;if(j==6)input_od.pose.sample_ms=600;
  if(j==7)test_check_session=1;if(j==8){input_od.pose.sample_ms=601;test_enter_advance_ms=1;}
  ChassisAuto_CanStep();CHECK(s_lateral_vy==1.0f && ff_L()==2);CHECK(g_chassis_auto_active && !cut_cause);ff_ODinvalid_L=ff_L();cases++;
 }
 ff_setup(-1,0,0);input_od.pose.sample_ms=601;ChassisAuto_CanStep();CHECK(ff_L()==4);cases++;
 /* Nonfinite OD rejects both terms; nonfinite fresh yaw rejects only FF. */
 {union{uint32_t u;float f;}bad;bad.u=0x7FC00000U;
  ff_setup(-1,4,0);s_lateral_vy=3;test_heading.yaw_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==3 && ff_L()==7);cases++;
  ff_setup(-1,4,0);s_lateral_vy=3;input_od.heading_offset_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==1 && ff_L()==2);cases++;
  bad.u=0x7F800000U;ff_setup(-1,4,0);s_lateral_vy=3;test_heading.yaw_deg=bad.f;ChassisAuto_CanStep();CHECK(s_lateral_vy==3 && ff_L()==7);cases++;
 }
 /* Finite offsets/path checked outside before fmod/tan; real wrap freshness. */
 ff_setup(-1,0,0);clock_ms=5;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=0xFFFFFFFAU;input_od.pose.sample_ms=5;s_lateral_tick_ms=0xFFFFFFA1U;ChassisAuto_CanStep();CHECK(ff_L()==4);cases++;
 /* FF computed anew during wheels2..4 cannot mutate accepted batch/state. */
 ff_setup(-1,0,0);ChassisAuto_CanStep();memcpy(saved,s_commands[2],sizeof(saved));saved_tick=s_lateral_tick_ms;first=s_lateral_vy;
 for(k=1;k<4;k++){clock_ms+=10;input_od.pose.sample_ms=clock_ms;test_heading.sample_ms=test_heading.angle_ms=test_heading.rate_ms=clock_ms;test_heading.yaw_deg=1;ChassisAuto_CanStep();ChassisAuto_CanStep();CHECK(memcmp(saved,s_commands[2],sizeof(saved))==0 && s_lateral_vy==first && s_lateral_tick_ms==saved_tick);cases++;}
 for(j=0;j<3;j++){
  ff_setup(-1,0,0);ChassisAuto_CanStep();
  if(j==0)s_stop_due=1;if(j==1){s_stop_due=s_finish_requested=1;}if(j==2){ChassisAuto_SafetyUpdate(0);}
  ChassisAuto_CanStep();CHECK(s_lateral_vy==0 && !s_lateral_tick_valid && s_lateral_applied_deci==0);cases++;
 }
 test_heading_override=0;test_entry_origin=0;test_check_base_valid=test_check_session=test_read_advance_ms=0;
 {uint32_t r=observe_distance_control();if(r)return r;}
 /* Real pure C integer invariants across both signs and low-speed boundaries. */
 for(b=0;b<=100;b+=10)for(h=-20;h<=20;h+=10)for(sign=-1;sign<=1;sign+=2){
  CHECK(Chassis_Baseline40Budget((uint32_t)b,h*.1f,&c,&limit,&factor));
  l=limit*sign;CHECK(Chassis_Baseline40Commands((uint32_t)b,c,l,cmd));
  CHECK(cmd[0].speed_raw+cmd[1].speed_raw+cmd[2].speed_raw+cmd[3].speed_raw==40*b);
  for(i=0;i<4;i++){CHECK(cmd[i].speed_raw<=10*(b+5));CHECK(cmd[i].speed_raw<=3000);
   CHECK(cmd[i].direction==g_chassis_forward_direction[i] && cmd[i].address==i+1);}
  CHECK(l<=30 && l>=-30);cases++;
 }
 memset(cmd,0xA5,sizeof(cmd));memcpy(saved,cmd,sizeof(cmd));
 CHECK(!Chassis_Baseline40Commands(1U,20,1,cmd));CHECK(memcmp(cmd,saved,sizeof(cmd))==0);
 CHECK(!Chassis_Baseline40Budget(101U,0,&c,&limit,&factor));
 setup();od=input_od;od.pose.x_mm=od.origin_x_mm;od.pose.y_mm=od.origin_y_mm;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 CHECK(Auto_LateralTarget(&od,100U,1399U,&target_valid)>9.9f);
 CHECK(Auto_LateralTarget(&od,100U,1400U,&target_valid)==0.0f);
 CHECK(Auto_LateralTarget(&od,101U,1001U,&target_valid)==0.0f);
 od.pose.valid=0;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 od.pose.reasons=1;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 od.pose.continuity_lost=1;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 od.pose.local_origin=1;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 od.anchor_valid=0;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)==0.0f);od=input_od;
 od.pose.x_mm=2133.0f;CHECK(Auto_LateralTarget(&od,100U,1001U,&target_valid)<-9.9f);
 /* Real float state: first/same tick, retained residue, bounded long dt, wrap. */
 CHECK(Auto_LateralStepLocked(1000U,10.0f,0,&c,&l));CHECK(s_lateral_vy==0 && l==0);
 CHECK(Auto_LateralStepLocked(1000U,10.0f,0,&c,&l));CHECK(s_lateral_vy==0 && l==0);
 for(k=1;k<=100;k++)CHECK(Auto_LateralStepLocked(1000U+k,10.0f,0,&c,&l));
 CHECK(s_lateral_vy>1.99f && s_lateral_vy<2.01f && l>=4);
 s_lateral_vy=0.0f;CHECK(Auto_LateralStepLocked(2100U,10.0f,0,&c,&l));CHECK(s_lateral_vy==2.0f);
 s_lateral_vy=0.0f;s_lateral_tick_ms=0xFFFFFFFAU;
 CHECK(Auto_LateralStepLocked(5U,10.0f,0,&c,&l));CHECK(s_lateral_vy>.219f && s_lateral_vy<.221f);
 s_lateral_vy=10.0f;s_translation_base_rpm=1U;CHECK(Auto_LateralStepLocked(6U,10.0f,2.0f,&c,&l));
 CHECK(s_lateral_vy==0 && l==0 && c==10);
 s_lateral_vy=10.0f;s_translation_base_rpm=0U;CHECK(Auto_LateralStepLocked(7U,10.0f,0,&c,&l));CHECK(s_lateral_vy==0 && l==0);
 /* Actual CanStep: no send for unquantized state/heading variations. */
 setup();ChassisAuto_CanStep();CHECK(sends==0 && s_lateral_vy==0);
 clock_ms++;input_od.pose.sample_ms=clock_ms;s_heading_pending_rpm=.01f;ChassisAuto_CanStep();
 CHECK(sends==0 && generations==0 && s_lateral_vy>.019f);
 /* Start batch, then mutate each pending input while wheels2..4 go out. */
 setup();s_lateral_vy=9.0f;s_lateral_tick_valid=1U;s_lateral_tick_ms=900U;
 ChassisAuto_CanStep();CHECK(sends==1 && s_motor_index==1 && g_chassis_auto_state==CHASSIS_AUTO_START);
 memcpy(saved,s_commands[2],sizeof(saved));saved_tick=s_lateral_tick_ms;first=s_lateral_vy;
 CHECK(s_lateral_applied_deci>0 && last_source==3U);CHECK((int32_t)(saved[0].speed_raw-saved[1].speed_raw-saved[2].speed_raw+saved[3].speed_raw)==4*s_lateral_applied_deci);
 for(k=1;k<4;k++){
  clock_ms+=10U;input_od.pose.x_mm=2133.0f;input_od.pose.sample_ms=clock_ms;
  s_heading_pending_rpm=-1.2f;s_translation_base_rpm=99U;
  ChassisAuto_CanStep();CHECK(sends==k); /* inter-wheel monitor */
  CHECK(memcmp(saved,s_commands[2],sizeof(saved))==0 && s_lateral_vy==first && s_lateral_tick_ms==saved_tick);
  ChassisAuto_CanStep();CHECK(sends==k+1U);CHECK(sent_raw[k]==saved[k].speed_raw && sent_address[k]==k+1U);
  CHECK(memcmp(saved,s_commands[2],sizeof(saved))==0 && s_lateral_vy==first && s_lateral_tick_ms==saved_tick);
 }
 CHECK(g_chassis_auto_state==CHASSIS_AUTO_RUN);
 clock_ms+=10U;ChassisAuto_CanStep();CHECK(sends==5U && s_translation_applied_base==99U);
 CHECK((int32_t)(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw-s_commands[2][3].speed_raw)==0);CHECK((int32_t)(-s_commands[2][0].speed_raw+s_commands[2][1].speed_raw-s_commands[2][2].speed_raw+s_commands[2][3].speed_raw)==-48);
 CHECK(s_heading_applied_rpm< -1.19f && s_lateral_vy<first && s_lateral_tick_ms==clock_ms);
 CHECK(s_commands[2][0].speed_raw+s_commands[2][1].speed_raw+s_commands[2][2].speed_raw+s_commands[2][3].speed_raw==3960U);
 /* Actual stop/finish beats every partially sent batch. No new lateral mix. */
 for(j=0U;j<2U;j++)for(k=1U;k<=3U;k++){
  setup();s_lateral_vy=9.0f;s_lateral_tick_valid=1U;s_lateral_tick_ms=900U;ChassisAuto_CanStep();
  for(i=1;i<(int32_t)k;i++){ChassisAuto_CanStep();ChassisAuto_CanStep();}
  CHECK(sends==k);memcpy(saved,s_commands[2],sizeof(saved));saved_sends=sends;
  s_stop_due=1U;if(j)s_finish_requested=1U;
  clock_ms+=10U;input_od.pose.x_mm=2133.0f;ChassisAuto_CanStep();
  CHECK(sends==saved_sends && stops==1U && g_chassis_auto_state==CHASSIS_AUTO_STOP);
  CHECK(s_lateral_vy==0 && !s_lateral_tick_valid && s_lateral_applied_deci==0);
  CHECK(memcmp(saved,s_commands[2],sizeof(saved))==0);cases++;
 }
 /* Old/stale OD asks only lateral target0, keeps running HWT path. */
 setup();s_lateral_vy=9.0f;s_lateral_tick_valid=1U;s_lateral_tick_ms=900U;clock_ms=1400U;
 ChassisAuto_CanStep();CHECK(s_lateral_vy==7.0f && g_chassis_auto_active && !cut_cause);
 CHECK(lock_depth==0U);cases++;
 setup();s_lateral_vy=9.0f;s_lateral_tick_valid=1U;s_lateral_applied_deci=20;
 ChassisAuto_SafetyUpdate(0U);CHECK(!g_chassis_auto_active && g_chassis_auto_state==CHASSIS_AUTO_CANCELLED);
 CHECK(!s_lateral_enabled && s_lateral_vy==0 && s_lateral_applied_deci==0 && !s_lateral_tick_valid);saved_sends=sends;saved_tick=stops;ChassisAuto_CanStep();CHECK(sends==saved_sends && stops==saved_tick);
 CHECK(lock_depth==0U);cases++;return 0U;
}
