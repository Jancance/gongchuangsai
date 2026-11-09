#include "extension.h"
#include "can.h"
#include "extension_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

volatile uint32_t g_extension_read_request = 0U;
volatile uint32_t g_extension_state = 0U;
volatile uint32_t g_extension_version_result = 0U;
volatile uint32_t g_extension_status_result = 0U;
volatile uint32_t g_extension_options_result = 0U;
volatile uint32_t g_extension_error = 0U;
CanRxMsg g_extension_version_reply;
CanRxMsg g_extension_status_reply;
CanRxMsg g_extension_options_reply;
volatile uint32_t g_extension_fw_type = 0xFFFFFFFFU;

void Extension_Service(uint32_t ready, uint32_t lift_busy)
{
    uint32_t request = g_extension_read_request, result;
    g_extension_read_request = 0U;
    /* 运动请求或动作占用时不插入身份读取，不排队。 */
    if (ExtensionHome_IsBusy() || Extension_MotionIsBusy() || g_extension_motion_request != 0U) return;
    if (ready != 1U || lift_busy != 0U)
    {
        if (request != 0U || (g_extension_state >= 1U && g_extension_state <= 3U))
        { g_extension_state = 5U; g_extension_error = 7U; }
        else if (ready != 1U) g_extension_state = 0U;
        return;
    }
    /* 忙时重复请求不排队、不重启；失败不自动重试。 */
    if (request == 1U && !(g_extension_state >= 1U && g_extension_state <= 3U))
    {
        g_extension_version_result = 0U;
        g_extension_status_result = 0U;
        g_extension_options_result = 0U;
        g_extension_error = 0U;
        memset(&g_extension_version_reply, 0, sizeof(g_extension_version_reply));
        memset(&g_extension_status_reply, 0, sizeof(g_extension_status_reply));
        memset(&g_extension_options_reply, 0, sizeof(g_extension_options_reply));
        g_extension_state = 1U;
    }
    switch (g_extension_state)
    {
    case 1U:
        result = CAN1_ReadVersion(6U, &g_extension_version_reply);
        g_extension_version_result = result;
        break;
    case 2U:
        result = CAN1_ReadStatus(6U, &g_extension_status_reply);
        g_extension_status_result = result;
        break;
    case 3U:
        result = CAN1_ReadMotorOptions(6U, &g_extension_options_reply);
        g_extension_options_result = result;
        if (result == 1U) g_extension_fw_type = (g_extension_options_reply.Data[2] >> 1) & 1U;
        break;
    default: return;
    }
    if (result == 1U) g_extension_state++;
    else { g_extension_error = result; g_extension_state = 5U; }
}

/* 单次低速相对小步，仅由原唯一CAN任务调用；上电不自动动作。 */
volatile uint32_t g_extension_motion_request = 0U;
volatile uint32_t g_extension_use_mm = EXTENSION_DEFAULT_USE_MM;
volatile float g_extension_distance_mm = EXTENSION_DEFAULT_MM;
volatile uint32_t g_extension_target_pulses = 0U;
volatile float g_extension_target_mm = 0.0f;
volatile uint32_t g_extension_motion_state = 0U;
volatile uint32_t g_extension_motion_result = 0U;
volatile uint32_t g_extension_enable_result = 0U;
volatile uint32_t g_extension_move_result = 0U;
volatile uint32_t g_extension_stop_result = 0U;
volatile uint32_t g_extension_motion_options_result = 0U;
volatile uint32_t g_extension_flags = 0U;
volatile int32_t g_extension_speed_rpm = 0;
volatile int32_t g_extension_speed_deci_rpm = 0;
volatile uint32_t g_extension_x_legacy_mstep = EXTENSION_X_LEGACY_MSTEP;
volatile int64_t g_extension_delta_raw = 0;
volatile uint32_t g_extension_motion_elapsed_ms = 0U;
volatile uint32_t g_extension_stall_pending = 0U;
volatile uint32_t g_extension_stall_elapsed_ms = 0U;
volatile uint32_t g_extension_stall_max_ms = 0U;
volatile uint32_t g_extension_stall_events = 0U;
volatile uint32_t g_extension_motion_direction = 0U;
CanRxMsg g_extension_motion_reply;
CanRxMsg g_extension_control_reply;
static TickType_t s_extension_started, s_extension_poll, s_extension_last_service;
static uint32_t s_extension_phase = 0U;
static int64_t s_extension_before_position = 0;
static TickType_t s_extension_stall_since;
static uint32_t s_extension_stall_active = 0U;
static uint32_t s_extension_target_pulses = EXTENSION_JOG_PULSES;

uint32_t Extension_MotionIsBusy(void)
{
    return (g_extension_motion_state >= 1U && g_extension_motion_state <= 6U) ? 1U : 0U;
}

static uint32_t Extension_StopCommand(void)
{
    const uint8_t command[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    g_extension_stop_result = CAN1_ExtensionExchange(command, 4U, &g_extension_control_reply);
    s_extension_stall_active = 0U; g_extension_stall_pending = 0U;
    return g_extension_stop_result;
}

static void Extension_MotionFault(uint32_t reason)
{
    /* 停止失败也保留失败锁存，不把发送尝试当作实际停车。 */
    (void)Extension_StopCommand();
    g_extension_motion_result = reason;
    g_extension_motion_state = 9U;
}

static int64_t Extension_SignedPosition(uint8_t sign, uint32_t raw)
{
    return sign ? -(int64_t)raw : (int64_t)raw;
}

/* 接单时换算并锁存；先检查浮点范围再转整数，拒绝NaN/无穷/越界。 */
static uint32_t Extension_LatchDistance(void)
{
    uint32_t units = g_extension_use_mm, pulses = EXTENSION_JOG_PULSES;
    float distance = g_extension_distance_mm, requested;
    if (!(EXTENSION_PULSES_PER_MM > 0.0f)) return 0U;
    if (units == 1U)
    {
        if (!(distance > 0.0f &&
              distance <= (float)EXTENSION_JOG_PULSES / EXTENSION_PULSES_PER_MM)) return 0U;
        requested = distance * EXTENSION_PULSES_PER_MM;
        if (!(requested >= 0.5f && requested <= (float)EXTENSION_JOG_PULSES)) return 0U;
        pulses = (uint32_t)(requested + 0.5f);
    }
    else if (units != 0U) return 0U;
    if (pulses == 0U || pulses > EXTENSION_JOG_PULSES) return 0U;
    s_extension_target_pulses = pulses;
    g_extension_target_pulses = pulses;
    g_extension_target_mm = (float)pulses / EXTENSION_PULSES_PER_MM;
    return 1U;
}

static uint32_t Extension_BuildMove(uint8_t out[15])
{
    return CAN1_ArmXBuildMove(6U, (uint8_t)g_extension_motion_direction, s_extension_target_pulses,
                              EXTENSION_X_SPEED_DECI_RPM, EXTENSION_X_ACCEL_RPM_S,
                              EXTENSION_X_DECEL_RPM_S, out);
}

void Extension_MotionService(uint32_t ready, uint32_t other_busy)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t request = g_extension_motion_request, result;
    uint8_t command[15], sign = 0U;
    uint32_t raw = 0U;
    int32_t speed = 0;
    int64_t delta;
    static uint8_t s_direction = 0U;
    g_extension_motion_request = 0U;
    if (ExtensionHome_IsBusy())
    {
        if (request == 3U) { g_extension_home_request = 2U; return; }
        other_busy = 1U;
    }

    if (request == 3U)
    {
        result = Extension_StopCommand();
        g_extension_motion_result = (result == 1U) ? 2U : 4U;
        g_extension_motion_state = (result == 1U) ? 8U : 9U;
        return;
    }
    if (ready != 1U || other_busy != 0U)
    {
        if (Extension_MotionIsBusy()) Extension_MotionFault(6U);
        else if (request != 0U)
        { g_extension_motion_result = 6U; g_extension_motion_state = 9U; }
        return;
    }
    /* 失败不自动重试；须显式停止成功后再接下一次请求。 */
    if (g_extension_motion_state == 9U) return;
    if (!Extension_MotionIsBusy() && (request == 1U || request == 2U))
    {
        if (Extension_LatchDistance() != 1U)
        {
            g_extension_target_pulses = 0U; g_extension_target_mm = 0.0f;
            g_extension_motion_result = 8U; g_extension_motion_state = 9U;
            return; /* 未发运动命令，不自动修正非法距离。 */
        }
        s_direction = (request == 1U) ? 0U : 1U;
        g_extension_motion_direction = s_direction;
        g_extension_motion_state = 1U;
        g_extension_motion_result = 0U;
        g_extension_enable_result = 0U; g_extension_move_result = 0U;
        g_extension_stop_result = 0U; g_extension_motion_options_result = 0U;
        g_extension_delta_raw = 0; g_extension_speed_rpm = 0;
        g_extension_motion_elapsed_ms = 0U;
        s_extension_stall_active = 0U; g_extension_stall_pending = 0U;
        g_extension_stall_elapsed_ms = 0U; g_extension_stall_max_ms = 0U;
        g_extension_stall_events = 0U;
        s_extension_started = now; s_extension_last_service = now;
        s_extension_phase = 0U;
    }
    if (!Extension_MotionIsBusy()) return;
    if (s_extension_stall_active)
    {
        g_extension_stall_elapsed_ms = (uint32_t)((now - s_extension_stall_since) * portTICK_PERIOD_MS);
        if (g_extension_stall_elapsed_ms > g_extension_stall_max_ms)
            g_extension_stall_max_ms = g_extension_stall_elapsed_ms;
    }
    /* 忙时新启动请求已消费，不排队、不延长超时；方向用内部锁存值。 */
    g_extension_motion_direction = s_direction;
    g_extension_motion_elapsed_ms = (uint32_t)((now - s_extension_started) * portTICK_PERIOD_MS);
    if ((TickType_t)(now - s_extension_started) >= pdMS_TO_TICKS(EXTENSION_MOVE_TIMEOUT_MS))
    { Extension_MotionFault(3U); return; }
    if ((TickType_t)(now - s_extension_last_service) > pdMS_TO_TICKS(EXTENSION_SERVICE_GAP_MS))
    { Extension_MotionFault(7U); return; }
    s_extension_last_service = now;
    switch (g_extension_motion_state)
    {
    case 1U:
        g_extension_motion_options_result = CAN1_ReadMotorOptions(EXTENSION_CAN_ADDRESS,
                                                                 &g_extension_motion_reply);
        if (g_extension_motion_options_result == 1U) g_extension_fw_type = (g_extension_motion_reply.Data[2] >> 1) & 1U;
        if (g_extension_motion_options_result != 1U ||
            (g_extension_motion_reply.Data[2] & 0x02U) != 0U ||
            !CAN1_ArmXConfigure(6U, g_extension_motion_reply.Data[2], g_extension_x_legacy_mstep))
        { g_extension_motion_result = 4U; g_extension_motion_state = 9U; }
        else g_extension_motion_state = 2U;
        break;
    case 2U:
        result = CAN1_ReadStatus(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply);
        if (result != 1U) { Extension_MotionFault(4U); break; }
        g_extension_flags = g_extension_motion_reply.Data[1];
        if ((g_extension_flags & 0x0CU) != 0U)
        { Extension_MotionFault(5U); break; }
        g_extension_motion_state = (g_extension_flags & 1U) ? 4U : 3U;
        s_extension_phase = 0U;
        break;
    case 3U:
        command[0] = 0xF3U; command[1] = 0xABU; command[2] = 1U;
        command[3] = 0U; command[4] = 0x6BU;
        g_extension_enable_result = CAN1_ExtensionExchange(command, 5U, &g_extension_control_reply);
        if (g_extension_enable_result != 1U) { Extension_MotionFault(4U); break; }
        g_extension_motion_state = 4U; s_extension_phase = 0U;
        break;
    case 4U:
        if (s_extension_phase == 0U)
        {
            result = CAN1_ReadSpeed(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply, &speed);
            if (result != 1U) { Extension_MotionFault(4U); break; }
            g_extension_speed_deci_rpm = speed; g_extension_speed_rpm = speed / 10;
            if (speed != 0) { Extension_MotionFault(5U); break; }
            s_extension_phase = 1U;
        }
        else
        {
            result = CAN1_ReadPosition(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply, &sign, &raw);
            if (result != 1U) { Extension_MotionFault(4U); break; }
            s_extension_before_position = Extension_SignedPosition(sign, raw);
            g_extension_motion_state = 5U;
        }
        break;
    case 5U:
        if (!Extension_BuildMove(command)) { Extension_MotionFault(4U); break; }
        g_extension_move_result = CAN1_ExtensionExchange(command, 15U, &g_extension_control_reply);
        if (g_extension_move_result != 1U) { Extension_MotionFault(4U); break; }
        g_extension_motion_state = 6U; s_extension_phase = 0U;
        s_extension_poll = xTaskGetTickCount();
        break;
    case 6U:
        if ((TickType_t)(now - s_extension_poll) < pdMS_TO_TICKS(EXTENSION_POLL_MS)) break;
        if (s_extension_phase == 0U)
        {
            result = CAN1_ReadStatus(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply);
            if (result != 1U) { Extension_MotionFault(4U); break; }
            g_extension_flags = g_extension_motion_reply.Data[1];
            /* 保护位/失能立即停；0x04单次低速大电流标志不冒充已持续保护。 */
            if ((g_extension_flags & 0x08U) != 0U || !(g_extension_flags & 1U))
            { Extension_MotionFault(5U); break; }
            if (g_extension_flags & 0x04U)
            {
                if (!s_extension_stall_active)
                {
                    s_extension_stall_active = 1U; s_extension_stall_since = now;
                    g_extension_stall_pending = 1U; g_extension_stall_elapsed_ms = 0U;
                    g_extension_stall_events++;
                }
                /* 每次新鲜状态复核；持续到界仍停，整体10秒和服务超时不延长。 */
                if ((TickType_t)(now - s_extension_stall_since) >= pdMS_TO_TICKS(EXTENSION_STALL_CONFIRM_MS))
                { Extension_MotionFault(5U); break; }
            }
            else
            {
                s_extension_stall_active = 0U; g_extension_stall_pending = 0U;
                g_extension_stall_elapsed_ms = 0U;
            }
            s_extension_phase = 1U;
        }
        else if (s_extension_phase == 1U)
        {
            result = CAN1_ReadPosition(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply, &sign, &raw);
            if (result != 1U) { Extension_MotionFault(4U); break; }
            g_extension_delta_raw = Extension_SignedPosition(sign, raw) - s_extension_before_position;
            s_extension_phase = 2U;
        }
        else
        {
            result = CAN1_ReadSpeed(EXTENSION_CAN_ADDRESS, &g_extension_motion_reply, &speed);
            if (result != 1U) { Extension_MotionFault(4U); break; }
            g_extension_speed_deci_rpm = speed; g_extension_speed_rpm = speed / 10;
            delta = g_extension_delta_raw;
            if ((g_extension_flags & 2U) && !(g_extension_flags & 0x0CU) &&
                speed == 0 && (delta >= 1 || delta <= -1))
            {
                /* 确认包不足以完成；还需位置变化、到位及静止反馈。保持锁轴。 */
                g_extension_motion_result = 1U; g_extension_motion_state = 7U;
            }
            s_extension_phase = 0U; s_extension_poll = now;
        }
        break;
    default: break;
    }
}

/* 缩回端机械限位为原点候选；本阶段只读本机参数，绝不触发回零。 */
volatile uint32_t g_extension_home_read_request = 0U;
volatile uint32_t g_extension_home_read_result = 0U;
ExtensionHomeParameters g_extension_home_params;
uint8_t g_extension_home_raw[17];
CanRxMsg g_extension_home_replies[3];

static uint32_t Extension_HomeRead16(const uint8_t *p)
{
    return ((uint32_t)p[0] << 8) | p[1];
}

void Extension_HomeReadService(uint32_t ready, uint32_t other_busy)
{
    uint32_t request = g_extension_home_read_request;
    uint8_t *p = g_extension_home_raw;
    ExtensionHomeParameters decoded;
    g_extension_home_read_request = 0U;
    if (request == 0U)
    {
        if (ready != 1U && g_extension_home_read_result == 1U)
        {
            g_extension_home_read_result = 0U;
            memset(&g_extension_home_params, 0, sizeof(g_extension_home_params));
        }
        return;
    }
    g_extension_home_read_result = 0U;
    memset(&g_extension_home_params, 0, sizeof(g_extension_home_params));
    memset(g_extension_home_raw, 0, sizeof(g_extension_home_raw));
    memset(g_extension_home_replies, 0, sizeof(g_extension_home_replies));
    if (request != 1U || ready != 1U || other_busy != 0U ||
        Extension_MotionIsBusy() || g_extension_motion_request != 0U ||
        (g_extension_state >= 1U && g_extension_state <= 3U))
    {
        g_extension_home_read_result = 7U;
        return;
    }
    g_extension_home_read_result = CAN1_ReadExtensionHomeParameters(p, g_extension_home_replies);
    if (g_extension_home_read_result != 1U) return;
    if (p[0] != 0x22U || p[16] != 0x6BU || p[1] > 5U || p[2] > 1U || p[15] > 1U ||
        Extension_HomeRead16(&p[3]) > 3000U || Extension_HomeRead16(&p[9]) > 3000U ||
        Extension_HomeRead16(&p[11]) > 5000U)
    {
        /* 上限只作数据解释合理性检查，不是28机械安全参数。 */
        g_extension_home_read_result = 6U;
        return;
    }
    decoded.mode = p[1]; decoded.direction = p[2];
    decoded.rpm = Extension_HomeRead16(&p[3]);
    decoded.timeout_ms = ((uint32_t)p[5] << 24) | ((uint32_t)p[6] << 16) |
                         ((uint32_t)p[7] << 8) | p[8];
    decoded.detect_rpm = Extension_HomeRead16(&p[9]);
    decoded.detect_current_ma = Extension_HomeRead16(&p[11]);
    decoded.detect_time_ms = Extension_HomeRead16(&p[13]);
    decoded.auto_home = p[15];
    g_extension_home_params = decoded;
}
