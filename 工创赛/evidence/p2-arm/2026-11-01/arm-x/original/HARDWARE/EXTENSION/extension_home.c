#include "extension_home.h"
#include "extension.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_extension_home_request = 0U;
volatile uint32_t g_extension_home_state = 0U, g_extension_home_result = 0U;
volatile uint32_t g_extension_home_flags = 0U, g_extension_home_reported = 0U;
volatile uint32_t g_extension_home_start_result = 0U;
volatile uint32_t g_extension_home_abort_result = 0U, g_extension_home_stop_result = 0U;
volatile uint32_t g_extension_home_elapsed_ms = 0U;
static TickType_t s_started, s_polled, s_last_service;
static uint32_t s_may_be_running = 0U, s_seen_running = 0U;
static CanRxMsg s_reply;

uint32_t ExtensionHome_IsBusy(void)
{
    return ((g_extension_home_state >= 1U && g_extension_home_state <= 7U) ||
            s_may_be_running != 0U) ? 1U : 0U;
}

static uint32_t ExtensionHome_AbortAndStop(void)
{
    const uint8_t stop[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    int32_t speed = 0;
    uint32_t verified_idle = 0U;
    g_extension_home_reported = 0U;
    g_extension_home_abort_result = CAN1_AbortExtensionHome(&s_reply);
    g_extension_home_stop_result = CAN1_ExtensionExchange(stop, 4U, &s_reply);
    if (g_extension_home_abort_result != 1U && g_extension_home_stop_result == 1U)
    {
        /* 已自行结束时9C可能返回条件不满足；再以新鲜闲置及静止反馈核实。 */
        if (CAN1_ReadExtensionHomeStatus(&s_reply) == 1U &&
            (s_reply.Data[1] & 3U) == 3U && !(s_reply.Data[1] & 4U) &&
            CAN1_ReadSpeed(6U, &s_reply, &speed) == 1U && speed == 0)
            verified_idle = 1U;
    }
    if (g_extension_home_stop_result == 1U &&
        (g_extension_home_abort_result == 1U || verified_idle))
    {
        s_may_be_running = 0U;
        CAN1_ExtensionHomeSessionEnd();
        return 1U;
    }
    /* 启动后若退出或停止未确认，继续占用互锁，等待显式中止重试/外部停机。 */
    return 0U;
}

static void ExtensionHome_Fail(uint32_t reason)
{
    if (s_may_be_running) (void)ExtensionHome_AbortAndStop();
    else CAN1_ExtensionHomeSessionEnd();
    g_extension_home_reported = 0U;
    g_extension_home_result = reason;
    g_extension_home_state = 9U;
}

void ExtensionHome_Service(uint32_t ready, uint32_t other_busy)
{
    uint32_t request = g_extension_home_request, result;
    TickType_t now = xTaskGetTickCount();
    const uint8_t enable[5] = {0xF3U, 0xABU, 1U, 0U, 0x6BU};
    int32_t speed = 0;
    g_extension_home_request = 0U;

    if (request == 2U || ((ExtensionHome_IsBusy() || g_extension_home_state == 9U) &&
                         g_extension_motion_request == 3U))
    {
        g_extension_motion_request = 0U;
        result = ExtensionHome_AbortAndStop();
        g_extension_home_result = result ? 2U : 4U;
        g_extension_home_state = result ? 10U : 9U;
        return;
    }
    if (ready != 1U || other_busy != 0U)
    {
        if (g_extension_home_state == 9U)
        { g_extension_home_reported = 0U; return; }
        if (ExtensionHome_IsBusy() || request != 0U) ExtensionHome_Fail(6U);
        if (ready != 1U) g_extension_home_reported = 0U;
        return;
    }
    if (g_extension_home_state == 9U)
    {
        if (g_extension_motion_request == 1U || g_extension_motion_request == 2U)
        { g_extension_motion_request = 0U; g_extension_motion_result = 6U; }
        return;
    }
    if (request == 1U && !ExtensionHome_IsBusy())
    {
        if (g_extension_motion_request != 0U || Extension_MotionIsBusy() ||
            g_extension_motion_state == 9U ||
            (g_extension_state >= 1U && g_extension_state <= 3U))
        { ExtensionHome_Fail(6U); return; }
        g_extension_home_reported = 0U; g_extension_home_result = 0U;
        g_extension_home_start_result = 0U; g_extension_home_abort_result = 0U;
        g_extension_home_stop_result = 0U; g_extension_home_elapsed_ms = 0U;
        s_may_be_running = 0U; s_seen_running = 0U;
        g_extension_home_state = 1U;
        s_started = now; s_last_service = now;
    }
    if (!ExtensionHome_IsBusy()) return;
    /* 忙时请求不排队，不改超时；普通小步不得与回零并行。 */
    if (g_extension_motion_request != 0U)
    { g_extension_motion_request = 0U; g_extension_motion_result = 6U; }
    g_extension_home_elapsed_ms = (uint32_t)((now - s_started) * portTICK_PERIOD_MS);
    if ((TickType_t)(now - s_started) >= pdMS_TO_TICKS(s_may_be_running ?
            EXTENSION_HOME_HOST_TIMEOUT_MS : EXTENSION_HOME_PREP_TIMEOUT_MS))
    { ExtensionHome_Fail(3U); return; }
    if ((TickType_t)(now - s_last_service) > pdMS_TO_TICKS(EXTENSION_HOME_SERVICE_GAP_MS))
    { ExtensionHome_Fail(8U); return; }
    s_last_service = now;

    switch (g_extension_home_state)
    {
    case 1U:
        result = CAN1_ReadMotorOptions(6U, &s_reply);
        if (result != 1U || (s_reply.Data[2] & 2U) == 0U)
        { ExtensionHome_Fail(4U); break; }
        g_extension_home_state = 2U;
        break;
    case 2U:
        /* 复用已验证的只读解码；由本回零状态机串行调用，不与外部读参并行。 */
        g_extension_home_read_request = 1U;
        Extension_HomeReadService(1U, 0U);
        if (g_extension_home_read_result != 1U) { ExtensionHome_Fail(4U); break; }
        if (g_extension_home_params.direction != EXTENSION_HOME_DIRECTION ||
            g_extension_home_params.rpm != EXTENSION_HOME_RPM ||
            g_extension_home_params.timeout_ms != EXTENSION_HOME_DRIVER_TIMEOUT_MS ||
            g_extension_home_params.detect_rpm != EXTENSION_HOME_DETECT_RPM ||
            g_extension_home_params.detect_current_ma != EXTENSION_HOME_DETECT_CURRENT_MA ||
            g_extension_home_params.detect_time_ms != EXTENSION_HOME_DETECT_TIME_MS ||
            g_extension_home_params.auto_home != 0U)
        { ExtensionHome_Fail(7U); break; }
        /* mode为菜单值；本次启动命令显式选02无限位碰撞，不写菜单或默认参数。 */
        g_extension_home_state = 3U;
        break;
    case 3U:
        result = CAN1_ReadStatus(6U, &s_reply);
        if (result != 1U) { ExtensionHome_Fail(4U); break; }
        if (s_reply.Data[1] & 0x0CU) { ExtensionHome_Fail(5U); break; }
        if (!(s_reply.Data[1] & 1U) && CAN1_ExtensionExchange(enable, 5U, &s_reply) != 1U)
        { ExtensionHome_Fail(4U); break; }
        g_extension_home_state = 4U;
        break;
    case 4U:
        result = CAN1_ReadExtensionHomeStatus(&s_reply);
        if (result != 1U) { ExtensionHome_Fail(4U); break; }
        g_extension_home_flags = s_reply.Data[1];
        if ((g_extension_home_flags & 3U) != 3U || (g_extension_home_flags & 0x3CU))
        {
            if (g_extension_home_flags & 4U) s_may_be_running = 1U;
            ExtensionHome_Fail(5U); break;
        }
        result = CAN1_ReadSpeed(6U, &s_reply, &speed);
        if (result != 1U) { ExtensionHome_Fail(4U); break; }
        if (speed != 0)
        { s_may_be_running = 1U; ExtensionHome_Fail(5U); break; }
        g_extension_home_state = 5U;
        break;
    case 5U:
        /* 应答丢失也按可能已启动处理，失败时退出回零并停车。 */
        s_may_be_running = 1U; s_started = xTaskGetTickCount();
        g_extension_home_start_result = CAN1_StartExtensionHome(&s_reply);
        if (g_extension_home_start_result != 1U) { ExtensionHome_Fail(4U); break; }
        g_extension_home_state = 6U; s_polled = xTaskGetTickCount();
        break;
    case 6U:
        if ((TickType_t)(now - s_polled) < pdMS_TO_TICKS(EXTENSION_HOME_POLL_MS)) break;
        result = CAN1_ReadExtensionHomeStatus(&s_reply);
        if (result != 1U) { ExtensionHome_Fail(4U); break; }
        g_extension_home_flags = s_reply.Data[1]; s_polled = xTaskGetTickCount();
        if ((g_extension_home_flags & 3U) != 3U || (g_extension_home_flags & 0x38U))
        { ExtensionHome_Fail(5U); break; }
        if (g_extension_home_flags & 4U) s_seen_running = 1U;
        /* 默认空闲位不算成功，必须见本轮运行或启动后的完成通知。 */
        if (!(g_extension_home_flags & 0x0CU) &&
            (s_seen_running || g_can_extension_home_done_notice))
            g_extension_home_state = 7U;
        break;
    case 7U:
        result = CAN1_ReadSpeed(6U, &s_reply, &speed);
        if (result != 1U) { ExtensionHome_Fail(4U); break; }
        if (speed == 0)
        {
            g_extension_home_result = 1U; g_extension_home_state = 8U;
            g_extension_home_reported = 1U; s_may_be_running = 0U;
            CAN1_ExtensionHomeSessionEnd();
        }
        else { g_extension_home_state = 6U; s_polled = xTaskGetTickCount(); }
        break;
    default: break;
    }
}
