#include "lift_home.h"
#include "lift.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_lift_home_request = 0U;
volatile uint32_t g_lift_home_state = 0U, g_lift_home_result = 0U;
volatile uint32_t g_lift_home_flags = 0U, g_lift_home_reported = 0U;
volatile uint32_t g_lift_home_start_result = 0U;
volatile uint32_t g_lift_home_abort_result = 0U, g_lift_home_stop_result = 0U;
static TickType_t s_started = 0U, s_polled = 0U;
static uint32_t s_sent = 0U, s_seen_running = 0U;
static CanRxMsg s_reply;

uint32_t LiftHome_IsBusy(void)
{
    return g_lift_home_state >= 1U && g_lift_home_state <= 7U;
}

static uint32_t LiftHome_AbortAndStop(void)
{
    const uint8_t stop[4] = {0xFEU, 0x98U, 0U, 0x6BU};
    g_lift_home_abort_result = CAN1_AbortLiftHome(&s_reply);
    g_lift_home_stop_result = CAN1_LiftExchange(stop, 4U, &s_reply);
    s_sent = 0U;
    g_lift_home_reported = 0U;
    return g_lift_home_abort_result == 1U && g_lift_home_stop_result == 1U;
}

static void LiftHome_Fail(uint32_t why)
{
    if (s_sent) (void)LiftHome_AbortAndStop();
    g_lift_home_reported = 0U;
    g_lift_home_result = why;
    g_lift_home_state = 9U;
}

void LiftHome_Service(uint32_t ready)
{
    uint32_t request = g_lift_home_request, result;
    TickType_t now = xTaskGetTickCount();
    uint8_t enable[5] = {0xF3U, 0xABU, 1U, 0U, 0x6BU};
    int32_t speed = 0;
    g_lift_home_request = 0U;
    if (!ready)
    {
        if (LiftHome_IsBusy() || request != 0U)
        { g_lift_home_state = 9U; g_lift_home_result = 6U; }
        g_lift_home_reported = 0U; s_sent = 0U;
        return;
    }
    /* 原停止请求在回零期间必须先退出回零，再发普通立即停止。 */
    if (request == 2U || ((LiftHome_IsBusy() || g_lift_home_state == 9U) && g_lift_motion_request == 3U))
    {
        g_lift_motion_request = 0U;
        result = LiftHome_AbortAndStop();
        g_lift_home_result = result ? 2U : 4U;
        g_lift_home_state = result ? 10U : 9U;
        return;
    }
    if (g_lift_home_state == 9U)
    {
        if (g_lift_motion_request == 1U || g_lift_motion_request == 2U)
        { g_lift_motion_request = 0U; g_lift_motion_result = 6U; }
        return;
    }
    if (request == 1U && !LiftHome_IsBusy())
    {
        if (g_lift_motion_request != 0U || g_lift_motion_state == 9U ||
            (g_lift_motion_state >= 1U && g_lift_motion_state <= 6U))
        { LiftHome_Fail(6U); return; }
        g_lift_home_reported = 0U; g_lift_home_result = 0U;
        g_lift_home_start_result = 0U; g_lift_home_abort_result = 0U;
        g_lift_home_stop_result = 0U; s_sent = 0U; s_seen_running = 0U;
        g_lift_home_state = 1U; s_started = now;
    }
    if (!LiftHome_IsBusy()) return;
    /* 忙时正向请求不排队，不重置超时；不放行正常3000RPM小步。 */
    if (g_lift_motion_request != 0U)
    { g_lift_motion_request = 0U; g_lift_motion_result = 6U; }
    if ((TickType_t)(now - s_started) >= pdMS_TO_TICKS(s_sent ? 11000U : 2500U))
    { LiftHome_Fail(3U); return; }
    switch (g_lift_home_state)
    {
    case 1U:
        result = CAN1_ReadMotorOptions(5U, &s_reply);
        if (result != 1U || (s_reply.Data[2] & 2U) == 0U)
        { LiftHome_Fail(4U); break; }
        g_lift_home_state = 2U;
        break;
    case 2U:
        if (Lift_ReadHomeParametersNow() != 1U) { LiftHome_Fail(4U); break; }
        /* 仅允许本机刚实测的低速参数，不擅自改阈值或上电自动回零。 */
        if (g_lift_home_params.direction != 1U || g_lift_home_params.rpm != 30U ||
            g_lift_home_params.timeout_ms != 10000U || g_lift_home_params.detect_rpm != 300U ||
            g_lift_home_params.detect_current_ma != 800U || g_lift_home_params.detect_time_ms != 60U ||
            g_lift_home_params.auto_home != 0U)
        { LiftHome_Fail(7U); break; }
        g_lift_home_state = 3U;
        break;
    case 3U:
        result = CAN1_ReadStatus(5U, &s_reply);
        if (result != 1U) { LiftHome_Fail(4U); break; }
        if (s_reply.Data[1] & 0x0CU) { LiftHome_Fail(5U); break; }
        if (!(s_reply.Data[1] & 1U) && CAN1_LiftExchange(enable, 5U, &s_reply) != 1U)
        { LiftHome_Fail(4U); break; }
        g_lift_home_state = 4U;
        break;
    case 4U:
        result = CAN1_ReadLiftHomeStatus(&s_reply);
        if (result != 1U) { LiftHome_Fail(4U); break; }
        g_lift_home_flags = s_reply.Data[1];
        if ((g_lift_home_flags & 3U) != 3U || (g_lift_home_flags & 0x3CU))
        { LiftHome_Fail(5U); break; }
        g_lift_home_state = 5U;
        break;
    case 5U:
        /* 即使命令应答丢失，也按可能已启动处理，失败时尝试中断。 */
        s_sent = 1U; s_started = xTaskGetTickCount();
        g_lift_home_start_result = CAN1_StartLiftHome(&s_reply);
        if (g_lift_home_start_result != 1U) { LiftHome_Fail(4U); break; }
        g_lift_home_state = 6U; s_polled = xTaskGetTickCount();
        break;
    case 6U:
        if ((TickType_t)(now - s_polled) < pdMS_TO_TICKS(50U)) break;
        result = CAN1_ReadLiftHomeStatus(&s_reply);
        if (result != 1U) { LiftHome_Fail(4U); break; }
        g_lift_home_flags = s_reply.Data[1]; s_polled = xTaskGetTickCount();
        if ((g_lift_home_flags & 3U) != 3U || (g_lift_home_flags & 0x38U))
        { LiftHome_Fail(5U); break; }
        if (g_lift_home_flags & 4U) s_seen_running = 1U;
        /* 上电默认空闲位绝不能被当成此次回零成功。 */
        if (!(g_lift_home_flags & 0x0CU) && (s_seen_running || g_can_lift_home_done_notice))
            g_lift_home_state = 7U;
        break;
    case 7U:
        result = CAN1_ReadSpeed(5U, &s_reply, &speed);
        if (result != 1U) { LiftHome_Fail(4U); break; }
        if (speed == 0)
        {
            g_lift_home_result = 1U; g_lift_home_state = 8U;
            g_lift_home_reported = 1U; s_sent = 0U;
        }
        else { g_lift_home_state = 6U; s_polled = xTaskGetTickCount(); }
        break;
    default: break;
    }
}
