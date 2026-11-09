#include "lift.h"
#include "lift_config.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

const uint32_t g_lift_address = LIFT_CAN_ADDRESS;
volatile uint32_t g_lift_state = 0U;
volatile uint32_t g_lift_read_request = 0U;
volatile uint32_t g_lift_read_attempts = 0U;
volatile uint32_t g_lift_version_result = 0U;
volatile uint32_t g_lift_status_result = 0U;
CanRxMsg g_lift_version_reply;
CanRxMsg g_lift_status_reply;
static TickType_t s_wait_started = 0U;
static uint32_t s_wait_ms = 0U;

void Lift_Service(uint32_t power_ready)
{
    TickType_t now = xTaskGetTickCount();
    if (power_ready != 1U)
    {
        /* 保留本次原始回复方便观察，供电恢复再新建读取会话。 */
        g_lift_state = 0U;
        return;
    }
    if (g_lift_read_request != 0U)
    {
        g_lift_read_request = 0U;
        g_lift_state = 0U;
    }
    if (g_lift_state == 0U)
    {
        memset(&g_lift_version_reply, 0, sizeof(g_lift_version_reply));
        memset(&g_lift_status_reply, 0, sizeof(g_lift_status_reply));
        g_lift_version_result = 0U;
        g_lift_status_result = 0U;
        g_lift_read_attempts = 0U;
        s_wait_started = now;
        s_wait_ms = LIFT_POWER_SETTLE_MS;
        g_lift_state = 1U;
        return;
    }
    if (g_lift_state == 1U)
    {
        if ((TickType_t)(now - s_wait_started) < pdMS_TO_TICKS(s_wait_ms))
        {
            return;
        }
        g_lift_state = 2U;
    }
    if (g_lift_state == 2U)
    {
        /* 通用单帧查询支持非零地址，现有底盘运动地址保护不变。 */
        g_lift_read_attempts++;
        g_lift_version_result = CAN1_ReadVersion(LIFT_CAN_ADDRESS,
                                                 &g_lift_version_reply);
        if (g_lift_version_result == 1U)
        {
            g_lift_state = 3U;
        }
        else if (g_lift_read_attempts < LIFT_READ_MAX_ATTEMPTS)
        {
            s_wait_started = xTaskGetTickCount();
            s_wait_ms = LIFT_READ_RETRY_MS;
            g_lift_state = 1U;
        }
        else
        {
            g_lift_state = 5U;
        }
        return;
    }
    if (g_lift_state == 3U)
    {
        g_lift_status_result = CAN1_ReadStatus(LIFT_CAN_ADDRESS,
                                               &g_lift_status_reply);
        g_lift_state = (g_lift_status_result == 1U) ? 4U : 5U;
    }
}
