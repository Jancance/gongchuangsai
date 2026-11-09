#include "extension.h"
#include "can.h"
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

void Extension_Service(uint32_t ready, uint32_t lift_busy)
{
    uint32_t request = g_extension_read_request, result;
    g_extension_read_request = 0U;
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
        break;
    default: return;
    }
    if (result == 1U) g_extension_state++;
    else { g_extension_error = result; g_extension_state = 5U; }
}
