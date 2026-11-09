#include "arm_power_mode.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_arm_menu_power_state = 0U;
static TickType_t s_menu_started = 0U;

uint32_t ArmPower_IsMenuMode(void)
{
    return APP_ARM_MENU_POWER_TEST;
}

/* 仅监督任务调用：单次最多五分钟，关请求后才能重新开始。 */
uint32_t ArmPower_FilterRequest(uint32_t request)
{
    if (ArmPower_IsMenuMode() == 0U)
    {
        return request;
    }
    if (request != 1U)
    {
        g_arm_menu_power_state = 0U;
        return 0U;
    }
    if (g_arm_menu_power_state == 0U)
    {
        s_menu_started = xTaskGetTickCount();
        g_arm_menu_power_state = 1U;
    }
    if ((g_arm_menu_power_state == 2U) ||
        ((TickType_t)(xTaskGetTickCount() - s_menu_started) >=
         pdMS_TO_TICKS(ARM_MENU_POWER_TIMEOUT_MS)))
    {
        g_arm_menu_power_state = 2U;
        return 0U;
    }
    return 1U;
}
