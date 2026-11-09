#include "arm_power_mode.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t g_arm_menu_power_state = 0U;
#if !APP_ARM_LIFT_TEST
static TickType_t s_menu_started = 0U;
#endif

uint32_t ArmPower_IsMenuMode(void)
{
    return APP_ARM_MENU_POWER_TEST;
}

uint32_t ArmPower_IsIsolatedMode(void)
{
    return (APP_ARM_MENU_POWER_TEST || APP_ARM_LIFT_TEST) ? 1U : 0U;
}

/* 菜单模式五分钟上限；升降实操由操作员关电，不在动作中定时松电。 */
uint32_t ArmPower_FilterRequest(uint32_t request)
{
#if APP_ARM_LIFT_TEST
    g_arm_menu_power_state = (request == 1U) ? 1U : 0U;
    return request;
#else
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
#endif
}
