#include "watchdog.h"
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_rcc.h"

/* 不冻结验证配置：CPU调试暂停时看门狗继续计时 */
#define WATCHDOG_DEBUG_FREEZE 1U
#define WATCHDOG_WAIT_LIMIT   1000000U
#define WATCHDOG_RELOAD       999U

volatile uint32_t g_watchdog_started = 0U;
volatile uint32_t g_watchdog_feed_count = 0U;
volatile uint32_t g_reset_iwdg = 0U;
volatile uint32_t g_reset_flags = 0U;

/* 调度前调用：1成功，2低速时钟未就绪，3寄存器更新超时 */
uint32_t Watchdog_Init(void)
{
    uint32_t wait_count = 0U;

    g_watchdog_started = 0U;
    g_watchdog_feed_count = 0U;
    /* 先保存上次复位原因，再清硬件标志，避免下次误认旧事件 */
    g_reset_flags = RCC->CSR & 0xFE000000U;
    g_reset_iwdg =
        (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) ? 1U : 0U;
    RCC_ClearFlag();

    if (WATCHDOG_DEBUG_FREEZE == 1U)
    {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
    }
    else
    {
        DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, DISABLE);
    }

    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
    {
        wait_count++;
        if (wait_count >= WATCHDOG_WAIT_LIMIT)
        {
            return 2U;
        }
    }

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(WATCHDOG_RELOAD);
    IWDG_Enable();
    wait_count = 0U;
    while ((IWDG_GetFlagStatus(IWDG_FLAG_PVU) != RESET) ||
           (IWDG_GetFlagStatus(IWDG_FLAG_RVU) != RESET))
    {
        wait_count++;
        if (wait_count >= WATCHDOG_WAIT_LIMIT)
        {
            return 3U;
        }
    }
    IWDG_ReloadCounter();
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    g_watchdog_started = 1U;
    return 1U;
}

/* 正常运行只由监督检查通过后的分支调用 */
void Watchdog_Feed(void)
{
    if (g_watchdog_started != 1U)
    {
        return;
    }
    IWDG_ReloadCounter();
    g_watchdog_feed_count++;
}
