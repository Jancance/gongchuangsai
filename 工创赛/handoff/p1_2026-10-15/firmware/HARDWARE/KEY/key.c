#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "Key.h"

/* PC13低电平表示按下，先确认释放再接受按下事件 */
static uint8_t g_key_last_raw = 1U;
static uint8_t g_key_release_seen = 0U;
static TickType_t g_key_change_tick = 0U;

/* 初始化启动按键输入，使用上拉 */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    g_key_last_raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    g_key_release_seen = 0U;
    g_key_change_tick = 0U;
}

/* 稳定按下产生一次事件，长按不重复，上电按住不触发 */
uint8_t Key_GetEvent(void)
{
    uint8_t raw;
    TickType_t now;

    raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    now = xTaskGetTickCount();

    if (raw != g_key_last_raw)
    {
        g_key_last_raw = raw;
        g_key_change_tick = now;
    }

    if ((TickType_t)(now - g_key_change_tick) <
        pdMS_TO_TICKS(APP_START_KEY_DEBOUNCE_MS))
    {
        return 0U;
    }

    if (raw != 0U)
    {
        g_key_release_seen = 1U;
    }
    else if (g_key_release_seen != 0U)
    {
        g_key_release_seen = 0U;
        return 1U;
    }

    return 0U;
}
