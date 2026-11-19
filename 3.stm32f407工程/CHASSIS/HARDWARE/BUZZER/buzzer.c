#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "Buzzer.h"

/* 当前鸣叫也计入剩余次数，只由任务A操作 */
static uint32_t g_buzzer_remaining = 0U;
static uint8_t g_buzzer_on = 0U;
static TickType_t g_buzzer_phase_tick = 0U;
static TickType_t g_buzzer_on_ticks;

/* 初始化PB0，默认关闭蜂鸣器 */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    g_buzzer_remaining = 0U;
    g_buzzer_on = 0U;
}

/* 调度启动后请求一组提示音，0次保持关闭 */
static void Buzzer_StartTimed(uint32_t count, uint32_t duration_ms)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    g_buzzer_remaining = count;
    g_buzzer_on_ticks = pdMS_TO_TICKS(duration_ms);
    g_buzzer_on = 0U;

    if (count != 0U)
    {
        g_buzzer_phase_tick = xTaskGetTickCount();
        g_buzzer_on = 1U;
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    }
}

void Buzzer_Start(uint32_t count)
{
    Buzzer_StartTimed(count, APP_BEEP_ON_MS);
}

void Buzzer_StartPulse(uint32_t duration_ms)
{
    Buzzer_StartTimed(1U, duration_ms);
}

/* 分时推进响停，不使用忙等延时 */
void Buzzer_Update(void)
{
    TickType_t now;
    TickType_t duration;

    if (g_buzzer_remaining == 0U)
    {
        return;
    }

    now = xTaskGetTickCount();
    duration = g_buzzer_on ? g_buzzer_on_ticks : pdMS_TO_TICKS(APP_BEEP_GAP_MS);

    /* 无符号时间差兼容节拍计数回绕 */
    if ((TickType_t)(now - g_buzzer_phase_tick) < duration)
    {
        return;
    }

    g_buzzer_phase_tick = now;

    if (g_buzzer_on != 0U)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        g_buzzer_on = 0U;
        g_buzzer_remaining--;
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
        g_buzzer_on = 1U;
    }
}
