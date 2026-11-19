#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "gripper.h"
#include "gripper_config.h"

/* 集中参数：标称映射起点，后续按真实开合位置调整。 */

static uint8_t s_ready = 0U;
static uint8_t s_running = 0U;

static void Gripper_PinMode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpio);
}

void Gripper_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    uint32_t timer_hz;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    Gripper_PinMode(GPIO_Mode_OUT);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* PB9/CH4 shares TIM4 20ms with PB6/CH1 and PB7/CH2. No timer reset. */
    s_ready = 0U;
    s_running = 0U;
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_hz *= 2U;
    }
    if ((timer_hz < 1000000U) || ((timer_hz % 1000000U) != 0U) ||
        ((timer_hz / 1000000U) > 65536U) ||
        (JIAZHUA_PULSE_MIN_US >= JIAZHUA_PULSE_MAX_US) ||
        (JIAZHUA_PULSE_MAX_US >= JIAZHUA_PERIOD_US) ||
        !(JIAZHUA_ANGLE_MAX_DEG > 0.0f))
    {
        return;
    }
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000U - 1U);
    timer.TIM_Period = JIAZHUA_PERIOD_US - 1U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &timer);
    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    channel.TIM_OCPolarity = TIM_OCPolarity_High;
    channel.TIM_Pulse = 0U;
    TIM_OC4Init(TIM4, &channel);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
    /* 默认不输出位置脉冲，显式调用jiazhua后才开始。 */
    s_ready = 1U;
}

uint8_t jiazhua(float angle_deg)
{
    uint32_t pulse_us;

    /* 此写法同时拒绝越界、NaN和无穷值，不把错误角度夹到端点。 */
    if ((s_ready == 0U) ||
        !(angle_deg >= 0.0f && angle_deg <= JIAZHUA_ANGLE_MAX_DEG))
    {
        return 0U;
    }
    pulse_us = JIAZHUA_PULSE_MIN_US + (uint32_t)(
        angle_deg * (float)(JIAZHUA_PULSE_MAX_US - JIAZHUA_PULSE_MIN_US)
        / JIAZHUA_ANGLE_MAX_DEG + 0.5f);
    TIM_SetCompare4(TIM4, pulse_us);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    if (s_running == 0U)
    {
        if ((TIM4->CR1 & TIM_CR1_CEN) == 0U)
            TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
        TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Enable);
        TIM_Cmd(TIM4, ENABLE);
        Gripper_PinMode(GPIO_Mode_AF);
        s_running = 1U;
    }
    /* 已运行时在下一PWM周期更新，不反复清零计数器。 */
    return 1U;
}

void Gripper_Stop(void)
{
    if (s_ready != 0U)
    {
        Gripper_PinMode(GPIO_Mode_OUT);
        TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Disable);
        TIM_SetCompare4(TIM4, 0U);
        s_running = 0U;
    }
}

uint8_t Gripper_PulseApplied(void)
{ return s_running && TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET; }
