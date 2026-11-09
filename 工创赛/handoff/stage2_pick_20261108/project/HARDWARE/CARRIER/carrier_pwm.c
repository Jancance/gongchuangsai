#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "carrier_config.h"
#include "carrier_pwm.h"

static uint32_t s_ready = 0U;
static uint32_t s_output = 0U;

static void CarrierPwm_PinMode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpio);
}

uint32_t CarrierPwm_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    uint32_t timer_hz;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    CarrierPwm_PinMode(GPIO_Mode_OUT);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_DeInit(TIM3);
    s_ready = 0U;
    s_output = 0U;
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_hz *= 2U;
    if (timer_hz < 1000000U || timer_hz % 1000000U != 0U ||
        timer_hz / 1000000U > 65536U ||
        CARRIER_PULSE_MIN_US >= CARRIER_PULSE_MAX_US ||
        CARRIER_PULSE_MAX_US >= CARRIER_PERIOD_US ||
        CARRIER_PERIOD_US > 65536U) return 0U;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000U - 1U);
    timer.TIM_Period = CARRIER_PERIOD_US - 1U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &timer);
    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    channel.TIM_OCPolarity = TIM_OCPolarity_High;
    channel.TIM_Pulse = 0U;
    TIM_OC4Init(TIM3, &channel);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
    s_ready = 1U;
    return 1U;
}

uint32_t CarrierPwm_Write(uint32_t pulse_us)
{
    if (s_ready != 1U || pulse_us < CARRIER_PULSE_MIN_US ||
        pulse_us > CARRIER_PULSE_MAX_US) return 0U;
    TIM_SetCompare4(TIM3, pulse_us);
    if (!s_output)
    {
        TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
        TIM_Cmd(TIM3, ENABLE);
        CarrierPwm_PinMode(GPIO_Mode_AF);
        s_output = 1U;
    }
    return 1U;
}

void CarrierPwm_Off(void)
{
    CarrierPwm_PinMode(GPIO_Mode_OUT);
    TIM_Cmd(TIM3, DISABLE);
    TIM_SetCompare4(TIM3, 0U);
    s_output = 0U;
}
