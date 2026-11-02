#include "stm32f4xx.h"
#include "led.h"

/* ³õÊ¼»¯PE7×´Ì¬µÆ£¬Ä¬ÈÏÏ¨Ãð */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_ResetBits(GPIOE, GPIO_Pin_7);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* ·­×ª×´Ì¬µÆ */
void LED_Turn(void)
{
    GPIO_ToggleBits(GPIOE, GPIO_Pin_7);
}
