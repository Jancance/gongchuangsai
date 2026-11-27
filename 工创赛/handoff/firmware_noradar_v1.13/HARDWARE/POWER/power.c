#include "stm32f4xx.h"
#include "power.h"

/* 初始化PE2，默认关闭VM允许请求 */
void Power_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    /* 先锁存低电平，再配置输出，避免初始化时误开启 */
    GPIO_ResetBits(GPIOE, GPIO_Pin_2);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* 请求开启VM，实际供电仍受外部开关控制 */
void Power_Enable(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

/* 撤销VM允许请求 */
void Power_Disable(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_2);
}
