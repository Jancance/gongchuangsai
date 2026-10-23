#include "stm32f4xx.h"
#include "power.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* 关闭后保留上次开启时刻，用于失败后取证。 */
static volatile uint32_t s_vm_command_active = 0U;
static volatile uint32_t s_vm_enable_seen = 0U;
static volatile TickType_t s_vm_enable_tick = 0U;
static volatile uint32_t s_vm_enable_count = 0U;

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
    s_vm_command_active = s_vm_enable_seen = 0U;
    s_vm_enable_tick = 0U;
    s_vm_enable_count = 0U;
}

/* 请求开启VM，实际供电仍受外部开关控制 */
void Power_Enable(void)
{
#if APP_UART_LINK_TEST
    /* 串口测试不能通过手改请求或接收字符串开启VM。 */
    Power_Disable();
    return;
#else
    taskENTER_CRITICAL();
    if ((s_vm_command_active == 0U) || ((GPIOE->ODR & GPIO_Pin_2) == 0U))
    {
        GPIO_SetBits(GPIOE, GPIO_Pin_2);
        s_vm_enable_tick = xTaskGetTickCount();
        s_vm_enable_seen = 1U;
        s_vm_enable_count++;
        s_vm_command_active = 1U;
    }
    else GPIO_SetBits(GPIOE, GPIO_Pin_2);
    taskEXIT_CRITICAL();
#endif
}

/* 撤销VM允许请求 */
void Power_Disable(void)
{
    /* 紧急关断不调用RTOS，不增加临界区或等待。 */
    GPIO_ResetBits(GPIOE, GPIO_Pin_2);
    s_vm_command_active = 0U;
}

void Power_ReadCommandStatus(PowerCommandStatus *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    status->active = ((s_vm_command_active != 0U) &&
        ((GPIOE->ODR & GPIO_Pin_2) != 0U)) ? 1U : 0U;
    status->enable_seen = s_vm_enable_seen;
    status->enable_tick = (uint32_t)s_vm_enable_tick;
    status->enable_count = s_vm_enable_count;
    taskEXIT_CRITICAL();
}
