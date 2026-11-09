#include "screen.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define SCREEN_BAUD_RATE 9600U
#define SCREEN_TEXT_LIMIT 96U
#define SCREEN_TX_TIMEOUT_MS 150U

volatile uint32_t g_screen_init_done = 0U;
volatile uint32_t g_screen_command_count = 0U;

/* 调度前只配置外设，不发送文字、不等待屏幕 */
void Screen_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    g_screen_init_done = 0U;
    g_screen_command_count = 0U;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = SCREEN_BAUD_RATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART4, &uart);
    /* 本步仅发送；没有接收中断，不把发完当成屏幕应答 */
    USART_Cmd(UART4, ENABLE);
    g_screen_init_done = 1U;
}

/* 本模块只有任务A调用；不得在中断或关闭调度的临界区发送 */
static uint32_t Screen_Write(const uint8_t *data, uint32_t length)
{
    TickType_t start;
    TickType_t timeout = pdMS_TO_TICKS(SCREEN_TX_TIMEOUT_MS);
    uint32_t i;

    if ((g_screen_init_done != 1U) || (__get_IPSR() != 0U) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
    {
        return SCREEN_NOT_READY;
    }
    start = xTaskGetTickCount();
    for (i = 0U; i < length; i++)
    {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
        {
            if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
            {
                return SCREEN_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
        {
            return SCREEN_TIMEOUT;
        }
        USART_SendData(UART4, data[i]);
    }
    /* 等待最后一字节发完，同样计入整帧150毫秒上限 */
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
    {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
        {
            return SCREEN_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return SCREEN_OK;
}

/* 最多95个字符；参数异常时不发送半条命令 */
static uint32_t Screen_TextLength(const char *text, uint32_t *length)
{
    uint32_t i;

    if (text == NULL)
    {
        return SCREEN_BAD_PARAM;
    }
    for (i = 0U; i < SCREEN_TEXT_LIMIT; i++)
    {
        if (text[i] == '\0')
        {
            *length = i;
            return SCREEN_OK;
        }
    }
    return SCREEN_BAD_PARAM;
}

uint32_t Screen_SendByte(uint8_t data)
{
    return Screen_Write(&data, 1U);
}

uint32_t Screen_SendString(const char *text)
{
    uint32_t length;

    if (Screen_TextLength(text, &length) != SCREEN_OK)
    {
        return SCREEN_BAD_PARAM;
    }
    return Screen_Write((const uint8_t *)text, length);
}

uint32_t Screen_SendCommand(const char *command)
{
    uint8_t frame[SCREEN_TEXT_LIMIT + 6U];
    uint32_t length;
    uint32_t i;
    uint32_t result;

    if ((Screen_TextLength(command, &length) != SCREEN_OK) ||
        (length == 0U))
    {
        return SCREEN_BAD_PARAM;
    }
    /* 先终止上次可能中断的残帧，再发送本条完整命令 */
    for (i = 0U; i < 3U; i++)
    {
        frame[i] = 0xFFU;
    }
    for (i = 0U; i < length; i++)
    {
        frame[i + 3U] = (uint8_t)command[i];
    }
    for (i = 0U; i < 3U; i++)
    {
        frame[length + 3U + i] = 0xFFU;
    }
    result = Screen_Write(frame, length + 6U);
    if (result == SCREEN_OK)
    {
        g_screen_command_count++;
    }
    return result;
}

/* 保留原两行完整任务码；这里只检查显示边界，不判搬运合法性 */
uint32_t Screen_ShowTaskCode(const char *task_code)
{
    char line1[9];
    char line2[8];
    char command[40];
    uint32_t length;
    uint32_t i;
    uint32_t result;

    if ((Screen_TextLength(task_code, &length) != SCREEN_OK) ||
        (length != 15U))
    {
        return SCREEN_BAD_PARAM;
    }
    for (i = 0U; i < 15U; i++)
    {
        if (((task_code[i] < '0') || (task_code[i] > '9')) &&
            (task_code[i] != '+'))
        {
            return SCREEN_BAD_PARAM;
        }
    }
    for (i = 0U; i < 8U; i++)
    {
        line1[i] = task_code[i];
    }
    line1[8] = '\0';
    for (i = 0U; i < 7U; i++)
    {
        line2[i] = task_code[i + 8U];
    }
    line2[7] = '\0';

    result = Screen_SendCommand("page 0");
    if (result != SCREEN_OK)
    {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20U));
    /* 两行长度已固定，40字节缓冲足以放下命令和结束零 */
    sprintf(command, "page0.t0.txt=\"%s\"", line1);
    result = Screen_SendCommand(command);
    if (result != SCREEN_OK)
    {
        return result;
    }
    sprintf(command, "page0.t1.txt=\"%s\"", line2);
    return Screen_SendCommand(command);
}

