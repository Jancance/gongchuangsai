#include "stm32f4xx.h"
#include "usart.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "queue.h"

/* 当前只允许任务A提交请求，中断负责逐字节发送 */
static uint8_t g_tx_buffer[APP_SERIAL_MOTION_CONTROL ? 768U : 64U];
static volatile uint16_t g_tx_length = 0U;
static volatile uint16_t g_tx_position = 0U;
/* 接收中断写入，任务A读取 */
static QueueHandle_t g_rx_queue = NULL;
volatile uint32_t g_uart1_rx_errors = 0U;

/* 初始化PA9发送，115200波特率，8位数据、无校验、1停止位 */
void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* 每次开机创建一次接收队列 */
    g_rx_queue = xQueueCreate(64U, sizeof(uint8_t));
    configASSERT(g_rx_queue != NULL);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200U;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    /* 调度启动前不开启接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0U;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* 返回1表示接受请求，0表示忙或文本过长，不等待发送 */
uint8_t USART1_SendString(const char *text)
{
    uint16_t length = 0U;

    if ((text == 0) || (g_tx_length != 0U))
    {
        return 0U;
    }

    while (text[length] != '\0')
    {
        if (length >= sizeof(g_tx_buffer))
        {
            return 0U;
        }

        g_tx_buffer[length] = (uint8_t)text[length];
        length++;
    }

    if (length == 0U)
    {
        return 1U;
    }

    g_tx_position = 0U;
    g_tx_length = length;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    return 1U;
}

/* 接收中断入队，保留发送，按需要请求任务切换 */
void USART1_IRQHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint8_t data;

    /* 读取数据寄存器清除溢出错误 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
        g_uart1_rx_errors++;
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        data = (uint8_t)USART_ReceiveData(USART1);

        /* 中断中使用带FromISR的队列接口 */
        if (xQueueSendFromISR(g_rx_queue, &data,
                             &higher_priority_task_woken) != pdPASS)
        {
            g_uart1_rx_errors++;
        }
    }

    /* 保留原来的发送功能 */
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (g_tx_position < g_tx_length)
        {
            USART_SendData(USART1, g_tx_buffer[g_tx_position]);
            g_tx_position++;
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            g_tx_length = 0U;
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/* 调度运行后开启接收，清除启动前可能留下的数据 */
void USART1_EnableReceive(void)
{
    configASSERT(g_rx_queue != NULL);
    (void)USART_GetFlagStatus(USART1, USART_FLAG_RXNE);
    (void)USART_ReceiveData(USART1);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/* 从队列取一个字节，无数据立即返回 */
uint8_t USART1_ReadByte(uint8_t *data)
{
    if ((data == NULL) || (g_rx_queue == NULL))
    {
        return 0U;
    }

    return (xQueueReceive(g_rx_queue, data, 0U) == pdPASS) ? 1U : 0U;
}
/* 等待一个字节，等待期间不阻塞其他任务 */

uint8_t USART1_WaitByte(uint8_t *data, uint32_t timeout_ms)
{
    if ((data == NULL) || (g_rx_queue == NULL))
    {
        return 0U;
    }

    return (xQueueReceive(g_rx_queue, data,
                          pdMS_TO_TICKS(timeout_ms)) == pdPASS) ? 1U : 0U;
}
