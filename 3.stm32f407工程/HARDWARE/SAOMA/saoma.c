#include "saoma.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* 同时支持回车换行结束，或无结束符时50ms字节间隔结束 */
#define SAOMA_GAP_MS 50U
#define SAOMA_QUEUE_LENGTH 128U
#define SAOMA_DRAIN_LIMIT 64U

typedef struct
{
    TickType_t tick;
    uint8_t data;
} SaomaByte;
static QueueHandle_t s_queue = NULL;
static char s_frame[SAOMA_TASK_LENGTH + 1U];
static uint32_t s_length = 0U;
static uint32_t s_bad = 0U;
static uint32_t s_seen_errors = 0U;
static uint32_t s_active = 0U;
static TickType_t s_last_tick = 0U;
char SAOMA_Data[SAOMA_TASK_LENGTH + 1U] = {0};
volatile uint8_t SAOMA_DataReady = 0U;
volatile uint32_t g_saoma_rx_count = 0U;
volatile uint32_t g_saoma_rx_errors = 0U;
volatile uint32_t g_saoma_accept_count = 0U;
volatile uint32_t g_saoma_reject_count = 0U;
volatile uint32_t g_saoma_last_result = 0U;

void SAOMA_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    s_queue = xQueueCreate(SAOMA_QUEUE_LENGTH, sizeof(SaomaByte));
    configASSERT(s_queue != NULL);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 9600U;
    uart.USART_Mode = USART_Mode_Rx;
    USART_Init(UART5, &uart);
    USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
    /* 沿用主函数Group4，不照搬F103的Group2 */
    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(UART5, ENABLE);
}

/* 调度运行后，由任务A开启接收 */
void SAOMA_EnableReceive(void)
{
    configASSERT(s_queue != NULL);
    (void)UART5->SR;
    (void)UART5->DR;
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
}

void UART5_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    SaomaByte item;
    uint32_t status = UART5->SR;
    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE |
                   USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) != 0U)
    {
        item.data = (uint8_t)UART5->DR;
        if ((status & (USART_FLAG_ORE | USART_FLAG_FE |
                       USART_FLAG_NE | USART_FLAG_PE)) != 0U)
        {
            g_saoma_rx_errors++;
        }
        else if ((status & USART_FLAG_RXNE) != 0U)
        {
            item.tick = xTaskGetTickCountFromISR();
            g_saoma_rx_count++;
            if (xQueueSendFromISR(s_queue, &item, &wake) != pdPASS)
            {
                g_saoma_rx_errors++;
            }
        }
    }
    portYIELD_FROM_ISR(wake);
}

/* 只检查四段数字+加号的基本外形，不冻结颜色/重复等官方语义 */
static void SAOMA_Finish(void)
{
    uint32_t i;
    uint32_t valid = (s_bad == 0U && s_length == 15U) ? 1U : 0U;
    if (s_active == 0U) return;
    for (i = 0U; i < s_length && valid != 0U; i++)
    {
        if (i == 3U || i == 7U || i == 11U)
        {
            if (s_frame[i] != '+') valid = 0U;
        }
        else if (s_frame[i] < '0' || s_frame[i] > '9') valid = 0U;
    }
    /* 短临界区交接，UART5不会在检查丢字节后插入新错误 */
    taskENTER_CRITICAL();
    if (g_saoma_rx_errors != s_seen_errors) valid = 0U;
    if (valid != 0U)
    {
        s_frame[15] = '\0';
        memcpy(SAOMA_Data, s_frame, sizeof(SAOMA_Data));
        SAOMA_DataReady = 1U;
        g_saoma_accept_count++;
        g_saoma_last_result = 1U;
    }
    else
    {
        SAOMA_DataReady = 0U;
        g_saoma_reject_count++;
        g_saoma_last_result = 2U;
    }
    taskEXIT_CRITICAL();
    s_length = 0U;
    s_bad = 0U;
    s_active = 0U;
}

void SAOMA_Update(void)
{
    SaomaByte item;
    uint32_t n;
    TickType_t gap = pdMS_TO_TICKS(SAOMA_GAP_MS);
    /* 丢字节后清掉不完整队列，直到结束符/新间隔再重新收帧 */
    if (s_seen_errors != g_saoma_rx_errors)
    {
        taskENTER_CRITICAL();
        s_seen_errors = g_saoma_rx_errors;
        (void)xQueueReset(s_queue);
        taskEXIT_CRITICAL();
        s_length = 0U;
        s_bad = 1U;
        s_active = 1U;
        s_last_tick = xTaskGetTickCount();
        SAOMA_DataReady = 0U;
        g_saoma_last_result = 3U;
    }
    for (n = 0U; n < SAOMA_DRAIN_LIMIT; n++)
    {
        if (s_seen_errors != g_saoma_rx_errors) break;
        if (xQueueReceive(s_queue, &item, 0U) != pdPASS) break;
        if (s_active != 0U && (TickType_t)(item.tick - s_last_tick) >= gap)
        {
            SAOMA_Finish();
        }
        s_last_tick = item.tick;
        if (item.data == '\r' || item.data == '\n')
        {
            SAOMA_Finish();
            continue;
        }
        s_active = 1U;
        if (s_length < SAOMA_TASK_LENGTH)
        {
            s_frame[s_length++] = (char)item.data;
        }
        else s_bad = 1U;
    }
    /* 队列尚有待处理数据时，不用当前时间误判帧已结束 */
    if (s_active != 0U && uxQueueMessagesWaiting(s_queue) == 0U &&
        (TickType_t)(xTaskGetTickCount() - s_last_tick) >= gap)
    {
        SAOMA_Finish();
    }
}
