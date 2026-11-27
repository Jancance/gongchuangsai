#include "radar_uart_port.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "radar_client.h"

static uint8_t *s_rx_buffer;
static uint16_t s_rx_length;
static volatile uint16_t s_rx_position;
static volatile uint16_t s_rx_read;
static uint32_t s_last_tick, s_ms, s_tick_remainder;
static volatile uint8_t s_rx_active;
static uint8_t *s_tx_buffer;
static uint16_t s_tx_length;
static uint16_t s_tx_position;
static volatile uint8_t s_tx_active;

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length)
{
    uint32_t key;
    if (buffer == NULL || length < 2U) return false;
    key=RadarUartPort_EnterCritical();
    s_rx_buffer = buffer;
    s_rx_length = length;
    s_rx_position = 0U;
    s_rx_read = 0U;
    s_rx_active = 1U;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    RadarUartPort_ExitCritical(key);
    return true;
}

bool RadarUartPort_ReceiveIsActive(void)
{
    return s_rx_active != 0U;
}

bool RadarUartPort_ReadByte(uint8_t *byte)
{
    uint32_t key = RadarUartPort_EnterCritical();
    bool available = s_rx_read != s_rx_position;
    if (available) { *byte = s_rx_buffer[s_rx_read]; s_rx_read = (s_rx_read+1U)%s_rx_length; }
    RadarUartPort_ExitCritical(key);
    return available;
}

bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length)
{
    uint32_t primask = RadarUartPort_EnterCritical();
    if (data == NULL || length == 0U || s_tx_active != 0U)
    {
        RadarUartPort_ExitCritical(primask);
        return false;
    }
    s_tx_buffer = data;
    s_tx_length = length;
    s_tx_position = 0U;
    s_tx_active = 1U;
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    RadarUartPort_ExitCritical(primask);
    return true;
}

void RadarUartPort_AbortTransmit(void)
{
    uint32_t primask = RadarUartPort_EnterCritical();
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART3, USART_IT_TC, DISABLE);
    s_tx_active = 0U;
    s_tx_buffer = NULL;
    s_tx_length = 0U;
    s_tx_position = 0U;
    RadarUartPort_ExitCritical(primask);
}

uint32_t RadarUartPort_GetTick(void)
{
    uint32_t tick = (uint32_t)xTaskGetTickCount();
    uint32_t delta = tick-s_last_tick;
    uint64_t scaled = (uint64_t)delta*1000U+s_tick_remainder;
    s_last_tick=tick; s_ms+=(uint32_t)(scaled/configTICK_RATE_HZ);
    s_tick_remainder=(uint32_t)(scaled%configTICK_RATE_HZ);
    return s_ms;
}

uint32_t RadarUartPort_EnterCritical(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

void RadarUartPort_ExitCritical(uint32_t previous_primask)
{
    if (previous_primask == 0U) __enable_irq();
}

void RadarUartPort_USART3_IRQHandler(void)
{
    uint32_t status = USART3->SR;
    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U)
    {
        (void)USART3->DR;
        RadarClient_OnUartErrorInterrupt(status);
    }
    else if ((status & USART_SR_RXNE) != 0U && s_rx_active != 0U)
    {
        uint8_t byte = (uint8_t)USART3->DR;
        uint16_t next = (s_rx_position+1U)%s_rx_length;
        if (next == s_rx_read) {
            s_rx_active=0U; USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
            RadarClient_OnUartErrorInterrupt(USART_SR_ORE);
        } else { s_rx_buffer[s_rx_position]=byte; s_rx_position=next; }
    }

    if ((status & USART_SR_TXE) != 0U && s_tx_active != 0U &&
        USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        if (s_tx_position < s_tx_length)
        {
            USART3->DR = s_tx_buffer[s_tx_position++];
            if (s_tx_position >= s_tx_length)
            {
                USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
                USART_ITConfig(USART3, USART_IT_TC, ENABLE);
            }
        }
    }
    if ((status & USART_SR_TC) != 0U && s_tx_active != 0U &&
        USART_GetITStatus(USART3, USART_IT_TC) != RESET)
    {
        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
        s_tx_active = 0U;
        RadarClient_OnUartTxCompleteInterrupt();
    }
}

void USART3_IRQHandler(void) { RadarUartPort_USART3_IRQHandler(); }
