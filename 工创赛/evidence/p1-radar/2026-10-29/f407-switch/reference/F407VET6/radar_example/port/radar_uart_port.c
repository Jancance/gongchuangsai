#include "radar_uart_port.h"

static bool rx_dma_hardware_enabled(const DMA_HandleTypeDef *dma)
{
    return dma != NULL && dma->Instance != NULL &&
           (dma->Instance->CR & DMA_SxCR_EN) != 0U;
}

static void clear_rx_dma_flags(DMA_HandleTypeDef *dma)
{
    uint32_t flags = __HAL_DMA_GET_TC_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_HT_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_TE_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_DME_FLAG_INDEX(dma) |
                     __HAL_DMA_GET_FE_FLAG_INDEX(dma);
    __HAL_DMA_CLEAR_FLAG(dma, flags);
}

static bool force_rx_dma_reinit(UART_HandleTypeDef *uart,
                                DMA_HandleTypeDef *dma)
{
    CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    (void)HAL_DMA_Abort(dma);
    __HAL_DMA_DISABLE(dma);
    if (rx_dma_hardware_enabled(dma)) return false;

    /* 硬件已停止时才修正可能滞留在BUSY/ABORT/ERROR的HAL状态。 */
    if (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY)
        dma->State = HAL_DMA_STATE_READY;
    if (HAL_DMA_DeInit(dma) != HAL_OK || HAL_DMA_Init(dma) != HAL_OK)
        return false;

    uart->RxState = HAL_UART_STATE_READY;
    uart->RxXferCount = 0U;
    uart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
    return true;
}

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length)
{
    UART_HandleTypeDef *uart = RADAR_UART_HANDLE;
    DMA_HandleTypeDef *dma;
    HAL_StatusTypeDef status;
    bool receive_dirty;

    if (buffer == NULL || length == 0U || uart->hdmarx == NULL)
        return false;
    dma = uart->hdmarx;
    if (dma->Init.Mode != DMA_CIRCULAR) return false;
    receive_dirty = uart->RxState != HAL_UART_STATE_READY ||
                    HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY ||
                    HAL_DMA_GetError(dma) != HAL_DMA_ERROR_NONE ||
                    (uart->Instance->CR3 & USART_CR3_DMAR) != 0U ||
                    rx_dma_hardware_enabled(dma);

    if (receive_dirty && HAL_UART_AbortReceive(uart) != HAL_OK) {
        if (!force_rx_dma_reinit(uart, dma)) return false;
    } else if (HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY ||
               HAL_DMA_GetError(dma) != HAL_DMA_ERROR_NONE ||
               (uart->Instance->CR3 & USART_CR3_DMAR) != 0U ||
               rx_dma_hardware_enabled(dma)) {
        if (!force_rx_dma_reinit(uart, dma)) return false;
    }

    /* 读取SR/DR一次会同时清除PE、FE、NE和ORE。 */
    __HAL_UART_CLEAR_PEFLAG(uart);
    clear_rx_dma_flags(dma);
    uart->ErrorCode = HAL_UART_ERROR_NONE;
    dma->ErrorCode = HAL_DMA_ERROR_NONE;

    status = HAL_UART_Receive_DMA(uart, buffer, length);
    if (status == HAL_OK)
        __HAL_DMA_DISABLE_IT(dma, DMA_IT_HT);
    return status == HAL_OK;
}

bool RadarUartPort_ReceiveIsActive(void)
{
    DMA_HandleTypeDef *dma = RADAR_UART_HANDLE->hdmarx;
    return dma != NULL &&
           RADAR_UART_HANDLE->RxState == HAL_UART_STATE_BUSY_RX &&
           HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY &&
           HAL_DMA_GetError(dma) == HAL_DMA_ERROR_NONE &&
           (RADAR_UART_HANDLE->Instance->CR3 & USART_CR3_DMAR) != 0U &&
           rx_dma_hardware_enabled(dma);
}

uint16_t RadarUartPort_GetReceivePosition(uint16_t buffer_length)
{
    uint16_t position;
    if (RADAR_UART_HANDLE->hdmarx == NULL || buffer_length == 0U) return 0U;
    position = (uint16_t)(buffer_length -
               __HAL_DMA_GET_COUNTER(RADAR_UART_HANDLE->hdmarx));
    return position >= buffer_length ? 0U : position;
}

bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length)
{
    return data != NULL && length > 0U &&
           HAL_UART_Transmit_DMA(RADAR_UART_HANDLE, data, length) == HAL_OK;
}

void RadarUartPort_AbortTransmit(void)
{
    (void)HAL_UART_AbortTransmit(RADAR_UART_HANDLE);
}

uint32_t RadarUartPort_GetTick(void)
{
    return HAL_GetTick();
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
