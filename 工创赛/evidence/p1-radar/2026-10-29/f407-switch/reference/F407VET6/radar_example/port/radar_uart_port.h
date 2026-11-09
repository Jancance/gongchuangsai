#ifndef RADAR_UART_PORT_H
#define RADAR_UART_PORT_H

#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

/* USART1 is the radar module link. USART2 is the optional demo console.
 * Keep USART1 at 115200 8N1, RX DMA circular, TX DMA normal. */
#define RADAR_UART_HANDLE       (&huart1)
#define RADAR_DEBUG_UART_HANDLE (&huart2)

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length);
bool RadarUartPort_ReceiveIsActive(void);
uint16_t RadarUartPort_GetReceivePosition(uint16_t buffer_length);
bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length);
void RadarUartPort_AbortTransmit(void);
uint32_t RadarUartPort_GetTick(void);
uint32_t RadarUartPort_EnterCritical(void);
void RadarUartPort_ExitCritical(uint32_t previous_primask);

#endif

