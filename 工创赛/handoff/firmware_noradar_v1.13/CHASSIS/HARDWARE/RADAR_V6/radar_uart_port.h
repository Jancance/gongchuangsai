#ifndef RADAR_UART_PORT_H
#define RADAR_UART_PORT_H

#include <stdbool.h>
#include <stdint.h>

bool RadarUartPort_StartReceive(uint8_t *buffer, uint16_t length);
bool RadarUartPort_ReceiveIsActive(void);
bool RadarUartPort_ReadByte(uint8_t *byte);
void RadarUartPort_USART3_IRQHandler(void);
bool RadarUartPort_StartTransmit(uint8_t *data, uint16_t length);
void RadarUartPort_AbortTransmit(void);
uint32_t RadarUartPort_GetTick(void);
uint32_t RadarUartPort_EnterCritical(void);
void RadarUartPort_ExitCritical(uint32_t previous_primask);

#endif
