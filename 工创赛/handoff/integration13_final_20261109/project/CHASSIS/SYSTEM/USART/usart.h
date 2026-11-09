#ifndef USART_MODULE_H
#define USART_MODULE_H

#include <stdint.h>

void USART1_Config(void);
uint8_t USART1_SendString(const char *text);
void USART1_EnableReceive(void);
uint8_t USART1_ReadByte(uint8_t *data);
/* 接收溢出或队列满的错误事件数 */
extern volatile uint32_t g_uart1_rx_errors;
uint8_t USART1_WaitByte(uint8_t *data, uint32_t timeout_ms);

/* Dedicated Orange Pi vision transport: PC6/PC7, 115200 8N1. */
void USART6_VisionConfig(void);
void USART6_VisionEnableReceive(void);
uint8_t USART6_VisionReadByte(uint8_t *data);
uint8_t USART6_VisionSend(const uint8_t *data, uint16_t length);
extern volatile uint32_t g_uart6_rx_errors;

#endif
