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

#endif
