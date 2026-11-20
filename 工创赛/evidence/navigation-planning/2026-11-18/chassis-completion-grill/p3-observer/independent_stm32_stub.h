#ifndef INDEPENDENT_STM32_STUB_H
#define INDEPENDENT_STM32_STUB_H

#include <stdint.h>

enum {
    DISABLE, ENABLE, GPIOD, USART3, RCC_AHB1Periph_GPIOD,
    RCC_APB1Periph_USART3, GPIO_PinSource8, GPIO_PinSource9,
    GPIO_AF_USART3, GPIO_Pin_8, GPIO_Pin_9, GPIO_Mode_AF,
    GPIO_Speed_50MHz, GPIO_PuPd_UP, USART_Mode_Rx, USART_Mode_Tx,
    USART_IT_RXNE, USART_IT_TXE, USART_IT_TC, USART3_IRQn
};
typedef struct { uint32_t GPIO_Pin, GPIO_Mode, GPIO_Speed, GPIO_PuPd; } GPIO_InitTypeDef;
typedef struct { uint32_t USART_BaudRate, USART_Mode; } USART_InitTypeDef;
typedef struct {
    uint32_t NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority;
    uint32_t NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

static inline void RCC_AHB1PeriphClockCmd(uint32_t x, uint32_t y) { (void)x; (void)y; }
static inline void RCC_APB1PeriphClockCmd(uint32_t x, uint32_t y) { (void)x; (void)y; }
static inline void GPIO_PinAFConfig(uint32_t x, uint32_t y, uint32_t z) { (void)x; (void)y; (void)z; }
static inline void GPIO_StructInit(GPIO_InitTypeDef *init) { (void)init; }
static inline void GPIO_Init(uint32_t x, GPIO_InitTypeDef *init) { (void)x; (void)init; }
static inline void USART_StructInit(USART_InitTypeDef *init) { (void)init; }
static inline void USART_Init(uint32_t x, USART_InitTypeDef *init) { (void)x; (void)init; }
static inline void USART_ITConfig(uint32_t x, uint32_t y, uint32_t z) { (void)x; (void)y; (void)z; }
static inline void NVIC_Init(NVIC_InitTypeDef *init) { (void)init; }
static inline void USART_Cmd(uint32_t x, uint32_t y) { (void)x; (void)y; }

#endif
