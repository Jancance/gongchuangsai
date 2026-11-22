#ifndef SAOMA_H
#define SAOMA_H
#include <stdint.h>
#define SAOMA_TASK_LENGTH 15U
extern char SAOMA_Data[SAOMA_TASK_LENGTH + 1U];
extern volatile uint8_t SAOMA_DataReady;
extern volatile uint32_t g_saoma_rx_count;
extern volatile uint32_t g_saoma_rx_errors;
extern volatile uint32_t g_saoma_accept_count;
extern volatile uint32_t g_saoma_reject_count;
/* 0未收完整帧，1基本格式通过，2长度/格式拒绝，3串口丢字节 */
extern volatile uint32_t g_saoma_last_result;
void SAOMA_Init(void);
void SAOMA_EnableReceive(void);
void SAOMA_Update(void);
#endif
