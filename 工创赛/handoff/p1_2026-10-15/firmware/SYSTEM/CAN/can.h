#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
/* 在FreeRTOS任务中调用一次，返回测试结果 */
uint32_t CAN1_LoopbackTest(void);
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply);
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply);
void CAN1_ResetSession(void);
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, int32_t *rpm);
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw);
                           
#endif
