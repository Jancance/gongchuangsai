#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
/* 首次控制故障锁存，只供遥测读取，不从串口清除。 */
extern volatile uint32_t g_can_ctrl_fault_latched;
/* 在FreeRTOS任务中调用一次，返回测试结果 */
uint32_t CAN1_LoopbackTest(void);
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply);
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply);
void CAN1_ResetSession(void);
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, int32_t *rpm);
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw);

/* 使能控制：enabled为0松轴，为1锁轴；地址限定1～4 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply);

/* 立即停止；地址限定1～4 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply);

/* Emm速度控制：speed_raw是原始命令值，暂不换算为RPM */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply);
                             
#endif
