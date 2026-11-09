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

/* 使能控制：enabled为0松轴，为1锁轴；地址限定1～4 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply);

/* 立即停止；地址限定1～4 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply);

/* Emm速度控制：speed_raw是原始命令值，暂不换算为RPM */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint8_t acceleration,
                             CanRxMsg *reply);
                             
uint32_t CAN1_LiftExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply);
uint32_t CAN1_ReadMotorOptions(uint8_t address, CanRxMsg *reply);

uint32_t CAN1_ReadLiftHomeParameters(uint8_t out[17], CanRxMsg replies[3]);

extern volatile uint32_t g_can_lift_home_done_notice;
uint32_t CAN1_StartLiftHome(CanRxMsg *reply);
uint32_t CAN1_AbortLiftHome(CanRxMsg *reply);
uint32_t CAN1_ReadLiftHomeStatus(CanRxMsg *reply);

uint32_t CAN1_ExtensionExchange(const uint8_t *request, uint8_t length, CanRxMsg *reply);

uint32_t CAN1_ReadExtensionHomeParameters(uint8_t out[17], CanRxMsg replies[3]);

extern volatile uint32_t g_can_extension_home_done_notice;
uint32_t CAN1_StartExtensionHome(CanRxMsg *reply);
uint32_t CAN1_AbortExtensionHome(CanRxMsg *reply);
uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg *reply);
void CAN1_ExtensionHomeSessionEnd(void);


#include "arm_x.h"
uint32_t CAN1_ArmXConfigure(uint8_t address, uint8_t options, uint32_t legacy_mstep);
uint32_t CAN1_ArmXBuildMove(uint8_t address, uint8_t direction, uint32_t pulses, uint16_t speed_deci_rpm, uint16_t acceleration, uint16_t deceleration, uint8_t out[15]);
#endif
