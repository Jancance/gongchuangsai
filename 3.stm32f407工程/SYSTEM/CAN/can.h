#ifndef CAN_H
#define CAN_H
#include "zdt_emm_protocol.h"
#include "chassis_load_diag.h"

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
/* 首次控制故障锁存，只供遥测读取，不从串口清除。 */
extern volatile uint32_t g_can_ctrl_fault_latched;
/* 在FreeRTOS任务中调用一次，返回测试结果 */
uint32_t CAN1_LoopbackTest(void);
/* 四台Emm菜单核对后的源码放行状态，离线工程始终为0。 */
uint32_t CAN1_EmmSettingsConfirmed(void);
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply);
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply);
void CAN1_ResetSession(void);
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t address, CanRxMsg *reply, float *rpm);
/* 返回物理RPM的float；Emm反馈为整数RPM，不除10。 */
uint32_t CAN1_ReadSpeed(uint8_t address, CanRxMsg *reply, float *rpm);
/* 返回符号及Emm原生65536计数/圈，不做二次缩放。 */
uint32_t CAN1_ReadPosition(uint8_t address, CanRxMsg *reply,
                           uint8_t *sign, uint32_t *raw);

/* 使能控制：enabled为0松轴，为1锁轴；地址限定1～4 */
uint32_t CAN1_SetMotorEnable(uint8_t address, uint8_t enabled,
                             CanRxMsg *reply);

/* 立即停止；地址限定1～4 */
uint32_t CAN1_StopMotor(uint8_t address, CanRxMsg *reply);

/* Emm速度控制：输入缩小10倍，加速度为0..255档。 */
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address, uint8_t direction,
                             uint16_t speed_raw, uint16_t acceleration,
                             CanRxMsg *reply);
uint32_t CAN1_StageEmmSpeedRaw(uint8_t address,uint8_t direction,uint16_t speed_raw,
    uint16_t acceleration,CanRxMsg *reply);
#endif
