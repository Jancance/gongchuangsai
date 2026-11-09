#ifndef CAN_H
#define CAN_H
#include <stdint.h>
typedef struct{uint32_t StdId,ExtId;uint8_t IDE,RTR,DLC,Data[8];}CanRxMsg;
extern volatile uint32_t g_can_ctrl_fault_latched;
uint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadSpeed(uint8_t,CanRxMsg*,float*);
uint32_t CAN1_ReadPosition(uint8_t,CanRxMsg*,uint8_t*,uint32_t*);
uint32_t CAN1_SetMotorEnable(uint8_t,uint8_t,CanRxMsg*);
uint32_t CAN1_StopMotor(uint8_t,CanRxMsg*);
uint32_t CAN1_SetEmmSpeedRaw(uint8_t,uint8_t,uint16_t,uint8_t,CanRxMsg*);
uint32_t CAN1_ReadEmmSystemSnapshot(uint8_t,CanRxMsg*,float*);
#endif
