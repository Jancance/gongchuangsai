#include "stm32f4xx_can.h"
uint32_t CAN1_ReadVersion(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadStatus(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadMotorOptions(uint8_t,CanRxMsg*);
uint32_t CAN1_ReadSpeed(uint8_t,CanRxMsg*,int32_t*);
uint32_t CAN1_ReadPosition(uint8_t,CanRxMsg*,uint8_t*,uint32_t*);
uint32_t CAN1_ExtensionExchange(const uint8_t*,uint8_t,CanRxMsg*);
uint32_t CAN1_ReadExtensionHomeParameters(uint8_t*,CanRxMsg*);
extern volatile uint32_t g_can_extension_home_done_notice;
uint32_t CAN1_StartExtensionHome(CanRxMsg*);
uint32_t CAN1_AbortExtensionHome(CanRxMsg*);
uint32_t CAN1_ReadExtensionHomeStatus(CanRxMsg*);
void CAN1_ExtensionHomeSessionEnd(void);
