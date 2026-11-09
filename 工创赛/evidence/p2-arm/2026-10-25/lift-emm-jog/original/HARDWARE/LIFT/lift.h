#ifndef LIFT_H
#define LIFT_H
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

extern const uint32_t g_lift_address;
/* 0等供电，1等驱动启动，2读版本，3读状态，4读取完成，5读取失败。 */
extern volatile uint32_t g_lift_state;
extern volatile uint32_t g_lift_read_request;
extern volatile uint32_t g_lift_read_attempts;
extern volatile uint32_t g_lift_version_result;
extern volatile uint32_t g_lift_status_result;
extern CanRxMsg g_lift_version_reply;
extern CanRxMsg g_lift_status_reply;

/* 只由现有CAN任务调用；本阶段不包含使能/失能/运动。 */
void Lift_Service(uint32_t power_ready);
#endif
