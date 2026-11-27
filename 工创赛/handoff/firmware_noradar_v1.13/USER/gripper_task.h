#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H
#include <stdint.h>
#include "FreeRTOS.h"
extern volatile float g_jiazhua_debug_angle;
extern volatile uint32_t g_jiazhua_debug_request;
/* 1仅PWM写入；4未就绪/其他轴忙/动作组占用；2撤销信号，非张开。 */
extern volatile uint32_t g_jiazhua_debug_result;
extern volatile BaseType_t g_gripper_task_create;
extern volatile uint32_t g_gripper_signal_stop_count;
void Gripper_TaskInit(void);
void Gripper_SetAccess(uint32_t ready,uint32_t other_busy,uint32_t sequence_busy);
uint32_t Gripper_IsCommandBusy(void);
/* 动作组内部邮箱；返回票号，0为拒绝。结果0等待/1写PWM/2拒绝。 */
uint32_t Gripper_ControlledCommand(float angle);
uint32_t Gripper_ControlledResult(uint32_t ticket);
void Gripper_CancelControlled(void);
#endif
