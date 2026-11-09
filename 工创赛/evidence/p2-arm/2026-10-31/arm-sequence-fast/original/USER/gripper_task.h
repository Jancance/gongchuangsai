#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"

/* Watch：先填angle，再写request=1；request=2撤销PWM。 */
extern volatile float g_jiazhua_debug_angle;
extern volatile uint32_t g_jiazhua_debug_request;
/* 0未接受/角度无效，1指令已写入，2已撤销信号，3未知请求。 */
extern volatile uint32_t g_jiazhua_debug_result;
extern volatile BaseType_t g_gripper_task_create;

void Gripper_TaskInit(void);

#endif
