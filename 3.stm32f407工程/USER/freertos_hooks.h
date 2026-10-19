#ifndef FREERTOS_HOOKS_H
#define FREERTOS_HOOKS_H

#include <stdint.h>

/* 故障码：0正常，1堆分配失败，2任务栈溢出，3断言失败 */
extern volatile uint32_t g_rtos_fault;
extern volatile uint32_t g_rtos_assert_line;

void App_RtosAssertFailed(uint32_t line);

#endif
