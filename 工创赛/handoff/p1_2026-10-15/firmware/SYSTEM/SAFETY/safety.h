#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include "FreeRTOS.h"

/* 当前关键任务编号：不把状态灯任务当成安全证明 */
#define SAFETY_TASK_APP 0U
#define SAFETY_TASK_RX  1U
#define SAFETY_TASK_KEY 2U

/* 应用监督故障，与g_rtos_fault分开记录 */
extern volatile uint32_t g_safety_fault;
extern volatile uint32_t g_safety_ready;
extern volatile uint32_t g_safety_count;
/* CAN健康和首次CAN故障上下文；节点位0～3对应地址1～4 */
extern volatile uint32_t g_can_guard_ready;
extern volatile uint32_t g_can_guard_bad_nodes;
/* 原因：1必要采集关闭，2有效数据缺失/过期，3堵转/保护 */
extern volatile uint32_t g_can_guard_cause;

void Safety_Init(void);
void Safety_Report(uint32_t task_id);
void Safety_Update(uint32_t vm_request, uint32_t can_test_result);

void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks);

#endif
