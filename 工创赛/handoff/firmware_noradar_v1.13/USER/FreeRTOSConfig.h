#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"
#include "freertos_hooks.h"

/* 抢占调度；每个节拍为1毫秒 */
#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      1000U

/* 使用32位节拍计数 */
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS

#define configMAX_PRIORITIES                    5U
#define configMINIMAL_STACK_SIZE                128U
#define configMAX_TASK_NAME_LEN                 16U

/* 使用heap_4动态分配，初始堆大小为20KB */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   (20U * 1024U)

/* 首次验证使用的功能配置 */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configUSE_MUTEXES                       1

/* 开启堆、栈和断言诊断 */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configASSERT(x)                         do { if (!(x)) App_RtosAssertFailed(__LINE__); } while (0)

/* F407具有4位NVIC优先级，数值越小优先级越高 */
/* 此段宏会用于ARMCC5内联汇编，整数不加U后缀 */
#define configPRIO_BITS                         __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 绑定启动文件中的三个异常入口 */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

/* 开启后续测试任务使用的原生接口 */
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_xTaskGetSchedulerState          1

/* 开启任务历史最小剩余栈查询 */
#define INCLUDE_uxTaskGetStackHighWaterMark    1

#endif
