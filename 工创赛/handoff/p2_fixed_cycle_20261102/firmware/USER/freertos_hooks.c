#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "power.h"

volatile uint32_t g_rtos_fault = 0U;
volatile uint32_t g_rtos_assert_line = 0U;

/* 停止执行，保留故障码供调试查看 */
static void App_RtosStop(uint32_t fault)
{
    g_rtos_fault = fault;
    __disable_irq();
    /* 故障时直接关闭VM，不等待任务运行 */
    Power_Disable();
    
    while (1)
    {
    }
}

/* 记录断言失败位置 */
void App_RtosAssertFailed(uint32_t line)
{
    g_rtos_assert_line = line;
    App_RtosStop(3U);
}

/* 内核动态分配失败 */
void vApplicationMallocFailedHook(void)
{
    App_RtosStop(1U);
}

/* 任务栈溢出 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    App_RtosStop(2U);
}
