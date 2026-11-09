#include "gripper_task.h"
#include "gripper.h"
#include "gripper_config.h"
#include "task.h"

volatile float g_jiazhua_debug_angle = GRIPPER_DEBUG_ANGLE_DEG;
volatile uint32_t g_jiazhua_debug_request = 0U;
volatile uint32_t g_jiazhua_debug_result = 0U;
volatile BaseType_t g_gripper_task_create = pdFAIL;

static TaskHandle_t s_gripper_task = NULL;

/* 调试请求集中在本任务执行，不占用主循环或串口接收任务。 */
static void Gripper_Task(void *argument)
{
    uint32_t request;
    float angle;
    (void)argument;

    for (;;)
    {
        /* 一起取得请求与参数，避免处理过程参数被其他任务改动。 */
        taskENTER_CRITICAL();
        request = g_jiazhua_debug_request;
        angle = g_jiazhua_debug_angle;
        g_jiazhua_debug_request = 0U;
        taskEXIT_CRITICAL();

        if (request == 1U)
        {
            g_jiazhua_debug_result = jiazhua(angle);
        }
        else if (request == 2U)
        {
            Gripper_Stop();
            g_jiazhua_debug_result = 2U;
        }
        else if (request != 0U)
        {
            g_jiazhua_debug_result = 3U;
        }
        vTaskDelay(pdMS_TO_TICKS(GRIPPER_TASK_PERIOD_MS));
    }
}

void Gripper_TaskInit(void)
{
    /* 只创建一次；初始化不会自动发出90度或其他角度命令。 */
    if (s_gripper_task == NULL)
    {
        g_gripper_task_create = xTaskCreate(Gripper_Task, "Gripper",
            GRIPPER_TASK_STACK_WORDS, NULL, GRIPPER_TASK_PRIORITY,
            &s_gripper_task);
        configASSERT(g_gripper_task_create == pdPASS);
    }
}
