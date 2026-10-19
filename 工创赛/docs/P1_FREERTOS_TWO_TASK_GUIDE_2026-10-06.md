# F407 FreeRTOS两个最小任务：用户手动接入与验收

日期：2026-10-06。实际工程仍在D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。用户已完成内核零错误零警告编译，但main尚未启动调度。本说明只给三个插入/替换片段，助手不修改该工程源码。

## 输入与范围

当前FreeRTOS V11.3.1、ARMCC5、RVDS/ARM_CM4F、heap_4、1ms tick和20KB heap已核对；SVC/PendSV/SysTick链接到port.o强定义。此次为两个无执行器动作任务的调度/节拍验证，非整车控制验收。LOC-03、RUN-06为E，PWR-01等R及官方Q无变化。

## 1. USER/main.c头部增加

保留stm32f4xx.h，在后面加入：

```c
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
```

## 2. 在int main(void)之前插入

```c
/* 测试状态，均可在Watch中观察 */
volatile uint32_t g_test_count_a = 0U;
volatile uint32_t g_test_count_b = 0U;
volatile TickType_t g_test_tick = 0U;
volatile BaseType_t g_test_create_a = pdFAIL;
volatile BaseType_t g_test_create_b = pdFAIL;
volatile uint32_t g_test_scheduler_returned = 0U;

/* 任务A：每100毫秒计数一次，并保存节拍快照 */
static void App_TestTaskA(void *argument)
{
    (void)argument;
    while (1)
    {
        g_test_count_a++;
        g_test_tick = xTaskGetTickCount();
        vTaskDelay(pdMS_TO_TICKS(100U));
    }
}

/* 任务B：每500毫秒计数一次 */
static void App_TestTaskB(void *argument)
{
    (void)argument;
    while (1)
    {
        g_test_count_b++;
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}
```

任务A、B分别更新自己的计数，A单独写tick快照；均使用原生vTaskDelay阻塞，不忙等。测试周期为100/500ms，不代表最终底盘周期或运动限值。

## 3. 替换原main函数

```c
int main(void)
{
    /* 更新系统时钟，配置全部优先级位为抢占优先级 */
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* 栈深度256个字，本端口每字4字节；检查创建结果 */
    g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                 256U, NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);

    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);

    /* 正常启动后由任务运行，通常不会从这里返回 */
    vTaskStartScheduler();

    /* 调度器异常返回，保留状态并进入现有断言诊断 */
    g_test_scheduler_returned = 1U;
    configASSERT(0);
    while (1) { }
}
```

任务优先级2/1仅为本次候选，5级配置范围内有效。每个任务栈256个32位字，即1024字节，实际栈/堆余量仍需后续实测。先创建两个任务并检查返回值，再启动调度；若内存分配失败先进入既有故障钩子，不继续假装成功。

## 4. 用户编译、下载与运行观察

保存、Rebuild通过后下载，使用当前实际可工作的硬件调试器；删除旧main循环和任务内部断点，避免暂停本身影响计数。Keil的Debug→Breakpoints窗口允许禁用/删除全部断点，见[官方说明](https://www.keil.com/support/man/docs/uv4cl/uv4cl_db_dbg_breakpnts.htm)。

调度启动后通常不返回main最后的while；CPU暂停时落在Idle/内核循环并不自动代表故障。通过连续两次运行后的变量增量判定，而不是等固定黄色箭头位置。

Watch添加：

```text
g_test_create_a
g_test_create_b
g_test_count_a
g_test_count_b
g_test_tick
g_test_scheduler_returned
g_rtos_fault
g_rtos_assert_line
*(unsigned int *)0xE000E014
```

先F5运行约2秒后停止，保存截图；再运行约2秒后停止，保存第二张。候选通过条件：

- 两个create值均1（pdPASS），g_test_scheduler_returned=0，g_rtos_fault=0。
- 两计数持续增加；两次暂停之间A增量约20、B约4，比例大约5:1，不对首次绝对计数硬判。
- tick快照持续增长，1000tick代表名义1秒；A只每100ms刷新，快照可能比暂停时真实tick早不满100ms。
- SysTick LOAD目标167999（0x0002903F），对应168MHz/1000-1。读寄存器是运行配置验证，物理节拍精度仍受实际时钟和操作计时影响。
- 无复位、HardFault或进入故障钩子；停止/恢复后仍继续计数。

若创建/调度失败，提供完整构建输出、Watch、当前PC及Call Stack，按已观察故障定位；不要为了让任务继续而删除configASSERT或栈/堆诊断。

## 本次助手验证及边界

使用临时main副本和实际工程现有内核对象执行ARMCC5编译/armlink链接：均通过，临时镜像Code=5408/RO=424/RW=156/ZI=22292。用户原main字节未改变，临时源/对象/镜像在结果记录后清理；只保留evidence/p1-foundation/2026-10-06/two_task_compile_link_probe.json。测试不包含实际板上调度，用户正式修改后的构建数值可以因选项而不同。

当前已完成编译级核对，任务运行/节拍和故障观察尚待用户截图。两个任务通过之后仍要继续基础栈/堆、故障监督及相关外设门禁，不把本次演示当整车RTOS验收。

