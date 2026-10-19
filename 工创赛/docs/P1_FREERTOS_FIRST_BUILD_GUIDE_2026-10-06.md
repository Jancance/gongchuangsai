# F407标准库＋FreeRTOS首次编译：用户手动操作

日期：2026-10-06。工程位置由用户保留在`D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/`。本轮只提供修改位置和中文片段，未创建/修改该工程的源码或uvprojx。目标先做到内核编译/链接通过，下一单项再创建两个任务验证调度；本步不宣称RTOS已运行。

## 已核对输入和要求

- 当前内核为V11.3.1，include/task.h版本宏已读；portable/RVDS/ARM_CM4F及heap_4文件存在。
- 当前ARMCC5.06、C99已勾选、F407VE/FPU2，工程已有CORE/USER/标准库include路径；基础HSE/PLL板上已进入main并读到168MHz。
- F4启动文件有SVC/PendSV/SysTick弱入口，本次通过端口函数名称映射提供强定义，不删除向量/弱入口，不再另写同名空C处理函数。
- R：PWR-01单电池等既有要求不变；E：LOC-03标准库＋FreeRTOS、RUN-06故障出口。官方Q项不变。配置数字是初始教学候选，非最终运动限值、非官方规则。
- [FreeRTOS Cortex-M中断约束](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)：F407使用4个优先级位；调用FromISR的中断受max-syscall阈值限制，后续逐项审查。

## 1. 用户在Keil加入文件和路径

新增分组FreeRTOS，只加入以下5个源文件（均相对实际工程根）：

```text
FreeRTOS/tasks.c
FreeRTOS/list.c
FreeRTOS/queue.c
FreeRTOS/portable/RVDS/ARM_CM4F/port.c
FreeRTOS/portable/MemMang/heap_4.c
```

其他架构/编译器port.c和其他heap实现不加入。后续用到软件定时器、事件组、流缓冲时再加入同版本对应源文件。

Options for Target → C/C++ → Include Paths在现有路径后增加：

```text
.\FreeRTOS\include
.\FreeRTOS\portable\RVDS\ARM_CM4F
```

现有`.\USER`、芯片和标准库宏保留。Floating Point Hardware应为Single Precision，符合port.c的__TARGET_FPU_VFP要求；C99当前已经启用。

## 2. 新建USER/freertos_hooks.h

```c
#ifndef FREERTOS_HOOKS_H
#define FREERTOS_HOOKS_H

#include <stdint.h>

/* 诊断代码：0正常，1堆分配失败，2任务栈溢出，3断言失败 */
extern volatile uint32_t g_rtos_fault;
extern volatile uint32_t g_rtos_assert_line;

void App_RtosAssertFailed(uint32_t line);

#endif
```

## 3. 新建USER/FreeRTOSConfig.h

```c
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"
#include "freertos_hooks.h"

/* 抢占调度，系统时钟在启动调度器前更新；每个节拍为1毫秒 */
#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      1000U
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                    5U
#define configMINIMAL_STACK_SIZE                128U
#define configMAX_TASK_NAME_LEN                 16U

/* 使用heap_4动态分配；20KB为首次验证候选，后续按实测调整 */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   (20U * 1024U)

/* 首次编译所需功能；暂不创建软件定时器任务 */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TIMERS                        0
#define configUSE_CO_ROUTINES                   0
#define configUSE_MUTEXES                       1

/* 保留堆、栈和断言诊断，出错时停在可观察位置 */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configASSERT(x)                         do { if (!(x)) App_RtosAssertFailed(__LINE__); } while (0)

/* F407的4位NVIC优先级；数值越小，中断优先级越高 */
#define configPRIO_BITS                         __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 将端口函数绑定到F407启动文件的三个异常入口 */
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

/* 开启后续最小任务验证使用的原生接口 */
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_xTaskGetSchedulerState          1

#endif
```

仅定义新的32位tick宏，不再同时定义configUSE_16_BIT_TICKS；当前V11.3.1对二者同时出现报错。128为StackType_t字数（本端口32位，每字4字节），不是字节数。5个任务优先级和20KB heap只是首个验证候选，后续任务/中断数量按资源实测调整。

2026-10-06 ARMCC5兼容性修正：这四行优先级宏使用无U后缀的整数，因为MAX_SYSCALL宏直接进入RVDS port的内联汇编mov立即数。临时配置对照实际重现A1586E并验证去后缀后port.c可编译；其他仅供C使用的U后缀保留。阈值仍为5<<4=0x50，内核优先级仍为15<<4=0xF0，优先级语义不变。

## 4. 新建USER/freertos_hooks.c并加入USER分组

```c
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"

volatile uint32_t g_rtos_fault = 0U;
volatile uint32_t g_rtos_assert_line = 0U;

/* 基础诊断停机，保留故障码供调试查看 */
static void App_RtosStop(uint32_t fault)
{
    g_rtos_fault = fault;
    __disable_irq();
    while (1) { }
}

/* 记录断言所在源文件行号 */
void App_RtosAssertFailed(uint32_t line)
{
    g_rtos_assert_line = line;
    App_RtosStop(3U);
}

/* 内核动态分配失败的诊断入口 */
void vApplicationMallocFailedHook(void)
{
    App_RtosStop(1U);
}

/* 任务栈溢出的诊断入口 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    App_RtosStop(2U);
}
```

该停机是首个诊断入口，不能代替最终硬件急停/健康监督，不在此处伪装所有执行器已断电。后续带动作模块时另接安全出口。

## 5. 首次Rebuild检查

main.c目前维持SystemCoreClockUpdate与空循环，先不调用vTaskStartScheduler。保存全部新文件，确认最后有换行，Rebuild；编译输出应包含5个内核/port/heap源和freertos_hooks.c，目标0错误0警告。用户提供完整输出后再检查链接器是否使用正确port强定义和内存余量。

下一单项：添加NVIC_PriorityGroup_4、两个无执行器动作的测试任务、逐一检查xTaskCreate结果并启动调度；使用实时计数/暂停后的tick和g_rtos_fault验证任务与节拍，同时审查SVC/PendSV/SysTick入口和栈/堆。配置通过不等于任务已运行。

本轮验证仅为本地文件版本、编译器/启动符号、必选宏和来源核对；未创建该工程源码、未自动编译/下载、未测RTOS调度。用户所有修改按中文注释手动操作。
