# 故障监督第一步：关键任务心跳与VM锁定

> 最新清理后启动通过（2026-10-12）：图d28f0034为VM请求0/RTOS故障0/监督fault0/局部ready1/创建1，监督余栈233字/APP208字、heap最小14072B，用户VM0V；已删临时变量cannot evaluate为正常。关键任务监督本轮（含Key失联关断/恢复锁定及清理）闭环，不重做下方历史测试。下一项P1_CAN_CONTINUOUS_STATUS_MANUAL_GUIDE_2026-10-12.md静态轮询采集，之后CAN故障接入Safety再看门狗；其他任务/调度异常及完整安全仍待验。

> 最新清理构建通过（2026-10-12）：用户已删除临时hold_key全局/分支，真实main只读确认无该符号、三处Safety_Report/监督任务/初始化保留，hooks故障关VM保留；正式Rebuild链接0错误0警告，Code14080/RO424/RW332/ZI22412、3秒。唯一下一项下载清理后的程序，F5约2秒后暂停，验RTOS故障0/监督故障0/局部ready1/VM请求0及实际VM0。Watch临时行可删，已有失联关断/恢复锁定不重做；正常恢复后接CAN持续健康。

> 最新失联/锁定实测通过（2026-10-12）：用户确认触发前VM请求1且VM已有电压；失联时hold1/fault04/ready0/VM0，恢复hold0后新图49c973dd仍VM请求1/fault04/ready0/RTOS故障0/VM0、count1120→1255。Key失联实际关VM及恢复后锁定已闭环。唯一下一项硬件关VM/退出调试，仅删main临时测试全局和循环头hold分支，保留Safety_Report及正式SAFETY/hook逻辑，用户Rebuild后正常启动检查。此前待确认条目为历史，不重做注入；CAN持续健康/看门狗与完整安全仍待验。

> 最新实机失联检测（2026-10-12）：图d9c72e0c为hold_key1/VM请求1、监督fault04/ready0、RTOS故障0；用户实测VM0V。失联识别及故障禁止状态已过，触发前VM是否已确认有电压尚未报告，不补造开启→关闭过程。下一项保持本次运行和硬件允许档、VM请求1，仅恢复hold_key0/F5约1秒，验故障04/ready0及VM0保持，并补触发前供电确认；两者到位再清理注入。此前构建/正常运行说明保留历史。

> 最新构建（2026-10-12）：用户已加入第6节临时Key失联变量/分支，完整Rebuild及链接0错误0警告，Code14108/RO424/RW336/ZI22408、3秒；真实main只读核对位置正确、正常按键/心跳保留。下一项用户下载运行，确认VM开启后hold_key1/F5约1秒，验监督fault04/ready0、RTOS故障0及实测VM0V；随后恢复hold0验证锁定，最后清理。本条替代此前“未收到完整日志”，不要求重复构建，实物失联/锁定仍待验。

> 用户实机正常运行已过（2026-10-12）：新图7d651aa1的count从0x126增至0x220、创建1/故障0/局部ready1，监督余栈233字=932B、heap最小14072B，VM请求0/RTOS故障0；用户补测VM0V。真实三文件及Keil接入已核，正常启动/计数增长/当前默认关闭门禁闭环。唯一下一项第6节临时Key失联代码和用户Rebuild，再实测关VM/故障锁定、清理。第1～5节保留历史，不要求重复；未收到完整Rebuild日志，不代填0错0警告。

> 私人候选编译已完成：safety.c、接入main.c及临时失联测试版main三个单元均ARMCC5.06 update5 build528下0错误0警告。真实main哈希前后不变，私人文件已清理；未链接、下载或实机运行，不替代第5节用户构建/验收。

本步由用户手动修改实际F407工程；保留SPL、FreeRTOS原生API、原版本/状态查询、灯/蜂鸣/串口/按键及RTOS致命错误关VM。助手只准备本指南和私人编译检查，不改实际固件。

这是工程实现（E），不是新增竞赛规则。四台3A基本状态读取已经通过；本步先验关键任务失联与VM锁定。CAN连续采集、新鲜度、驱动器保护、看门狗和正式READY/启动仍待后续完成。以下g_safety_ready仅表示本步任务检查通过，不是整车READY或运动授权。

## 1. 新建SYSTEM/SAFETY/safety.h

在实际工程SYSTEM下新建SAFETY文件夹和safety.h，加入以下内容。头文件无需单独加入Keil编译列表。

```c
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

/* 当前关键任务编号：不把状态灯任务当成安全证明 */
#define SAFETY_TASK_APP 0U
#define SAFETY_TASK_RX  1U
#define SAFETY_TASK_KEY 2U

/* 应用监督故障，与g_rtos_fault分开记录 */
extern volatile uint32_t g_safety_fault;
extern volatile uint32_t g_safety_ready;
extern volatile uint32_t g_safety_count;

void Safety_Init(void);
void Safety_Report(uint32_t task_id);
void Safety_Update(uint32_t vm_request, uint32_t can_test_result);

#endif
```

## 2. 新建SYSTEM/SAFETY/safety.c

```c
#include "safety.h"
#include "FreeRTOS.h"
#include "task.h"
#include "power.h"

/* 仅用于当前静态平台，后续运动控制需另行冻结更快的限值 */
#define SAFETY_STARTUP_MS 1000U
#define SAFETY_APP_MS     1000U
#define SAFETY_RX_MS       300U
#define SAFETY_KEY_MS      100U

volatile uint32_t g_safety_fault = 0U;
volatile uint32_t g_safety_ready = 0U;
volatile uint32_t g_safety_count = 0U;

static TickType_t s_last_tick[3];
static uint32_t s_seen_mask = 0U;
static TickType_t s_start_tick = 0U;
static uint32_t s_started = 0U;
static uint32_t s_was_ready = 0U;

/* 调度前调用，默认不允许VM */
void Safety_Init(void)
{
    uint32_t i;

    g_safety_fault = 0U;
    g_safety_ready = 0U;
    g_safety_count = 0U;
    s_seen_mask = 0U;
    s_start_tick = 0U;
    s_started = 0U;
    s_was_ready = 0U;
    for (i = 0U; i < 3U; i++)
    {
        s_last_tick[i] = 0U;
    }
    Power_Disable();
}

/* 由相应任务在一轮实际工作结束后报告，不由其他任务代报 */
void Safety_Report(uint32_t task_id)
{
    if (task_id < 3U)
    {
        taskENTER_CRITICAL();
        s_last_tick[task_id] = xTaskGetTickCount();
        s_seen_mask |= 1UL << task_id;
        taskEXIT_CRITICAL();
    }
}

/* 只有监督任务调用；故障锁定后，正常心跳恢复也不能自动开VM */
void Safety_Update(uint32_t vm_request, uint32_t can_test_result)
{
    TickType_t now;
    TickType_t last[3];
    uint32_t seen;
    uint32_t bad = 0U;
    uint32_t i;

    g_safety_count++;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    seen = s_seen_mask;
    for (i = 0U; i < 3U; i++)
    {
        last[i] = s_last_tick[i];
    }
    taskEXIT_CRITICAL();

    if (s_started == 0U)
    {
        s_start_tick = now;
        s_started = 1U;
    }

    if (((seen & 0x01U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_APP]) >
         pdMS_TO_TICKS(SAFETY_APP_MS)))
    {
        bad |= 0x01U;
    }
    if (((seen & 0x02U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_RX]) >
         pdMS_TO_TICKS(SAFETY_RX_MS)))
    {
        bad |= 0x02U;
    }
    if (((seen & 0x04U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_KEY]) >
         pdMS_TO_TICKS(SAFETY_KEY_MS)))
    {
        bad |= 0x04U;
    }

    /* 回环结果0表示还在启动，其他非1结果表示已失败 */
    if ((can_test_result != 0U) && (can_test_result != 1U))
    {
        g_safety_fault |= 0x10U;
    }
    if ((s_was_ready != 0U) ||
        ((TickType_t)(now - s_start_tick) >=
         pdMS_TO_TICKS(SAFETY_STARTUP_MS)))
    {
        g_safety_fault |= bad;
        if (can_test_result == 0U)
        {
            g_safety_fault |= 0x08U;
        }
    }

    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    if ((g_safety_ready == 1U) && (vm_request == 1U))
    {
        Power_Enable();
    }
    else
    {
        Power_Disable();
    }
}
```

故障码是位掩码：01任务A、02串口接收任务、04按键任务、08回环启动未完成、10回环失败；可能组合。没有添加软件清除接口，普通任务恢复后故障仍保持，复位后重新检查且VM请求默认0。调试器强行改故障变量不属于恢复流程。

任务A暂兼版本/状态查询，两个请求同轮执行时原有等待预算较长，因此本步APP限值1秒，只用于静态平台。RX在没有串口字节时每100ms正常返回也报告；不把“没有收到字符”判成任务死亡。运行中P2心跳200ms减速/500ms停止目标不由这组测试参数替代。

## 3. Keil配置

在Project树SYSTEM分组中Add Existing Files，加入新建的safety.c。Options for Target → C/C++ → Include Paths追加 `.\SYSTEM\SAFETY`（与现有 `.\SYSTEM\CAN` 同级，现有路径保持）。

## 4. USER/main.c增量修改

在现有头文件区添加：

```c
#include "safety.h"
```

在现有任务创建结果变量附近添加：

```c
/* 监督任务创建结果及历史最小剩余栈 */
volatile BaseType_t g_safety_task_create = pdFAIL;
volatile UBaseType_t g_safety_stack_words = 0U;
```

任务A的while中，删除下列原有整块供电判断（不要删除两段CAN查询）：

```c
/* 临时供电测试，由现有任务A处理 */
if (g_vm_test_request == 1U)
{
    Power_Enable();
}
else
{
    Power_Disable();
}
```

现在正常供电输出由监督任务单独管理；RTOS故障hooks里的Power_Disable必须保留。任务A每轮末尾 `vTaskDelay(pdMS_TO_TICKS(100U));` 前添加：

```c
/* 一轮查询和平台工作结束后，报告真实进展 */
Safety_Report(SAFETY_TASK_APP);
```

串口接收任务App_UartRxTask的while中，在接收if/else整块之后、while结束大括号前添加：

```c
/* 有数据或正常空闲等待返回，均表示任务仍在运行 */
Safety_Report(SAFETY_TASK_RX);
```

按键任务App_KeyTask每轮末尾 `vTaskDelay(pdMS_TO_TICKS(1U));` 前添加：

```c
Safety_Report(SAFETY_TASK_KEY);
```

在 `int main(void)` 前添加监督任务函数：

```c
/* 监督不等待CAN或串口，检查后立即让出CPU */
static void App_SafetyTask(void *argument)
{
    (void)argument;

    while (1)
    {
        Safety_Update(g_vm_test_request, g_can_test_result);
        g_safety_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}
```

main中Power_Init后、任何xTaskCreate之前添加：

```c
Safety_Init();
```

main中原四个任务创建/断言之后、vTaskStartScheduler之前添加：

```c
/* 优先级4，高于当前接收3、任务A/按键2；无阻塞外设操作 */
g_safety_task_create = xTaskCreate(App_SafetyTask, "Safety",
                                  256U, NULL, 4U, NULL);
configASSERT(g_safety_task_create == pdPASS);
```

保留原四任务栈和优先级、UART中断优先级、FreeRTOSConfig、CAN两个手动查询入口及所有hooks；本步不加IWDG喂狗。

## 5. 本次先验正常启动

先保持硬件开关在关VM档，退出调试，完成上述修改、保存、Rebuild；将完整构建结果发回。通过后用户下载运行，F5让程序运行约2秒再暂停，查看：

```text
g_vm_test_request
g_rtos_fault
g_safety_task_create
g_safety_fault
g_safety_ready
g_safety_count
g_safety_stack_words
g_heap_min_bytes
```

预期VM请求0、实际VM0V、RTOS故障0、监督创建1、监督故障0、局部任务检查ready1，count恢复运行后持续增加。栈/堆以实测为准，不预填。CPU暂停时软件监督无法继续执行，已有独立硬件切断仍有效；本步尚未加看门狗。

本轮先提交构建，再按上述正常运行门禁验收；不重复四台手动查询或断言注入。之后再做任务失联/锁定测试及CAN健康接入，不能只凭“能编译”宣布监督完成。

## 6. 下一轮任务失联验证说明

仅在第5节正常运行通过后执行，保持静态无运动命令。临时在main全局添加 `volatile uint32_t g_safety_test_hold_key = 0U;`，App_KeyTask的while一进入时加入：

```c
/* 临时测试：模拟按键任务失联，监督任务仍可运行 */
if (g_safety_test_hold_key == 1U)
{
    vTaskDelay(pdMS_TO_TICKS(1U));
    continue;
}
```

用户构建/下载、正常运行后恢复硬件允许档，请求VM1确认供电；设置测试变量1/F5，约1秒后暂停，预期g_safety_fault含04、ready0、VM0V、g_rtos_fault0。把测试变量改0/F5，故障与VM关闭仍保持，说明没有自动重新允许。之后硬件关VM，删除临时全局和if，保留正式监督；用户重新构建/下载后正常启动验证，不删或强清正式故障码。

这只验证允许调度仍运行时某个关键任务失联。高优先级死循环/关闭中断/调度器停止仍须看门狗与硬件链路覆盖；监控任务不覆盖自身彻底挂住。本步没有把电机手动快照当持续CAN健康，后续CAN监督未接入前不放行实际运动。

## 7. 需求和证据

已检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），官方Q不变。四台状态原始帧与汇总在evidence/p1-foundation/2026-10-12/four_motor_status_basic_pass.json；私人编译结果另记。用户实际三文件添加/构建链接、监督任务时序/栈堆、任务失联关VM/故障锁定仍待证据。助手不改实际源码、PCB或代发硬件命令。
