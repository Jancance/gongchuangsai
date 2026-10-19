# 软件故障停机时关闭VM：手动补丁与静态验证

> 最新实机：断言fault3/VM0通过，临时变量/注入if已从真实main清理，永久末尾断言和hooks关VM保留，用户截图fault0/VM请求0恢复正常。Watch旧测试变量cannot evaluate可删，不重做断言。下一项P1_CAN_STATUS_READ_MANUAL_GUIDE_2026-10-11.md的3A只读状态；下文为已完成验证接入记录，完整看门狗/故障安全仍待验。

当前四台CAN1单节点及共总线版本读取通过；复位默认VM0V、CPU暂停且请求1时硬件开关关VM0V均有实测。现有USER/freertos_hooks.c的App_RtosStop仅记录故障、禁中断、空循环，没有撤销PE2允许输出。下一项在公共停机入口关闭VM，并用一次临时断言验证该路径，不发电机运动命令。

以下修改由用户在原项目手动完成；助手只编译私人副本，不改真实固件。

## 1. freertos_hooks.c增加Power_Disable

文件头现有include后加入：

```c
#include "power.h"
```

在App_RtosStop的__disable_irq后加入一行：

```c
    /* 故障时直接撤销VM允许输出，不等待任务运行 */
    Power_Disable();
```

修改后该函数为：

```c
static void App_RtosStop(uint32_t fault)
{
    g_rtos_fault = fault;
    __disable_irq();

    /* 故障时直接撤销VM允许输出，不等待任务运行 */
    Power_Disable();

    while (1)
    {
    }
}
```

Power_Disable是已有标准库GPIO直接操作，不调用FreeRTOS、不等待节拍，在禁中断后仍可执行。当前Power_Init在创建任务前完成；早期尚未配置GPIO时的故障不靠此单行代表完整安全链。现有故障码1堆失败、2栈溢出、3断言，共用此入口；本轮只实测断言路径，不制造真实内存损坏。

## 2. main.c临时断言注入

在全局变量区加入：

```c
/* 临时断言测试：默认关闭，验证后删除 */
volatile uint32_t g_test_force_assert = 0U;
```

在App_TestTaskA的while(1)内、g_test_count_a++之前加入：

```c
        /* 本轮只验证公共故障停机的VM关断 */
        if (g_test_force_assert != 0U)
        {
            configASSERT(0);
        }
```

测试标志默认0，不自动启动注入。保留原地址查询和VM默认关闭，未添加使能/速度/位置等运动命令。

## 3. 用户静态验证

1. 保持硬件开关关闭，先撤销上轮软件请求0/F5；手动补丁、Rebuild成功后下载，先F5运行主控到灯闪。
2. 硬件开关置既有软件控制允许档，Stop设VM请求1/F5，等驱动启动，并确认VM实际有电。四轮静止，只发过版本读取。
3. Stop将g_test_force_assert设1，F5约1秒再Stop。预期停在App_RtosStop、g_rtos_fault=3、实测VM近0V；灯停止闪烁符合禁中断诊断循环，不代表关断失败。软件VM请求可仍1，其值不等于PE2实际输出。
4. 若VM未关闭，使用已实测可独立关断的硬件开关断VM，保留故障截图/电压。
5. 测试完成保持硬件关闭；此故障循环不会因把测试标志改0就恢复，复位才退出。移除main的临时变量和注入if，保留hooks中的Power_Disable，用户再次Rebuild/下载完成清理后再接续。

这一试验验证断言公共停机入口正常执行时能关闭VM，不代表任意内存损坏、所有ISR上下文、看门狗/自动重启互锁或完整硬件急停安全已验。后续仍按已完成成品板及原八阶段推进，实车实际运动从完整装机载物低速开始（MOT-04）。

## 需求与证据

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E）；官方Q不变。本轮私人hooks/main候选单元编译、原源码哈希及清理证据单独记录；用户真实构建、断言VM实测及临时注入清理仍待完成。
