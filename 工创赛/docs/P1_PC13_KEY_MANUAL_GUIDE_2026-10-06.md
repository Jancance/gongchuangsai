# PC13启动按键输入：手动接线、消抖与事件验证

2026-10-07最新验收：用户回报自锁复位/释放重接步骤“没有问题”，最终截图count1/create1/fault0、Key剩余栈932字节/heap15200字节。PC13自锁输入保持复位后须释放再重接的门禁通过（前半count0为用户口述，最后1有截图），不重复本项。下一步PA1两档选择，连接与语义已询问；未实现整车READY/运行锁存或动作允许。

2026-10-07器件澄清：用户实际使用自锁开关，下面“瞬时常开按钮”是此前教学假设，不能视为必须换件的正式要求。现有边沿代码可沿用：PC13-GND稳定断开先使能下一次事件，稳定接通产生一次，保持接通不重复。松开手不能解除自锁，需再次操作使其断开。复位边界操作改为保持自锁接通→RST/F5运行→count0；先F5恢复→解除自锁断开0.1秒以上→重新接通并保持2秒→count1。断开不是已实现的停车命令，也不是硬件急停；本阶段只计事件。该变更为实际器件/操作解释（E），RUN-01等R和官方Q不变，无需为此修改代码。

最新结果：用户事件/内存截图count3、create1、fault0、Key最小剩余栈233字=932字节、heap当前/历史最小15200字节，Idle/PSP正常。实际源确认序列测试已清理，Key已接入，基础输入及本轮内存门禁通过。下一单项按住启动键复位后运行→count0；先恢复运行再释放至少0.1秒、重新长按2秒→count1，确认开机抑制/长按不重复。只查看结果，不再改代码/下载；精密滤波时间/整车启动未由本图验收。

2026-10-06。当前UART独立接收/100字节/内存门禁通过（RX剩余868字节、heap16328字节）。用户先确认按钮未接，随后最新“继续按钮接好了”，已据用户更新连接状态，实际输入事件未验。此步只计按下事件，不接运动输出/整车启动状态机；按键不是RST。检查RUN-01/PWR-01（R）、LOC-03/RUN-06（E）；官方Q不变。

## 先清理USER/main.c临时RX序列比较

删除全局`volatile uint32_t g_serial_rx_test_mismatches = 0U;`，把相邻注释改为“接收任务创建结果和栈余量”。
在App_UartRxTask只删除以下内层比较及对应测试注释，保留外层WaitByte、计数/last更新和else内HWM：

```c
if (data != (uint8_t)('0' + (g_serial_rx_count % 10U)))
{
    g_serial_rx_test_mismatches++;
}
```

Watch中旧mismatches行可移除。其他UART/LED/蜂鸣/RTOS功能保留。

## 接线

用户当前使用自锁开关：选接通时导通、断开时不导通的一对触点，两端接主控PC13启动输入接口及GND，无需另接电源；再次操作解除自锁才相当于软件释放。此前2026-10-06教程按常开瞬时按钮描述，差异已在本文件顶部及执行日志记录，现不要求更换器件。若用四脚轻触或多触点开关，按真实触点通断识别端子，不以外观猜同组。没有按钮时仅在已确认PC13/GND启动接口接通/断开可模拟输入，不能拿BOOT0/RST或供电端子替代。

图纸和现有计划为PC13低有效、10k上拉3V3；软件另开内部上拉，避免未接按钮时浮动。只配置GPIOC13，不改UART PA10的NOPULL。初始释放需连续稳定20ms，按住上电不产生事件，长按不重复；1kHz扫描/20ms为既有工程候选，不是官方规则数值。

## USER/app_config.h最后endif前增加

```c
/* 启动按键连续稳定时间，单位：毫秒 */
#define APP_START_KEY_DEBOUNCE_MS    20U
```

保留用户已选择的蜂鸣次数等配置。

## HARDWARE/KEY/Key.h与Key.c

用户在实际工程D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/HARDWARE下建KEY文件夹。Key.h：

```c
#ifndef KEY_H
#define KEY_H

#include <stdint.h>

void Key_Init(void);
uint8_t Key_GetEvent(void);

#endif
```

Key.c：

```c
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "Key.h"

/* PC13低电平表示按下，先确认释放再接受按下事件 */
static uint8_t g_key_last_raw = 1U;
static uint8_t g_key_release_seen = 0U;
static TickType_t g_key_change_tick = 0U;

/* 初始化启动按键输入，使用上拉 */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    g_key_last_raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    g_key_release_seen = 0U;
    g_key_change_tick = 0U;
}

/* 稳定按下产生一次事件，长按不重复，上电按住不触发 */
uint8_t Key_GetEvent(void)
{
    uint8_t raw;
    TickType_t now;

    raw = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
    now = xTaskGetTickCount();

    if (raw != g_key_last_raw)
    {
        g_key_last_raw = raw;
        g_key_change_tick = now;
    }

    if ((TickType_t)(now - g_key_change_tick) <
        pdMS_TO_TICKS(APP_START_KEY_DEBOUNCE_MS))
    {
        return 0U;
    }

    if (raw != 0U)
    {
        g_key_release_seen = 1U;
    }
    else if (g_key_release_seen != 0U)
    {
        g_key_release_seen = 0U;
        return 1U;
    }

    return 0U;
}
```

## Keil及USER/main.c

把Key.c加入HARDWARE分组，C Include Paths追加`.\HARDWARE\KEY`，保留其他路径。main头部加`#include "Key.h"`，全局变量后加入：

```c
/* 启动按键事件数、输入任务创建结果及栈余量 */
volatile uint32_t g_start_key_count = 0U;
volatile BaseType_t g_key_task_create = pdFAIL;
volatile UBaseType_t g_key_stack_words = 0U;
```

在main前增加：

```c
/* 按既有输入方案每1毫秒扫描，只记录启动按键事件 */
static void App_KeyTask(void *argument)
{
    uint32_t diagnostic_count = 0U;

    (void)argument;

    while (1)
    {
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
        }

        diagnostic_count++;
        if (diagnostic_count >= 100U)
        {
            g_key_stack_words = uxTaskGetStackHighWaterMark(NULL);
            diagnostic_count = 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
```

main在USART1_Config后、创建任务前加入：

```c
/* 调度前初始化启动按键 */
Key_Init();
```

在原三个任务创建检查后、vTaskStartScheduler前加入：

```c
/* 本轮按键扫描任务，栈256字、优先级2 */
    g_key_task_create = xTaskCreate(App_KeyTask, "Key",
                                    256U, NULL, 2U, NULL);
    configASSERT(g_key_task_create == pdPASS);
```

此为第四业务/测试任务，栈256字、优先级2为候选，保留RX3/A2/B1；每1ms让出CPU，Key_GetEvent按真实tick稳定窗口判断，不使用忙等延时或EXTI13，避免既有机械输入EXTI线路共用问题。新任务内存需记录，本轮未冻结最终整车优先级/周期/栈。

## 编译证据与实物测试

私下复制实际main/config到工作区scratch并加入新Key模块及序列清理，用ARMCC5编译Key/main均0错误0警告，原用户源哈希未变。仅两源编译，未完整链接/下载；临时副本清理，保留pc13_key_private_compile_probe.json。实际文件创建/修改/编译/下载由用户操作。

保存、Rebuild、下载并F5运行，首次无按键g_start_key_count应0。短按/松开一次→1；再按住约2秒，计数变2后保持2；松开后再按一次→3。释放和按下应各超过20ms；长按上电应保持0，释放后新按才加1，但先完成普通短按/长按门禁再拓展此启动边界。保持LED闪烁。

Watch查看g_start_key_count、g_key_task_create（1）、g_key_stack_words、g_heap_free_bytes、g_heap_min_bytes、g_rtos_fault（0）；GPIO状态/事件与内存记录后才能通过本项。新输入识别只为后续一键启动铺底，完整起点选择PA1、锁存run_id、自检READY、动作允许及硬件安全仍另实现/验收，不让本测试事件直接触发电机。
