# PB0有源蜂鸣器候选：可调次数与分时更新手动教学

2026-10-06。用户明确“不知道是不是有源，先按照有源来做”，这是已授权的工程候选（E），不是器件类型已确认。按所给原理图PB0经S8050驱动，高电平开启、低电平关闭。检查LOC-03、RUN-06（E）、PWR-01（R）；官方Q不变。PB1舵机所用TIM3周期不改。

最新实物结果：0U/1U/3U均经用户确认对应静音/次数、结束关闭且LED继续闪，当前可调次数实物检查闭环，不再重复。当前源码计数已为2U，按用户选择保留，不自动恢复1U；原默认建议为1U。下一项USART1调试发送。完整型号与独立时间测量未提供，提示含义仍仅基础平台就绪。

实际工程保留D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程，以下由用户逐步手动填写，助手不创建固件文件、不改uvprojx。当前声响仅验证已通过的基础平台就绪；不是雷达/整车初始化完成。

## 1 参数入口USER/app_config.h

```c
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 基础初始化提示次数：0静音，1默认，可改为3 */
#define APP_INIT_BEEP_COUNT    1U
/* 单次鸣叫及两次之间的间隔，单位毫秒 */
#define APP_BEEP_ON_MS         200U
#define APP_BEEP_GAP_MS        200U

#endif
```

当前任务A每100ms调用更新，响/停时长建议用100ms整数倍；时长约值受调度影响，不以此产生精确声频。

## 2 HARDWARE/Buzzer/Buzzer.h

用户新建Buzzer目录及.h/.c。Buzzer.h：

```c
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Start(uint32_t count);
void Buzzer_Update(void);

#endif
```

## 3 HARDWARE/Buzzer/Buzzer.c

```c
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "Buzzer.h"

/* 当前鸣叫也计入剩余次数；只由任务A操作 */
static uint32_t g_buzzer_remaining = 0U;
static uint8_t g_buzzer_on = 0U;
static TickType_t g_buzzer_phase_tick = 0U;

/* 初始化PB0，默认关闭蜂鸣器 */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    g_buzzer_remaining = 0U;
    g_buzzer_on = 0U;
}

/* 调度启动后请求一组提示音，0次保持关闭 */
void Buzzer_Start(uint32_t count)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    g_buzzer_remaining = count;
    g_buzzer_on = 0U;

    if (count != 0U)
    {
        g_buzzer_phase_tick = xTaskGetTickCount();
        g_buzzer_on = 1U;
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
    }
}

/* 分时推进响停，不忙等、不调用延时 */
void Buzzer_Update(void)
{
    TickType_t now;
    TickType_t duration;

    if (g_buzzer_remaining == 0U)
    {
        return;
    }

    now = xTaskGetTickCount();
    duration = pdMS_TO_TICKS(g_buzzer_on ?
                            APP_BEEP_ON_MS : APP_BEEP_GAP_MS);

    /* 无符号时间差兼容节拍计数回绕 */
    if ((TickType_t)(now - g_buzzer_phase_tick) < duration)
    {
        return;
    }

    g_buzzer_phase_tick = now;

    if (g_buzzer_on != 0U)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        g_buzzer_on = 0U;
        g_buzzer_remaining--;
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_0);
        g_buzzer_on = 1U;
    }
}
```

该状态机每次Start从头发一组，最后一次响结束后剩余归0并关PB0。当前只由任务A调用Start/Update，不支持不同任务无锁并发；未来提示请求可用队列交给单一服务任务。Init在调度前，Start在调度后。任务死锁或暂停可能使输出停在高电平，软件诊断循环不能自动替代输出关断或硬件安全；不宣称已实现安全监督。

## 4 Keil配置

把Buzzer.c加入HARDWARE分组；Include Paths追加`.\HARDWARE\Buzzer`，保留LED子目录及原路径。头文件在USER已有搜索目录，不能把仅父目录HARDWARE视为自动递归搜索。

## 5 USER/main.c插入

头部加：

```c
#include "Buzzer.h"
#include "app_config.h"
```

main中的LED_Init后、创建任务前加入：

```c
/* 调度前配置PB0并保持关闭 */
Buzzer_Init();
```

App_TestTaskA中的(void)argument之后、while(1)之前加入：

```c
/* 本次只提示已验收的基础平台就绪，启动时请求一次 */
Buzzer_Start(APP_INIT_BEEP_COUNT);
```

同一任务while内、g_test_count_a++之后加入：

```c
/* 周期更新蜂鸣，任务正常计数及延时保持原样 */
Buzzer_Update();
```

Start不能放进while，否则每100ms重启提示组可能变成持续响。保留LED及RTOS诊断，不新建第三任务。

## 6 构建与实物验收

默认1次：保存、Rebuild、下载并复位启动/F5，运行时应只响一次约200ms，之后静音且LED继续闪。
改3：仅改APP_INIT_BEEP_COUNT为3U，再构建下载复位，响三次、间隔约200ms。
改0：同理不响。完成后用户选实际需要次数，默认建议恢复1U。

未发声或只咔一下不能直接确认为无源；先核对PB0约200ms高脉冲、供电、驱动与器件型号，停止增加DC通电时长或更改公共TIM3。当前有源只是用户指定候选。至少默认1次与LED同时正常的实物结果返回后再继续，0/3仍分别待验证。没有提供实际模块编译/下载/音响证据前，本项不判通过。
