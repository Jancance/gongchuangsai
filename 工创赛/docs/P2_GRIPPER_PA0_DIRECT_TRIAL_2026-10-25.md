# PA0夹爪：直接进行一次空载试转

2026-10-25按用户要求，跳过额外电平/波形检查，先做一次有界实操，异常再排查。保留编译、默认不动作、有限时长这几项实现约束。代码由用户手动录入实际工程，助手不修改外部源、不烧录。

## 本次实际要做什么

断电后，将舵机输出轴上的舵盘/连杆与夹爪机构脱开，使舵机能空载转动；不强扭输出轴。首次位置未知，不能直接带夹爪夹到端点。本次使用PA0信号、现有5V及共地，其他轴/底盘动力保持关闭，不按PC13启动旧底盘测试。

这是20ms周期、1500→1550→1500us各约0.5秒的信号试验，最后撤销PWM；1500us不是已经标定的机械零点，结束不代表物理到位或必然失能。两种脉宽只是本次E类试验参数，不是已经取得该舵机原厂规格或夹爪开合参数。用纸标记/观察输出轴相邻位置变化即可，不扫500～2500us全行程。若堵转、持续响或异常，切断舵机5V，不反复触发。

## 1. 替换两个已有文件

工程：D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。

HARDWARE/GRIPPER/gripper.h：

```c
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdint.h>

/* Watch：1触发一次试转，2中止；任务接收后自动清零。 */
extern volatile uint32_t g_gripper_test_request;
/* 0等待，1试转中，2信号序列结束，3时钟配置不满足条件。 */
extern volatile uint32_t g_gripper_test_state;

void Gripper_Init(void);
void Gripper_TestTask(void *argument);

#endif
```

HARDWARE/GRIPPER/gripper.c：

```c
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gripper.h"

volatile uint32_t g_gripper_test_request = 0U;
volatile uint32_t g_gripper_test_state = 0U;
static uint32_t s_clock_ready = 0U;

/* 改信号脚模式；GPIO输出模式始终预置低电平。 */
static void Gripper_PinMode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);
}

static void Gripper_StopSignal(void)
{
    Gripper_PinMode(GPIO_Mode_OUT);
    TIM_Cmd(TIM2, DISABLE);
    TIM_SetCompare1(TIM2, 0U);
}

void Gripper_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef channel;
    uint32_t timer_hz;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    Gripper_PinMode(GPIO_Mode_OUT);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    /* F407：APB1非1分频时，定时器时钟是PCLK1的2倍。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_hz *= 2U;
    }
    s_clock_ready = 0U;
    g_gripper_test_request = 0U;
    g_gripper_test_state = 0U;
    if ((timer_hz < 1000000U) ||
        ((timer_hz % 1000000U) != 0U) ||
        ((timer_hz / 1000000U) > 65536U))
    {
        g_gripper_test_state = 3U;
        return;
    }
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000U - 1U);
    timer.TIM_Period = 19999U;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_OCStructInit(&channel);
    channel.TIM_OCMode = TIM_OCMode_PWM1;
    channel.TIM_OutputState = TIM_OutputState_Enable;
    channel.TIM_OCPolarity = TIM_OCPolarity_High;
    channel.TIM_Pulse = 0U;
    TIM_OC1Init(TIM2, &channel);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    /* 默认计数器关闭、PA0仍为GPIO低电平，不上电自动试转。 */
    s_clock_ready = 1U;
}

void Gripper_TestTask(void *argument)
{
    TickType_t started = 0U;
    TickType_t elapsed;
    uint32_t active = 0U;
    uint32_t command;
    (void)argument;

    for (;;)
    {
        command = g_gripper_test_request;
        if (command != 0U)
        {
            g_gripper_test_request = 0U;
        }
        if (command == 2U)
        {
            Gripper_StopSignal();
            active = 0U;
            g_gripper_test_state = s_clock_ready ? 0U : 3U;
        }
        else if ((command == 1U) && (active == 0U) && s_clock_ready)
        {
            TIM_SetCompare1(TIM2, 1500U);
            TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
            TIM_Cmd(TIM2, ENABLE);
            Gripper_PinMode(GPIO_Mode_AF);
            started = xTaskGetTickCount();
            active = 1U;
            g_gripper_test_state = 1U;
        }
        /* 运行中再次写1不延长试验。 */
        if (active != 0U)
        {
            elapsed = (TickType_t)(xTaskGetTickCount() - started);
            if (elapsed >= pdMS_TO_TICKS(1500U))
            {
                Gripper_StopSignal();
                active = 0U;
                g_gripper_test_state = 2U;
            }
            else if (elapsed >= pdMS_TO_TICKS(1000U))
            {
                TIM_SetCompare1(TIM2, 1500U);
            }
            else if (elapsed >= pdMS_TO_TICKS(500U))
            {
                TIM_SetCompare1(TIM2, 1550U);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
```

说明：1MHz计数时1个计数为1us，20000个计数为20ms。程序按现有F407时钟树计算分频，不固定套用83。约1.5秒的软件任务计时依赖RTOS正常调度，不是独立硬件切电保护；结束仅撤销信号。已有共享电源/安全模块不被此代码重配。

## 2. 接入工程

- Keil的标准库分组加入现成的StdPeriph_Driver/src/stm32f4xx_tim.c；当前工程此前未加入此文件，否则会出现TIM_*链接未定义。
- 保留main.c已有#include "gripper.h"及Power_Init后的Gripper_Init调用。
- 在main.c全局变量区添加：

```c
volatile BaseType_t g_gripper_task_create = pdFAIL;
```

- 在main()的vTaskStartScheduler();前添加：

```c
g_gripper_task_create = xTaskCreate(Gripper_TestTask, "Gripper",
                                  256U, NULL, 2U, NULL);
configASSERT(g_gripper_task_create == pdPASS);
```

只新增夹爪任务，不修改原底盘任务/宏，也不用PC13作为试转触发。保留PA0专用夹爪、PB9未来补光、PB1三槽盘的分配。

## 3. 编译后直接试一次

1. 确保舵机与夹爪机构已脱开、空载。先断电接回PA0信号、5V和GND，主控与舵机共地；底盘和其他机构动力保持关闭。
2. Rebuild无错误后下载，F5连续运行，不按PC13、不单步、不更改旧底盘请求。
3. Watch加入g_gripper_test_request和g_gripper_test_state；只把request改为1一次，不反复写。
4. 观察输出轴：可能先转到1500us对应位置，再小幅偏转、回到第一位置，约1.5秒后state为2。实际轴不动/持续响/state为3或其他异常则停止试验，回传现象再针对性排查。state2仅表示信号序列发送结束，不是舵机到位证明。

回传“是否转动、是否小幅往返、是否异常响、最终state”即可，不额外测波形。下一项根据实际效果确定夹爪装配基准和有限开合范围，不能直接把本次空载序列当装机夹爪动作组。

## 记录与规则

本次为用户授权的调试直接实操路线（E），替代前序先测PWM波形的安排。已核R PWR-01/RUN-01/FIELD-07及MECH-01/02；E RUN-06/LOC-03。正式自主入口和本调试入口分开；既有官方Q保持待技术答疑确认。

编译/实物结果以用户后续日志/现象为证据；该文本不表示已下载、转动、带载或动作组通过。周期/脉宽是本次试验设定，准确控制范围、供电带载能力、机械开合端点、到位与停止能力仍待实际动作确认。没有修改外部真实固件或自动启动硬件。
