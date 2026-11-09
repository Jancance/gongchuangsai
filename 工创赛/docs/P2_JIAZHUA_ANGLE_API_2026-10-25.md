# 夹爪角度接口：jiazhua(90);

按用户2026-10-25要求，用jiazhua(角度)调用夹爪。只读实际工程时，gripper.c/h仍为低电平初始化版本，尚未加入上一次试转任务，因此本版直接替换这两个文件即可，不必再加入固定往返试验任务。

## 用法

```c
jiazhua(90);       /* 请求目标90度，不是相对当前位置再转90度 */
jiazhua(120);      /* 改为请求目标120度 */
Gripper_Stop();    /* 停止发送PWM，不等于舵机已停止/失能 */
```

使用英文括号和英文分号。jiazhua返回1表示指令已写入PWM，0表示角度非法或初始化失败，不表示物理到位；不阻塞等待。270度/500～2500us为本次可调整的标称线性换算起点（E），未取得本台原厂对应规格/实物角度标定；90度是该映射下目标，尚不是经过尺量的实际机械角度。首次脱开夹爪机构空载试，不用0/270端点扫行程。

## 1. 替换HARDWARE/GRIPPER/gripper.h

```c
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdint.h>

void Gripper_Init(void);
uint8_t jiazhua(float angle_deg);
void Gripper_Stop(void);

#endif
```

## 2. 替换HARDWARE/GRIPPER/gripper.c

```c
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "gripper.h"

/* 集中参数：标称映射起点，后续按真实开合位置调整。 */
#define JIAZHUA_ANGLE_MAX_DEG  270.0f
#define JIAZHUA_PULSE_MIN_US   500U
#define JIAZHUA_PULSE_MAX_US   2500U
#define JIAZHUA_PERIOD_US      20000U

static uint8_t s_ready = 0U;
static uint8_t s_running = 0U;

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
    s_ready = 0U;
    s_running = 0U;
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_hz *= 2U;
    }
    if ((timer_hz < 1000000U) || ((timer_hz % 1000000U) != 0U) ||
        ((timer_hz / 1000000U) > 65536U) ||
        (JIAZHUA_PULSE_MIN_US >= JIAZHUA_PULSE_MAX_US) ||
        (JIAZHUA_PULSE_MAX_US >= JIAZHUA_PERIOD_US) ||
        !(JIAZHUA_ANGLE_MAX_DEG > 0.0f))
    {
        return;
    }
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000U - 1U);
    timer.TIM_Period = JIAZHUA_PERIOD_US - 1U;
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
    /* 默认不输出位置脉冲，显式调用jiazhua后才开始。 */
    s_ready = 1U;
}

uint8_t jiazhua(float angle_deg)
{
    uint32_t pulse_us;

    /* 此写法同时拒绝越界、NaN和无穷值，不把错误角度夹到端点。 */
    if ((s_ready == 0U) ||
        !(angle_deg >= 0.0f && angle_deg <= JIAZHUA_ANGLE_MAX_DEG))
    {
        return 0U;
    }
    pulse_us = JIAZHUA_PULSE_MIN_US + (uint32_t)(
        angle_deg * (float)(JIAZHUA_PULSE_MAX_US - JIAZHUA_PULSE_MIN_US)
        / JIAZHUA_ANGLE_MAX_DEG + 0.5f);
    TIM_SetCompare1(TIM2, pulse_us);
    if (s_running == 0U)
    {
        TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
        TIM_Cmd(TIM2, ENABLE);
        Gripper_PinMode(GPIO_Mode_AF);
        s_running = 1U;
    }
    /* 已运行时在下一PWM周期更新，不反复清零计数器。 */
    return 1U;
}

void Gripper_Stop(void)
{
    if (s_ready != 0U)
    {
        Gripper_PinMode(GPIO_Mode_OUT);
        TIM_Cmd(TIM2, DISABLE);
        TIM_SetCompare1(TIM2, 0U);
        s_running = 0U;
    }
}
```

调用后PWM持续保持该目标，直到下一次jiazhua或Gripper_Stop，不采用上版固定往返/1.5秒撤销信号；这是位置舵机调试接口，动作组的等待到位/超时由后续任务负责。仅由一个任务负责调用此驱动，其他任务不同时改TIM2/PA0。

## 3. Keil接入及直接实操

- 保留main.c现有#include "gripper.h"和Power_Init后的Gripper_Init。
- 在标准库分组加入StdPeriph_Driver/src/stm32f4xx_tim.c；只加入一次。
- 不需要加入上一版Gripper_TestTask或g_gripper_test_request。若自己已经加入，移除旧试转任务创建，避免它与jiazhua同时控制TIM2。
- Rebuild无误后下载，在自己的调试任务需要夹爪动作的位置调用jiazhua(90);。不要把角度调用放进Gripper_Init，否则变成开机立即转动。

现在就想用Watch方便调角度，可在USER/main.c全局变量区加入：

```c
volatile float g_jiazhua_debug_angle = 90.0f;
volatile uint32_t g_jiazhua_debug_request = 0U;
volatile uint32_t g_jiazhua_debug_result = 0U;
```

在现有App_TestTaskA的while (1)循环内、g_test_count_a++;之前加入下面一小段，保留其余代码，不增加阻塞等待或新的任务：

```c
if (g_jiazhua_debug_request == 2U)
{
    g_jiazhua_debug_request = 0U;
    Gripper_Stop();
    g_jiazhua_debug_result = 2U;
}
else if (g_jiazhua_debug_request == 1U)
{
    g_jiazhua_debug_request = 0U;
    g_jiazhua_debug_result = jiazhua(g_jiazhua_debug_angle);
}
```

Rebuild无误后下载、F5运行；先在Watch填angle=90，再写request=1一次，之后改angle=100再写request=1。需要撤销信号写request=2；不要按旧PC13或写底盘运动请求。result=1只说明PWM命令已发出，result=2只说明已撤销信号。未触发请求时默认不动作；调试变量与接口不作为正式自主动作组入口。

需要直接体验一轮时，可在一个用户调试任务里执行下面的有限片段一次；这是动作片段，不是完整任务函数，不要放进无限循环中无休止重复：

```c
jiazhua(90);
vTaskDelay(pdMS_TO_TICKS(800U));
jiazhua(100);
vTaskDelay(pdMS_TO_TICKS(800U));
jiazhua(90);
vTaskDelay(pdMS_TO_TICKS(800U));
Gripper_Stop();
```

首次仍脱开机构空载，观察是否转到两处不同位置；不增加电平/波形检查前置，有异常再回传编译错误或实际现象。上面800ms仅调试观察间隔，既不是已验证到位时间，也不是限力保护。正式固定动作组后续再用实物反馈确定位置、速度及超时，不把指令返回值当成功抓放。

## 需求与证据

已核R PWR-01/RUN-01/FIELD-07、E RUN-06/LOC-03；角度接口及标称换算为E类用户工程决定，Q官方未决项不变。实际PA0/5V由用户确认，静态0.01V及此前编译通过不证明新PWM接口已运行。本版未修改外部真实工程；新代码/实物角度/机械范围/到位/供电带载尚待用户实操结果。
