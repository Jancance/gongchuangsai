# CAN1静默回环手动验收（2026-10-10）

> 实机结果更新：用户截图result1/fault0，任务A最小剩余栈212字=848字节、heap当前/最小15200字节，实际源模式/完整帧比较一致，CAN头已补。控制器内部验收通过，不重做本页步骤、不缩任务栈。下一项实际电机侧第二终端辨识及正常模式只读版本，不能把本结果当线束/TJA/终端/电机通信通过。证据can1_silent_loopback_pass.json/png。

基线：competition_requirements_2027.md。用户已经提供CAN标准库参与Rebuild且完整链接0错误0警告的日志，编译器ARMCC5.06 update5 build528，Code10876/RW256/ZI22360。实际源检查：CAN源已入工程，USER/stm32f4xx_conf.h仍缺CAN头引用；先补后按本指南手动加入模块。助手不改原C/H或Keil工程，不交付完整固件。

本步骤只验F407的bxCAN控制器，使用静默＋回环模式，内部自发自收；电机VM保持关闭，不发实际电机命令。ST官方bxCAN资料说明组合静默/回环不会影响外部总线：https://www.st.com/resource/en/product_training/STM32F7_Peripheral_bxCAN.pdf；F4同为bxCAN见ST技术人员说明：https://community.st.com/stm32-mcus-60/using-can-bxcan-in-normal-mode-with-stm32-microcontrollers-part-1-151183。资料用于bxCAN概念，不复制HAL或F7引脚。

## 1. 补CAN头

USER/stm32f4xx_conf.h现有stm32f4xx_usart.h后、assert_param前加入：

```c
#include "stm32f4xx_can.h"
```

## 2. 新建SYSTEM/CAN/can.h

工程根目录D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程，手动新建SYSTEM/CAN目录与can.h：

```c
#ifndef CAN_H
#define CAN_H

#include "stm32f4xx.h"

/* 在FreeRTOS任务中调用一次，返回测试结果 */
uint32_t CAN1_LoopbackTest(void);

#endif
```

## 3. 新建SYSTEM/CAN/can.c

```c
#include "can.h"
#include "stm32f4xx_can.h"
#include "FreeRTOS.h"
#include "task.h"

/* 初始化CAN1，成功返回1，时钟不匹配返回2，初始化失败返回3 */
static uint32_t CAN1_InitLoopback(void)
{
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_GetClocksFreq(&clocks);
    if (clocks.PCLK1_Frequency != 42000000U)
    {
        return 2U;
    }

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);

    /* 按成品PCB使用PD0接收、PD1发送，不启用旧STB候选引脚 */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &gpio);

    CAN_StructInit(&can);
    /* 静默加回环：测试帧只在控制器内部流转 */
    can.CAN_Mode = CAN_Mode_Silent_LoopBack;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;
    /* 42MHz / 6 / (1 + 11 + 2) = 500kbit/s */
    can.CAN_Prescaler = 6U;
    can.CAN_NART = ENABLE;
    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success)
    {
        return 3U;
    }

    /* 测试阶段用过滤器0接收全部帧，送入FIFO0 */
    filter.CAN_FilterNumber = 0U;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0U;
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0U;
    filter.CAN_FilterMaskIdLow = 0U;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    return 1U;
}

/* 一次完整回环测试，不开启VM，不使用电机协议 */
uint32_t CAN1_LoopbackTest(void)
{
    CanTxMsg tx;
    CanRxMsg rx;
    TickType_t start_tick;
    uint32_t result;
    uint8_t mailbox;
    uint8_t status;
    uint8_t i;

    result = CAN1_InitLoopback();
    if (result != 1U)
    {
        return result;
    }

    tx.StdId = 0U;
    tx.ExtId = 0x12345U;
    tx.IDE = CAN_Id_Extended;
    tx.RTR = CAN_RTR_Data;
    tx.DLC = 8U;
    for (i = 0U; i < 8U; i++)
    {
        tx.Data[i] = (uint8_t)(0x10U + i);
    }

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 4U;
    }

    start_tick = xTaskGetTickCount();
    while (1)
    {
        status = CAN_TransmitStatus(CAN1, mailbox);
        if (status == CAN_TxStatus_Ok)
        {
            break;
        }
        if ((status == CAN_TxStatus_Failed) ||
            ((TickType_t)(xTaskGetTickCount() - start_tick) >=
             pdMS_TO_TICKS(100U)))
        {
            CAN_CancelTransmit(CAN1, mailbox);
            return 4U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    start_tick = xTaskGetTickCount();
    while (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
    {
        if ((TickType_t)(xTaskGetTickCount() - start_tick) >=
            pdMS_TO_TICKS(100U))
        {
            return 5U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    CAN_Receive(CAN1, CAN_FIFO0, &rx);
    if ((rx.IDE != CAN_Id_Extended) ||
        (rx.RTR != CAN_RTR_Data) ||
        (rx.ExtId != tx.ExtId) || (rx.DLC != tx.DLC))
    {
        return 6U;
    }
    for (i = 0U; i < 8U; i++)
    {
        if (rx.Data[i] != tx.Data[i])
        {
            return 6U;
        }
    }

    return 1U;
}
```

返回码：0为全局初值/尚未完成，1通过，2 PCLK1不是42MHz，3初始化失败，4无邮箱/发送失败或超时，5接收超时，6帧属性或数据不一致。FIFO中比较扩展帧属性、ID、长度、8字节内容，不仅凭发送成功。收发等待分别100ms且使用原生vTaskDelay，让其他任务运行；不新增ISR/队列/任务。模式固定为CAN_Mode_Silent_LoopBack，本轮不改正常模式。

## 4. 加入Keil并修改main

Keil添加新can.c到分组，Include Paths追加`.\SYSTEM\CAN`，保留现有路径。

main.c头文件区加入：

```c
#include "can.h"
```

main.c全局区加入：

```c
/* CAN1内部回环结果：0未完成，1通过，其他值见返回码 */
volatile uint32_t g_can_test_result = 0U;
```

在App_TestTaskA()里，Buzzer_Start(APP_INIT_BEEP_COUNT)后、while循环前加入：

```c
/* 调度启动后只执行一次CAN内部回环，函数内部包含初始化 */
g_can_test_result = CAN1_LoopbackTest();
```

保留原Power_Init、临时g_vm_test_request、LED/蜂鸣/串口/按键任务。VM请求保持默认0，电机供电断开。把测试放在任务启动处而非每100ms重复执行；函数需要调度/节拍，不放调度器启动前调用。CAN源现阶段只有内部自检，将来再逐步加入正常模式和电机协议。

## 5. 用户构建下载与验收

保存、Rebuild，0错误0警告后下载，F5运行约1秒，再Stop看Watch：

```text
g_can_test_result
g_rtos_fault
g_test_count_a
g_stack_a_words
```

期待result1、fault0、原状态灯继续闪烁/其他任务继续运行；回报实际截图或数值。栈余量观察新增嵌套调用，保留任务A256字；没有证据前不缩栈。不要用本结果宣称TJA1050、线束、500k物理波形、电机地址/协议、终端或CAN真实收发已通过，实际版本读取另验。

## 6. 需求与未闭环

R：PWR-01；E：LOC-03、RUN-06、MOT-04。官方Q不变。工程未闭环：静默回环运行、电机端第二终端与分支时序、正常模式只读版本及X/Emm实际类型；VM独立软件关闭/按键强制关/复位安全和带载各自验证。用户手动代码风格保持，不改成品PCB或源工程。代码私下编译/原源哈希证据与用户真实构建/运行分开记录。
