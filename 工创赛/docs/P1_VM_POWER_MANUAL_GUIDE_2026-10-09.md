# PE2与VM静态供电手动接入（2026-10-09）

> 最新现场接续：用户已补main154行Power_Init，确认“VM有电压了”；开启环节依据用户口述接受，具体电压尚未报告。现在只验证关闭：电机VM负载继续断开，g_vm_test_request=0并F5运行约1秒，测VM对GND。不重复已修Init或重建模块，硬件开关/复位默认禁止/带载和CAN仍另验。证据vm_enable_after_init_fix.json；此前漏调及request1低电压见vm_request1_missing_power_init.json/png，保留历史。

用户原工程：D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/gongxun-f407.uvprojx。用户自行修改，助手不改原C/H或Keil工程。标准外设库、原生FreeRTOS和中文注释保持；保留LED/Buzzer/UART/Key及现有任务。

图纸重读见P1_CAN_VM_SCHEMATIC_RECHECK_2026-10-09.md。用户已测Q2输入侧22.34V、VM_EN=0.016V：上游电压到达，VM_EN是低电平。当前程序尚无PE2输出初始化/允许请求，VM关闭符合该控制条件；尚未证明开关档位或Q2/Q6故障。本轮先实现默认关闭的PE2请求，电机VM供电断开的情况下手动测试。

## 1. 测试准备

先全断电，断开电机VM供电负载；若35/28/云台也接这条VM，保持其VM负载断开。本轮只给主控/电源板正常供电并测输出，不让电机上电运行。开关公共2接1（VM_IO）才由PE2控制；2接3（GND）强制关闭。机械按下/弹起对应待实际测量，不能因为标“急停”就认为按下允许供电。

## 2. 新建模块

在工程根目录手动建立HARDWARE/POWER，新建power.h：

```c
#ifndef POWER_H
#define POWER_H

void Power_Init(void);
void Power_Enable(void);
void Power_Disable(void);

#endif
```

新建power.c：

```c
#include "stm32f4xx.h"
#include "power.h"

/* PE2只输出VM允许请求，实际供电还受外部开关控制 */
void Power_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* 先将输出锁存为低，再切换输出模式，默认关闭VM */
    GPIO_ResetBits(GPIOE, GPIO_Pin_2);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* 请求开启VM，外部开关须将VM_EN接到VM_IO */
void Power_Enable(void)
{
    GPIO_SetBits(GPIOE, GPIO_Pin_2);
}

/* 撤销VM允许请求 */
void Power_Disable(void)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_2);
}
```

Keil将power.c加入现有工程分组；Options for Target→C/C++→Include Paths追加`.\HARDWARE\POWER`，保留原搜索路径。不需要新增标准库源，因为现有GPIO/RCC已接入。

## 3. main.c只改三处

第一处，现有#include之后加入头文件与临时请求：

```c
#include "power.h"

/* 临时静态电源测试：0关闭，1请求开启，电机供电须断开 */
volatile uint32_t g_vm_test_request = 0U;
```

第二处，main()的Key_Init()之后、创建任务之前加入（必须实际调用，只有power.c里的函数定义不会执行初始化）：

```c
/* 调度前初始化VM允许请求，默认保持关闭 */
Power_Init();
```

第三处，App_TestTaskA()的while循环内，Buzzer_Update()之后加入：

```c
/* 临时测试由任务A处理，不改变其他任务 */
if (g_vm_test_request == 1U)
{
    Power_Enable();
}
else
{
    Power_Disable();
}
```

请求由原TaskA每100ms处理，不增加任务、不更改SysTick、优先级或已验串口接收。只有值1请求高；其他值关闭。这是电机VM负载断开的临时测量入口，不是比赛自动启动/完整安全状态机，PC13仍只计数。Power_Init不在上电时自动启用VM。

## 4. 用户构建、下载与测量

1. 保存、Rebuild，检查0错误0警告，再下载、进入调试并F5运行。默认g_vm_test_request=0。
2. Watch加入g_vm_test_request；Stop后将值改为1，必须F5恢复，让TaskA真正输出PE2。
3. 电机VM负载仍断开，万用表直流电压档，黑表笔电源板GND，记录：
   - PE2/VM_IO：可测主控开关第1脚，允许请求时应接近主控逻辑供电（约3.2～3.3V）。
   - VM_EN：电源板R13左端或H2第6脚，软件控制档应接近PE2。
   - VM输出：软件允许且Q6/Q2正常时，应接近本次Q2输入22.34V。
4. 如PE2高、VM_EN仍低，先查开关是否2接3的关闭档或控制排线；无需重复测CAN电阻。负载已断开的前提下切换开关一次，再记录VM_EN；不从按钮外观定义允许/急停机械状态。若PE2和VM_EN都高而VM仍无电压，再进入Q6/Q2及门极实测。
5. 测完Stop将请求改0，再F5运行使Power_Disable执行。Stop只暂停CPU，不会自动撤销已输出的高电平，不能把Stop当关闭VM。

回报格式：

```text
g_vm_test_request=1并F5后：
PE2/VM_IO：___ V
VM_EN：___ V
VM：___ V
开关机械状态：按下 / 弹起
```

确认静态供电与关闭行为后再清理临时请求和TaskA测试分支，保留默认关闭的Power模块，后续由整车状态/安全流程接管。没有验证前不直接将PC13启动事件接Power_Enable，也不恢复电机带载动作。

## 5. 验证范围与需求

私下临时副本分别编译power.c及插入候选的main.c，ARMCC5.06 update5 build528、标准库/现有FreeRTOS头文件下均0错误0警告，用户原main/conf/uvprojx字节保持不变。证据evidence/p1-foundation/2026-10-09/vm_power_private_compile_probe.json。这只证明两编译单元兼容，不是用户完整链接、下载或实际PE2/VM电压通过；临时源码/对象清理，不交付完整固件。

R检查RUN-01/PWR-01，E检查LOC-03/RUN-06/MOT-04/PWR-02；官方Q不变。原理图/电路逻辑、静态开启关闭、急停/默认禁止、所有负载和实际通信分别验收。硬件安全链与故障停电尚未通过，本模块不能替代硬件急停。
