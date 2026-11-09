# 夹爪PA0第1步：建模块并编译，初始化保持低电平

用户确认当前夹爪众灵20KG/270°/单轴PWM，供电5V，信号PA0，PA0不接补光；以后需要补光用PB9。该接线工程决定取代旧PB9夹爪/PA0补光规划，不自动修改PCB或供电，不把5V用户反馈当动态带载测量。

实际工程D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程，gongxun-f407.uvprojx。本步由用户手动修改，助手不改真实工程，不生成运动命令。先只建初始化模块，PA0保持低电平，不生成PWM、不设置中点/开合角度；供电/复位时舵机自身行为尚未验收，不能承诺无信号时机械必然锁定或完全不动。

## 文件→位置→代码

1. 在HARDWARE下新建GRIPPER目录，添加gripper.h及gripper.c；Keil工程新增GRIPPER组，加入gripper.c，Include Paths追加.\HARDWARE\GRIPPER。

gripper.h：

```c
#ifndef GRIPPER_H
#define GRIPPER_H

void Gripper_Init(void);

#endif
```

gripper.c：

```c
#include "stm32f4xx.h"
#include "gripper.h"

/* 本步只初始化信号脚，保持低电平，不发送舵机位置脉冲。 */
void Gripper_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* 先把输出锁存置低，再切换为输出模式。 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);
}
```

2. USER/main.c顶部已有头文件后添加#include "gripper.h"；在main()原有Power_Init();后添加Gripper_Init();，不变动其余任务/测试宏/看门狗代码。
3. 只执行Rebuild，回传完整编译结果。预期无新增错误/警告，尤其不能缺gripper.h或Gripper_Init定义。本步不下载、不接通新供电、不写Watch运动请求。

## 解释与后续门禁

GPIO预置低后启用输出，避免初始化时先出现高电平；此时还没有TIM2/PWM。PA0后续可按TIM2_CH1准备硬件PWM，实际周期/脉宽需取得对应版本资料或明确试验边界，不假定通用270°等于可安全整程扫描。当前实际main已存在底盘/航向测试任务，不能为测舵机而未经核对直接启动整机旧运动入口。

后续应先检查PWM波形再安排与机械端点分离的首次舵机试验，首次位置指令可能引起大幅移动，不把1500us当夹爪当前角度。加入占空比/角度接口及RTOS请求状态机属于后续步骤，本页编译通过不等于其已实现。

已核R PWR-01/RUN-01/FIELD-07，E RUN-06/LOC-03；Q官方未决项不变。证据为用户PA0/5V/无补光反馈及实际工程源码只读核对；本页代码尚待用户手动录入、编译，无下载或实物验收。
