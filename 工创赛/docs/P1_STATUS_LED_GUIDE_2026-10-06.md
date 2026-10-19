# PE7状态灯：标准库与FreeRTOS手动接入

2026-10-06。两测试任务基础检查与临时断言清理恢复已通过，本次只验证状态灯GPIO。依据用户已给原理图及P1_STM32F407VET6_PIN_PLAN，状态灯为PE7、高电平亮；实际焊接/亮灭待验收。检查LOC-03、RUN-06（E）、PWR-01（R），官方Q不变。助手只维护本教程，不在实际工程写代码。

最新验收：用户确认状态灯亮0.5秒、灭0.5秒，PE7实物亮灭本项通过；实际main及工程路径亦已只读核对。证据为用户口述，未独立计量频率。下一项PB0可调初始化蜂鸣服务，等待用户确认有源/无源或型号，不再重复LED测试。

实际工程目录：D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。

1. 用户在HARDWARE下新建LED文件夹、LED.h及LED.c。头文件内容：

```c
#ifndef LED_H
#define LED_H

void LED_Init(void);
void LED_Turn(void);

#endif
```

2. LED.c手动填入：

```c
#include "stm32f4xx.h"
#include "LED.h"

/* 初始化PE7状态灯，高电平亮，默认熄灭 */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_ResetBits(GPIOE, GPIO_Pin_7);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* 翻转状态灯，由一个任务调用 */
void LED_Turn(void)
{
    GPIO_ToggleBits(GPIOE, GPIO_Pin_7);
}
```

仅配置PE7，不配置整个GPIOE端口、不接使能引脚。已有conf包含GPIO/RCC头，真实SPL声明以上API。接口采用F103式Init/Turn及独立.c/.h，代码注释中文；无HAL/LL包装。

3. Keil新建HARDWARE分组，Add Existing Files加入HARDWARE/LED/LED.c；C/C++的Include Paths追加`.\HARDWARE\LED`，保留已有路径。必须指向含led.h的LED子目录；仅填写`.\HARDWARE`不会递归查找子目录。2026-10-06首次LED构建正因实际搜索路径只到HARDWARE而失败，文件本身已建立、led.c也已加入；用户只需修正这一项再Rebuild，不需搬文件或改main。

4. USER/main.c头部加`#include "LED.h"`；main在NVIC_PriorityGroupConfig后、创建任务前加：

```c
/* 调度启动前初始化状态灯 */
LED_Init();
```

App_TestTaskB的while内、g_test_count_b++后加：

```c
/* 每500毫秒翻转一次，完整亮灭周期约1秒 */
LED_Turn();
```

保留原计数、栈查询、vTaskDelay(pdMS_TO_TICKS(500U))及故障钩子，不新建第三任务、不用F103 delay.c或重配SysTick。

5. 保存、Rebuild、下载后F5运行或退出调试后板上复位。观察状态灯亮约0.5秒/灭约0.5秒。运行时观察，不将Stop期间停止闪烁认定故障。不亮时检查PE7引脚电平、LED焊接/极性/限流电阻，先保留实际结果再定位，不换引脚。若示波器验证：PE7约0～3.2/3.3V，周期约1秒、占空比约50%；该实测结果尚未提供，不提前宣称准确。

完成条件：用户Rebuild零错误及实物可见亮灭或PE7有效波形。只证明该状态灯输出路径，不等同于业务健康监督、蜂鸣器、CAN/电机或硬件安全验收。
