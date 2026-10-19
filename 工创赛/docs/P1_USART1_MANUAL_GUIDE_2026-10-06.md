# USART1调试口：标准库TXE中断发送手动教学

日期2026-10-06。当前LED与蜂鸣器0/1/3次均经用户实物反馈通过。此步只做USART1发送验收，不接收外设命令、不接收雷达或控制电机。检查LOC-03、RUN-06（E）、PWR-01（R），官方Q不变；代码由用户手动加入实际F407工程，助手不写固件文件。

最新实物结果：XCOM COM4/115200/8N1已收到F407 FreeRTOS READY，R=21，USART1首次单向发送门禁通过。用户报告按钮复位需再烧录，正按P1_RESET_STANDALONE_GUIDE_2026-10-06.md独立排查；用户已澄清声音是正常配置次数后停止。复位/冷启动稳定性未通过，不重复首次发送验证、不归因为Flash丢失。下面空文件准备条目为历史进度，现实际代码/工程已接入。

当前手动准备情况：用户已建SYSTEM/USART/usart.c和usart.h（文件夹实际大写USART），读取时两个文件均0字节；工程源/搜索路径、conf和main调用尚未接入。按下面内容逐文件填写并Ctrl+S，再添加两个.c与.\SYSTEM\USART路径。用户请求详细操作后，聊天也直接列出内容与点击顺序；不重复创建文件、不用助手自动写固件。

## 引脚来源与历史冲突

旧P1_STM32F407VET6_PIN_PLAN的2026-08-28候选把PA9～PA12保留USB、标注不占USART1；用户提供成品主控原理图（更新2026-09-19，codex-clipboard-84c20a92-c899-47f2-8423-9d33f0459965.png、codex-clipboard-cfa6784c-8caa-4915-8d42-cfc8652fcbb8.png）显示USART1 TX=PA9、RX=PA10，H14为调试串口。依据用户明确成品PCB不改及已有图纸，本轮工程调试采用USART1 PA9/PA10，旧USB保留条目保留历史，不能同时将同一引脚分给USB用途。实际接线/输出波形与串口收到文本仍待验收；这不是官方规则变更。

当前只接主控USART1 TX/PA9到USB-TTL RXD及GND共地，适配器VCC、TXD暂不接主控；主控继续原供电。串口助手115200、8数据位、无校验、1停止位、无流控，文本显示，先打开端口再复位/F5，以免错过一次启动消息。

## 文件与库配置

实际根目录D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。用户新建SYSTEM/usart/usart.h与usart.c。Keil把StdPeriph_Driver/src/stm32f4xx_usart.c加入LIB分组，SYSTEM/usart/usart.c加入SYSTEM分组；C Include Paths追加`.\SYSTEM\usart`，保留其他路径。

USER/stm32f4xx_conf.h在现有标准库头文件旁加：

```c
#include "stm32f4xx_usart.h"
```

沿用F103模块名usart.c/h及USART1_Config风格，F407 GPIO时钟/AF重新配置，不照搬F103时钟或printf忙等。

## usart.h

```c
#ifndef USART_MODULE_H
#define USART_MODULE_H

#include <stdint.h>

void USART1_Config(void);
uint8_t USART1_SendString(const char *text);

#endif
```

## usart.c

```c
#include "stm32f4xx.h"
#include "usart.h"

/* 当前只允许任务A提交发送请求，中断负责逐字节发送 */
static uint8_t g_tx_buffer[64];
static volatile uint16_t g_tx_length = 0U;
static volatile uint16_t g_tx_position = 0U;

/* 初始化PA9发送，115200波特率，8位数据、无校验、1停止位 */
void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 115200U;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0U;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/* 返回1表示已接受，0表示忙或文本过长；不等待发送完成 */
uint8_t USART1_SendString(const char *text)
{
    uint16_t length = 0U;

    if ((text == 0) || (g_tx_length != 0U))
    {
        return 0U;
    }

    while (text[length] != '\0')
    {
        if (length >= sizeof(g_tx_buffer))
        {
            return 0U;
        }
        g_tx_buffer[length] = (uint8_t)text[length];
        length++;
    }

    if (length == 0U)
    {
        return 1U;
    }

    g_tx_position = 0U;
    g_tx_length = length;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    return 1U;
}

/* TXE由写入数据寄存器自动处理，此处不调用RTOS接口 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (g_tx_position < g_tx_length)
        {
            USART_SendData(USART1, g_tx_buffer[g_tx_position]);
            g_tx_position++;
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            g_tx_length = 0U;
        }
    }
}
```

最大一次64字节，只允许单一任务生产者；无队列、无动态内存、无RTOS ISR调用。忙/过长立即失败，不阻塞重试，不把“接受”当对方接收或物理发送完成。后续多任务日志另接统一队列/发送任务及健康检查。

## USER/main.c位置

头部加：

```c
#include "usart.h"
```

测试全局变量后加：

```c
/* 1表示调试文本请求被接受，不代表电脑已经收到 */
volatile uint32_t g_serial_send_ok = 0U;
```

main在Buzzer_Init后、创建任务前加：

```c
/* 初始化调试串口，本轮仅发送 */
USART1_Config();
```

任务A在(void)argument后、Buzzer_Start之前加：

```c
/* 只请求一次基础平台就绪文本 */
g_serial_send_ok = USART1_SendString("F407 FreeRTOS READY\r\n");
```

不把发送放进while，不改既有任务周期、LED、蜂鸣、RTOS内核。文本ASCII免除显示编码歧义；中文代码注释保留。

## 验收与限制

用户保存所有文件Rebuild下载，先打开正确COM及上述参数，再复位并让程序连续运行（F5）。期望收到一次F407 FreeRTOS READY，LED继续闪，蜂鸣按当前用户选择次数执行。app_config.h现读数已为2U，用户可按喜好保留2或恢复1，不自动替换；零次通过是用户之前的实物反馈，不把现在2U解释为那次零测量失败。

无消息时先看g_serial_send_ok、真实运行位置、COM/接收模式/线序及PA9波形；不先重做已通过的RTOS测试。Rx、队列日志、双向协议、硬件急停/输出默认禁止及业务最坏负载仍待后续检查。收到文本才能通过物理串口门禁；配置/API静态核对不是实际通信证据。
