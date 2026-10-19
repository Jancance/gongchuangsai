# UART4串口屏：保留原两行任务码的手动迁移

> 最新接续2026-10-13：用户实物报告UART4原屏第一行156+123+、第二行516+231，完整两行码显示通过；实际三文件/Keil接入已只读核对。原屏工程D:/新建文件夹4/工创赛/代码/串口屏/工创赛.HMI已找到，USART HMI正在打开并独占；未读取内部布局、不改原文件。唯一下一项用户在编辑器选page0/t0给画布和属性截图，再在工作副本增量加状态/故障，保留任务码；之后扫码/正式READY，八阶段不变。

## 1. 本步做什么

原F103工程不改，屏内原页面不覆盖。保留Screen_Init/SendByte/SendString/SendCommand/ShowTaskCode五个接口、9600/8N1，以及page0.t0/t1前8后7完整任务码；F407改UART4 PC10_TX/PC11_RX。发送函数新增结果返回，旧的忽略返回值调用方式仍可用，本步用结果诊断。启动满1秒后显示一次156+123+516+231；VM请求保持默认0，原CAN/按键/调试串口/监督/IWDG全部保留。

命令以三个FF结束，与原源及[厂家通信指南](https://www.tjc1688.com/contents/25/651.html)一致。本步帧前三个FF终止可能残留的半帧；结果1仅表示UART发完，不代表屏执行成功。每条整帧超时150ms、等待时让出CPU；只由任务A发送，不在监督/ISR/临界区使用。屏回包/触摸和任务故障显示后续接入，当前不把屏模块加入必需READY条件。

## 2. 新建HARDWARE/SCREEN/screen.h

在实际F407工程新建HARDWARE/SCREEN目录，新建下面头文件保存为screen.h，旧F103文件保持原样。

```c
#ifndef SCREEN_H
#define SCREEN_H

#include <stdint.h>

/* 结果1仅表示串口发完，屏幕实际显示仍需目视验收 */
#define SCREEN_OK        1U
#define SCREEN_BAD_PARAM 2U
#define SCREEN_NOT_READY 3U
#define SCREEN_TIMEOUT   4U

extern volatile uint32_t g_screen_init_done;
extern volatile uint32_t g_screen_command_count;

void Screen_Init(void);
uint32_t Screen_SendByte(uint8_t data);
uint32_t Screen_SendString(const char *text);
uint32_t Screen_SendCommand(const char *command);
uint32_t Screen_ShowTaskCode(const char *task_code);

#endif
```

## 3. 新建HARDWARE/SCREEN/screen.c

同目录新建screen.c并填入下面新模块。中文注释按当前工程编码保存，不替换整个main或其他模块。

```c
#include "screen.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define SCREEN_BAUD_RATE 9600U
#define SCREEN_TEXT_LIMIT 96U
#define SCREEN_TX_TIMEOUT_MS 150U

volatile uint32_t g_screen_init_done = 0U;
volatile uint32_t g_screen_command_count = 0U;

/* 调度前只配置外设，不发送文字、不等待屏幕 */
void Screen_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;

    g_screen_init_done = 0U;
    g_screen_command_count = 0U;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);

    USART_StructInit(&uart);
    uart.USART_BaudRate = SCREEN_BAUD_RATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART4, &uart);
    /* 本步仅发送；没有接收中断，不把发完当成屏幕应答 */
    USART_Cmd(UART4, ENABLE);
    g_screen_init_done = 1U;
}

/* 本模块只有任务A调用；不得在中断或关闭调度的临界区发送 */
static uint32_t Screen_Write(const uint8_t *data, uint32_t length)
{
    TickType_t start;
    TickType_t timeout = pdMS_TO_TICKS(SCREEN_TX_TIMEOUT_MS);
    uint32_t i;

    if ((g_screen_init_done != 1U) || (__get_IPSR() != 0U) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING))
    {
        return SCREEN_NOT_READY;
    }
    start = xTaskGetTickCount();
    for (i = 0U; i < length; i++)
    {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
        {
            if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
            {
                return SCREEN_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(1U));
        }
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
        {
            return SCREEN_TIMEOUT;
        }
        USART_SendData(UART4, data[i]);
    }
    /* 等待最后一字节发完，同样计入整帧150毫秒上限 */
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
    {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout)
        {
            return SCREEN_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
    return SCREEN_OK;
}

/* 最多95个字符；参数异常时不发送半条命令 */
static uint32_t Screen_TextLength(const char *text, uint32_t *length)
{
    uint32_t i;

    if (text == NULL)
    {
        return SCREEN_BAD_PARAM;
    }
    for (i = 0U; i < SCREEN_TEXT_LIMIT; i++)
    {
        if (text[i] == '\0')
        {
            *length = i;
            return SCREEN_OK;
        }
    }
    return SCREEN_BAD_PARAM;
}

uint32_t Screen_SendByte(uint8_t data)
{
    return Screen_Write(&data, 1U);
}

uint32_t Screen_SendString(const char *text)
{
    uint32_t length;

    if (Screen_TextLength(text, &length) != SCREEN_OK)
    {
        return SCREEN_BAD_PARAM;
    }
    return Screen_Write((const uint8_t *)text, length);
}

uint32_t Screen_SendCommand(const char *command)
{
    uint8_t frame[SCREEN_TEXT_LIMIT + 6U];
    uint32_t length;
    uint32_t i;
    uint32_t result;

    if ((Screen_TextLength(command, &length) != SCREEN_OK) ||
        (length == 0U))
    {
        return SCREEN_BAD_PARAM;
    }
    /* 先终止上次可能中断的残帧，再发送本条完整命令 */
    for (i = 0U; i < 3U; i++)
    {
        frame[i] = 0xFFU;
    }
    for (i = 0U; i < length; i++)
    {
        frame[i + 3U] = (uint8_t)command[i];
    }
    for (i = 0U; i < 3U; i++)
    {
        frame[length + 3U + i] = 0xFFU;
    }
    result = Screen_Write(frame, length + 6U);
    if (result == SCREEN_OK)
    {
        g_screen_command_count++;
    }
    return result;
}

/* 保留原两行完整任务码；这里只检查显示边界，不判搬运合法性 */
uint32_t Screen_ShowTaskCode(const char *task_code)
{
    char line1[9];
    char line2[8];
    char command[40];
    uint32_t length;
    uint32_t i;
    uint32_t result;

    if ((Screen_TextLength(task_code, &length) != SCREEN_OK) ||
        (length != 15U))
    {
        return SCREEN_BAD_PARAM;
    }
    for (i = 0U; i < 15U; i++)
    {
        if (((task_code[i] < '0') || (task_code[i] > '9')) &&
            (task_code[i] != '+'))
        {
            return SCREEN_BAD_PARAM;
        }
    }
    for (i = 0U; i < 8U; i++)
    {
        line1[i] = task_code[i];
    }
    line1[8] = '\0';
    for (i = 0U; i < 7U; i++)
    {
        line2[i] = task_code[i + 8U];
    }
    line2[7] = '\0';

    result = Screen_SendCommand("page 0");
    if (result != SCREEN_OK)
    {
        return result;
    }
    vTaskDelay(pdMS_TO_TICKS(20U));
    /* 两行长度已固定，40字节缓冲足以放下命令和结束零 */
    sprintf(command, "page0.t0.txt=\"%s\"", line1);
    result = Screen_SendCommand(command);
    if (result != SCREEN_OK)
    {
        return result;
    }
    sprintf(command, "page0.t1.txt=\"%s\"", line2);
    return Screen_SendCommand(command);
}
```

## 4. USER/main.c增量接入：五处

### 4.1 头部watchdog.h之后

```c
#include "screen.h"
```

### 4.2 g_watchdog_init_result定义之后

```c
/* 当前屏幕台架：上电请求显示一次，后续接扫码业务 */
volatile uint32_t g_screen_test_request = 1U;
volatile uint32_t g_screen_test_result = 0U;
```

### 4.3 App_TestTaskA开头

将原函数的开头改成下面几行，后面USART1接收开启、启动提示、蜂鸣、CAN回环和while循环都保留。

```c
static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();

    (void)argument;
```

### 4.4 循环内g_test_tick赋值之前

App_CanPollRefresh/Safety_CanReport仍保留。在`g_test_tick = xTaskGetTickCount();`这一行前插入：

```c
        /* 不阻塞启动等待；开机满1秒后，仅任务A发一次两行码 */
        if ((g_screen_test_request == 1U) &&
            (g_vm_test_request == 0U) &&
            ((TickType_t)(xTaskGetTickCount() - screen_start_tick) >=
             pdMS_TO_TICKS(1000U)))
        {
            g_screen_test_request = 0U;
            g_screen_test_result =
                Screen_ShowTaskCode("156+123+516+231");
        }
```

后面栈堆统计、Safety_Report与100ms延时全部保留。等待屏启动通过逐轮检查经过时间实现，不阻塞启动1秒。本步样例只显示、不解析、不触发搬运，后续扫码业务接入时撤掉自动样例并保留正式接口。不靠长时间Stop改变量启动显示，适应当前IWDG宏0。

### 4.5 main中USART1_Config之后

```c
    /* UART4仅初始化，实际发送放到任务A */
    Screen_Init();
```

后面的Power_Init/Safety_Init/IWDG初始化、五任务创建及断言、调度器保持原样；不新建额外任务，不由其他任务代报APP心跳。

## 5. 加入Keil并Rebuild

1. 在原HARDWARE分组或新建SCREEN分组，Add Existing Files加入新screen.c，不能选旧F103 screen.c。
2. Options for Target → C/C++ → Include Paths追加`.\HARDWARE\SCREEN`，保留所有现有路径和宏。stm32f4xx_usart.c已在工程内，不重复加同一文件。
3. 保存全部文件，Rebuild，发完整日志。本轮先做到正式构建，构建通过后再下载实屏验证。

## 6. 构建后的实屏门禁

用户正式构建通过后再下载。保持VM请求0/硬件关VM，F5正常运行约3秒。预期原屏第一行`156+123+`，第二行`516+231`，合起来为完整15字符四段码；给实际屏幕照片及VM读数。屏不显示时先核原页面控件/波特率和诊断，不能凭结果1当显示已经成功。

Watch可加g_screen_init_done/g_screen_test_request/g_screen_test_result/g_screen_command_count，保留RTOS及Safety故障/IWDG复位/APP余栈/heap和VM请求。预期初始化1、请求消费为0、结果1、完整命令数3，两个fault0、VM0。当前不冻结宏0下长时间暂停会复位，Watch可能为上一次暂停快照；实际显示和持续正常运行优先，不重做旧CAN/IWDG注入。

结果码：1UART发完；2空指针/长度或显示字符不合要求；3未初始化/未在调度运行的任务环境；4整帧超时。本步未解析回包/ACK，页面或控件不存在、波特率不匹配也可能返回1。初始化诊断1同样只是MCU配置完成，不是屏自检通过。

## 7. 需求及证据范围

检查DSP-01/DSP-02/QR-01/PWR-01（R），DSP-03/LOC-03/RUN-06（E）；QR-04（Q）继续待技术答疑确认。固定15字符和数字/加号限制保留旧样例显示功能并避免命令注入，不等于官方任务码精确语法/业务合法性冻结。真实扫码及解析在后续UART5步骤；先字符长度/内容验证后再发送，短码不会越界读。

字体至少12mm须实物测量，14～16mm为工程目标；真实正确抓放次数要与实际动作验收连接。当前不签屏全功能/双向接收/整车READY。用户已确认正常供电和同一块原屏；精确电压/TX电平及A01原厂完整规格仍未取得，标作资料/测量范围，不作为本轮用户明确推进的阻断项。

私人证据SCREEN_UART4_private_compile.json：仅两单元-c，不链接/烧录/硬件动作，五真实源SHA前后不变；临时私人文件按路径校验后清理。正式构建/实屏两行/发送错误和任务故障显示、字体与抓放统计仍待后续证据。
