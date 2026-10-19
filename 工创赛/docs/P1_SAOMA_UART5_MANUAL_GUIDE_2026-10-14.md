# 扫码模块UART5到原page0两行显示：手动迁移

2026-10-14。用户确认继续使用原F103工程同一块扫码模块；9600/8N1沿用实际旧源。本步仅接收/基本格式检查/显示实际原始码，不启动搬运。抓放统计按用户决定延后，page1不作为前置。以下为工程实现（E），R/Q见末节。

## 1. 修改范围与接收策略

新建HARDWARE/SAOMA/saoma.h及saoma.c，main增量接入；screen.c/h不用改。PD2 UART5_RX，AF8；本步RX-only，不发送扫码配置/触发指令，沿用扫码模块原来的扫描方式。NVIC仍Group4、UART5优先级6，接收中断启动后才开。中断将字节和时间戳入128项队列，TaskA每轮最多取64项；接收状态不在ISR中解析或调用屏幕。

一帧以CR/LF结束；无结束符时以实际字节间隔50ms结束，四段各3个数字/3个加号共15字符才通过。超过长度、字符/加号位置错误、串口丢字节均拒绝，丢字节后清队列并等边界重新同步。这个格式和50ms为当前旧模块的工程兼容方案，真实分帧仍需扫码实测；不是官方语义冻结。不校验颜色映射/重复规则或任务合法性，后续QR-04答疑和正式READY接入前继续补。

SAOMA_Data和SAOMA_DataReady保留，只有任务A发布/消费，ISR不改字符串。一个轮次遇到多帧时保留最新已完整通过的码；若最后一帧拒绝则不发布新结果，屏可能保留上次码，此时不能认为最新扫码已有效。记录accepted/rejected和last_result。每轮最多一次完整屏幕更新，原有有界发送与APP心跳保留。

## 2. 新建HARDWARE/SAOMA/saoma.h

```c
#ifndef SAOMA_H
#define SAOMA_H
#include <stdint.h>
#define SAOMA_TASK_LENGTH 15U
extern char SAOMA_Data[SAOMA_TASK_LENGTH + 1U];
extern volatile uint8_t SAOMA_DataReady;
extern volatile uint32_t g_saoma_rx_count;
extern volatile uint32_t g_saoma_rx_errors;
extern volatile uint32_t g_saoma_accept_count;
extern volatile uint32_t g_saoma_reject_count;
/* 0未收完整帧，1基本格式通过，2长度/格式拒绝，3串口丢字节 */
extern volatile uint32_t g_saoma_last_result;
void SAOMA_Init(void);
void SAOMA_EnableReceive(void);
void SAOMA_Update(void);
#endif
```

## 3. 新建HARDWARE/SAOMA/saoma.c

按当前工程GBK/ANSI保存中文注释。不得把旧F103 saoma.c加入F407。

```c
#include "saoma.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* 同时支持回车换行结束，或无结束符时50ms字节间隔结束 */
#define SAOMA_GAP_MS 50U
#define SAOMA_QUEUE_LENGTH 128U
#define SAOMA_DRAIN_LIMIT 64U

typedef struct
{
    TickType_t tick;
    uint8_t data;
} SaomaByte;
static QueueHandle_t s_queue = NULL;
static char s_frame[SAOMA_TASK_LENGTH + 1U];
static uint32_t s_length = 0U;
static uint32_t s_bad = 0U;
static uint32_t s_seen_errors = 0U;
static uint32_t s_active = 0U;
static TickType_t s_last_tick = 0U;
char SAOMA_Data[SAOMA_TASK_LENGTH + 1U] = {0};
volatile uint8_t SAOMA_DataReady = 0U;
volatile uint32_t g_saoma_rx_count = 0U;
volatile uint32_t g_saoma_rx_errors = 0U;
volatile uint32_t g_saoma_accept_count = 0U;
volatile uint32_t g_saoma_reject_count = 0U;
volatile uint32_t g_saoma_last_result = 0U;

void SAOMA_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    s_queue = xQueueCreate(SAOMA_QUEUE_LENGTH, sizeof(SaomaByte));
    configASSERT(s_queue != NULL);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 9600U;
    uart.USART_Mode = USART_Mode_Rx;
    USART_Init(UART5, &uart);
    USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);
    /* 沿用主函数Group4，不照搬F103的Group2 */
    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(UART5, ENABLE);
}

/* 调度运行后，由任务A开启接收 */
void SAOMA_EnableReceive(void)
{
    configASSERT(s_queue != NULL);
    (void)UART5->SR;
    (void)UART5->DR;
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
}

void UART5_IRQHandler(void)
{
    BaseType_t wake = pdFALSE;
    SaomaByte item;
    uint32_t status = UART5->SR;
    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE |
                   USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) != 0U)
    {
        item.data = (uint8_t)UART5->DR;
        if ((status & (USART_FLAG_ORE | USART_FLAG_FE |
                       USART_FLAG_NE | USART_FLAG_PE)) != 0U)
        {
            g_saoma_rx_errors++;
        }
        else if ((status & USART_FLAG_RXNE) != 0U)
        {
            item.tick = xTaskGetTickCountFromISR();
            g_saoma_rx_count++;
            if (xQueueSendFromISR(s_queue, &item, &wake) != pdPASS)
            {
                g_saoma_rx_errors++;
            }
        }
    }
    portYIELD_FROM_ISR(wake);
}

/* 只检查四段数字+加号的基本外形，不冻结颜色/重复等官方语义 */
static void SAOMA_Finish(void)
{
    uint32_t i;
    uint32_t valid = (s_bad == 0U && s_length == 15U) ? 1U : 0U;
    if (s_active == 0U) return;
    for (i = 0U; i < s_length && valid != 0U; i++)
    {
        if (i == 3U || i == 7U || i == 11U)
        {
            if (s_frame[i] != '+') valid = 0U;
        }
        else if (s_frame[i] < '0' || s_frame[i] > '9') valid = 0U;
    }
    /* 短临界区交接，UART5不会在检查丢字节后插入新错误 */
    taskENTER_CRITICAL();
    if (g_saoma_rx_errors != s_seen_errors) valid = 0U;
    if (valid != 0U)
    {
        s_frame[15] = '\0';
        memcpy(SAOMA_Data, s_frame, sizeof(SAOMA_Data));
        SAOMA_DataReady = 1U;
        g_saoma_accept_count++;
        g_saoma_last_result = 1U;
    }
    else
    {
        SAOMA_DataReady = 0U;
        g_saoma_reject_count++;
        g_saoma_last_result = 2U;
    }
    taskEXIT_CRITICAL();
    s_length = 0U;
    s_bad = 0U;
    s_active = 0U;
}

void SAOMA_Update(void)
{
    SaomaByte item;
    uint32_t n;
    TickType_t gap = pdMS_TO_TICKS(SAOMA_GAP_MS);
    /* 丢字节后清掉不完整队列，直到结束符/新间隔再重新收帧 */
    if (s_seen_errors != g_saoma_rx_errors)
    {
        taskENTER_CRITICAL();
        s_seen_errors = g_saoma_rx_errors;
        (void)xQueueReset(s_queue);
        taskEXIT_CRITICAL();
        s_length = 0U;
        s_bad = 1U;
        s_active = 1U;
        s_last_tick = xTaskGetTickCount();
        SAOMA_DataReady = 0U;
        g_saoma_last_result = 3U;
    }
    for (n = 0U; n < SAOMA_DRAIN_LIMIT; n++)
    {
        if (s_seen_errors != g_saoma_rx_errors) break;
        if (xQueueReceive(s_queue, &item, 0U) != pdPASS) break;
        if (s_active != 0U && (TickType_t)(item.tick - s_last_tick) >= gap)
        {
            SAOMA_Finish();
        }
        s_last_tick = item.tick;
        if (item.data == '\r' || item.data == '\n')
        {
            SAOMA_Finish();
            continue;
        }
        s_active = 1U;
        if (s_length < SAOMA_TASK_LENGTH)
        {
            s_frame[s_length++] = (char)item.data;
        }
        else s_bad = 1U;
    }
    /* 队列尚有待处理数据时，不用当前时间误判帧已结束 */
    if (s_active != 0U && uxQueueMessagesWaiting(s_queue) == 0U &&
        (TickType_t)(xTaskGetTickCount() - s_last_tick) >= gap)
    {
        SAOMA_Finish();
    }
}
```

## 4. USER/main.c五处增量修改

### 4.1 screen.h后添加头文件

```c
#include "saoma.h"
#include <stdio.h>
```

### 4.2 替换两项屏幕固定样例全局变量

删除旧g_screen_test_request、g_screen_test_result及其注释，改为：

```c
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_scan_display_result = 0U;
volatile uint32_t g_scan_display_count = 0U;
volatile uint32_t g_scan_log_send_ok = 0U;
```

### 4.3 App_TestTaskA开启USART1接收之后

```c
    SAOMA_EnableReceive();
```

保留函数开头screen_start_tick及原USART1/蜂鸣/CAN回环。它用于启动满1秒后才更新屏幕，不等待或卡住任务。

### 4.4 替换整个旧固定样例if块

删除原注释“不阻塞启动等待”开始到调用Screen_ShowTaskCode固定样例及其结束大括号的整块，位置在Safety_CanReport后、g_test_tick赋值前，改为：

```c
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        SAOMA_Update();
        if (SAOMA_DataReady != 0U &&
            (TickType_t)(xTaskGetTickCount() - screen_start_tick) >=
            pdMS_TO_TICKS(1000U))
        {
            char scan_log[24];
            SAOMA_DataReady = 0U;
            g_scan_display_result = Screen_ShowTaskCode(SAOMA_Data);
            if (g_scan_display_result == SCREEN_OK)
            {
                g_scan_display_count++;
            }
            /* 诊断发送接受不等于PC已经收到；忙时返回0 */
            sprintf(scan_log, "SCAN:%s\r\n", SAOMA_Data);
            g_scan_log_send_ok = USART1_SendString(scan_log);
        }
```

后面的g_test_tick、栈堆统计、Safety_Report及100ms延时仍保留。不要同时保留旧样例分支，否则会覆盖实际扫码结果。

### 4.5 main中的Safety_Init之后添加

```c
    SAOMA_Init();
```

之后仍原Watchdog_Init/断言/五任务创建，不改Group4，不新增任务。本步接收队列新增约1KB堆开销，实际栈堆和监督健康在下载后观察。

## 5. Keil加入并Rebuild

新增SAOMA分组或在HARDWARE中加入新saoma.c；头路径追加 .\HARDWARE\SAOMA。现有标准库usart.c及FreeRTOS queue.c已编入，不重复添加。UART5_IRQHandler只保留本模块一份，若原工程另有同名实现先核对冲突；本次只读未发现USER中同名定义。

保存，Rebuild并发完整日志。私人两单元-c编译通过不等于用户完整链接/实机已验，助手未改实际工程或烧录。

## 6. 正式构建通过后的下一实机门禁

下载后保持VM请求0，用原扫码方式先扫156+123+516+231。串口助手115200/8N1应出现SCAN:156+123+516+231，屏page0第一行156+123+、第二行516+231。再扫另一张码确认原页面能更新。提供实际屏幕/串口截图及VM；不要把上次残留屏幕文字当本次收到的数据，核SCAN行和接收计数。

可观察SAOMA_Data、g_saoma_rx_count、g_saoma_rx_errors、g_saoma_accept_count、g_saoma_reject_count、g_saoma_last_result、g_scan_display_result、g_scan_display_count，以及RTOS/Safety故障、APP余栈/heap。解析1只表示基本外形有效，显示结果1只表示UART发完。错误恢复/半帧/重复扫码和超长码在基本接通后定向验证；不长时间Stop编辑变量，当前IWDG宏0会复位。

## 7. 需求/限制

检查QR-01/DSP-01（R）、QR-03/LOC-03/RUN-06（E），QR-04（Q）待技术答疑确认。此模块只保留原始四段码，尚未形成两个批次的业务对象，不能宣称QR-01完整解析/比赛任务执行已验。基本长度与加号位置是旧模块显示格式工程输入，不冻结官方颜色映射/重复规则。实际模块完整型号未填，用户已确认同原F103模块；不再请求确认是否同一块或编造CY-SCAN01型号。

私人证据SAOMA_UART5_private_compile.json，四实际源SHA不变，无链接/下载/硬件动作；私人文件清理后记录cleaned。正式构建、真实扫码/屏显、异常恢复和完整业务合法性/正式READY仍待验。抓放统计/最终结果显示保持延后到全程可跑阶段，不新增诊断页。
