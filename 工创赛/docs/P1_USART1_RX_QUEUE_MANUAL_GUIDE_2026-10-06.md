# USART1接收：中断到FreeRTOS队列手动验证

最新验收：用户ABC截图count3、last0x43、接收错误事件0/fault0，Idle/PSP正常，基础RX交接通过。下一项按P1_USART1_RX_TASK_MANUAL_GUIDE_2026-10-06.md分离独立阻塞接收任务并验100字节突发；不再重复本ABC或电平测试。旧任务A100ms消费仅为基础演示，不沿用为生产协议吞吐设计。

2026-10-06。用户最新确认“有响，复位按键正常，进入下一步”，结束之前复位问题排查；不继续要求NRST测量，不重复烧录以“修复”已确认正常的按钮。USART1发送/LED/蜂鸣已具实物证据，下一项仅接收及FromISR队列交接。检查LOC-03、RUN-06（E）、PWR-01（R），官方Q不变。

## 当前证据与接线边界

最新测量：用户重测TXD/VCC均3.7V。已核对[ST官方DS8626 Rev12](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)Table8的PA10为FT、Table14的FT输入正常工作条件及脚注：主控正常供电且数字USART模式下可接本次3.7V输入，但VIN高于VDD+0.3时必须关闭内部上/下拉。以下USART GPIO配置因此增加NOPULL修正；先改代码并下载，再接TTL TXD到PA10，模块VCC不接主控。保持主控原供电并复位/F5开始新测试，清除下载时未接输入可能留下的计数。旧0.23V仅保留历史，不认定模块坏；不把所有F407脚或断电状态都归为同样耐压条件。

已私下用安装的ARMCC5编译拟修改的usart/main临时副本，两源零错误/零警告，原用户源SHA256不变。仅编译，不是完整链接/下载/板上验收；临时副本清理，保留uart1_rx_private_compile_probe.json。NVIC优先级6符合本机MAX_SYSCALL=5，保持PriorityGroup4。接收中断必须在任务A启动后才启用，避免调度前调用FromISR。

此前USB-TTL仅接RXD接主控PA9及GND。此次3.7V读数已在上述FT/NOPULL条件下核对，不再要求模块必须空闲3.3V。适配器VCC不接主控、主控原供电保持、不改PCB。先前两源私下编译验证为队列/IRQ方案；本轮NOPULL改动由同库有效宏支持，最新完整构建/实物接收仍待用户操作。

## 1 SYSTEM/USART/usart.h最后endif前加

```c
void USART1_EnableReceive(void);
uint8_t USART1_ReadByte(uint8_t *data);

/* 接收溢出或队列满的错误事件数 */
extern volatile uint32_t g_uart1_rx_errors;
```

## 2 SYSTEM/USART/usart.c头部及初始化调整

已有include后加：

```c
#include "FreeRTOS.h"
#include "queue.h"
```

已有发送变量后加：

```c
/* 接收中断写入，任务A读取 */
static QueueHandle_t g_rx_queue = NULL;
volatile uint32_t g_uart1_rx_errors = 0U;
```

USART1_Config的三个结构体声明后、时钟开启前加（只在每次开机调用一次Config）：

```c
/* 创建64字节接收队列，失败交给已有诊断 */
g_rx_queue = xQueueCreate(64U, sizeof(uint8_t));
configASSERT(g_rx_queue != NULL);
```

把GPIO_Pin原仅9改为：

```c
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
```

同时把原GPIO_PuPd_UP改为以下值（接入3.7V前必须完成）：

```c
/* PA10为FT输入，高于VDD+0.3V时关闭内部上下拉 */
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
```

在PA9 GPIO_PinAFConfig旁新增：

```c
GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
```

把USART_Mode改为：

```c
USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
```

初始化关闭TXE中断那行后新增：

```c
/* 调度启动前不开启接收中断 */
USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
```

保留NVIC优先级6、旧发送缓冲和USART1_SendString。

## 3 新增函数与替换原中断函数

在usart.c末尾增加以下两个函数（放在IRQ之前也可）：

```c
/* 调度运行后开启接收，先清除启动前可能留下的数据 */
void USART1_EnableReceive(void)
{
    configASSERT(g_rx_queue != NULL);
    (void)USART_GetFlagStatus(USART1, USART_FLAG_RXNE);
    (void)USART_ReceiveData(USART1);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/* 从队列取一个字节，无数据立即返回，不阻塞任务 */
uint8_t USART1_ReadByte(uint8_t *data)
{
    if ((data == NULL) || (g_rx_queue == NULL))
    {
        return 0U;
    }

    return (xQueueReceive(g_rx_queue, data, 0U) == pdPASS) ? 1U : 0U;
}
```

原USART1_IRQHandler整个函数及注释替换为：

```c
/* 接收中断入队，保留发送，按需要请求任务切换 */
void USART1_IRQHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint8_t data;

    /* 溢出数据不作为有效输入，读取数据寄存器清除错误 */
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
        g_uart1_rx_errors++;
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        data = (uint8_t)USART_ReceiveData(USART1);

        /* 中断中只能使用带FromISR的队列接口 */
        if (xQueueSendFromISR(g_rx_queue, &data,
                             &higher_priority_task_woken) != pdPASS)
        {
            g_uart1_rx_errors++;
        }
    }

    /* 保留已验证的发送中断 */
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

    portYIELD_FROM_ISR(higher_priority_task_woken);
}
```

不新增第二个同名IRQ。ORE时丢弃该错误输入并清除状态，队列满亦计错误；计数是错误事件数，不是精确丢字节数。此为基础输入验证，后续协议需要处理错误事件/帧完整性，不能忽视错误后继续使用旧目标。

## 4 USER/main.c插入

测试全局变量后加：

```c
/* 任务已经处理的接收字节数与最后一字节 */
volatile uint32_t g_serial_rx_count = 0U;
volatile uint32_t g_serial_rx_last = 0U;
```

App_TestTaskA的函数开头、(void)argument之前加局部变量：

```c
uint8_t rx_data;
```

同一任务在(void)argument后、发送READY文本之前加：

```c
/* FreeRTOS启动后，才允许接收中断使用队列接口 */
USART1_EnableReceive();
```

同一任务while内、Buzzer_Update后加：

```c
/* 本轮每次取一个字节，无数据时立即返回 */
if (USART1_ReadByte(&rx_data) != 0U)
{
    g_serial_rx_count++;
    g_serial_rx_last = rx_data;
}
```

保留其他main/任务/LED/蜂鸣/诊断；不再改Keil分组/搜索路径或加第三任务。当前测试每100ms消费一字节，只用于人手短串输入，不代表115200波特率的持续吞吐能力；后续协议接收需单独任务/批量处理/容量与最坏负载验证。

## 5 单次ABC验收

保存全部、Rebuild下载、进入Keil调试并F5正常运行。USB-TTL TXD电平确认兼容后才接PA10；串口助手仍115200/8N1，文本模式。先等READY，再发送一次ABC；取消发送HEX、发送新行、定时发送等选项，确保只发3字节。等待1秒后Stop，不Reset，在Watch加g_serial_rx_count、g_serial_rx_last、g_uart1_rx_errors、g_rtos_fault。

预期count3、last0x43(字符C)、errors0、fault0，LED/蜂鸣原功能正常。此代码不回显ABC到串口助手，验收以Watch为准；先前收到的READY仍保留。收到其他计数按真实发送内容/误差定位，不强行修改计数为3；不要在任务暂停时持续发数据。本项实物接收与TXD电平尚待用户证据，不提前签署通过。
