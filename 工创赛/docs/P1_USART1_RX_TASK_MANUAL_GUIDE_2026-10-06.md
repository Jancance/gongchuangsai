# USART1独立接收任务与100字节突发验收

最新结果：用户100字节截图S100、count100、last0x39、mismatches/errors/fault0、create1，Idle/PSP正常；随后补图HWM0xD9=217字=868字节、heap当前/历史最小0x3FC8=16328字节。此次突发和新任务内存门禁闭环，保持栈256字/heap20KB。下一必要清理由用户删main中的临时mismatches变量和数字序列比较内层if，保留通用WaitByte接收/统计/HWM及其他功能；保存Rebuild下载，准备PC13输入。不再重复本次测试。本项不证明持续满速或整车安全。

2026-10-06。ABC短串截图已确认count3、last0x43、接收错误事件0、fault0，PC在Idle/Thread/PSP；USART1两个方向与基本中断队列交接通过。应用错误计数当前仅覆盖ORE和队列满，不宣称全故障条件已测试。检查LOC-03、RUN-06（E）、PWR-01（R），官方Q不变。

原任务A每100ms取一个字节，不适合后续持续串口输入。此步增加独立接收任务，以xQueueReceive等待数据；FromISR入队可唤醒它，无数据只阻塞该任务。保持原A100ms/B500ms、启动提示/发送/GPIO NOPULL/优先级6，不再重复接线/电平或ABC试验。

## 1 SYSTEM/USART/usart.h增加接口

最后endif前加：

```c
uint8_t USART1_WaitByte(uint8_t *data, uint32_t timeout_ms);
```

旧ReadByte可保留，但本轮不再从任务A调用。

## 2 SYSTEM/USART/usart.c末尾新增

```c
/* 等待一个字节，超时只结束等待，不阻塞其他任务 */
uint8_t USART1_WaitByte(uint8_t *data, uint32_t timeout_ms)
{
    if ((data == NULL) || (g_rx_queue == NULL))
    {
        return 0U;
    }

    return (xQueueReceive(g_rx_queue, data,
                          pdMS_TO_TICKS(timeout_ms)) == pdPASS) ? 1U : 0U;
}
```

队列仍64字节，ISR保持原样、不忙等。100ms是等待无输入的超时，不是数据处理延迟/整车安全期限；有字节立即返回。

## 3 USER/main.c改变消费者

现有接收全局变量后加：

```c
/* 接收任务创建结果、栈余量及本轮序列错误数 */
volatile BaseType_t g_rx_task_create = pdFAIL;
volatile UBaseType_t g_rx_stack_words = 0U;
volatile uint32_t g_serial_rx_test_mismatches = 0U;
```

从App_TestTaskA删除局部uint8_t rx_data;及整个USART1_ReadByte的if块；保留在函数入口的USART1_EnableReceive、READY发送和Buzzer_Start，以及循环中的其他计数/诊断/延时，确保只有新任务消费接收队列。

在main前、任务B函数后新增：

```c
/* 独立接收任务：有数据立即处理，无数据进入等待 */
static void App_UartRxTask(void *argument)
{
    uint8_t data;

    (void)argument;

    while (1)
    {
        if (USART1_WaitByte(&data, 100U) != 0U)
        {
            /* 仅本轮检查重复的0123456789测试序列 */
            if (data != (uint8_t)('0' + (g_serial_rx_count % 10U)))
            {
                g_serial_rx_test_mismatches++;
            }

            g_serial_rx_count++;
            g_serial_rx_last = data;
        }
        else
        {
            /* 空闲时记录该任务历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
    }
}
```

序列比较及mismatches仅本次突发验收使用，非正式串口协议；后续转协议工作时替换此测试解析，不能将任意输入都按数字序列认定故障。HWM只在空闲超时更新，发送结束等1秒后再读以捕获本次低水位。256字栈/优先级3为本轮候选，不是最终整车任务配置。

main中两个测试任务创建检查后、vTaskStartScheduler前加入：

```c
/* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   256U, NULL, 3U, NULL);
    configASSERT(g_rx_task_create == pdPASS);
```

所有任务仍在调度前创建；调度器先运行接收任务，等待队列后任务A启动并开放RX。助手不改用户源/工程；已在工作区临时副本用真实ARMCC5验证新usart/main两源均0错误0警告，原用户源哈希一致，仅编译未链接/下载。临时副本清理，证据uart1_rx_task_private_compile_probe.json。

## 4 一次100字节测试

用户保存全部、Rebuild、下载、复位/F5正常运行，等READY出现后XCOM文本发送一次以下单行，不带额外空格/回车/换行，关发送新行/HEX/定时：

```text
0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789
```

发送长度100字节；XCOM S是累计值，看本次增量，不要求总S100。发送后等1秒，Stop而非Reset；Watch同时查看g_serial_rx_count、last、mismatches、g_uart1_rx_errors、g_rtos_fault、g_rx_task_create、g_rx_stack_words、g_heap_free_bytes、g_heap_min_bytes。预计count100（0x64）、last0x39（字符9）、mismatches/errors/fault0、create1，栈及堆余量非0并记录实际值，不预先冻结尺寸。LED仍正常；100字节过后不持续发或在暂停时发。

本项证明一次短突发的计数/序列与新任务余量，不能外推连续满速115200、长时间压力、雷达CRC/COBS/事务、控制实时性或硬件安全。通过后不继续重复同一100字节测试，进入下一基础模块。
