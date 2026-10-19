# 两测试任务栈与heap余量：手动测量步骤

日期：2026-10-06。当前两任务计数增长经视频与手动复位解释确认，基础运行门禁通过；本次仅测当前无动作测试的内存余量，不能视为完整整车/所有功能栈的最终尺寸验收。实际工程保持用户指定D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程。检查LOC-03、RUN-06（E），R和官方Q不变。

最新结果：用户已手动加入片段，并提供PC=0x0800099C、Thread/PSP、Idle清理检查处的新运行截图。A499/B100/tick49800、创建均1、fault/returned均0；A/B最小剩余栈均233字=932字节，heap当前/历史最低均17600字节。两简化任务本项检查通过；保留当前256字任务栈/20KB堆，不缩减。下一单项核对运行态SystemCoreClock与SysTick LOAD/CTRL低3位，详见P1_EXECUTION_LOG。实际业务任务、Idle和MSP仍另验。

历史：首次同值截图停在复位入口、尚未执行SystemInit/__main，因此当时仅作前次运行候选RAM读数，未据此直接通过。

## 修改位置与中文片段

1. USER/FreeRTOSConfig.h的INCLUDE接口配置段、最后#endif之前加入：

```c
/* 开启任务历史最小剩余栈查询 */
#define INCLUDE_uxTaskGetStackHighWaterMark      1
```

2. USER/main.c已有测试全局变量后加入：

```c
/* 栈余量单位为32位字，每字4字节 */
volatile UBaseType_t g_stack_a_words = 0U;
volatile UBaseType_t g_stack_b_words = 0U;
/* 堆余量单位为字节 */
volatile size_t g_heap_free_bytes = 0U;
volatile size_t g_heap_min_bytes = 0U;
```

3. App_TestTaskA的while内、vTaskDelay之前加入：

```c
/* NULL表示查询当前任务，记录历史最小剩余栈及堆余量 */
g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
g_heap_free_bytes = xPortGetFreeHeapSize();
g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
```

4. App_TestTaskB的while内、vTaskDelay之前加入：

```c
/* 查询任务B自己的历史最小剩余栈 */
g_stack_b_words = uxTaskGetStackHighWaterMark(NULL);
```

变量分别由对应任务写入，heap两个快照由A更新。标准stdint/stddef及接口已由现有FreeRTOS/task头提供，不需要创建第三个任务或修改port.c。修改及编译/下载由用户操作，助手不改源码。

## 运行观察与判断

保存、Rebuild通过后下载并复位启动，运行10～20秒，暂停观察四个新变量及现有g_rtos_fault/计数。这一轮不反复复位。记录截图与原始值，之后才能冻结当前测试候选的每任务256字/heap20KB。

- 栈高水位API返回该任务自启动以来最小剩余栈，单位是StackType_t字；本ARM_CM4F端口每字4字节。不是最大已用栈字节数。
- heap_free是当前未分配总字节；heap_min是启动以来最小未分配总字节，不是连续最大块，也不能单独证明没有碎片。
- 测试任务不再动态创建对象，初始化完成后的heap读数应稳定；fault应0、任务计数仍推进。栈余量接近0时需进一步分析或增加栈，不先把无故障当余量足够。
- 正常读数只覆盖这两个简化任务；Idle、MSP中断栈、实际外设/控制任务和最坏负载另行验证，不能沿用本次读数认定全车内存安全。

依据：[官方task.h高水位接口说明](https://github.com/FreeRTOS/FreeRTOS-Kernel/blob/V11.3.1/include/task.h)、[heap_4实现](https://github.com/FreeRTOS/FreeRTOS-Kernel/blob/V11.3.1/portable/MemMang/heap_4.c)、本机相同版本文件与用户新运行截图。片段由用户手动加入并在本板运行，助手未修改固件；验收范围仅为当前两测试任务。
