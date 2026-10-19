# CAN采集移入独立任务：用户手动修改指南

> 2026-10-15独立CAN采集/扫码并行通过：反馈count18/18/17/17→39/39/39/39，两mask0F/error0、两fault0/两ready1，CAN余栈297字/heap11320B；用户明确换码后屏幕正常更新。唯一下一项不改源，按P1_CAN_MISSING_NODE_STATIC_CHECK_2026-10-15.md做缺失ID1的启动状态保护：先VM请求0/反馈0运行并实测VM0，再断左后ID1一路，其余三台保留；第33行关断断点后VM请求1抓fault20/cause2/badmask1及实际VM0。不是带电插拔、运行中断线或运动验收；10ms旧超时根因未唯一确认。

2026-10-15，工程建议（E）。本轮先手动修改并Rebuild，构建通过后再按第6节静态验收。只改实际工程USER/main.c；助手未修改实际固件。当前源TaskA末尾已为50ms。原扫码、屏幕、按键/START锁码、看门狗、监督、手动版本/状态/速度/位置查询均保留。

## 已通过与本次目的

用户两图728bfb31/0ff8367a：feedback_enable1，四台计数从12/12/12/13增长到20/20/20/21，两反馈有效mask均0F、error0、RTOS/监督fault0、两ready1、ESR0。接受50ms静态持续采集计数/有效性门禁；未展示实时RPM/raw数组、精确采集年龄或新物理VM读数，不补造。1秒新鲜度仍仅静态候选，尚未验收行车闭环。

CAN现在和屏幕/扫码共用TaskA。此次将CAN初始化及查询全部移入独立任务，CAN每次交易后仍等待50ms；TaskA可每10ms处理扫码、屏幕和启动门禁。不能让TaskA和CAN任务同时调用CAN接口。没有增加电机使能、转动、清零或校准指令。

串口调试发送原本只有TaskA调用。移出手动查询后会出现两个任务共用发送缓冲区，故本次增加短临界区包装；只复制最多64字节、非阻塞，忙时仍返回0，不等待发送，不循环重试，不提高保护门限。

## 1. 增加三个观察变量

文件：USER/main.c。在 `g_safety_task_create` 及其注释之前增加：

```c
/* CAN独立采集任务的创建结果、循环次数和历史最小余栈 */
volatile BaseType_t g_can_task_create = pdFAIL;
volatile uint32_t g_can_task_count = 0U;
volatile UBaseType_t g_can_task_stack_words = 0U;

```

## 2. 给共用调试发送增加短临界区

在 `static void App_CanPollStep(void)` 前面的注释之前增加：

```c
/* 两个任务共用调试发送缓冲区，短临界区内只复制，不等待串口 */
static uint8_t App_DebugSendString(const char *text)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = USART1_SendString(text);
    taskEXIT_CRITICAL();
    return result;
}

```

然后把main.c中原有的5处发送调用名 `USART1_SendString` 改为 `App_DebugSendString`：START日志、POS日志、SPEED日志、启动READY文本、SCAN日志。按搜索结果逐个改完；**刚添加的包装函数内部那一处 `USART1_SendString(text)` 必须保留原名**，否则递归。

改完搜索：main.c中 `USART1_SendString(` 只剩包装内部1处；其他调用均为 `App_DebugSendString(`。不要改usart.c/h。

## 3. 将整段CAN查询代码剪切到新任务

在 `App_TestTaskA` 的while循环中，从以下注释开始剪切：

```c
        /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
```

一直剪切到下面调用的分号结束，**包括这个调用**：

```c
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
        g_can_poll_protect_mask, g_can_poll_tick);
```

这段包括完整if/else手动查询、`App_CanFeedbackStep()`、`App_CanPollRefresh()`、`App_CanFeedbackRefresh()` 和 `Safety_CanReport()`，不能只剪一个函数调用。末尾后面的 `/* 一轮最多处理64字节... */` 和 `SAOMA_Update()` 留在TaskA。

然后在 `App_TestTaskA` 定义之前新增下面函数，把刚剪切的整段粘贴到标注位置：

```c
/* CAN1只由本任务初始化和查询；每轮交易结束后至少等待50毫秒 */
static void App_CanTask(void *argument)
{
    (void)argument;
    g_can_test_result = CAN1_LoopbackTest();
    while (1)
    {
        g_can_task_count++;
        if (g_can_test_result == 1U)
        {
            /* 把第3步剪切的完整查询代码粘贴在这里，替换本注释 */
        }
        else
        {
            /* 回环失败时不发电机查询，监督沿用原启动故障判定 */
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
        }
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}
```

新任务放在 `App_StartGateUpdate()` 结束之后、`App_TestTaskA()` 之前即可，原有CAN辅助函数此时已定义，无需新增头文件或前置声明。

## 4. TaskA移除回环初始化，恢复应用处理10ms

TaskA开始部分删除这两行，CAN回环现在由新任务执行一次：

```c
    /* 调度启动后只执行一次，函数内部包含CAN初始化 */
    g_can_test_result = CAN1_LoopbackTest();
```

保留原USART接收/扫码接收开启、READY日志和启动蜂鸣。TaskA只剩蜂鸣更新、扫码/屏幕、START门禁、APP心跳和栈堆统计。

把TaskA末尾的50U改为10U：

```c
        vTaskDelay(pdMS_TO_TICKS(10U));
```

**只改TaskA**。新CAN任务末尾必须保持50U。此次TaskA的10ms不再决定CAN查询间隔，不能把新CAN任务也改为10ms。任务A和蜂鸣旧“每100毫秒”注释可分别改为“扫码、屏幕和启动门禁，轮间隔10毫秒”“按当前tick更新蜂鸣状态”。原CAN辅助函数“只由任务A调用”注释改为“只由CAN任务调用”。原CAN_POLL_VALID_MS前的100ms注释改成“CAN任务每轮交易后等待50ms，候选新鲜度1秒”，宏值不改。

## 5. main中创建任务，然后Rebuild

在 `vTaskStartScheduler()` 前、现有任务创建及configASSERT之后增加：

```c
    /* CAN栈384字，优先级3；监督仍为4 */
    g_can_task_create = xTaskCreate(App_CanTask, "CanPoll",
                                      384U, NULL, 3U, NULL);
    configASSERT(g_can_task_create == pdPASS);

```

CAN栈384字=1536字节是待实测余量的工程选择；优先级3，与UartRx同级，低于监督4。Key/TaskA2和LED1不改。新增CAN任务不喂狗、不直接开VM；原监督继续独立用状态时间戳/保护标志判定。现阶段没有新增CAN独立心跳槽，不能将本次编译或循环计数等同全部任务失效安全完成。

保存后Rebuild，先给完整构建日志。无需新增Keil文件组或包含路径。隔离副本已用实际包含路径/ARMCC5.06编译-c通过，无错误警告输出；未链接、未烧录，实际main哈希不变。该检查不能代替用户正式构建和实物验收。

## 6. 构建后再做的静态验收

当前轮只先完成第1～5节。用户正式构建通过后再下载，仍默认VM请求0/反馈0，调试冻结宏1。先看VM0、平台ready1、回环result1，新can_task_create=pdPASS(1)、can_task_count增长；can_task_stack_words必须有余量，记录堆最小值。

按原方法VM请求1使状态健康，再反馈enable1。保留首次故障断点；正常关VM调用且fault0不能当成新故障。手动查询请求维持0。比较约3秒及再2秒四台feedback_count都增加、三mask0F、错误0、两fault0/两ready1及CAN余栈/堆。随后测试未锁码时更换二维码，屏幕/SCAN仍更新且采集计数继续增加；START已锁码时保持原锁码行为，需正常复位建立未锁码会话，不能故障清零绕门禁。

如非零故障停住，保留现场再查，不放宽门限/无条件喂狗。当前尚未新增速度/位置数据失效到运动停止的完整控制策略，不能直接发运动命令。

## 需求与证据

已检查PWR-01（R，单内置电源约束不变）、LOC-03/RUN-06/MOT-04（E）。Q条目无变更、未替代待技术答疑确认。证据：CAN_feedback_50ms_two_snapshots_pass.json与两张用户原图、CAN_task_separation_private_compile.json。未闭环：真实任务分离后的构建/采集/扫码交互、CAN余栈/堆、运行失效策略、更快运动时效及实际底盘运动。10ms超时唯一根因仍未认定；本次隔离屏幕业务不宣称消除了全部超时原因。
