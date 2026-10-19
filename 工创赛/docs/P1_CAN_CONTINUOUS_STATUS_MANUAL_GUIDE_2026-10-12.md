# CAN连续状态采集第一步：四地址轮询与新鲜度

> 采集关闭/当前内存门禁已过（2026-10-12）：两图5c8d15d3/d707cf24为poll_enable0/VM请求1、valid0/四ageFFFFFFFF、结果/flags/count清0、RTOS/监督fault0/局部ready1。APP余栈201字804B、监督233字932B、heap最小14072B。基本连续采集/两次进展/关闭撤销门禁闭环，下一项P1_CAN_FAULT_GUARD_MANUAL_GUIDE_2026-10-12.md接CAN故障。该新步骤将采集默认值改1，并在VM开启且曾健康后关闭必要采集时锁定关VM；本指南默认0/只采集行为保留历史。

> 最新基本循环通过（2026-10-12）：用户两图四个result值均1、flags均03，counts从14/14/13/13增至26/25/25/25，valid均0F/protect0/最后交易ESR0，所见age为0～303tick。接受基本循环/两次进展与新鲜快照，不扩大为响应时延或长期负载通过。下一项关闭poll_enable0/F5约1秒后暂停，验valid0/四ageFFFFFFFF，并补当前RTOS/监督故障、APP余栈及heap最小；无需重建或重做健康轮询。CAN故障关VM仍待接入。

> 最新用户构建/下载通过（2026-10-12）：三文件已手改接入，真实源只读确认CAN会话复用/有限清FIFO/公开接口保留，任务A if/else-if/else及分支外刷新位置正确。正式Rebuild链接0错误0警告，Code14644/RO424/RW360/ZI22512、2秒；Erase/Programming完成、Verify OK、Application running，用户日志20:53:39下载结束。下一项第6节VM供电/驱动启动完成后启用采集，验四节点结果/新鲜度与两次计数增长；不重改第1～4节或重复构建，CAN故障关VM仍未接入。

> 私人候选编译已完成：can.c/main.c两个单元ARMCC5.06 update5 build528下0错误0警告，原can.c/can.h/main.c SHA前后不变，私人文件已清理；未实际链接/下载或进行循环测试。下一项用户按第1～4节修改三个文件并Rebuild，再验实机循环。

本轮由用户手动修改实际工程的can.c、can.h、main.c。保留版本查询、手动状态查询、启动静默回环、任务心跳和已通过的VM故障锁定，不改PCB或电机菜单，不发送运动/使能/校准命令。

本步是静态采集工程实现（E）：先验证四节点结果、成功计数、新鲜度及保护位观测，之后把这些结果接入Safety故障锁定并验证通信故障关VM，再接看门狗。当前采集尚不自动触发CAN故障关VM；g_can_poll_valid_mask表示数据新鲜，不表示整车允许运动。原任务监督仍有效。

## 1. SYSTEM/CAN/can.c保持正常模式会话

在include区结束后、CAN1_InitMode之前加入：

```c
/* 当前所有CAN调用都来自任务A，不允许其他任务并发使用 */
static uint32_t s_can1_normal_ready = 0U;

/* 标记下一次查询需重新初始化，不在这里发送或切换硬件模式 */
void CAN1_ResetSession(void)
{
    s_can1_normal_ready = 0U;
}
```

在CAN1_LoopbackTest内部局部变量声明之后、原result=CAN1_InitMode静默回环之前添加：

```c
CAN1_ResetSession();
```

在CAN1_ReadParameter内部，原uint8_t status声明之后添加（不是回环函数）：

```c
uint8_t flush_count;
```

仅在CAN1_ReadParameter内，将下面原块：

```c
/* 真实电机通信使用正常模式 */
result = CAN1_InitMode(CAN_Mode_Normal);
if (result != 1U)
{
    return result;
}
```

替换为：

```c
/* 同一连续会话只初始化一次，避免每条请求反复重置CAN */
if (s_can1_normal_ready == 0U)
{
    result = CAN1_InitMode(CAN_Mode_Normal);
    if (result != 1U)
    {
        return result;
    }
    s_can1_normal_ready = 1U;
}

/* FIFO0最多容纳3帧，有限清理前一请求遗留的数据 */
for (flush_count = 0U; flush_count < 3U; flush_count++)
{
    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0U)
    {
        break;
    }
    CAN_Receive(CAN1, CAN_FIFO0, &rx);
}
```

后面的tx构造、发送/接收有限等待、地址/功能码/长度/校验尾匹配保持。

把文件末尾两个短的公开函数完整替换为下面内容，函数名/参数/返回码和协议不变：

```c
/* 读取版本，失败后下一次查询重新建立正常模式会话 */
uint32_t CAN1_ReadVersion(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x1FU, 6U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}

/* 读取状态，不改变驱动器的使能或运动状态 */
uint32_t CAN1_ReadStatus(uint8_t address, CanRxMsg *reply)
{
    uint32_t result = CAN1_ReadParameter(address, 0x3AU, 3U, reply);
    if (result != 1U)
    {
        CAN1_ResetSession();
    }
    return result;
}
```

失败后只重建主控CAN会话，不清驱动器保护/故障、不自动重新允许VM。ESR仍在本次调用返回后、下一次查询前保存。

## 2. SYSTEM/CAN/can.h添加声明

在已有CAN1_ReadStatus声明后加入：

```c
void CAN1_ResetSession(void);
```

## 3. USER/main.c增加每节点快照

在原g_can_status_reply之后添加：

```c
/* 静态采集开关：默认0，驱动器上电完成后再由Watch改1 */
volatile uint32_t g_can_poll_enable = 0U;
/* 数组下标0/1/2/3对应地址1/2/3/4 */
volatile uint32_t g_can_poll_result[4] = {0U};
volatile uint32_t g_can_poll_flags[4] = {0U};
volatile TickType_t g_can_poll_tick[4] = {0U};
volatile uint32_t g_can_poll_count[4] = {0U};
volatile uint32_t g_can_poll_age_ticks[4] = {0U};
volatile uint32_t g_can_poll_valid_mask = 0U;
volatile uint32_t g_can_poll_protect_mask = 0U;
volatile uint32_t g_can_poll_esr = 0U;
CanRxMsg g_can_poll_reply;

static uint32_t s_can_poll_index = 0U;
static uint32_t s_can_poll_active = 0U;
/* 本步100ms查一台，名义四台约400ms，候选新鲜度1秒 */
#define CAN_POLL_VALID_MS 1000U
```

在原App_TestTaskA函数之前添加下面两个小函数。当前所有CAN交易仍只有任务A一个调用方，没有新增CAN任务/中断或并发调用。

```c
/* 每轮只采一台，保持成功计数/时间戳；关闭时清除本会话快照 */
static void App_CanPollStep(void)
{
    uint32_t i;
    uint32_t index;
    uint32_t result;

    if ((g_can_poll_enable != 1U) ||
        (g_vm_test_request != 1U) || (g_safety_ready != 1U))
    {
        CAN1_ResetSession();
        s_can_poll_active = 0U;
        s_can_poll_index = 0U;
        g_can_poll_valid_mask = 0U;
        g_can_poll_protect_mask = 0U;
        for (i = 0U; i < 4U; i++)
        {
            g_can_poll_result[i] = 0U;
            g_can_poll_flags[i] = 0U;
            g_can_poll_tick[i] = 0U;
            g_can_poll_count[i] = 0U;
            g_can_poll_age_ticks[i] = 0xFFFFFFFFU;
        }
        return;
    }

    if (s_can_poll_active == 0U)
    {
        CAN1_ResetSession();
        s_can_poll_index = 0U;
        s_can_poll_active = 1U;
    }

    index = s_can_poll_index;
    result = CAN1_ReadStatus((uint8_t)(index + 1U),
                            &g_can_poll_reply);
    g_can_poll_result[index] = result;
    g_can_poll_esr = CAN1->ESR;
    if (result == 1U)
    {
        g_can_poll_flags[index] = g_can_poll_reply.Data[1];
        g_can_poll_tick[index] = xTaskGetTickCount();
        g_can_poll_count[index]++;
    }
    else
    {
        /* 本节点失败后撤销其快照，不能继续使用旧状态 */
        g_can_poll_flags[index] = 0U;
        g_can_poll_tick[index] = 0U;
    }
    s_can_poll_index = (index + 1U) % 4U;
}

/* 每轮都计算新鲜度，手动查询占用总线时也不能冻结旧有效标志 */
static void App_CanPollRefresh(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t age;
    uint32_t i;
    uint32_t valid = 0U;
    uint32_t protect = 0U;

    for (i = 0U; i < 4U; i++)
    {
        g_can_poll_age_ticks[i] = 0xFFFFFFFFU;
        if ((g_can_poll_enable == 1U) &&
            (g_vm_test_request == 1U) && (g_safety_ready == 1U) &&
            (g_can_poll_result[i] == 1U) &&
            (s_can_poll_active == 1U))
        {
            /* 无符号相减，允许tick正常回绕 */
            age = (TickType_t)(now - g_can_poll_tick[i]);
            g_can_poll_age_ticks[i] = (uint32_t)age;
            if (age <= pdMS_TO_TICKS(CAN_POLL_VALID_MS))
            {
                valid |= 1UL << i;
                if ((g_can_poll_flags[i] & 0x0CU) != 0U)
                {
                    protect |= 1UL << i;
                }
            }
        }
    }
    g_can_poll_valid_mask = valid;
    g_can_poll_protect_mask = protect;
}
```

age单位为tick，当前配置1tick=1ms。FFFFFFFF表示无有效采样，不能解释为“0毫秒新数据”。保护mask按节点位置标识新鲜状态中的堵转/堵转保护位（Flags04/08）；未采样、失败或过期时valid位撤销，即使历史计数非0也不能使用旧数据。成功计数用于观察进展，不作为新鲜度条件；关闭采集/VM或正式监督失效时，下一轮清本会话计数/状态。ESR是最后一条交易的诊断快照，不代表四节点整个历史均无错误。

## 4. 任务A每轮只选择一条CAN交易

在App_TestTaskA中找到第二段手动状态查询，把它开头的 `if` 改成 `else if`，保留条件和内部完整代码。原版本查询仍是前面的if。

```c
/* VM请求开启后，手动读取一次电机状态 */
else if ((g_can_status_request == 1U) &&
         (g_vm_test_request == 1U))
```

在这整个手动状态查询分支结束的大括号之后、原g_test_tick赋值之前加入：

```c
else
{
    /* 无手动请求时，按地址1到4循环采状态 */
    App_CanPollStep();
}
App_CanPollRefresh();
```

最终顺序是版本if → 手动状态else if → 连续采集else → 每轮刷新新鲜度 → 原tick/内存/APP心跳/100ms延时。原版本/状态请求同时置1时分两轮处理，不能删除任何原分支。g_can_version_address继续仅为手动查询选地址，连续采集固定覆盖1～4。

不更改Safety模块、五个任务栈/优先级、UART中断/时钟或电机菜单。本步不把valid/protect直接接VM；下一步才加CAN故障锁定与实际故障测试。

## 5. 本轮先验用户Rebuild

用户保存三个文件、Rebuild，把完整错误/警告和Program Size发回。助手私人候选编译仅验证语法/接口，不替代用户构建链接与实机四节点循环。

## 6. 构建通过后静态采集门禁

用户下载/F5正常启动，硬件允许软件控制、VM请求1/F5，确认电机供电并启动完成；此时采集开关仍0，两个手动请求保持0。然后暂停设置g_can_poll_enable=1、F5约3秒后暂停。

Watch保留VM请求、RTOS/监督故障、APP余栈/heap，并添加：

```text
g_can_poll_enable
g_can_poll_result
g_can_poll_flags
g_can_poll_count
g_can_poll_age_ticks
g_can_poll_valid_mask
g_can_poll_protect_mask
g_can_poll_esr
```

展开四数组，预期四个result均1，四个成功计数均大于0，valid_mask为0x0F（四台数据均新鲜）、protect_mask通常0但以实测为准、ESR及两个fault0。Flags不写死03；出现保护位时先记录原因，不能用0清除来伪装正常。本轮只采状态，不人为堵转。

恢复F5约2秒再暂停，四个count应继续增加；回报两次截图，APP栈和heap以实测验收。不要把调试CPU暂停期间不更新误判为CAN掉线，tick同样暂停。需观察不同故障/持续负载时另做明确测试，不热插拔模块或用新增端接测量替代当前循环门禁。

采集0或VM请求0后有效mask应撤销；该边界在健康循环通过后验证。本步CAN信息尚未接入故障锁定，因此不是通信故障自动关VM验收，也不是完整CAN调度/运动控制架构。后续高速控制前须把交易从平台任务迁入独立CAN服务并重新验时序，新鲜度阈值和控制响应按实际设计冻结。

## 7. 需求与未闭环

已检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），官方Q不变；厂家3A短帧与04/08保护位沿用已验协议，不新增规则。证据为四台原状态帧、清理后的监督启动及现有源只读检查；私人编译结果另记。用户实际增量构建、循环/两次进展、关闭/失败失效、CAN故障接入Safety、看门狗及完整安全/运动仍待验，助手不改真实固件或操纵硬件。
