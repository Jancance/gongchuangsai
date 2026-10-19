# CAN故障接入VM锁定：手动增量修改

> 恢复采集请求后的锁定已过（反馈2026-10-12，记录更新2026-10-13）：图5e8841d5为poll1/fault20/两个ready0/bad_nodes0F/cause1，用户VM0V，监督count1499→1830继续运行。接受原因1必要采集失效关断及恢复请求后锁定；顶部VM请求/RTOS字段未显示，不代填。下一项P1_CAN_GUARD_STALE_PROTECT_TEST_MANUAL_2026-10-12.md临时main三处，分别验cause2旧采样时间和cause3软件保护输入，完成后清理再看门狗；不重复本指南已过步骤。

> 最新第6节关断通过（2026-10-12）：图d73e90dd采集0/故障20/平台与CAN ready0/bad_nodes0F/cause1，用户VM0V；第5节触发前VM有电已确认。接受关闭必要采集的实际关断，不等同真实断线/时间过期/保护均已验。唯一下一项保持本次运行、硬件软件允许档/VM请求1，仅恢复poll_enable1/F5约1秒，验故障20/ready0/原始节点原因与VM0保持；不复位、不清fault、不改源。恢复请求锁定与原因2/3后续测试仍待证据。

> 最新用户第5节接续（2026-10-12）：真实三文件已接入且只读核对位置/默认值正确；用户两图VM请求1/poll1/RTOS及监督fault0/局部ready1、四result1/valid0F，counts35/36/35/35→45/46/46/45，确认实际VM有电压。APP余栈804B/监督920B/heap最小14064B，正常供电/循环与本轮内存已具证据。新guard ready/bad_nodes/cause未显示，完整Rebuild日志未提供，不代填。下一项Watch添加三诊断变量，确认1/0/0后做第6节关闭必要采集关VM，再恢复采集验锁定；不重做源码接入或循环测试。

> 私人候选编译已通过：safety.c/main两个单元ARMCC5.06 update5 build528下0错误0警告，真实safety.c/h/main SHA前后不变，私人文件已清理；未链接/下载或实机触发新CAN故障。下一项用户按第1～3节增量修改三文件并Rebuild，再分步验健康/故障/锁定。

本步由用户手动修改SYSTEM/SAFETY/safety.h、safety.c和USER/main.c，原CAN驱动、版本/手动状态/循环采集、五个任务和RTOS故障关VM保留。四台基本循环/两次进展、关闭采集撤销快照及当前栈堆已经有实物证据。

本步工程策略（E）：VM请求开启后，先给驱动器3秒上电等待；一旦四台都健康，之后任何节点有效状态丢失/过期，或停止必要采集，都锁定CAN数据故障并关VM。新鲜状态中的堵转/保护标志立即进入保护故障，不等启动宽限结束。Fault32/64是项目软件位掩码，不是厂家故障码或竞赛规则。

现有100ms查一台、状态新鲜度1秒仅用于静态接入，不能作为行走时安全响应的最终参数。CAN独立服务、较快控制/采样、实际响应时间及看门狗仍须后续实现/验收；本步不发送运动、校准或驱动器使能命令，不解除完整装机载物低速门禁。

## 1. SYSTEM/SAFETY/safety.h添加CAN快照接口

在原stdint.h之后添加：

```c
#include "FreeRTOS.h"
```

在现有extern变量之后加入：

```c
/* CAN健康和首次CAN故障上下文；节点位0～3对应地址1～4 */
extern volatile uint32_t g_can_guard_ready;
extern volatile uint32_t g_can_guard_bad_nodes;
/* 原因：1必要采集关闭，2有效数据缺失/过期，3堵转/保护 */
extern volatile uint32_t g_can_guard_cause;
```

在函数声明区末尾、endif之前添加：

```c
void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks);
```

## 2. SYSTEM/SAFETY/safety.c新增CAN监督状态

在现有SAFETY_*_MS宏之后加入：

```c
/* 静态接入候选：先等待驱动器上电，已有健康后不重新宽限 */
#define SAFETY_CAN_BOOT_MS  3000U
#define SAFETY_CAN_FRESH_MS 1000U
#define SAFETY_FAULT_CAN_DATA    0x20U
#define SAFETY_FAULT_CAN_PROTECT 0x40U
```

在现有静态变量s_was_ready之后、Safety_Init之前加入：

```c
volatile uint32_t g_can_guard_ready = 0U;
volatile uint32_t g_can_guard_bad_nodes = 0U;
volatile uint32_t g_can_guard_cause = 0U;

/* 两个任务共享的快照字段，配合短临界区发布/读取 */
static volatile uint32_t s_can_enabled = 0U;
static volatile uint32_t s_can_valid = 0U;
static volatile uint32_t s_can_protect = 0U;
static volatile TickType_t s_can_ticks[4];
static uint32_t s_can_requested = 0U;
static uint32_t s_can_was_ready = 0U;
static TickType_t s_can_power_tick = 0U;
```

在Safety_Init内、原Power_Disable之前添加：

```c
g_can_guard_ready = 0U;
g_can_guard_bad_nodes = 0U;
g_can_guard_cause = 0U;
s_can_enabled = 0U;
s_can_valid = 0U;
s_can_protect = 0U;
s_can_requested = 0U;
s_can_was_ready = 0U;
s_can_power_tick = 0U;
for (i = 0U; i < 4U; i++)
{
    s_can_ticks[i] = 0U;
}
```

接着在Safety_Update函数之前加入以下两个函数。原Safety_Report和Safety_Init保留，不替换整文件。

```c
/* 任务A在刷新快照后发布，短临界区保证监督读到同一组数据 */
void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks)
{
    uint32_t i;

    taskENTER_CRITICAL();
    s_can_enabled = (enabled == 1U) ? 1U : 0U;
    s_can_valid = (ticks != NULL) ? (valid_mask & 0x0FU) : 0U;
    s_can_protect = protect_mask & s_can_valid;
    for (i = 0U; i < 4U; i++)
    {
        s_can_ticks[i] = (ticks != NULL) ? ticks[i] : 0U;
    }
    taskEXIT_CRITICAL();
}

/* 只由监督任务调用，不等待CAN，不操作接收FIFO */
static void Safety_CanGuardUpdate(uint32_t vm_request)
{
    TickType_t now;
    TickType_t ticks[4];
    uint32_t enabled;
    uint32_t valid;
    uint32_t protect;
    uint32_t fresh = 0U;
    uint32_t i;

    g_can_guard_ready = 0U;
    if (g_safety_fault != 0U)
    {
        /* 已锁定故障时保留首次CAN上下文，避免自身关电导致二次误报 */
        return;
    }
    if (vm_request != 1U)
    {
        /* 正常撤销供电不是通信故障，下次请求重新计上电等待 */
        s_can_requested = 0U;
        s_can_was_ready = 0U;
        return;
    }

    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    enabled = s_can_enabled;
    valid = s_can_valid;
    protect = s_can_protect;
    for (i = 0U; i < 4U; i++)
    {
        ticks[i] = s_can_ticks[i];
    }
    taskEXIT_CRITICAL();

    if (s_can_requested == 0U)
    {
        s_can_power_tick = now;
        s_can_requested = 1U;
        s_can_was_ready = 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        /* 监督自己重算年龄，不能只相信采集任务留下的有效mask */
        if (((valid & (1UL << i)) != 0U) &&
            ((TickType_t)(now - ticks[i]) <=
             pdMS_TO_TICKS(SAFETY_CAN_FRESH_MS)))
        {
            fresh |= 1UL << i;
        }
    }

    protect &= fresh;
    if (protect != 0U)
    {
        g_can_guard_bad_nodes = protect;
        g_can_guard_cause = 3U;
        g_safety_fault |= SAFETY_FAULT_CAN_PROTECT;
        return;
    }
    if ((enabled == 1U) && (fresh == 0x0FU))
    {
        s_can_was_ready = 1U;
        g_can_guard_ready = 1U;
        return;
    }

    if ((s_can_was_ready != 0U) ||
        ((TickType_t)(now - s_can_power_tick) >=
         pdMS_TO_TICKS(SAFETY_CAN_BOOT_MS)))
    {
        g_can_guard_bad_nodes =
            (enabled == 1U) ? (0x0FU & ~fresh) : 0x0FU;
        g_can_guard_cause = (enabled == 1U) ? 2U : 1U;
        g_safety_fault |= SAFETY_FAULT_CAN_DATA;
    }
}
```

最后在原Safety_Update中，找到原 `g_safety_ready = 0U;`，在它紧前面插入：

```c
/* 在最后决定VM允许之前，先纳入CAN故障判断 */
Safety_CanGuardUpdate(vm_request);
```

原APP/RX/KEY检查、回环结果检查、g_safety_ready计算和最后Power_Enable/Disable全部保留。新增CAN故障同样使用原g_safety_fault保持，恢复数据/切换请求都不清故障。不能把调用插到Power_Enable之后。

**两个ready的含义：** g_safety_ready仍表示平台任务正常且没有锁定故障，上电等待内可为1并允许VM；g_can_guard_ready只有VM请求开启且四台采样新鲜才为1。这样避免“电机没电→没有回复→一直不允许供电”的死循环。两者都不是正式整车READY；上电等待期间只供电/只读，不发运动。看门狗尚未接入。

## 3. USER/main.c发布快照并默认开启必要采集

把原采集开关声明及其注释改为：

```c
/* 默认开启必要采集；VM关闭或平台故障时仍不发送查询 */
volatile uint32_t g_can_poll_enable = 1U;
```

在任务A中，原App_CanPollRefresh调用之后、g_test_tick之前添加：

```c
/* 发布同一轮的采集状态，监督任务独立重算新鲜度 */
Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
                 g_can_poll_protect_mask, g_can_poll_tick);
```

保留原采集两个helper、手动查询if/else-if、三任务心跳、任务创建/优先级和Power_Init/Safety_Init。没有新增Keil文件或IncludePath。此前连续采集指南默认0是历史，本步改为默认1；VM请求仍默认0，不自动上电。

**行为变化：** 以前关闭采集只撤销数据；本步在VM已开启且CAN曾健康后，关闭必要采集会触发CAN数据故障并锁定VM关闭。正常停止请撤销VM请求；不能把停止必要反馈当成保持供电运行的模式。

## 4. 本轮先Rebuild

用户完成上述三文件增量修改、保存/Rebuild，提交完整日志。真实编译/下载/新监督正常运行与故障关断必须分别验收，不用私人编译代替实物。

## 5. 构建通过后正常上电/健康验证

用户下载/F5正常启动约2秒；VM请求0、RTOS/监督故障0、g_safety_ready1、g_can_guard_ready0是正常。硬件开关恢复软件允许档，设置VM请求1/F5约4秒后暂停，预期：

```text
g_vm_test_request = 1
g_can_poll_enable = 1
g_rtos_fault = 0
g_safety_fault = 0
g_safety_ready = 1
g_can_guard_ready = 1
g_can_poll_valid_mask = 0x0F
g_can_guard_bad_nodes = 0
g_can_guard_cause = 0
```

确认实际VM有电压；四节点count继续增加，APP/监督余栈与heap以实测验收，不填预测值。上电宽限3秒为工程候选，若驱动实际启动更久应依据实测调整，不清fault伪装通过。此时手动版本/状态请求保持0，避免连续手动请求挤占采集导致真实新鲜度失效。

## 6. 关闭必要采集：验证CAN故障关VM与锁定

仅在第5节四节点健康、VM有电实测通过后执行。硬件开关保持软件允许，VM请求仍1；暂停将g_can_poll_enable改0，F5约1秒后暂停，预期：

```text
g_safety_fault = 0x20  （十进制32）
g_safety_ready = 0
g_can_guard_ready = 0
g_can_guard_bad_nodes = 0x0F
g_can_guard_cause = 1
g_rtos_fault = 0
VM = 0 V
```

然后仅把g_can_poll_enable恢复1/F5约1秒，故障32/VM0仍保持，不能自动重新供电。不要在此期间复位或手动清故障。该测试验证“必要采集失效”路径，不能扩大为真实断线/超时、过期和实际驱动器堵转均已测试。没有临时测试代码需要清理，正常重启后VM请求0。

## 7. 后续过期/保护路径测试与验收边界

第6节通过后，还需有针对性验证保留旧mask但采样时间不再更新时仍能触发原因2，以及原因3保护路径。用明确标记的临时软件注入模拟采样停滞/保护数据，保留平台任务活跃，不故意让电机机械堵转、不热插拔插件；对应代码与实物测试另给，验证后清理。软件保护注入不等于厂家真实堵转阈值/保护动作已验收。

单次截图/一次正常循环不替代长期总线负载、独立CAN服务/较快控制时序、其他关键任务/调度异常、看门狗或正式就绪/启动/重新允许状态机。已有Key失联关断/锁定不重做，继续阶段1安全收尾后才接运动。

## 8. 需求与证据

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），官方Q不变。静态候选3秒等待/1秒新鲜度和堵转/保护即关VM均为E，不能写成官方规则或已实测响应性能。旧状态帧协议/保护位沿用厂家手册已核内容。证据为连续两次进展与关闭后无效/内存截图、实际源只读检查；私人编译结果另记。用户本轮构建/健康启动/必要采集失效与锁定、过期/保护路径、看门狗仍待验，助手不改真实固件或操纵硬件。
