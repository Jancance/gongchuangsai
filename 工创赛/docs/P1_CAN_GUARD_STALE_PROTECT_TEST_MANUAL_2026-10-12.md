# CAN监督过期/保护路径：一次临时修改，分两次验证

> 清理后正常启动已过2026-10-13：图23e8764d为VM请求0/采集1/RTOS及监督fault0/平台ready1/CAN ready0、节点原因0，用户VM0V；APP余栈832B/监督928B/heap最小14064B。临时测试代码清理/正式构建/正常启动门禁闭环，旧测试观察行已移除。下一项P1_IWDG_SUPERVISOR_MANUAL_GUIDE_2026-10-13.md接入看门狗，不再返回本指南重做注入。

> 最新清理构建通过2026-10-13：用户正式Rebuild链接0错误0警告，Code15212/RO424/RW396/ZI22524、6秒。真实main无四测试状态/局部index，原参数Safety_CanReport只调用一次，Refresh/三心跳/默认VM0/采集1及正式guard保留。旧“临时故障注入”注释尚有一行，仅为文字，可改“发布CAN采集快照供监督检查”，不要求为注释单独重建。唯一下一项下载清理后的新程序、F5约2秒，验VM请求0/RTOS及监督fault0/平台ready1/CAN ready0、实际VM0，旧测试Watch行可删除；随后看门狗，不重做故障注入。

> 最新保护复原锁定通过2026-10-13：图14708bc3为test_mode0/VM请求1/采集1、fault40/两个ready0/cause3/节点01、RTOS故障0/VM0，监督count484→819继续运行。过期与模拟保护触发、保护复原锁定已具证据；原因2复原未单独拍图，只保留记录说明，共同fault20锁定及模式0发布路径已有其他验证，不增加重复触发门禁。唯一下一项第7节清理四临时全局/局部test_index和发布分支，恢复原接口，用户Rebuild后正常启动，随后看门狗。不重做已过测试，真实驱动保护/断线和完整安全仍另验。

> 最新实机测试2026-10-13：用户正式Rebuild链接0错0警告（Code15336/RO424/RW408/ZI22544），main三处只读核对正确。第5节图5892fa0d为mode1/测试旧mask0F/fault20/cause2/bad_nodes02/VM0；第6节图ed9ff3ce为mode2/fault40/cause3/bad_nodes01/VM0，均VM请求1/RTOS故障0。过期及软件模拟保护触发/VM禁止状态已过；最新保护模式恢复0后的锁定尚待图。唯一下一项保持当前模式2这次运行，test_mode改0/F5约1秒，不复位/不清fault，验fault40/cause3/节点01/VM0保持，随后第7节一次清理。过期模式恢复0锁定未单独提供证据，不要求返回重做已过触发测试；真实驱动堵转/时延性能仍未验。

> 准备接续更新2026-10-13，文件名沿用建立时的2026-10-12。私人main.c候选ARMCC5.06 update5 build528下0错误0警告，真实main SHA前后不变，两个私人文件已清理；未链接/下载或触发新故障。唯一下一项用户第1～3节修改main.c三处并Rebuild，测试模式先保持0，再分两次实机验证。

必要采集关闭的fault20/cause1关VM及恢复采集请求后锁定已通过。本步验证另外两条分支：监督端收到旧采样时间时的cause2，以及保护输入的cause3。只由用户临时修改USER/main.c，保留can/safety模块及其参数；测试结束一次性清理。

以下是工程故障注入（E），不发送运动或校准命令，不热插拔模块，不故意机械堵转。保护输入是软件模拟，不能据此宣布厂家的实际堵转阈值/动作已验收。旧照片缺少顶部VM请求/RTOS字段时不补造，后续Watch建议精简后同时展示。

## 1. USER/main.c添加临时测试变量

先将硬件开关置关VM档、退出调试。在现有g_can_poll_reply之后、原s_can_poll_index之前添加以下整块：

```c
/* 临时测试：0正常，1冻结监督采样时间，2模拟第1台保护输入 */
volatile uint32_t g_can_guard_test_mode = 0U;
volatile uint32_t g_can_guard_test_valid_mask = 0U;
volatile TickType_t g_can_guard_test_ticks[4] = {0U};
static uint32_t s_can_guard_test_frozen = 0U;
```

## 2. App_TestTaskA增加一个局部循环变量

在该函数一开始，原 `(void)argument;` 之前添加：

```c
uint32_t test_index;
```

不要放到其他任务或整个文件的全局区。

## 3. 临时替换任务A的CAN快照发布调用

找到App_CanPollRefresh之后的原两行调用及其“发布同一轮”注释：

```c
Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
                 g_can_poll_protect_mask, g_can_poll_tick);
```

用以下整块替换。App_CanPollRefresh、上面的正常查询/轮询、下面的tick/栈堆/APP心跳/延时都保留：

```c
/* 临时故障注入，正式监督与正常轮询仍继续运行 */
if (g_can_guard_test_mode == 1U)
{
    if (s_can_guard_test_frozen == 0U)
    {
        /* 健康采集时只保存一次，后续持续发布旧有效位和旧时间 */
        g_can_guard_test_valid_mask = g_can_poll_valid_mask;
        for (test_index = 0U; test_index < 4U; test_index++)
        {
            g_can_guard_test_ticks[test_index] = g_can_poll_tick[test_index];
        }
        s_can_guard_test_frozen = 1U;
    }
    Safety_CanReport(g_can_poll_enable, g_can_guard_test_valid_mask,
                     g_can_poll_protect_mask, g_can_guard_test_ticks);
}
else
{
    s_can_guard_test_frozen = 0U;
    if (g_can_guard_test_mode == 2U)
    {
        /* 这里01是第1台节点位，不是厂家状态字的堵转位 */
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
                         g_can_poll_protect_mask | 0x01U, g_can_poll_tick);
    }
    else
    {
        /* 模式0保持原正式行为 */
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
                         g_can_poll_protect_mask, g_can_poll_tick);
    }
}
```

模式1冻结的是“发布给监督的数据时间”，正常总线读和平台心跳仍有进展，且不屏蔽现有真实保护mask。保存的有效mask在正常四台健康时应为0F；监督必须自己从旧tick判断过期。故障后正常采集数组会清零，但临时保存的mask/ticks继续保留，便于检查输入确实没有更新。

模式2在新鲜数据的监督输入中加入第1台保护节点位；正常g_can_poll_flags并没有被改成堵转状态。这是监督分支注入，不是实际驱动器保护事件，也不验证厂家内部保护检测阈值。

## 4. 本轮先保存/Rebuild

修改只有main.c一个文件、三处。用户Rebuild并提交完整日志；本轮先保持test_mode默认0。新构建/下载与各物理关断必须分别有证据，不以编译代替实测。

## 5. 测试一：保留有效位，但时间不再新鲜

构建通过后用户下载/F5正常启动，确认测试模式0、VM请求0、两个fault0。硬件恢复软件允许档，VM请求1/F5约4秒后暂停，确认四台result1/valid0F、g_can_guard_ready1、fault0且实际VM有电。不要带着上一cause1故障开始本测试，下载后正常启动会按默认请求0重新初始化。

在Watch保留/添加下面这些主要变量；为方便同屏查看，可暂时移除旧g_can_status_*、g_can_version_address等观察行，源码不删：

```text
g_vm_test_request
g_rtos_fault
g_safety_fault
g_safety_ready
g_can_poll_enable
g_can_guard_ready
g_can_guard_bad_nodes
g_can_guard_cause
g_can_guard_test_mode
g_can_guard_test_valid_mask
g_can_guard_test_ticks
g_safety_count
g_stack_a_words
g_safety_stack_words
g_heap_min_bytes
```

健康供电后暂停，设置test_mode=1、F5约2秒再暂停，预期：

```text
g_vm_test_request = 1
g_can_poll_enable = 1
g_can_guard_test_mode = 1
g_can_guard_test_valid_mask = 0x0F
g_safety_fault = 0x20   （32）
g_can_guard_cause = 2
g_can_guard_bad_nodes != 0
g_safety_ready = 0
g_can_guard_ready = 0
g_rtos_fault = 0
VM = 0 V
```

bad_nodes对应首先过期的节点，可能01/02/04/08或组合，不写死0F。正常轮询顺序使四个采样tick不同，本步不要求精确某个节点或精确响应耗时。

截图和VM测量先发回；确认后把test_mode改0/F5约1秒，fault20/cause2/VM0应继续保持，验证该路径也不自动重新允许。不要通过Watch把正式fault清0。截图中的保存mask0F仅表示“这份旧输入曾声称有效”，不是当前实时数据有效。

## 6. 测试二：软件模拟第1台保护输入

测试一过期和锁定通过后，先硬件关VM，正常复位/F5运行，不手动清故障。复位后确认test_mode0/VM请求0/两个fault0，再恢复硬件允许档、VM请求1/F5约4秒，确认CAN ready1/四台新鲜和实际VM有电。这是为了给独立保护分支一个没有旧锁定故障的启动条件。

保持硬件软件允许档和VM请求1，暂停设置test_mode=2、F5约1秒后暂停，预期：

```text
g_can_guard_test_mode = 2
g_safety_fault = 0x40   （64）
g_can_guard_cause = 3
g_can_guard_bad_nodes = 0x01
g_safety_ready = 0
g_can_guard_ready = 0
g_rtos_fault = 0
VM = 0 V
```

回报截图和电压。确认后把test_mode改0/F5约1秒，fault40/cause3/VM0应继续保持。数据数组在故障关电后可能清0，这不推翻保存的首次原因/节点。

## 7. 两项验证后一次清理

硬件关VM/退出调试，在main.c删除第1节四个临时变量（含临时注释）：

```c
volatile uint32_t g_can_guard_test_mode = 0U;
volatile uint32_t g_can_guard_test_valid_mask = 0U;
volatile TickType_t g_can_guard_test_ticks[4] = {0U};
static uint32_t s_can_guard_test_frozen = 0U;
```

删除App_TestTaskA开头的局部变量：

```c
uint32_t test_index;
```

把第3节从 `if (g_can_guard_test_mode == 1U)` 开始的完整临时if/else块（含内部mode2和mode0分支及临时注释）恢复为：

```c
/* 发布同一轮的采集状态，监督任务独立重算新鲜度 */
Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
                 g_can_poll_protect_mask, g_can_poll_tick);
```

上面的App_CanPollRefresh和下面的tick/栈堆/APP心跳/延时继续保留；全部正式can/safety、RX/KEY心跳及原查询不删。现在先保存/Rebuild发完整日志，再下载运行验证清理后的正常状态。

用户Rebuild/下载后F5正常启动，验test_mode等已删除符号cannot evaluate（Watch行可删）、VM请求0、RTOS/监督fault0、局部ready1、CAN ready0及实际VM0。清理完才进入看门狗接入/调试暂停与故障复位检查，不留测试模式进入后续控制。

## 8. 需求与证据

已检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），官方Q不变。旧cause1关断/锁定、正常采集/内存已有记录；本步属于标记清楚的软件故障注入，只验证监督时间判断/保护分支与实际VM输出，不代替真实驱动堵转、断线/噪声/全负载、独立CAN时序及完整安全。私人main编译结果另记，实际新构建/两个测试/锁定/清理仍待用户证据，助手不改真实源码或操纵硬件。
