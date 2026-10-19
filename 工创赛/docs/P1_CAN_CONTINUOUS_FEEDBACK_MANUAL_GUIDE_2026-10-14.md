> 2026-10-15：50ms静态持续采集两图已过：四台count12/12/12/13→20/20/20/21，各增8；两反馈mask0F、error0、两fault0/两ready1、ESR0。唯一下一项用户按P1_CAN_TASK_SEPARATION_MANUAL_GUIDE_2026-10-15.md第1～5节手动main-only分离CAN任务后Rebuild；CAN延时50ms，扫码屏幕TaskA改10ms，原CAN回环/查询单一任务拥有，main调试发送加短临界区以免两任务共缓冲区竞争。隔离副本ARMCC -c无错误警告，未链接/烧录，实际源哈希不变；分离后的正式构建、静态/扫码交互与栈堆仍待实测，不直接进入运动。

# 四轮状态/转速/位置持续采集：静态阶段

2026-10-14建立，2026-10-15按间隔对照更新。四台35/36单次读取已过，位置sign0/raw为1、1、3、3。用户只增量改main.c，CAN接口/旧手动请求和3A保护不删。本步为静态采集原型（E），仍同一TaskA独占CAN；实际运动前还须独立CAN服务/控制时序、更快反馈新鲜度及完整READY，不直接用于行车。

## 1. 顺序与时效

每轮只一条请求，按状态1→速度1→位置1→状态2…到4再回1；手动位置/速度/版本/状态优先分支不变。TaskA轮间隔现用50ms（10ms下曾出现ID1查询超时，100ms与50ms状态对照均有健康现场），避免12条轮询把状态3A的1秒新鲜度拖过期。名义轮间隔合计600ms另加实际收发/任务时间，不保证固定采样频率。原3A保护/新鲜度仍由监督检查；速度/位置独立结果/时间戳/有效掩码，失败立即无效，旧raw仅诊断，不能只看数字。

反馈默认关闭，VM默认0。开启反馈后每台有数组，下标0..3对应地址1..4；转速有正负，位置仍sign/raw分开。两类数据当前新鲜度候选1秒，未宣称适合闭环运动；后续控制必须检查反馈标志并建立失效停车策略，不能只有3A状态通过就允许运动。

## 2. main.c新增全局/静态变量

在CAN_POLL_VALID_MS定义之后加：

```c
/* 台架持续反馈：默认0，用户静态供电健康后设1 */
volatile uint32_t g_can_feedback_enable = 0U;
volatile uint32_t g_can_feedback_speed_result[4] = {0U};
volatile int32_t g_can_feedback_speed_rpm[4] = {0};
volatile uint32_t g_can_feedback_position_result[4] = {0U};
volatile uint32_t g_can_feedback_position_sign[4] = {0U};
volatile uint32_t g_can_feedback_position_raw[4] = {0U};
volatile TickType_t g_can_feedback_speed_tick[4] = {0U};
volatile TickType_t g_can_feedback_position_tick[4] = {0U};
volatile uint32_t g_can_feedback_speed_age_ticks[4] = {
    0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU};
volatile uint32_t g_can_feedback_position_age_ticks[4] = {
    0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU};
volatile uint32_t g_can_feedback_count[4] = {0U};
volatile uint32_t g_can_feedback_speed_valid_mask = 0U;
volatile uint32_t g_can_feedback_position_valid_mask = 0U;
volatile uint32_t g_can_feedback_error_count = 0U;
static uint32_t s_feedback_phase = 0U;
static uint32_t s_feedback_index = 0U;
/* 当前静态候选阈值，运动控制前重新验证更快的时效 */
#define CAN_FEEDBACK_VALID_MS 1000U
```

## 3. 新增三个函数

在App_StartGateUpdate之前加下面三个函数，原App_CanPollStep和App_CanPollRefresh全部保留。

```c
/* 关闭反馈时撤销有效性，原状态保护轮询继续 */
static void App_CanFeedbackClear(void)
{
    uint32_t i;
    s_feedback_phase = 0U;
    s_feedback_index = 0U;
    g_can_feedback_speed_valid_mask = 0U;
    g_can_feedback_position_valid_mask = 0U;
    g_can_feedback_error_count = 0U;
    for (i = 0U; i < 4U; i++)
    {
        g_can_feedback_speed_result[i] = 0U;
        g_can_feedback_position_result[i] = 0U;
        g_can_feedback_speed_tick[i] = 0U;
        g_can_feedback_position_tick[i] = 0U;
        g_can_feedback_speed_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_position_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_count[i] = 0U;
    }
}

/* 只由任务A调用，一轮一个请求：状态、转速、位置再换节点 */
static void App_CanFeedbackStep(void)
{
    CanRxMsg reply;
    uint32_t result;
    int32_t rpm = 0;
    uint8_t sign = 0U;
    uint32_t raw = 0U;
    uint32_t index;

    if (g_can_feedback_enable != 1U || g_can_poll_enable != 1U ||
        g_vm_test_request != 1U || g_safety_ready != 1U)
    {
        App_CanFeedbackClear();
        App_CanPollStep();
        return;
    }
    if (s_feedback_phase == 0U)
    {
        /* 原状态函数首次启动会从节点0开始 */
        s_feedback_index = (s_can_poll_active == 0U) ? 0U : s_can_poll_index;
        App_CanPollStep();
        if (g_can_poll_result[s_feedback_index] != 1U)
        {
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 1U;
        return;
    }
    index = s_feedback_index;
    if (s_feedback_phase == 1U)
    {
        result = CAN1_ReadSpeed((uint8_t)(index + 1U), &reply, &rpm);
        g_can_feedback_speed_result[index] = result;
        if (result == 1U)
        {
            g_can_feedback_speed_rpm[index] = rpm;
            g_can_feedback_speed_tick[index] = xTaskGetTickCount();
        }
        else
        {
            g_can_feedback_speed_tick[index] = 0U;
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 2U;
    }
    else
    {
        result = CAN1_ReadPosition((uint8_t)(index + 1U), &reply, &sign, &raw);
        g_can_feedback_position_result[index] = result;
        if (result == 1U)
        {
            g_can_feedback_position_sign[index] = sign;
            g_can_feedback_position_raw[index] = raw;
            g_can_feedback_position_tick[index] = xTaskGetTickCount();
            if (g_can_poll_result[index] == 1U &&
                g_can_feedback_speed_result[index] == 1U)
            {
                g_can_feedback_count[index]++;
            }
        }
        else
        {
            g_can_feedback_position_tick[index] = 0U;
            g_can_feedback_error_count++;
        }
        s_feedback_phase = 0U;
    }
}

/* 手动查询占用总线时，依然每轮计算反馈年龄并撤销过期数据 */
static void App_CanFeedbackRefresh(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t age;
    uint32_t i;
    uint32_t speed_mask = 0U;
    uint32_t position_mask = 0U;
    for (i = 0U; i < 4U; i++)
    {
        g_can_feedback_speed_age_ticks[i] = 0xFFFFFFFFU;
        g_can_feedback_position_age_ticks[i] = 0xFFFFFFFFU;
        if (g_can_feedback_enable == 1U && g_can_poll_enable == 1U &&
            g_vm_test_request == 1U && g_safety_ready == 1U)
        {
            if (g_can_feedback_speed_result[i] == 1U)
            {
                age = (TickType_t)(now - g_can_feedback_speed_tick[i]);
                g_can_feedback_speed_age_ticks[i] = (uint32_t)age;
                if (age <= pdMS_TO_TICKS(CAN_FEEDBACK_VALID_MS))
                    speed_mask |= 1UL << i;
            }
            if (g_can_feedback_position_result[i] == 1U)
            {
                age = (TickType_t)(now - g_can_feedback_position_tick[i]);
                g_can_feedback_position_age_ticks[i] = (uint32_t)age;
                if (age <= pdMS_TO_TICKS(CAN_FEEDBACK_VALID_MS))
                    position_mask |= 1UL << i;
            }
        }
    }
    g_can_feedback_speed_valid_mask = speed_mask;
    g_can_feedback_position_valid_mask = position_mask;
}
```

## 4. TaskA三个修改点

1. 只改原无手动请求else分支中的App_CanPollStep调用为：

```c
                App_CanFeedbackStep();
```

2. 原App_CanPollRefresh调用之后添加：

```c
        App_CanFeedbackRefresh();
```

原Safety_CanReport/扫码/启动请求和心跳/栈堆统计保持；不要把原App_CanPollRefresh替换掉。

3. App_TestTaskA末尾当前保留50U（此前10U建议撤回；当前源已核对50U）：

```c
        vTaskDelay(pdMS_TO_TICKS(50U));
```

其他任务延时和Key1ms不改。TaskA旧注释可改为“每轮一个CAN请求，轮间隔50毫秒”；蜂鸣Update仍按tick判断，不因更勤更新而改鸣响时长。

## 5. 保存/Rebuild

只改main.c，无新文件/头路径。私人main.c ARMCC5.06 update5 build528 -c 0错0警告，真实main SHA不变，私有两文件核路径后清理，无链接/烧录。用户正式Rebuild并给日志，过后再实测。

## 6. 构建后的首次持续采集

用户下载后维持静止，台架宏1；VM允许档/暂停设g_vm_test_request1，F5到平台及CAN ready1/fault0，再暂停设g_can_feedback_enable1。手动位置/速度/版本/状态请求都保持0。F5运行约3秒暂停观察：

- g_can_poll_valid_mask=0F、g_can_feedback_speed_valid_mask=0F、g_can_feedback_position_valid_mask=0F。
- 两类result[0..3]都1、speed_rpm[0..3]静止约0，position sign/raw按实际位置，不强求永远1/1/3/3。
- g_can_feedback_count[0..3]都大于0；再F5约2秒后暂停，四项都增加。
- 两类age_ticks均未超1000tick（当前1tick=1ms），failed/未见数据则FFFFFFFF；error_count正常为0。
- 两fault0/两ready1，APP余栈/监督栈和heap记录新的实际值。

给两次计数/有效掩码与故障/内存截图。自动采集不逐条打印串口，避免泛洪；手动请求日志仍可用。若失败或旧数据，只看result/age/mask判断，不以静止0值判数据新鲜。关闭enable0时两反馈mask清0、age失效，原3A轮询继续；计数/错误会话重置，旧数据值可能保留，仅作历史诊断。

## 7. 限制与后续

原状态保护继续；单条查询最长由原CAN超时限定，扫码显示仍有有界等待。TaskA其他工作可影响刷新，当前静态原型不能签100Hz/硬实时或闭环车体性能。速度/位置无效尚未接成新的运动停止条件，实际运动前须独立CAN服务、反馈失效策略/测得时序和几何方向限值。不要为保持ready而放宽旧3A保护阈值。

检查PWR-01（R）、LOC-03/RUN-06/MOT-04（E），QR-04（Q）不变。静态数据连续有效不等于真实位姿/里程、两批次业务/完整READY；完整装机载物低速按原阶段执行，抓放统计仍延后。证据CAN_feedback_private_compile.json及四轮单次位置/转速原图；新增采集正式构建/计数增长和时效仍待用户。
