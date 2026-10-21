# 真实HWT航向输入检查：手动接入，只显示计算结果

2026-10-20。上一轮用户Rebuild 36956/564/1308/25324、0错0警告/4秒，板上虚拟自检result1/case0、ccw={5,3,1,0,0}、cw={-5,-3,-1,0,0}、跨界2度、两fault0、VM请求0，接受纯计算用例的板上门禁。截图没有新增物理VM电压或APP余栈/堆测量，不补造。

下一步使用真实HWT航向和角速度输入，只显示目标、误差、建议转向轮速和状态。保持现有静态模式，代码不操作CAN/VM，不触发校准或自动转动。程序显示的建议轮速不能等同于真实轮速或运动许可。助手只提供说明/隔离核对，实际代码由用户修改。

## 1. 保持当前静态设置

app_config.h继续使用上一轮已经设置的值，不用再修改：

```c
#define APP_HWT_STATIC_TEST 1U
#define APP_HWT_TURN_TEST 0U
```

本轮只改main.c，上一轮heading_controller头文件及虚拟自检函数保留，不再改控制器源码。

## 2. main.c加入真实输入预览函数

在 `static void App_TestTaskA(void *argument)` 前、上一轮App_HeadingDryRun函数结束后，添加以下整段：

```c
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 1开始一次预览，2取消；默认0，拒绝请求不会留待以后自动执行。 */
volatile uint32_t g_heading_live_request = 0U;
/* 相对本次开始时的航向，第一轮保持0度；范围仅用于静态预览。 */
volatile float g_heading_live_relative_deg = 0.0f;
volatile float g_heading_live_target_deg = 0.0f;
volatile float g_heading_live_error_deg = 0.0f;
volatile int32_t g_heading_live_command_rpm = 0;
volatile uint32_t g_heading_live_state = HEADING_CTRL_IDLE;
volatile uint32_t g_heading_live_cause = 0U;
volatile uint32_t g_heading_live_stop_required = 0U;
/* 拒绝原因：1本次上电已用，2静态条件不满足，3HWT未就绪，4角度无效。 */
volatile uint32_t g_heading_live_reject = 0U;
static HeadingController s_heading_live_controller = {0};
static uint32_t s_heading_live_used = 0U;

static uint32_t App_HeadingLiveHealthy(uint32_t now)
{
    /* 沿现有解析器500ms新鲜度，仅用于本次不驱动电机的输入检查。 */
    return ((g_hwt_version_result == 1U) && (g_hwt_valid_mask == 3U) &&
        (g_hwt_angle_age_ms <= 500U) && (g_hwt_rate_age_ms <= 500U) &&
        ((uint32_t)(now - g_hwt_update_tick) <= 100U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}

/* 只由TaskA在HWT101_Update之后调用，不读写电机或VM输出。 */
static void App_HeadingLiveStep(void)
{
    HeadingControllerOutput output = {0};
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t request = g_heading_live_request;
    uint32_t healthy = App_HeadingLiveHealthy(now);
    uint32_t static_ok = ((g_vm_test_request == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U)) ? 1U : 0U;
    float relative;

    g_heading_live_request = 0U;
    if ((request == 2U) &&
        ((s_heading_live_controller.state == HEADING_CTRL_TURNING) ||
         (s_heading_live_controller.state == HEADING_CTRL_SETTLING)))
        HeadingController_Cancel(&s_heading_live_controller, &output);
    if (request == 1U)
    {
        g_heading_live_reject = 0U;
        relative = g_heading_live_relative_deg;
        if (s_heading_live_used != 0U) g_heading_live_reject = 1U;
        else if (static_ok == 0U) g_heading_live_reject = 2U;
        else if (healthy == 0U) g_heading_live_reject = 3U;
        else if (!(relative >= -90.0f && relative <= 90.0f))
            g_heading_live_reject = 4U; /* 此写法也拒绝NaN/无穷值。 */
        else if (HeadingController_Start(&s_heading_live_controller,
            &g_heading_controller_default_config,
            g_hwt_yaw_deg + relative, g_hwt_yaw_deg, now) != 1U)
            g_heading_live_reject = 4U;
        else s_heading_live_used = 1U;
    }
    /* 失效时传0，让控制器锁定故障并把建议轮速撤为0。 */
    (void)HeadingController_Step(&s_heading_live_controller, now,
        g_hwt_yaw_deg, g_hwt_wz_dps,
        ((healthy != 0U) && (static_ok != 0U)) ? 1U : 0U, &output);
    g_heading_live_state = output.state;
    g_heading_live_cause = output.cause;
    g_heading_live_target_deg = output.target_deg;
    g_heading_live_error_deg = output.error_deg;
    g_heading_live_command_rpm = output.command_rpm;
    g_heading_live_stop_required = output.stop_required;
}
#endif
```

目标在接受请求时锁定为“当时航向＋relative”，过程中修改relative不会改变正在计算的目标。每次上电只接受一次有效开始，完成/取消/超时后需复位再测；拒绝的请求当轮消费，不自动排队。state沿控制器定义：0空闲、1计算转向、2稳定检查、3完成、4故障、5取消。cause2表示本次数据/静态条件失效，cause3为20秒超时，cause4为取消。当前500ms窗口仅用于输入检查，不批准为行车反馈时效。

## 3. TaskA循环接入

在现有 `HWT101_Update();` 后、`SAOMA_Update();` 前添加：

```c
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
#endif
```

必须放在while内跟随真实新数据更新。上一轮App_HeadingDryRun仍只在while前调用一次，不能替换或重复调用它。不要把g_heading_live_command_rpm接到CAN，也不要写g_vm_test_request=1。

## 4. 当前先Rebuild

保存main.c并Rebuild，把完整日志发来。助手不会修改实际main、app_config、控制器或工程。本轮新增调用仍没有电机动作入口。

## 5. 构建确认后的第一轮：当前朝向作为目标

保持外部VM关闭，下载新静态程序。持续Run约3秒，确认HWT版本1、valid_mask3、两fault0；不触发HWT校准请求。

第一轮relative保持0。将g_heading_live_request置1一次，继续Run约2秒后Stop。预期state3/cause0/reject0、command_rpm0、stop_required1，目标接近接受请求时的真实g_hwt_yaw_deg，误差在候选±2度内。VM请求仍0，APP余栈和堆需观察。结果表示真实输入下静止到位逻辑通过，不表示小车主动转过任何角度。

Watch可移出上一轮dry的5项（只移除观察项，不删变量），保留以下变量并观察APP余栈/堆：

```text
g_heading_live_request
g_heading_live_relative_deg
g_heading_live_target_deg
g_heading_live_error_deg
g_heading_live_command_rpm
g_heading_live_state
g_heading_live_cause
g_heading_live_reject
g_heading_live_stop_required
g_hwt_yaw_deg
g_vm_test_request
g_rtos_fault
g_safety_fault
g_stack_a_words
g_heap_min_bytes
```

若未通过，不手改state/result/cause；保留截图，依据reject和cause定位。运行中暂停会中断传感器处理，本轮只在约2秒后暂停取结果，不通过单步模拟连续新数据。

## 6. 后续输入方向检查（第一轮通过后再做）

复位新会话，HWT正常后把relative置45、request置1，车体保持静止。开始约2秒时应看到误差约+45、command_rpm5、state1；此数值只是计算输出，小车不转动。设置request2并继续Run后应取消为state5/cause4/command0/stop_required1。反向可在下一次复位测试relative=-45，计算输出应为-5。若20秒仍未到位，会正常超时cause3；本预览没有电机去自动完成目标。当前轮无需做手动转车或这些附加动作，先完成构建和第5节首图。

## 依据与未闭环

R：RUN-01/PWR-01；E：LOC-03/05、RUN-06、MOT-04；官方Q不变。上一轮板上虚拟自检及当前源修正已过。当前真实输入预览代码是待用户手改/构建/板上验收的下一步；CAN/HWT运行期停车、目标角执行器集成、物理转角误差、停止与实时X/Y均不能据此签通过。
