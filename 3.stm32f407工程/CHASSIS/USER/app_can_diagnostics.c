#include "app_can_diagnostics.h"
#include "freertos_hooks.h"
#include "startup_axes.h"
#include "chassis_auto_test.h"
#include "safety.h"
#include "usart.h"
#include <stdio.h>

extern volatile uint32_t g_vm_test_request;

/* CAN1回环结果：0未完成，1通过 */
volatile uint32_t g_can_test_result = 0U;
/* 查询地址：1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_version_address = 1U;
/* 实时位置只读，不清零；原始幅值与符号分别保留 */
volatile uint32_t g_can_position_address = 1U;
volatile uint32_t g_can_position_request = 0U;
volatile uint32_t g_can_position_result = 0U;
volatile uint32_t g_can_position_sign = 0U;
volatile uint32_t g_can_position_raw = 0U;
volatile uint32_t g_can_position_esr = 0U;
CanRxMsg g_can_position_reply;
/* 静态停止命令测试：仅由独立CAN任务处理 */
/* 地址1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_stop_address = 1U;

/* 默认0不发送；设为1后申请发送一次，处理时清零 */
volatile uint32_t g_can_stop_request = 0U;

/* 返回1只表示收到02应答，不代表物理停车已验证 */
volatile uint32_t g_can_stop_result = 0U;

/* 保存CAN错误寄存器和电机原始应答 */
volatile uint32_t g_can_stop_esr = 0U;
CanRxMsg g_can_stop_reply;
/* 单次静态失能测试：固定松轴，不提供使能输入 */
volatile uint32_t g_can_disable_address = 1U;
volatile uint32_t g_can_disable_request = 0U;
/* 1仅表示收到02应答，实际失能需读3A的Ens_TF位确认 */
volatile uint32_t g_can_disable_result = 0U;
volatile uint32_t g_can_disable_esr = 0U;
CanRxMsg g_can_disable_reply;
/* 单轮静态锁轴测试：直接发送使能，不发送速度指令 */
volatile uint32_t g_can_enable_address = 1U;
volatile uint32_t g_can_enable_request = 0U;
/* 1仅为使能02应答；7地址错误，9条件拒绝；实际锁轴需实测 */
volatile uint32_t g_can_enable_result = 0U;
volatile uint32_t g_can_enable_esr = 0U;
CanRxMsg g_can_enable_reply;
/* 默认不动作；正向映射0/1/0/1、raw1（菜单Disable时1RPM），共同运行约10秒 */
volatile uint32_t g_can_jog_request = 0U;
/* 默认1只演练时序，不发F6/FE；实转需在触发前明确改0 */
volatile uint32_t g_can_jog_dry_run = 1U;
/* 本次锁定模式，只读观察；不能在运行中把演练切为实转 */
volatile uint32_t g_can_jog_mode = 1U;
static uint32_t s_can_jog_dry_run = 1U;
/* 9条件拒绝，10四轮状态检查失败；启动/停止各自保留底层结果 */
volatile uint32_t g_can_jog_result = 0U;
volatile uint32_t g_can_jog_check_result = 0U;
volatile uint32_t g_can_jog_start_result = 0U;
volatile uint32_t g_can_jog_stop_result = 0U;
volatile uint32_t g_can_jog_esr = 0U;
CanRxMsg g_can_jog_reply;
CanRxMsg g_can_jog_stop_reply;
static uint32_t s_can_jog_preparing = 0U;
static uint32_t s_can_jog_check_index = 0U;
/* 1全失能，2确认全失能，3四轮使能，4确认四轮使能，5逐台发速度 */
volatile uint32_t g_can_jog_prepare_stage = 0U;
volatile uint32_t g_can_jog_prepare_result[4] = {0U};
static uint32_t s_can_jog_poll_between = 0U;
static uint32_t s_can_jog_start_index = 0U;
static uint32_t s_can_jog_stop_index = 0U;
static uint32_t s_can_jog_session = 0U;
/* 正向定义按现场观察：1左后/2右后/3左前/4右前，2与4需反相 */
volatile uint32_t g_can_jog_enable_result[4] = {0U};
volatile uint32_t g_can_jog_start_results[4] = {0U};
volatile uint32_t g_can_jog_stop_results[4] = {0U};
volatile uint32_t g_can_jog_start_mask = 0U;
volatile uint32_t g_can_jog_stop_mask = 0U;


/* 最后一笔自动停止应答，逐节点结果由Safety保存 */
CanRxMsg g_motion_auto_stop_reply;
volatile uint32_t g_motion_auto_stop_esr = 0U;
/* 静态零速度测试：仅由独立CAN任务处理 */
volatile uint32_t g_can_zero_address = 1U;
volatile uint32_t g_can_zero_request = 0U;

/* 返回1只表示收到02应答，实际速度需另行读取确认 */
volatile uint32_t g_can_zero_result = 0U;
volatile uint32_t g_can_zero_esr = 0U;
CanRxMsg g_can_zero_reply;


/* 只读转速请求：地址1左后、2右后、3左前、4右前 */
volatile uint32_t g_can_speed_address = 1U;
volatile uint32_t g_can_speed_request = 0U;
volatile uint32_t g_can_speed_result = 0U;
volatile float g_can_speed_rpm = 0;
volatile uint32_t g_can_speed_esr = 0U;
CanRxMsg g_can_speed_reply;
/* 版本读取仅由Watch手动触发一次 */
volatile uint32_t g_can_version_request = 0U;
volatile uint32_t g_can_version_result = 0U;
volatile uint32_t g_can_version_esr = 0U;
CanRxMsg g_can_version_reply;
/* 状态查询沿用g_can_version_address选择目标 */
volatile uint32_t g_can_status_request = 0U;
volatile uint32_t g_can_status_result = 0U;
volatile uint32_t g_can_status_esr = 0U;
volatile uint32_t g_can_status_flags = 0U;
volatile TickType_t g_can_status_tick = 0U;
CanRxMsg g_can_status_reply;
/* 默认开启必要采集；VM关闭或平台故障时仍不发送查询 */
volatile uint32_t g_can_poll_enable = 1U;
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
/* 台架持续反馈：默认0，用户静态供电健康后设1 */
volatile uint32_t g_can_feedback_enable = 0U;
/* 静态时效诊断：1暂停向Safety发布两类反馈快照，CAN采集仍运行 */
/* 默认0；撤VM、关闭反馈或故障时自动清零，不用作运动控制 */
volatile uint32_t g_motion_diag_hold_report = 0U;
/* 静态超时注入：1跳过自动停止领取，仍执行原采集和状态保护 */
/* 默认0；撤VM、关闭反馈或故障时与发布暂停一起清零 */
volatile uint32_t g_motion_diag_hold_stop_service = 0U;
/* 静态失败注入：1令下一笔自动停止不发帧，模拟返回4并消费 */
/* 默认0；只验证应用失败分支，不冒充真实CAN硬件故障 */
volatile uint32_t g_motion_diag_fail_stop_once = 0U;
/* 已执行注入次数，撤VM不清，复位清零 */
volatile uint32_t g_motion_diag_fail_stop_count = 0U;
/* 成功发布快照的次数，不是电机采样次数 */
volatile uint32_t g_motion_diag_report_count = 0U;
volatile uint32_t g_can_feedback_speed_result[4] = {0U};
volatile float g_can_feedback_speed_rpm[4] = {0};
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

static uint8_t App_DebugSendString(const char *text)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = USART1_SendString(text);
    taskEXIT_CRITICAL();
    return result;
}

/* 每轮只采一台，保持成功计数/时间戳；关闭时清除本会话快照 */
static void App_CanPollStep(void)
{
    uint32_t i;
    uint32_t index;
    uint32_t result;

    if ((g_can_poll_enable != 1U) ||
        (g_vm_test_request != 1U && !g_startup_axes_vm_request) || (g_safety_ready != 1U))
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
void App_CanPollRefresh(void)
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
            (g_vm_test_request == 1U || g_startup_axes_vm_request) && (g_safety_ready == 1U) &&
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
void App_CanFeedbackStep(void)
{
    CanRxMsg reply;
    uint32_t result;
    float rpm = 0;
    uint8_t sign = 0U;
    uint32_t raw = 0U;
    uint32_t index;

    if (g_can_feedback_enable != 1U || g_can_poll_enable != 1U ||
        (g_vm_test_request != 1U && !g_startup_axes_vm_request) || g_safety_ready != 1U)
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
            ChassisAuto_ReportIdleSpeed(index,rpm,g_can_feedback_speed_tick[index]);
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
void App_CanFeedbackRefresh(void)
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


/* CAN1只由本任务初始化和查询；每轮交易结束后至少等待50毫秒 */
/* 仅用于人工静态锁轴，不是正式运动许可；发送前检查条件 */
static uint32_t App_CanEnableTestReady(void)
{
    if(!CAN1_EmmSettingsConfirmed())return 0U;
    return ((g_vm_test_request == 1U) &&
            (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
            (g_can_poll_enable == 1U) &&
            (g_wheel_test_cut_vm == 0U) &&
            (g_chassis_auto_cut_vm == 0U) &&
            (g_motion_diag_fault_latched == 0U) &&
            (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
            (g_motion_auto_stop_cut_vm == 0U) &&
            (g_motion_diag_hold_report == 0U) &&
            (g_motion_diag_hold_stop_service == 0U) &&
            (g_motion_diag_fail_stop_once == 0U) &&
            (g_can_stop_request == 0U) &&
            (g_can_disable_request == 0U)) ? 1U : 0U;
}

/* 该测试占用控制入口；拒绝同时挂起的其他人工交易，避免事后补发 */
uint32_t App_CanJogOtherRequest(void)
{
    return ((g_can_stop_request != 0U) || (g_can_disable_request != 0U) ||
            (g_can_enable_request != 0U) || (g_can_zero_request != 0U) ||
            (g_can_speed_request != 0U) || (g_can_position_request != 0U) ||
            (g_can_version_request != 0U) || (g_can_status_request != 0U)) ? 1U : 0U;
}

static void App_CanJogClearRequests(void)
{
    g_can_stop_request = 0U;
    g_can_disable_request = 0U;
    g_can_enable_request = 0U;
    g_can_zero_request = 0U;
    g_can_speed_request = 0U;
    g_can_position_request = 0U;
    g_can_version_request = 0U;
    g_can_status_request = 0U;
}

uint32_t App_CanJogReady(void)
{
    return ((App_CanEnableTestReady() == 1U) &&
            (g_can_feedback_enable == 0U) &&
            (App_CanJogOtherRequest() == 0U)) ? 1U : 0U;
}

/* 一轮一台，按各轮正向映射发帧，不使用广播；失败停止后续启动 */
static void App_CanJogStartStep(void)
{
    uint32_t index = s_can_jog_start_index;
    uint32_t result;
    CanRxMsg reply = {0};
    if ((index >= 4U) || (Safety_WheelTestStartAllowed(s_can_jog_session) != 1U))
        return;
    if (s_can_jog_dry_run == 1U) result = 1U; /* 演练模拟，不发F6 */
    else result = CAN1_SetEmmSpeedRaw((uint8_t)(index + 1U),
        g_chassis_forward_direction[index], 1U, 0U, &reply);
    g_can_jog_start_results[index] = result;
    g_can_jog_start_result = result;
    g_can_jog_result = result;
    g_can_jog_reply = reply;
    g_can_jog_esr = CAN1->ESR;
    if (result == 1U) g_can_jog_start_mask |= 1UL << index;
    s_can_jog_start_index++;
    if ((result != 1U) || (s_can_jog_start_index == 4U))
        Safety_WheelTestStartReport(s_can_jog_session, result);
}

/* 每轮至多一笔交易；全失能确认、四轮使能确认后才启动四轮 */
static void App_CanJogService(void)
{
    uint32_t session;
    uint32_t index;
    CanRxMsg reply = {0};

    /* 重复触发不会续时，也不留在队列中等待下一轮供电 */
    g_can_jog_request = 0U;
    if (App_CanJogReady() != 1U)
    {
        g_can_jog_result = 9U;
        s_can_jog_preparing = 0U;
        App_CanJogClearRequests();
        Safety_WheelTestAbort();
        return;
    }
    if (s_can_jog_preparing == 1U)
    {
        if (g_wheel_test_state != WHEEL_TEST_PREPARING)
        {
            s_can_jog_preparing = 0U;
            return;
        }
        if (s_can_jog_poll_between == 1U)
        {
            s_can_jog_poll_between = 0U;
            App_CanPollStep();
            return;
        }
        index = s_can_jog_check_index;
        if (g_can_jog_prepare_stage == 1U)
        {
            g_can_jog_prepare_result[index] = CAN1_SetMotorEnable(
                (uint8_t)(index + 1U), 0U, &reply);
            g_can_jog_reply = reply;
            g_can_jog_esr = CAN1->ESR;
            if (g_can_jog_prepare_result[index] != 1U)
            {
                g_can_jog_result = 10U;
                s_can_jog_preparing = 0U;
                Safety_WheelTestPrepareFailed();
                return;
            }
            s_can_jog_poll_between = 1U;
            s_can_jog_check_index++;
            if (s_can_jog_check_index == 4U)
            {
                s_can_jog_check_index = 0U;
                g_can_jog_prepare_stage = 2U;
            }
            return;
        }
        if (g_can_jog_prepare_stage == 3U)
        {
            g_can_jog_check_result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 1U, &reply);
            g_can_jog_enable_result[index] = g_can_jog_check_result;
            g_can_jog_reply = reply;
            g_can_jog_esr = CAN1->ESR;
            if (g_can_jog_check_result != 1U)
            {
                g_can_jog_result = 10U;
                s_can_jog_preparing = 0U;
                Safety_WheelTestPrepareFailed();
                return;
            }
            s_can_jog_poll_between = 1U;
            s_can_jog_check_index++;
            if (s_can_jog_check_index == 4U)
            {
                s_can_jog_check_index = 0U;
                g_can_jog_prepare_stage = 4U;
            }
            return;
        }
        if ((g_can_jog_prepare_stage == 2U) || (g_can_jog_prepare_stage == 4U))
        {
            uint32_t expected = (g_can_jog_prepare_stage == 4U) ? 1U : 0U;
            g_can_jog_check_result = CAN1_ReadStatus((uint8_t)(index + 1U), &reply);
            g_can_jog_reply = reply;
            g_can_jog_esr = CAN1->ESR;
            if ((g_can_jog_check_result != 1U) ||
                ((reply.Data[1] & 0x0DU) != expected))
            {
                g_can_jog_result = 10U;
                s_can_jog_preparing = 0U;
                Safety_WheelTestPrepareFailed();
                return;
            }
            g_can_poll_result[index] = 1U;
            g_can_poll_flags[index] = reply.Data[1];
            g_can_poll_tick[index] = xTaskGetTickCount();
            g_can_poll_count[index]++;
            s_can_jog_check_index++;
            if (s_can_jog_check_index == 4U)
            {
                s_can_jog_check_index = 0U;
                g_can_jog_prepare_stage = (g_can_jog_prepare_stage == 2U) ? 3U : 5U;
            }
            return;
        }
        s_can_jog_preparing = 0U;
        if (Safety_WheelTestArm(&session) != 1U)
        {
            g_can_jog_result = 9U;
            return;
        }
        s_can_jog_session = session;
        s_can_jog_start_index = 0U;
        s_can_jog_stop_index = 0U;
        App_CanJogStartStep();
        return;
    }
    if (g_wheel_test_state == WHEEL_TEST_STARTING)
    {
        App_CanJogStartStep();
        return;
    }
    if (Safety_WheelTestTakeStop(&session) == 1U)
        s_can_jog_stop_index = 0U;
    if ((g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STOPPING))
    {
        index = s_can_jog_stop_index;
        if (index >= 4U) return;
        if (s_can_jog_dry_run == 1U)
            g_can_jog_stop_result = 1U; /* 演练模拟，不发FE */
        else
            g_can_jog_stop_result = CAN1_StopMotor((uint8_t)(index + 1U), &reply);
        g_can_jog_stop_results[index] = g_can_jog_stop_result;
        g_can_jog_stop_reply = reply;
        g_can_jog_esr = CAN1->ESR;
        if (g_can_jog_stop_result == 1U) g_can_jog_stop_mask |= 1UL << index;
        s_can_jog_stop_index++;
        if ((g_can_jog_stop_result != 1U) || (s_can_jog_stop_index == 4U))
            Safety_WheelTestStopReport(s_can_jog_session, g_can_jog_stop_result);
    }
    else if (g_wheel_test_active == 1U)
    {
        /* 运动窗口只保留必要状态轮询，不插入其他人工查询 */
        App_CanPollStep();
    }
}

const uint32_t *AppCanDiagnostics_JogPreparing(void)
{
    return &s_can_jog_preparing;
}

void AppCanDiagnostics_Dispatch(void)
{
        if (g_chassis_auto_active == 1U)
        {
            g_chassis_auto_request = 0U;
            if (g_vm_test_request != 1U)
            {
                App_CanJogClearRequests();
                g_can_jog_request = 0U;
                ChassisAuto_SafetyUpdate(g_vm_test_request);
            }
            else if ((App_CanJogReady() != 1U) || (g_can_jog_request != 0U))
            {
                App_CanJogClearRequests();
                g_can_jog_request = 0U;
                ChassisAuto_Abort();
            }
            else ChassisAuto_CanStep();
        }
        else if (g_chassis_auto_request == 1U)
        {
            g_chassis_auto_request = 0U;
            if ((App_CanJogReady() == 1U) && (g_wheel_test_active == 0U) &&
                (s_can_jog_preparing == 0U) && (g_can_jog_request == 0U) &&
                (g_chassis_auto_dry_run <= 1U))
                (void)ChassisAuto_Begin(g_chassis_auto_dry_run);
            else g_chassis_auto_result = 9U;
        }
        else if ((s_can_jog_preparing == 1U) || (g_wheel_test_active == 1U))
        {
            App_CanJogService();
        }
        else if (g_can_jog_request == 1U)
        {
            CanRxMsg empty_reply = {0};
            g_can_jog_request = 0U;
            g_can_jog_result = 0U;
            g_can_jog_check_result = 0U;
            g_can_jog_start_result = 0U;
            g_can_jog_stop_result = 0U;
            g_can_jog_reply = empty_reply;
            g_can_jog_stop_reply = empty_reply;
            if ((App_CanJogReady() == 1U) && (g_wheel_test_active == 0U) &&
                (g_wheel_test_cut_vm == 0U) && (g_can_jog_dry_run <= 1U))
            {
                s_can_jog_dry_run = g_can_jog_dry_run;
                g_can_jog_mode = s_can_jog_dry_run;
                if (Safety_WheelTestPrepare() == 1U)
                {
                    uint32_t i;
                    for (i = 0U; i < 4U; i++)
                    {
                        g_can_jog_prepare_result[i] = 0U;
                        g_can_jog_enable_result[i] = 0U;
                        g_can_jog_start_results[i] = 0U;
                        g_can_jog_stop_results[i] = 0U;
                    }
                    g_can_jog_start_mask = 0U;
                    g_can_jog_stop_mask = 0U;
                    s_can_jog_start_index = 0U;
                    s_can_jog_stop_index = 0U;
                    s_can_jog_check_index = 0U;
                    s_can_jog_poll_between = 0U;
                    g_can_jog_prepare_stage = 1U;
                    s_can_jog_preparing = 1U;
                }
                else g_can_jog_result = 9U;
            }
            else g_can_jog_result = 9U;
        }
        else if ((g_motion_auto_stop_state == MOTION_STOP_PENDING) &&
            (g_motion_diag_hold_stop_service != 1U))
        {
            uint8_t address;
            uint32_t session;
            uint32_t result;
            if ((g_vm_test_request == 1U) &&
                (Safety_MotionStopTake(&address, &session) == 1U))
            {
                if (g_motion_diag_fail_stop_once == 1U)
                {
                    CanRxMsg empty_reply = {0};
                    g_motion_diag_fail_stop_once = 0U;
                    g_motion_diag_fail_stop_count++;
                    /* 没有发送；清旧应答，返回与发送失败一致的4 */
                    g_motion_auto_stop_reply = empty_reply;
                    result = 4U;
                }
                else
                {
                    result = CAN1_StopMotor(address, &g_motion_auto_stop_reply);
                }
                g_motion_auto_stop_esr = CAN1->ESR;
                Safety_MotionStopReport(address, session, result);
            }
        }
        else if (g_can_stop_request == 1U)
        {
            uint32_t address = g_can_stop_address;
            CanRxMsg reply = {0};

            /* 先清请求，条件不满足也不留待以后自动执行 */
            g_can_stop_request = 0U;
            g_can_stop_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_stop_result = 7U;
            }
            else if ((g_vm_test_request != 1U) ||
                     (g_rtos_fault != 0U) ||
                     (g_safety_fault != 0U) ||
                     (g_safety_ready != 1U) ||
                     (g_can_guard_ready != 1U))
            {
                /* 9是本测试入口拒绝码，表示未满足测试条件 */
                g_can_stop_result = 9U;
            }
            else
            {
                g_can_stop_result =
                    CAN1_StopMotor((uint8_t)address, &reply);
            }

            g_can_stop_reply = reply;
            g_can_stop_esr = CAN1->ESR;
        }
        /* 单次失能：停止优先，固定enabled=0，不提供重新使能 */
        else if (g_can_disable_request == 1U)
        {
            uint32_t address = g_can_disable_address;
            CanRxMsg reply = {0};

            /* 拒绝也消费请求，不等条件恢复后自动发送 */
            g_can_disable_request = 0U;
            g_can_disable_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_disable_result = 7U;
            }
            else if ((g_vm_test_request != 1U) ||
                     (g_rtos_fault != 0U) ||
                     (g_safety_fault != 0U) ||
                     (g_safety_ready != 1U) ||
                     (g_can_guard_ready != 1U) ||
                     (g_motion_diag_fault_latched != 0U) ||
                     (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
                     (g_motion_auto_stop_cut_vm != 0U) ||
                     (g_motion_diag_hold_report != 0U) ||
                     (g_motion_diag_hold_stop_service != 0U) ||
                     (g_motion_diag_fail_stop_once != 0U))
            {
                /* 静态测试条件不满足，返回9且不发送 */
                g_can_disable_result = 9U;
            }
            else
            {
                g_can_disable_result =
                    CAN1_SetMotorEnable((uint8_t)address, 0U, &reply);
            }

            g_can_disable_reply = reply;
            g_can_disable_esr = CAN1->ESR;
        }
        /* 停止/失能优先；只发F3使能，不在失能状态发送F6零速 */
        else if (g_can_enable_request == 1U)
        {
            uint32_t address = g_can_enable_address;
            CanRxMsg reply = {0};

            /* 地址本轮冻结，请求先消费，拒绝或失败均不自动重试 */
            g_can_enable_request = 0U;
            g_can_enable_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_enable_result = 7U;
            }
            else if (App_CanEnableTestReady() != 1U)
            {
                g_can_enable_result = 9U;
            }
            else
            {
                g_can_enable_result = CAN1_SetMotorEnable(
                    (uint8_t)address, 1U, &reply);
            }

            g_can_enable_reply = reply;
            g_can_enable_esr = CAN1->ESR;
        }
        /* 静态零速度测试：停止请求优先，不提供非零速度输入 */
        else if (g_can_zero_request == 1U)
        {
            uint32_t address = g_can_zero_address;
            CanRxMsg reply = {0};

            /* 先消费请求，拒绝后不等待条件恢复再自动发送 */
            g_can_zero_request = 0U;
            g_can_zero_result = 0U;

            if ((address < 1U) || (address > 4U))
            {
                g_can_zero_result = 7U;
            }
            else if ((g_vm_test_request != 1U) ||
                     (g_rtos_fault != 0U) ||
                     (g_safety_fault != 0U) ||
                     (g_safety_ready != 1U) ||
                     (g_can_guard_ready != 1U))
            {
                /* 9是测试入口拒绝码，未满足条件时不发帧 */
                g_can_zero_result = 9U;
            }
            else
            {
                /* 方向固定0、速度固定0、加速度参数固定10 */
                /* 10仅为本次静态测试参数，不代表整车运动限值 */
                g_can_zero_result = CAN1_SetEmmSpeedRaw(
                    (uint8_t)address, 0U, 0U, 10U, &reply);
            }

            g_can_zero_reply = reply;
            g_can_zero_esr = CAN1->ESR;
        }
        else if ((g_can_position_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_position_address;
            uint8_t sign = 0U;
            uint32_t raw = 0U;
            char log[64];
            g_can_position_request = 0U;
            g_can_position_result = 0U;
            g_can_position_sign = 0U;
            g_can_position_raw = 0U;
            if (address >= 1U && address <= 4U)
            {
                g_can_position_result = CAN1_ReadPosition(
                    (uint8_t)address, &g_can_position_reply, &sign, &raw);
            }
            else
            {
                g_can_position_result = 7U;
            }
            g_can_position_esr = CAN1->ESR;
            if (g_can_position_result == 1U)
            {
                g_can_position_sign = sign;
                g_can_position_raw = raw;
            }
            /* 结果非1时SIGN/RAW字段只是无效占位 */
            sprintf(log, "POS:A%lu R%lu SIGN%lu RAW%lu\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_position_result,
                    (unsigned long)g_can_position_sign,
                    (unsigned long)g_can_position_raw);
            (void)App_DebugSendString(log);
        }
        /* 已有四台基本状态就绪后，每轮只发一个只读转速请求 */
        else if ((g_can_speed_request == 1U) && (g_vm_test_request == 1U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U))
        {
            uint32_t address = g_can_speed_address;
            float rpm = 0;
            char log[48];
            g_can_speed_request = 0U;
            g_can_speed_result = 0U;
            g_can_speed_rpm = 0;
            if (address >= 1U && address <= 4U)
            {
                g_can_speed_result = CAN1_ReadSpeed((uint8_t)address,
                                                  &g_can_speed_reply, &rpm);
            }
            else
            {
                g_can_speed_result = 7U;
            }
            g_can_speed_esr = CAN1->ESR;
            if (g_can_speed_result == 1U) g_can_speed_rpm = rpm;
            /* 结果非1时RPM字段不作为有效反馈 */
            sprintf(log, "SPEED:A%lu R%lu RPM%ld\r\n",
                    (unsigned long)address,
                    (unsigned long)g_can_speed_result,
                    (long)ZdtEmm_RpmDeci(g_can_speed_rpm));
            (void)App_DebugSendString(log);
        }
        /* VM已请求开启后，读取指定电机版本 */
        else if ((g_can_version_request == 1U) &&
            (g_vm_test_request == 1U))
        {
            uint32_t address = g_can_version_address;

            g_can_version_request = 0U;
            g_can_version_result = 0U;

            if ((address >= 1U) && (address <= 4U))
            {
                g_can_version_result =
                    CAN1_ReadVersion((uint8_t)address,
                                     &g_can_version_reply);
            }
            else
            {
                /* 地址无效，本次不发送 */
                g_can_version_result = 7U;
            }

            g_can_version_esr = CAN1->ESR;
        }
        /* VM请求开启后，手动读取一次电机状态 */
        else if ((g_can_status_request == 1U) &&
            (g_vm_test_request == 1U))
        {
            uint32_t address = g_can_version_address;

            g_can_status_request = 0U;
            g_can_status_result = 0U;
            g_can_status_flags = 0U;
            g_can_status_tick = 0U;

            if ((address >= 1U) && (address <= 4U))
            {
                g_can_status_result =
                    CAN1_ReadStatus((uint8_t)address,
                                    &g_can_status_reply);
            }
            else
            {
                g_can_status_result = 7U;
            }

            g_can_status_esr = CAN1->ESR;

            /* 只在查询成功时保存状态字和采样时间 */
            if (g_can_status_result == 1U)
            {
                g_can_status_flags = g_can_status_reply.Data[1];
                g_can_status_tick = xTaskGetTickCount();
            }
        }
            else
            {
                /* 无手动请求时，按地址1到4循环采状态 */
                 App_CanFeedbackStep();
            }
}
