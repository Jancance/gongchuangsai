#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "led.h"
#include "Buzzer.h"
#include "app_config.h"
#include "usart.h"
#include "Key.h"
#include "power.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "chassis_auto_test.h"
#include "hwt101.h"
#include "safety.h"
#include "watchdog.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>
#include "heading_controller.h"
#include "heading_session.h"

/* 临时静态供电测试：0关闭，1请求开启 */
volatile uint32_t g_vm_test_request = 0U;
/* 测试状态，可在Watch中观察 */
volatile uint32_t g_test_count_a = 0U;
volatile uint32_t g_test_count_b = 0U;
volatile TickType_t g_test_tick = 0U;
volatile BaseType_t g_test_create_a = pdFAIL;
volatile BaseType_t g_test_create_b = pdFAIL;
volatile uint32_t g_test_scheduler_returned = 0U;
/* 1表示发送请求被接受，不代表电脑已经收到 */
volatile uint32_t g_serial_send_ok = 0U;
/* 已处理的接收字节数与最后一个字节 */
volatile uint32_t g_serial_rx_count = 0U;
volatile uint32_t g_serial_rx_last = 0U;
/* 接收任务创建结果、栈余量及本轮序列错误数 */
volatile BaseType_t g_rx_task_create = pdFAIL;
volatile UBaseType_t g_rx_stack_words = 0U;
/* 启动按键事件数、任务创建结果及栈余量 */
volatile uint32_t g_start_key_count = 0U;
volatile BaseType_t g_key_task_create = pdFAIL;
volatile UBaseType_t g_key_stack_words = 0U;
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
volatile int32_t g_can_speed_rpm = 0;
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
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_scan_display_result = 0U;
volatile uint32_t g_scan_display_count = 0U;
volatile uint32_t g_scan_log_send_ok = 0U;
/* 当前已接模块的启动请求门禁，尚未驱动底盘 */
volatile uint32_t g_start_gate_ready = 0U;
volatile uint32_t g_start_request = 0U;
volatile uint32_t g_start_accept_count = 0U;
volatile uint32_t g_start_deny_count = 0U;
char g_start_task_code[16] = {0};
#if !APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST
static uint32_t s_start_gate_revision = 0U;
#endif
static uint32_t s_start_pending = 0U;
static uint32_t s_start_pending_revision = 0U;

static uint32_t s_can_poll_index = 0U;
static uint32_t s_can_poll_active = 0U;
/* 本步100ms查一台，名义四台约400ms，候选新鲜度1秒 */
#define CAN_POLL_VALID_MS 1000U
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
/* CAN独立采集任务的创建结果、循环次数和历史最小余栈 */
volatile BaseType_t g_can_task_create = pdFAIL;
volatile uint32_t g_can_task_count = 0U;
volatile UBaseType_t g_can_task_stack_words = 0U;

/* 监督任务创建结果及历史最小剩余栈 */
volatile BaseType_t g_safety_task_create = pdFAIL;
volatile UBaseType_t g_safety_stack_words = 0U;

/* 栈余量单位为字，每字4字节 */
volatile UBaseType_t g_stack_a_words = 0U;
volatile UBaseType_t g_stack_b_words = 0U;
/* 堆余量单位为字节 */
volatile size_t g_heap_free_bytes = 0U;
volatile size_t g_heap_min_bytes = 0U;

/* 两个任务共用调试发送缓冲区，短临界区内只复制，不等待串口 */
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


/* 只有任务A调用；先检查当前平台，再确认按键对应的扫码版本 */
static void App_StartGateUpdate(void)
{
    uint32_t platform_ok;
    uint32_t code_ok;
    uint32_t accepted = 0U;
    uint32_t i;
    char log[24];

    taskENTER_CRITICAL();
    platform_ok = (g_rtos_fault == 0U && g_safety_fault == 0U &&
                   g_safety_ready == 1U) ? 1U : 0U;
    if (platform_ok == 0U)
    {
        g_start_request = 0U;
    }
    code_ok = (g_saoma_last_result == 1U &&
               g_saoma_accept_count != 0U &&
               g_scan_display_result == SCREEN_OK) ? 1U : 0U;
    g_start_gate_ready = (platform_ok != 0U && code_ok != 0U &&
                          g_start_request == 0U) ? 1U : 0U;
#if !APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST
    s_start_gate_revision = g_saoma_accept_count;
#endif
    if (s_start_pending != 0U)
    {
        s_start_pending = 0U;
        if (g_start_gate_ready != 0U &&
            s_start_pending_revision == g_saoma_accept_count)
        {
            for (i = 0U; i < sizeof(g_start_task_code); i++)
            {
                g_start_task_code[i] = SAOMA_Data[i];
            }
            g_start_request = 1U;
            g_start_gate_ready = 0U;
            g_start_accept_count++;
            accepted = 1U;
        }
        else
        {
            g_start_deny_count++;
        }
    }
    taskEXIT_CRITICAL();
    if (accepted != 0U)
    {
        sprintf(log, "START:%s\r\n", g_start_task_code);
        (void)App_DebugSendString(log);
    }
}
/* CAN1只由本任务初始化和查询；每轮交易结束后至少等待50毫秒 */
/* 仅用于人工静态锁轴，不是正式运动许可；发送前检查条件 */
static uint32_t App_CanEnableTestReady(void)
{
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
static uint32_t App_CanJogOtherRequest(void)
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

static uint32_t App_CanJogReady(void)
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
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 以下仅显示任务间数据，不作为运动允许信号。 */
volatile uint32_t g_heading_bus_valid = 0U;
volatile int32_t g_heading_bus_rpm = 0;
volatile uint32_t g_heading_bus_stop = 1U;
volatile uint32_t g_heading_bus_sequence = 0U;
volatile uint32_t g_heading_publish_result = 0U;
#endif
#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
#if configTICK_RATE_HZ != 1000U
#error Heading_source_requires_1000Hz_tick
#endif
#define HEADING_SOURCE_FRESH_MS 100U
typedef struct
{
    uint32_t sequence, sample_ms, angle_ms, rate_ms, healthy;
    float yaw_deg, rate_dps;
} AppHeadingSource;

static AppHeadingSource s_heading_source = {0};
static uint32_t s_heading_source_ready = 0U;
volatile uint32_t g_heading_source_valid = 0U;
volatile uint32_t g_heading_source_sequence = 0U;
volatile uint32_t g_heading_source_age_ms = 0U;
volatile uint32_t g_heading_source_angle_age_ms = 0U;
volatile uint32_t g_heading_source_rate_age_ms = 0U;
volatile float g_heading_source_yaw_deg = 0.0f;
volatile float g_heading_source_rate_dps = 0.0f;

/* 唯一生产者TaskA：紧接HWT101_Update调用，不代表运动许可。 */
static void App_HeadingSourcePublish(void)
{
    AppHeadingSource value = {0};
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    value.sample_ms = now;
    /* 用真实接收帧龄期还原时间，不能用本次发布时刻冒充新帧。 */
    value.angle_ms = g_hwt_update_tick - g_hwt_angle_age_ms;
    value.rate_ms = g_hwt_update_tick - g_hwt_rate_age_ms;
    value.yaw_deg = g_hwt_yaw_deg;
    value.rate_dps = g_hwt_wz_dps;
    value.healthy = ((g_hwt_version_result == 1U) &&
        (g_hwt_valid_mask == 3U) &&
        (g_hwt_angle_age_ms < HEADING_SOURCE_FRESH_MS) &&
        (g_hwt_rate_age_ms < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - g_hwt_update_tick) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.angle_ms) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.rate_ms) < HEADING_SOURCE_FRESH_MS) &&
        (value.yaw_deg >= -180.0f && value.yaw_deg <= 180.0f) &&
        (value.rate_dps >= -2000.0f && value.rate_dps <= 2000.0f) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
    /* 序号只在完整发布时递增；读者不修改它，允许uint32回绕。 */
    value.sequence = s_heading_source.sequence + 1U;
    s_heading_source = value;
    s_heading_source_ready = 1U;
    taskEXIT_CRITICAL();
}

/* 唯一观察者CAN任务：只取快照，禁止从这些Watch变量驱动电机。 */
static void App_HeadingSourceObserve(void)
{
    AppHeadingSource value;
    uint32_t now, ready, valid;
    taskENTER_CRITICAL();
    value = s_heading_source;
    ready = s_heading_source_ready;
    now = (uint32_t)xTaskGetTickCount();
    taskEXIT_CRITICAL();
    valid = ((ready == 1U) && (value.healthy == 1U) &&
        ((uint32_t)(now - value.sample_ms) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.angle_ms) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.rate_ms) < HEADING_SOURCE_FRESH_MS)) ? 1U : 0U;
    g_heading_source_sequence = value.sequence;
    g_heading_source_age_ms = now - value.sample_ms;
    g_heading_source_angle_age_ms = now - value.angle_ms;
    g_heading_source_rate_age_ms = now - value.rate_ms;
    g_heading_source_yaw_deg = value.yaw_deg;
    g_heading_source_rate_dps = value.rate_dps;
    /* 失效仍保留原始观测值供排查，必须同时检查valid。 */
    g_heading_source_valid = valid;
}
#endif
#if APP_HEADING_SESSION_TEST
/* 只允许CAN任务操作会话对象；Safety只锁存trip并撤销执行。 */
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
volatile uint32_t g_heading_exec_request = 0U; /* 1开始一次，2取消 */
volatile float g_heading_exec_relative_deg = 5.0f;
volatile uint32_t g_heading_exec_phase = HR_IDLE;
volatile uint32_t g_heading_exec_trip = 0U;
volatile uint32_t g_heading_exec_state = HS_IDLE;
volatile uint32_t g_heading_exec_cause = 0U;
volatile uint32_t g_heading_exec_action = HS_ACTION_NONE;
volatile int32_t g_heading_exec_rpm = 0;
volatile float g_heading_exec_target_deg = 0.0f;
volatile float g_heading_exec_error_deg = 0.0f;
volatile uint32_t g_heading_exec_backend_session = 0U;
static HeadingSession s_heading_exec = {0};
static uint32_t s_heading_exec_wait_ms = 0U;
static float s_heading_exec_relative = 0.0f;

/* trip:1取消，2源失效/配置冲突，3平台/互斥，4等待超时，
 * 5执行器会话归属，6会话/提交失败，7运行总截止。首次原因锁存。 */
/* 首次失败快照：仅记录，不改变原有判定、截止或VM动作。 */
volatile uint32_t g_heading_fault_phase = 0U;
volatile uint32_t g_heading_fault_source_flags = 0U;
volatile uint32_t g_heading_fault_sample_age_ms = 0U;
volatile uint32_t g_heading_fault_angle_age_ms = 0U;
volatile uint32_t g_heading_fault_rate_age_ms = 0U;
volatile uint32_t g_heading_fault_hwt_update_age_ms = 0U;
volatile uint32_t g_heading_fault_tick_ms = 0U;

static void App_HeadingRuntimeTrip(uint32_t cause)
{
    uint32_t now, flags;
    taskENTER_CRITICAL();
    if (g_heading_exec_trip == 0U)
    {
        now = (uint32_t)xTaskGetTickCount();
        g_heading_fault_phase = g_heading_exec_phase;
        g_heading_fault_tick_ms = now;
        g_heading_fault_sample_age_ms = now - s_heading_source.sample_ms;
        g_heading_fault_angle_age_ms = now - s_heading_source.angle_ms;
        g_heading_fault_rate_age_ms = now - s_heading_source.rate_ms;
        g_heading_fault_hwt_update_age_ms = now - g_hwt_update_tick;
        flags = 0U;
        if (s_heading_source_ready != 1U) flags |= 0x001U;
        if (s_heading_source.healthy != 1U) flags |= 0x002U;
        if (g_heading_fault_sample_age_ms >= HS_SOURCE_MS) flags |= 0x004U;
        if (g_heading_fault_angle_age_ms >= HS_SOURCE_MS) flags |= 0x008U;
        if (g_heading_fault_rate_age_ms >= HS_SOURCE_MS) flags |= 0x010U;
        if (g_hwt_cal_active != 0U) flags |= 0x020U;
        if (g_hwt_cal_request != 0U) flags |= 0x040U;
        if (g_hwt_cal_end_tx_result == 2U) flags |= 0x080U;
        if (g_hwt_version_state != 0U) flags |= 0x100U;
        if (g_hwt_version_request != 0U) flags |= 0x200U;
        if (g_hwt_a7_request != 0U) flags |= 0x400U;
        g_heading_fault_source_flags = flags;
        g_heading_exec_trip = cause;
    }
    g_heading_exec_rpm = 0;
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    taskEXIT_CRITICAL();
}


static uint32_t App_HeadingRuntimeExclusive(void)
{
    return ((g_chassis_auto_request == 0U) &&
        (g_can_jog_request == 0U) && (s_can_jog_preparing == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U) &&
        (App_CanJogOtherRequest() == 0U) && (g_can_feedback_enable == 0U) &&
        (g_can_poll_enable == 1U) && (g_can_test_result == 1U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_motion_diag_hold_report == 0U) &&
        (g_motion_diag_hold_stop_service == 0U) &&
        (g_motion_diag_fail_stop_once == 0U)) ? 1U : 0U;
}

/* 由调用者在临界区内使用，独立复查真实帧年龄及配置互斥。 */
static uint32_t App_HeadingRuntimeSource(uint32_t now)
{
    return ((s_heading_source_ready == 1U) && (s_heading_source.healthy == 1U) &&
        ((uint32_t)(now - s_heading_source.sample_ms) < HS_SOURCE_MS) &&
        ((uint32_t)(now - s_heading_source.angle_ms) < HS_SOURCE_MS) &&
        ((uint32_t)(now - s_heading_source.rate_ms) < HS_SOURCE_MS) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}

static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *status)
{
    return ((status->session == g_heading_exec_backend_session) &&
        (status->heading_mode == 1U) && (status->segment == 2U) &&
        (status->dry_run == 0U)) ? 1U : 0U;
}

/* Safety每20ms调用；不修改会话对象，不发CAN，不等待CAN任务。 */
static void App_HeadingRuntimeSafety(void)
{
    ChassisAutoStatus status;
    uint32_t now, normal_done;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    (void)ChassisAuto_ReadStatus(&status);
    if ((g_heading_exec_phase != HR_WAIT) && (g_heading_exec_phase != HR_RUN))
    {
        if (status.active != 0U) App_HeadingRuntimeTrip(5U);
        g_vm_test_request = 0U;
    }
    else if (g_heading_exec_trip != 0U)
        App_HeadingRuntimeTrip(g_heading_exec_trip);
    else if (g_heading_exec_request == 2U) App_HeadingRuntimeTrip(1U);
    else if (!App_HeadingRuntimeSource(now)) App_HeadingRuntimeTrip(2U);
    else if (!App_HeadingRuntimeExclusive() || (g_vm_test_request != 1U))
        App_HeadingRuntimeTrip(3U);
    else if (g_heading_exec_phase == HR_WAIT)
    {
        if ((uint32_t)(now - s_heading_exec_wait_ms) >= 5000U)
            App_HeadingRuntimeTrip(4U);
        else if (status.active != 0U || status.result != 0U)
            App_HeadingRuntimeTrip(5U);
    }
    else
    {
        normal_done = ((g_heading_exec_state == HS_STOPPING) &&
            App_HeadingRuntimeOwns(&status) && (status.active == 0U) &&
            (status.result == 1U) && (status.rotation_stop_mask == 0x0FU)) ? 1U : 0U;
        if (!App_HeadingRuntimeOwns(&status)) App_HeadingRuntimeTrip(5U);
        else if ((uint32_t)(now - s_heading_exec_wait_ms) >=
                 (HS_RUN_MS + HS_STOP_MS)) App_HeadingRuntimeTrip(7U);
        else if (!normal_done && ((status.active != 1U) ||
                 (status.result != 0U) || (g_safety_ready != 1U) ||
                 (g_can_guard_ready != 1U))) App_HeadingRuntimeTrip(3U);
    }
    taskEXIT_CRITICAL();
}

/* 只由唯一CAN任务调用，每轮计算后最多执行原执行器的一步交易。 */
static void App_HeadingRuntimeStep(void)
{
    HeadingSessionInput input = {0};
    HeadingSessionOutput output = {0};
    ChassisAutoStatus status;
    uint32_t request, accepted = 0U, now;

    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    request = g_heading_exec_request;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_heading_exec_phase >= HR_DONE)
    {
        /* 终态保持观察结果；新request不排队，上电后只运行一次。 */
        g_vm_test_request = 0U;
        taskEXIT_CRITICAL();
        return;
    }
    if ((request == 2U) &&
        ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN)))
        App_HeadingRuntimeTrip(1U);
    if ((request == 1U) && (g_heading_exec_phase == HR_IDLE))
    {
        s_heading_exec_relative = g_heading_exec_relative_deg;
        /* 首轮只接受正负5度，不从Watch开放任意大角度。 */
        if (((s_heading_exec_relative != 5.0f) && (s_heading_exec_relative != -5.0f)) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != 0U) || (g_heading_exec_trip != 0U) ||
            (status.active != 0U) || (status.result != 0U) ||
            (status.cut_vm != 0U) ||
            !(s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f))
            App_HeadingRuntimeTrip(3U);
        else
        {
            s_heading_exec_wait_ms = now;
            g_heading_exec_phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    if ((g_heading_exec_phase == HR_WAIT) && (g_heading_exec_trip == 0U))
    {
        if (!App_HeadingRuntimeSource(now)) App_HeadingRuntimeTrip(2U);
        else if (!App_HeadingRuntimeExclusive() || g_vm_test_request != 1U)
            App_HeadingRuntimeTrip(3U);
        else if ((uint32_t)(now - s_heading_exec_wait_ms) >= 5000U)
            App_HeadingRuntimeTrip(4U);
        else if (App_CanJogReady() == 1U)
        {
            input.now_ms = now;
            input.sample_ms = s_heading_source.sample_ms;
            input.sequence = s_heading_source.sequence;
            input.healthy = input.platform_ok = 1U;
            input.yaw_deg = s_heading_source.yaw_deg;
            input.rate_dps = s_heading_source.rate_dps;
            input.backend_active = status.active;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            if (HeadingSession_Start(&s_heading_exec, &input,
                s_heading_exec_relative, &output) != 1U)
                App_HeadingRuntimeTrip(6U);
            else
            {
                accepted = ChassisAuto_BeginHeading(0U, output.command_rpm);
                HeadingSession_ReportBegin(&s_heading_exec, accepted, &output);
                if (accepted != 1U) App_HeadingRuntimeTrip(6U);
                else
                {
                    (void)ChassisAuto_ReadStatus(&status);
                    g_heading_exec_backend_session = status.session;
                    g_heading_exec_phase = HR_RUN;
                    s_heading_exec_wait_ms = now;
                }
            }
        }
    }
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_HeadingRuntimeOwns(&status)) App_HeadingRuntimeTrip(5U);
        else
        {
            input.now_ms = now;
            input.sample_ms = s_heading_source.sample_ms;
            input.sequence = s_heading_source.sequence;
            input.healthy = App_HeadingRuntimeSource(now);
            input.platform_ok = ((App_HeadingRuntimeExclusive() == 1U) &&
                (g_vm_test_request == 1U) &&
                ((App_CanJogReady() == 1U) ||
                 ((s_heading_exec.state == HS_STOPPING) &&
                  (status.active == 0U) && (status.result == 1U) &&
                  (status.rotation_stop_mask == 0x0FU)))) ? 1U : 0U;
            input.yaw_deg = s_heading_source.yaw_deg;
            input.rate_dps = s_heading_source.rate_dps;
            input.backend_active = status.active;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            if (output.action == HS_ACTION_UPDATE)
            {
                accepted = ChassisAuto_UpdateHeading(output.command_rpm);
                HeadingSession_ReportUpdate(&s_heading_exec, accepted, &output);
            }
            else if (output.action == HS_ACTION_STOP)
            {
                accepted = ChassisAuto_RequestStop();
                HeadingSession_ReportStop(&s_heading_exec, accepted, &output);
            }
            if (output.action == HS_ACTION_ABORT) App_HeadingRuntimeTrip(6U);
            else if (output.action == HS_ACTION_COMPLETE)
            {
                g_heading_exec_phase = HR_DONE;
                g_vm_test_request = 0U;
            }
        }
    }
    if (g_heading_exec_trip != 0U)
    {
        if (g_heading_exec_trip == 1U)
        {
            HeadingSession_Cancel(&s_heading_exec, &output);
            g_heading_exec_phase = HR_CANCELLED;
        }
        else
        {
            /* 只让CAN任务终结会话；故障不伪装成正常到位。 */
            input.healthy = input.platform_ok = 0U;
            input.backend_result = 2U;
            input.now_ms = now;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            g_heading_exec_phase = HR_FAULT;
        }
        output.action = HS_ACTION_ABORT;
        output.command_rpm = 0;
    }
    g_heading_exec_state = output.state;
    g_heading_exec_cause = output.cause;
    g_heading_exec_action = output.action;
    g_heading_exec_rpm = output.command_rpm;
    if (g_heading_exec_phase != HR_IDLE)
    {
        g_heading_exec_target_deg = s_heading_exec.controller.target_deg;
        g_heading_exec_error_deg = s_heading_exec.controller.error_deg;
    }
    taskEXIT_CRITICAL();

    /* 真正的CAN交易放在临界区外，仍只有这一个CAN任务。 */
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
        ChassisAuto_CanStep();
}
#endif

static void App_CanTask(void *argument)
{
    (void)argument;
    g_can_test_result = CAN1_LoopbackTest();
    while (1)
    {
        g_can_task_count++;
        #if APP_HEADING_SESSION_TEST
                App_HeadingRuntimeStep();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
                App_HeadingSourceObserve();
                {
                    ChassisHeadingRequest heading_request = {0, 1U, 0U};
                    uint32_t valid = ChassisAuto_HeadingRead(&heading_request);
                    /* 只观察：不会把这里的轮速发给电机。 */
                    g_heading_bus_rpm = heading_request.command_rpm;
                    g_heading_bus_stop = heading_request.stop_required;
                    g_heading_bus_sequence = heading_request.sequence;
                    g_heading_bus_valid = valid;
                }
        #endif
        if (g_can_test_result == 1U)
        {
            
        /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
        /* 静态停止测试优先，每轮仍只执行一笔CAN交易 */
        /* 自动停止优先，每轮只执行一笔CAN交易 */
        #if APP_HEADING_SESSION_TEST
                if (g_chassis_auto_active == 0U) App_CanPollStep();
        #endif
                if (APP_HEADING_SESSION_TEST == 0U)
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
            int32_t rpm = 0;
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
                    (long)g_can_speed_rpm);
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
            } /* 原调试交易只在非目标航向模式执行 */
        App_CanPollRefresh();
        App_CanFeedbackRefresh();
        /* 临时故障注入，正式监督与正常轮询仍继续运行 */
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
        g_can_poll_protect_mask, g_can_poll_tick);
        /* 诊断暂停仅冻结Safety的两类快照；原3A状态保护照常发布 */
        if ((g_can_feedback_enable != 1U) || (g_can_poll_enable != 1U) ||
            (g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
            (g_rtos_fault != 0U) || (g_safety_fault != 0U))
        {
            /* 会话结束或故障不保留诊断暂停，仍发布无效快照 */
            g_motion_diag_hold_report = 0U;
            g_motion_diag_hold_stop_service = 0U;
            g_motion_diag_fail_stop_once = 0U;
        }
        if (g_motion_diag_hold_report != 1U)
        {
            /* 将真实采样时间交给Safety，不能替换成报告时刻 */
            Safety_MotionFeedbackReport(
                ((g_can_feedback_enable == 1U) &&
                 (g_can_poll_enable == 1U) && (g_vm_test_request == 1U) &&
                 (g_safety_ready == 1U) && (g_rtos_fault == 0U) &&
                 (g_safety_fault == 0U)) ? 1U : 0U,
                g_can_feedback_speed_valid_mask,
                g_can_feedback_position_valid_mask,
                g_can_feedback_speed_tick, g_can_feedback_position_tick);
            g_motion_diag_report_count++;
        }
        }
        else
        {
            /* 回环失败时不发电机查询，监督沿用原启动故障判定 */
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
            g_motion_diag_hold_report = 0U;
            g_motion_diag_hold_stop_service = 0U;
            g_motion_diag_fail_stop_once = 0U;
            Safety_MotionFeedbackReport(0U, 0U, 0U, NULL, NULL);
            g_motion_diag_report_count++;
        }
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_HEADING_SESSION_TEST ? 10U : 50U));
    }
}

/* 仅虚拟数据的软件自检：0未执行，1通过，2失败。case非0定位失败组。 */
volatile uint32_t g_heading_dry_result = 0U;
volatile uint32_t g_heading_dry_case = 0U;
volatile int32_t g_heading_dry_ccw[5] = {0};
volatile int32_t g_heading_dry_cw[5] = {0};
volatile float g_heading_dry_cross_error = 0.0f;

static void App_HeadingDryRun(void)
{
    HeadingController controller = {0};
    HeadingControllerOutput output = {0};
    const HeadingControllerConfig *config = &g_heading_controller_default_config;
    static const float yaw[2][5] = {
        {0.0f, 25.0f, 39.0f, 44.0f, 44.0f},
        {0.0f, -25.0f, -39.0f, -44.0f, -44.0f}
    };
    static const uint32_t tick[5] = {0U, 100U, 200U, 300U, 1000U};
    static const int32_t expected[5] = {5, 3, 1, 0, 0};
    uint32_t direction, i, expected_state;
    int32_t expected_command;
    float error;

    g_heading_dry_result = 2U;
    /* 两组各5次计算，虚拟tick不会让任务真实等待这些时间。 */
    for (direction = 0U; direction < 2U; direction++)
    {
        g_heading_dry_case = direction + 1U;
        if (HeadingController_Start(&controller, config,
            (direction == 0U) ? 45.0f : -45.0f, 0.0f, 0U) != 1U) return;
        for (i = 0U; i < 5U; i++)
        {
            if (HeadingController_Step(&controller, tick[i], yaw[direction][i],
                0.0f, 1U, &output) != 1U) return;
            expected_command = (direction == 0U) ? expected[i] : -expected[i];
            if (direction == 0U) g_heading_dry_ccw[i] = output.command_rpm;
            else g_heading_dry_cw[i] = output.command_rpm;
            expected_state = (i < 3U) ? HEADING_CTRL_TURNING :
                ((i == 3U) ? HEADING_CTRL_SETTLING : HEADING_CTRL_DONE);
            if ((output.command_rpm != expected_command) ||
                (output.state != expected_state) ||
                (output.cause != HEADING_CAUSE_NONE) ||
                (output.stop_required != ((i < 3U) ? 0U : 1U))) return;
        }
    }

    /* +179到-179最短误差为+2度，在当前2度候选容差内。 */
    g_heading_dry_case = 3U;
    if (HeadingController_Start(&controller, config, -179.0f, 179.0f, 0U) != 1U)
        return;
    if (HeadingController_Step(&controller, 0U, 179.0f, 0.0f, 1U, &output) != 1U)
        return;
    g_heading_dry_cross_error = output.error_deg;
    if ((output.error_deg < 1.999f) || (output.error_deg > 2.001f) ||
        (output.state != HEADING_CTRL_SETTLING) || (output.command_rpm != 0)) return;
    if (HeadingController_Step(&controller, 700U, -179.0f, 0.0f, 1U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_DONE) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 4U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, 1U, 0.0f, 0.0f, 0U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_SENSOR_INVALID) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 5U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, config->timeout_ms,
        0.0f, 0.0f, 1U, &output) != 1U) return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_TIMEOUT) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 6U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    HeadingController_Cancel(&controller, &output);
    if ((output.state != HEADING_CTRL_CANCELLED) ||
        (output.cause != HEADING_CAUSE_CANCELLED) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    /* 验证刚修正的两角分别归一化，不允许误返回0。 */
    g_heading_dry_case = 7U;
    error = HeadingController_ErrorDeg(179.0f, -3500.0f);
    if ((error < 78.999f) || (error > 79.001f)) return;
    error = HeadingController_ErrorDeg(-179.0f, 3500.0f);
    if ((error < -79.001f) || (error > -78.999f)) return;

    g_heading_dry_case = 0U;
    g_heading_dry_result = 1U;
}

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
/* 仅供Watch观察：转换结果与四轮目标，不发送CAN。 */
volatile uint32_t g_heading_live_encode_result = 0U;
volatile ChassisMotorCommand g_heading_live_motor[4] = {0};
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
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT] = {0};
    uint32_t i;
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
        /* 将本轮航向输出转换为四轮指令，仅发布观察结果。 */
    g_heading_live_encode_result =
        Chassis_HeadingCommands(output.command_rpm, commands);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_heading_live_motor[i] = commands[i];
    }
    /* TaskA发布完整的一轮计算结果；CAN任务只读取预览。 */
    g_heading_publish_result = ChassisAuto_HeadingPublish(
        output.command_rpm, output.stop_required);
}
#endif

/* 任务A：每100毫秒计数一次 */
static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    SAOMA_EnableReceive();
    HWT101_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
    /* 已验收的基础平台就绪后，只请求一次提示音 */
    Buzzer_Start(APP_INIT_BEEP_COUNT);
        /* 一次性虚拟角度自检，不驱动小车。 */
    App_HeadingDryRun();
    while (1)
    {
        g_test_count_a++;
        
        /* 每100毫秒更新响停状态 */
        Buzzer_Update();
        HWT101_Update();
        #if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
        App_HeadingSourcePublish();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        SAOMA_Update();
        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */
        if (g_start_request != 0U)
        {
            SAOMA_DataReady = 0U;
        }
        if (SAOMA_DataReady != 0U &&
            (TickType_t)(xTaskGetTickCount() - screen_start_tick) >=
            pdMS_TO_TICKS(1000U))
        {
            char scan_log[24];
            SAOMA_DataReady = 0U;
            g_scan_display_result = Screen_ShowTaskCode(SAOMA_Data);
            if (g_scan_display_result == SCREEN_OK)
            {
                g_scan_display_count++;
            }
            /* 诊断发送接受不等于PC已经收到；忙时返回0 */
            sprintf(scan_log, "SCAN:%s\r\n", SAOMA_Data);
            g_scan_log_send_ok = App_DebugSendString(scan_log);
        }
        App_StartGateUpdate();
        g_test_tick = xTaskGetTickCount();
        /* 查询当前任务的最小剩余栈及堆余量 */
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 一轮查询和平台工作结束后，报告真实进展 */
        Safety_Report(SAFETY_TASK_APP);
        
        
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/* 任务B：每500毫秒计数一次 */
static void App_TestTaskB(void *argument)
{
    (void)argument;

    while (1)
    {
        g_test_count_b++;
        /* 每500毫秒翻转一次，完整亮灭周期约1秒 */
        LED_Turn();
        /* 查询任务B自己的最小剩余栈 */
        g_stack_b_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

/* 独立接收任务：有数据立即处理，无数据进入等待 */
static void App_UartRxTask(void *argument)
{
    uint8_t data;

    (void)argument;

    while (1)
    {
        if (USART1_WaitByte(&data, 100U) != 0U)
        {
            g_serial_rx_count++;
            g_serial_rx_last = data;
        }
        else
        {
            /* 空闲时记录历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
        /* 有数据或正常空闲等待返回，均表示任务仍在运行 */
        Safety_Report(SAFETY_TASK_RX);
    }
}

/* 每1毫秒扫描，只记录启动按键事件 */
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST
/* 0等按键、2等CAN就绪、3测试中、4完成、5取消/失败。
 * 本开发固件每次上电仅一次；再次按键不会重启，复位后才允许重测。 */
volatile uint32_t g_chassis_key_state = 0U;
volatile uint32_t g_chassis_key_cause = 0U; /* 1取消、2门控、3就绪超时、4执行失败、5HWT条件 */
static TickType_t s_chassis_key_tick = 0U;

#if APP_HWT_TURN_TEST
volatile uint32_t g_hwt_turn_request = 0U; /* 1开始一次实转，2取消；默认0 */
volatile float g_hwt_turn_yaw_start = 0.0f, g_hwt_turn_yaw_end = 0.0f;
volatile float g_hwt_turn_yaw_delta = 0.0f;
static float s_hwt_turn_last_yaw;
static uint32_t s_hwt_turn_angle_count;
/* 采样任务与数据均须新鲜，不把旧valid_mask当成新数据。 */
static uint32_t App_HwtTurnHealthy(void)
{
    return ((g_hwt_version_result == 1U) && (g_hwt_valid_mask == 3U) &&
        ((uint32_t)xTaskGetTickCount() - g_hwt_update_tick <= 100U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}
#endif

static void App_ChassisKeyStop(uint32_t cause)
{
    taskENTER_CRITICAL();
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    g_chassis_key_state = 5U;
    g_chassis_key_cause = cause;
    taskEXIT_CRITICAL();
}

/* 只提交请求；不等待、不调用CAN、不直接接通电源。 */
static void App_ChassisKeyStep(uint32_t event)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t state = g_chassis_key_state;
    if (event != 0U)
    {
        if ((state >= 1U) && (state <= 3U))
        { App_ChassisKeyStop(1U); return; }
        if (state != 0U) return;
        if ((g_vm_test_request != 0U) || (g_rtos_fault != 0U) ||
            (g_safety_fault != 0U) ||
            (g_chassis_auto_active != 0U) || (g_chassis_auto_request != 0U) ||
            (g_chassis_auto_cut_vm != 0U) || (g_wheel_test_active != 0U) ||
            (g_wheel_test_cut_vm != 0U) || (s_can_jog_preparing != 0U) ||
            (g_can_jog_request != 0U) || (App_CanJogOtherRequest() != 0U) ||
            (g_can_feedback_enable != 0U) || (g_motion_diag_fault_latched != 0U) ||
            (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
            (g_motion_auto_stop_cut_vm != 0U))
        { App_ChassisKeyStop(2U); return; }
        g_vm_test_request = 1U;
        g_chassis_key_state = 2U;
        s_chassis_key_tick = now;
        return;
    }
    if ((state == 0U) || (state >= 4U)) return;
    if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
        ((state == 3U) && (g_safety_ready != 1U)) || (g_can_feedback_enable != 0U) ||
        (g_can_poll_enable != 1U) || (g_can_jog_request != 0U) ||
        (App_CanJogOtherRequest() != 0U) ||
        (g_motion_diag_hold_report != 0U) || (g_motion_diag_hold_stop_service != 0U) ||
        (g_motion_diag_fail_stop_once != 0U))
    { App_ChassisKeyStop(2U); return; }
    if (state == 2U)
    {
        if (g_vm_test_request != 1U)
        { App_ChassisKeyStop(2U); return; }
        if ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(APP_CHASSIS_KEY_READY_MS))
        { App_ChassisKeyStop(3U); return; }
        if ((g_can_test_result == 1U) && (App_CanJogReady() == 1U))
        {
            taskENTER_CRITICAL();
            g_chassis_auto_result = 0U;
            g_chassis_auto_dry_run = 0U; /* 按键明确选择实转；Watch入口默认仍为演练 */
            g_chassis_auto_request = 1U;
            g_chassis_key_state = 3U;
            s_chassis_key_tick = now;
            taskEXIT_CRITICAL();
        }
    }
    else if (state == 3U)
    {
        if ((g_chassis_auto_request == 0U) && (g_chassis_auto_active == 0U) &&
            (g_chassis_auto_result != 0U))
        {
            if (g_chassis_auto_result == 1U)
            { g_chassis_key_state = 4U; g_vm_test_request = 0U; }
            else App_ChassisKeyStop(4U);
        }
        else if ((g_vm_test_request != 1U) ||
            ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(50000U)))
        { App_ChassisKeyStop(4U); }
    }
}
#if APP_HWT_TURN_TEST
/* 只由Watch开始；PC13只取消活动测试。重复请求不续时、不排队。 */
static void App_HwtTurnStep(uint32_t request, uint32_t key_event)
{
    uint32_t state = g_chassis_key_state;
    if (((request == 2U) || (key_event != 0U)) && (state >= 2U) && (state <= 3U))
    { App_ChassisKeyStop(1U); return; }
    if (((state == 2U) || (state == 3U)) && !App_HwtTurnHealthy())
    { App_ChassisKeyStop(5U); return; }
    if ((request == 1U) && (state == 0U))
    {
        if (!App_HwtTurnHealthy()) { App_ChassisKeyStop(5U); return; }
        g_hwt_turn_yaw_start = g_hwt_turn_yaw_end = g_hwt_yaw_deg;
        g_hwt_turn_yaw_delta = 0.0f;
        s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
        App_ChassisKeyStep(1U);
        return;
    }
    if (((state == 2U) || (state == 3U)) &&
        (s_hwt_turn_angle_count != g_hwt_angle_count))
    {
        float delta = g_hwt_yaw_deg - s_hwt_turn_last_yaw;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        g_hwt_turn_yaw_delta += delta;
        g_hwt_turn_yaw_end = s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
    }
    App_ChassisKeyStep(0U);
}
/* Safety任务也独立检查，Watch直接改VM不能绕过显式测试会话。 */
static void App_HwtTurnSafety(void)
{
    if ((g_chassis_key_state == 2U) || (g_chassis_key_state == 3U))
    {
        if (!App_HwtTurnHealthy()) App_ChassisKeyStop(5U);
    }
    else g_vm_test_request = 0U;
}
#endif
#endif

static void App_KeyTask(void *argument)
{
    uint32_t diagnostic_count = 0U;

    (void)argument;

    while (1)
    {
#if APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* PC13只取消，不用按键启动本次开发试验。 */
            taskENTER_CRITICAL();
            if ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN))
                g_heading_exec_request = 2U;
            taskEXIT_CRITICAL();
        }
#elif APP_HWT_STATIC_TEST
        if (Key_GetEvent() != 0U) g_start_key_count++;
#elif APP_HWT_TURN_TEST
        {
            uint32_t event = Key_GetEvent();
            uint32_t request = g_hwt_turn_request;
            g_hwt_turn_request = 0U;
            if (event != 0U) g_start_key_count++;
            App_HwtTurnStep(request, event);
        }
#elif APP_CHASSIS_KEY_TEST_ENABLE
        {
            uint32_t event = Key_GetEvent();
            if (event != 0U) g_start_key_count++;
            App_ChassisKeyStep(event);
        }
#else
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* 未就绪的按键当场拒绝，不留到扫码后自动启动 */
            taskENTER_CRITICAL();
            if (g_start_gate_ready != 0U && g_start_request == 0U)
            {
                s_start_pending_revision = s_start_gate_revision;
                s_start_pending = 1U;
            }
            else
            {
                g_start_deny_count++;
            }
            taskEXIT_CRITICAL();
        }

#endif
        diagnostic_count++;
        if (diagnostic_count >= 100U)
        {
            g_key_stack_words = uxTaskGetStackHighWaterMark(NULL);
            diagnostic_count = 0U;
        }
        Safety_Report(SAFETY_TASK_KEY);
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

/* 监督不等待CAN或串口，检查后立即让出CPU */
static void App_SafetyTask(void *argument)
{
    (void)argument;

    while (1)
    {
#if APP_HEADING_SESSION_TEST
        App_HeadingRuntimeSafety();
#elif APP_HWT_STATIC_TEST
        g_vm_test_request = 0U;
#elif APP_HWT_TURN_TEST
        App_HwtTurnSafety();
#endif
        Safety_Update(g_vm_test_request, g_can_test_result);
        g_safety_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

int main(void)
{
    /* 更新时钟，将4位优先级全部用于抢占优先级 */
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* 调度启动前初始化状态灯 */
    LED_Init();
    /* 调度前初始化蜂鸣器，保持关闭 */
    Buzzer_Init();
    /* 初始化调试串口，本轮仅发送 */
    USART1_Config();
    /* 调度前初始化启动按键 */
    /* UART4仅初始化，实际发送放到任务A */
    Screen_Init();
    Key_Init();
    /* 初始化PE2，默认关闭VM允许请求 */
    Power_Init();
    Safety_Init();
    SAOMA_Init();
    HWT101_Init();
    g_watchdog_init_result = Watchdog_Init();
    configASSERT(g_watchdog_init_result == 1U);
    
    /* 每个任务栈为256个字，即1024字节 */
    g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                 256U, NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);
    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);
    /* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   256U, NULL, 3U, NULL);
    configASSERT(g_rx_task_create == pdPASS);
    /* 本轮按键扫描任务，栈256字、优先级2 */
    g_key_task_create = xTaskCreate(App_KeyTask, "Key",
                                    256U, NULL, 2U, NULL);
    configASSERT(g_key_task_create == pdPASS);
    /* 优先级4，高于当前接收3、任务A/按键2；无阻塞外设操作 */
    g_safety_task_create = xTaskCreate(App_SafetyTask, "Safety",
                                      256U, NULL, 4U, NULL);
    configASSERT(g_safety_task_create == pdPASS);
    
    
    
    /* CAN栈384字，优先级3；监督仍为4 */
    g_can_task_create = xTaskCreate(App_CanTask, "CanPoll",
                                      384U, NULL, 3U, NULL);
    configASSERT(g_can_task_create == pdPASS);
    /* 启动后由任务运行 */
    vTaskStartScheduler();

    /* 调度器异常返回时记录并停止 */
    g_test_scheduler_returned = 1U;
    configASSERT(0);

    while (1)
    {
    }
}
