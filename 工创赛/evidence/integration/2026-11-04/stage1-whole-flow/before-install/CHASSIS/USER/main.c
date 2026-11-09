#define VEHICLE_PARAMETER_TABLES
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "led.h"
#include "Buzzer.h"
#include "app_config.h"
#include "startup_pose.h"
#include "startup_axes.h"
#include "vehicle_arm.h"
#include "vehicle_mission.h"
#include "route_plan.h"
extern volatile RouteCheck g_route_check;
#include "dm_can2.h"
#include "radar_servo.h"
#include "radar_lift.h"
#include "radar_zone_switch.h"
#include "fine_rpm_telemetry.h"
#include "app_entry.h"
#include "usart.h"
#include "Key.h"
#include "power.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "chassis_auto_test.h"
#include "chassis_odometry.h"
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
extern volatile ChassisOdometryStatus g_chassis_odometry;
#endif
#include "hwt101.h"
#include "safety.h"
#include "watchdog.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>
#include <string.h>
#include "heading_controller.h"
#include "heading_session.h"
#include "route_heading_reference.h"
#include "motion_action_plan.h"
#include "motion_control.h"
#include "runtime_route.h"
#include "motion_distance_limits.h"
#include "vehicle_log_config.h"
#include "field_map.h"

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
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_stage1_stop_requested=0U;
volatile uint32_t g_stage1_motion_skipped=0U;
volatile uint32_t g_scan_display_result = 0U;
volatile uint32_t g_scan_display_count = 0U;
volatile uint32_t g_scan_log_send_ok = 0U;
/* 当前已接模块的启动请求门禁，尚未驱动底盘 */
volatile uint32_t g_start_gate_ready = 0U;
volatile uint32_t g_start_request = 0U;
volatile uint32_t g_start_accept_count = 0U;
volatile uint32_t g_start_deny_count = 0U;
char g_start_task_code[16] = {0};
#if APP_IS_COMPETITION || (!APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST)
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
static void App_CanFeedbackStep(void)
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
#if APP_IS_COMPETITION || (!APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST)
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
#if APP_IS_COMPETITION
        AppCompetitionEntry_Submit(g_start_task_code,platform_ok);
#endif
        sprintf(log, "START:%s\r\n", g_start_task_code);
        (void)App_DebugSendString(log);
    }
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
    uint32_t sequence, sample_ms, angle_ms, rate_ms, healthy, valid_mask;
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

/* Invalid frame age stays invalid; normal uint32 timer wrap remains supported. */
static __inline uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit)
{ return (s->valid_mask & bit) ? now-((bit==1U)?s->angle_ms:s->rate_ms) : 0xFFFFFFFFU; }
/* 唯一生产者TaskA：紧接HWT101_Update调用，不代表运动许可。 */
static void App_HeadingSourcePublish(void)
{
    AppHeadingSource value = {0};
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    value.sample_ms = now;
    /* 用真实接收帧龄期还原时间，不能用本次发布时刻冒充新帧。 */
    value.valid_mask=g_hwt_valid_mask & 3U;
    if(g_hwt_angle_age_ms==0xFFFFFFFFU)value.valid_mask &= ~1U;
    if(g_hwt_rate_age_ms==0xFFFFFFFFU)value.valid_mask &= ~2U;
    value.angle_ms=(value.valid_mask & 1U) ? g_hwt_update_tick-g_hwt_angle_age_ms : 0U;
    value.rate_ms=(value.valid_mask & 2U) ? g_hwt_update_tick-g_hwt_rate_age_ms : 0U;
    value.yaw_deg = g_hwt_yaw_deg;
    value.rate_dps = g_hwt_wz_dps;
    value.healthy = ((g_hwt_version_result == 1U) &&
        (value.valid_mask == 3U) &&
        (g_hwt_angle_age_ms < HEADING_SOURCE_FRESH_MS) &&
        (g_hwt_rate_age_ms < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - g_hwt_update_tick) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,1U)) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,2U)) < HEADING_SOURCE_FRESH_MS) &&
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
        ((uint32_t)(App_HeadingFrameAge(&value,now,1U)) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,2U)) < HEADING_SOURCE_FRESH_MS)) ? 1U : 0U;
    g_heading_source_sequence = value.sequence;
    g_heading_source_age_ms = now - value.sample_ms;
    g_heading_source_angle_age_ms = App_HeadingFrameAge(&value,now,1U);
    g_heading_source_rate_age_ms = App_HeadingFrameAge(&value,now,2U);
    g_heading_source_yaw_deg = value.yaw_deg;
    g_heading_source_rate_dps = value.rate_dps;
    /* 失效仍保留原始观测值供排查，必须同时检查valid。 */
    g_heading_source_valid = valid;
}
#endif
#if APP_HEADING_SESSION_TEST
/* 只允许CAN任务操作会话对象；Safety只锁存trip并撤销执行。 */
#if APP_TEST_MODE == 3U
static uint32_t s_combined_keep_vm, s_combined_powered_segment;
static uint32_t s_combined_base_rpm, s_combined_run_ms;
static float s_combined_reference_mm_s;
static float s_combined_distance_mm;
static uint32_t s_motion_session_open, s_motion_has_action, s_motion_list_submit;
static uint32_t s_motion_turn_max_rpm = 5U;
#define App_StageBaseRpm() (s_combined_base_rpm)
#define App_StageRunMs() (s_combined_run_ms)
#define App_StageDistanceMm() (s_combined_distance_mm)
#define App_StageDoneVm() (s_combined_keep_vm)
#define App_StageStartVm() (s_combined_powered_segment)
#else
#define App_StageBaseRpm() ((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM)
#define App_StageRunMs() ((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_RUN_MS : APP_LATERAL_HOLD_RUN_MS)
#define App_StageDistanceMm() (0.0f)
#define App_StageDoneVm() (0U)
#define App_StageStartVm() (0U)
#endif
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
volatile uint32_t g_heading_exec_request = 0U; /* 1开始一次，2取消 */
volatile float g_heading_exec_relative_deg = 30.0f;
volatile uint32_t g_heading_exec_phase = HR_IDLE;
volatile uint32_t g_heading_exec_trip = 0U;
volatile uint32_t g_heading_exec_state = HS_IDLE;
volatile uint32_t g_heading_exec_cause = 0U;
volatile uint32_t g_heading_exec_action = HS_ACTION_NONE;
volatile float g_heading_exec_rpm = 0.0f;
volatile float g_heading_exec_target_deg = 0.0f;
volatile float g_heading_exec_error_deg = 0.0f;
volatile uint32_t g_heading_exec_backend_session = 0U;
/* 0未检查、1等待PE2开启、2上电等待、3等待四轮新状态、4就绪。 */
volatile uint32_t g_heading_power_wait_state = 0U;
volatile uint32_t g_heading_power_on_age_ms = 0U;
volatile uint32_t g_heading_power_post_settle_mask = 0U;
/* 分阶段HWT快照；只记录，不改变运动、停车或VM控制。 */
typedef struct
{
    uint32_t captured, valid;
    uint32_t event_ms, angle_sample_ms, angle_age_ms, rate_age_ms;
    uint32_t source_sequence, pe2_active;
    float yaw_deg, error_deg, rate_dps;
} HeadingStopAngleSample;
typedef struct
{
    uint32_t started, zero_submit_accepted;
    uint32_t off_observed_ms;
    uint32_t post_status; /* 0等待，1已采到关闭后新帧，2两秒内无有效帧。 */
    float target_deg;
    HeadingStopAngleSample start;
    HeadingStopAngleSample zero_request;
    HeadingStopAngleSample reached;
    HeadingStopAngleSample completed;
    HeadingStopAngleSample off_seen;
    HeadingStopAngleSample after_off;
} HeadingStopTrace;
volatile HeadingStopTrace g_heading_stop_trace = {0};
static HeadingSession s_heading_exec = {0};
/* CAN-task-only preparation buffers; keep large temporary objects off task stack. */
static RouteHeadingReference s_route_heading_reference;
static float s_route_heading_target;
static uint32_t s_route_heading_turn_active;
volatile float g_route_heading_targets[8], g_route_heading_start_yaw[8];
static HeadingSession s_heading_prepared;
static ChassisAutoPreparedHeading s_heading_backend_prepared;
static void App_MotionFaultCapture(uint32_t cause);
extern volatile uint32_t g_motion_fault_diag_valid;
volatile uint32_t g_heading_prepare_cycles,g_heading_prepare_max_cycles;
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
        App_MotionFaultCapture(cause);
        g_heading_fault_phase = g_heading_exec_phase;
        g_heading_fault_tick_ms = now;
        g_heading_fault_sample_age_ms = now - s_heading_source.sample_ms;
        g_heading_fault_angle_age_ms = App_HeadingFrameAge(&s_heading_source,now,1U);
        g_heading_fault_rate_age_ms = App_HeadingFrameAge(&s_heading_source,now,2U);
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
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,1U)) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,2U)) < HS_SOURCE_MS) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}

/* 仅在现有WAIT及临界区内检查；不阻塞、不请求VM、不发CAN。 */
static uint32_t App_HeadingPowerReady(uint32_t now)
{
    PowerCommandStatus power;
    TickType_t on_age, sample_age, settle_ticks;
    uint32_t i, mask = 0U;
    Power_ReadCommandStatus(&power);
    g_heading_power_post_settle_mask = 0U;
    g_heading_power_on_age_ms = 0xFFFFFFFFU;
    if ((power.active != 1U) || (power.enable_seen != 1U))
    {
        g_heading_power_wait_state = 1U;
        return 0U;
    }
    on_age = (TickType_t)((TickType_t)now - (TickType_t)power.enable_tick);
    g_heading_power_on_age_ms = (uint32_t)(on_age * portTICK_PERIOD_MS);
    settle_ticks = pdMS_TO_TICKS(APP_HEADING_POWER_SETTLE_MS);
    if (on_age < settle_ticks)
    {
        g_heading_power_wait_state = 2U;
        return 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        sample_age = (TickType_t)((TickType_t)now - g_can_poll_tick[i]);
        /* 必须是本次开启并完成等待后的成功新帧，不接受等待前样本。 */
        if ((g_can_poll_result[i] == 1U) && (g_can_poll_count[i] != 0U) &&
            ((g_can_poll_flags[i] & 0x0CU) == 0U) &&
            (sample_age <= pdMS_TO_TICKS(CAN_POLL_VALID_MS)) &&
            (sample_age <= (TickType_t)(on_age - settle_ticks)))
            mask |= 1UL << i;
    }
    g_heading_power_post_settle_mask = mask;
    g_heading_power_wait_state = (mask == 0x0FU) ? 4U : 3U;
    return (mask == 0x0FU) ? 1U : 0U;
}

/* 调用者持有临界区；captured只表示事件发生，valid才表示源新鲜。 */
static void App_HeadingStopCapture(volatile HeadingStopAngleSample *sample,
    uint32_t now)
{
    PowerCommandStatus power;
    if (sample->captured != 0U) return;
    Power_ReadCommandStatus(&power);
    sample->valid = App_HeadingRuntimeSource(now);
    sample->event_ms = now;
    sample->angle_sample_ms = s_heading_source.angle_ms;
    sample->angle_age_ms = App_HeadingFrameAge(&s_heading_source,now,1U);
    sample->rate_age_ms = App_HeadingFrameAge(&s_heading_source,now,2U);
    sample->source_sequence = s_heading_source.sequence;
    sample->pe2_active = power.active;
    sample->yaw_deg = s_heading_source.yaw_deg;
    sample->error_deg = HeadingController_ErrorDeg(
        g_heading_stop_trace.target_deg, s_heading_source.yaw_deg);
    sample->rate_dps = s_heading_source.rate_dps;
    sample->captured = 1U;
}

/* 每轮观察包括终态；不等待、不延长VM供电，不发送任何动作。 */
static void App_HeadingStopObserve(uint32_t now)
{
    PowerCommandStatus power;
    uint32_t off_age;
    if (g_heading_stop_trace.started == 0U) return;
    Power_ReadCommandStatus(&power);
    if ((g_heading_stop_trace.off_seen.captured == 0U) && (power.active == 0U))
    {
        g_heading_stop_trace.off_observed_ms = now;
        App_HeadingStopCapture(&g_heading_stop_trace.off_seen, now);
    }
    if ((g_heading_stop_trace.off_seen.captured == 0U) ||
        (g_heading_stop_trace.post_status != 0U)) return;
    off_age = now - g_heading_stop_trace.off_observed_ms;
    if (off_age >= 2000U)
    {
        g_heading_stop_trace.post_status = 2U;
        return;
    }
    /* 至少一秒后采新角度帧，不能用关断前或缓存中的旧样本。 */
    if ((off_age >= 1000U) && (App_HeadingRuntimeSource(now) == 1U) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,1U)) <= off_age - 1000U))
    {
        App_HeadingStopCapture(&g_heading_stop_trace.after_off, now);
        g_heading_stop_trace.post_status = 1U;
    }
}

static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *status)
{
    return ((status->session == g_heading_exec_backend_session) &&
        (status->heading_mode == 1U) && (status->segment == 2U) &&
        (status->forward_hold_mode == 0U) && (status->dry_run == 0U)) ? 1U : 0U;
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
        g_vm_test_request = (g_heading_exec_phase == HR_DONE) ? App_StageDoneVm() : 0U;
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
    uint32_t request, accepted = 0U, now, record_zero = 0U;
    uint32_t critical_cycle,prepare_cycle,prepared_ok,expected_session;
    uint32_t uart_base,overflow_base,checksum_base;

    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    critical_cycle=HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    App_HeadingStopObserve(now);
    request = g_heading_exec_request;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_heading_exec_phase >= HR_DONE)
    {
        /* 终态保持观察结果；新request不排队，上电后只运行一次。 */
        g_vm_test_request = (g_heading_exec_phase == HR_DONE) ? App_StageDoneVm() : 0U;
        HWT101_TimingEnd(0U,critical_cycle);
        taskEXIT_CRITICAL();
        return;
    }
    if ((request == 2U) &&
        ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN)))
        App_HeadingRuntimeTrip(1U);
    if ((request == 1U) && (g_heading_exec_phase == HR_IDLE))
    {
        s_heading_exec_relative = g_heading_exec_relative_deg;
        /* 本轮只接受正负5度或30度，不允许Watch任意扩大角度。 */
        if (((s_heading_exec_relative != 5.0f) && (s_heading_exec_relative != -5.0f) &&
             (s_heading_exec_relative != 30.0f) && (s_heading_exec_relative != -30.0f) &&
             !((APP_TEST_MODE == 3U) &&
               (((s_heading_exec_relative >= 5.0f) && (s_heading_exec_relative <= 90.0f)) ||
                ((s_heading_exec_relative <= -5.0f) && (s_heading_exec_relative >= -90.0f))))) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != App_StageStartVm()) || (g_heading_exec_trip != 0U) ||
            (status.active != 0U) || (status.result != 0U) ||
            (status.cut_vm != 0U) ||
            !(s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f))
            App_HeadingRuntimeTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
#endif
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
        else if ((App_CanJogReady() == 1U) && (App_HeadingPowerReady(now) == 1U))
        {
            input.now_ms = now;
#if APP_TEST_MODE == 3U
            input.max_rpm = s_motion_turn_max_rpm;
#endif
            input.sample_ms = s_heading_source.sample_ms;
            input.sequence = s_heading_source.sequence;
            input.healthy = input.platform_ok = 1U;
            input.yaw_deg = s_heading_source.yaw_deg;
            input.rate_dps = s_heading_source.rate_dps;
            input.backend_active = status.active;
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            expected_session=status.session;
            uart_base=g_hwt_uart_errors;overflow_base=g_hwt_overflows;
            checksum_base=g_hwt_checksum_errors;
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();
            prepare_cycle=HWT101_TimingBegin();
            HeadingSession_Init(&s_heading_prepared);
            prepared_ok=s_route_heading_turn_active ?
                HeadingSession_StartReference(&s_heading_prepared,&input,
                    s_heading_exec_relative,s_route_heading_target,&output) :
                HeadingSession_Start(&s_heading_prepared,&input,
                    s_heading_exec_relative,&output);
            if(prepared_ok)prepared_ok=ChassisAuto_PrepareHeading(0U,output.command_rpm,
                s_heading_prepared.turn_trial ? s_heading_prepared.initial_sign : 0,
                &s_heading_backend_prepared);
            g_heading_prepare_cycles=HWT101_TimingBegin()-prepare_cycle;
            if(g_heading_prepare_cycles>g_heading_prepare_max_cycles)
                g_heading_prepare_max_cycles=g_heading_prepare_cycles;
            taskENTER_CRITICAL();critical_cycle=HWT101_TimingBegin();
            now=(uint32_t)xTaskGetTickCount();
            (void)ChassisAuto_ReadStatus(&status);
            /* Preparation never grants motion: recheck cancellation, source epoch and ownership. */
            if(g_heading_exec_request==2U)App_HeadingRuntimeTrip(1U);
            else if(g_heading_exec_trip!=0U) { /* Keep the first safety failure. */ }
            else if(!App_HeadingRuntimeSource(now) || now-input.sample_ms>=HS_SOURCE_MS ||
                uart_base!=g_hwt_uart_errors || overflow_base!=g_hwt_overflows ||
                checksum_base!=g_hwt_checksum_errors)App_HeadingRuntimeTrip(2U);
            else if(g_heading_exec_phase!=HR_WAIT || !App_HeadingRuntimeExclusive() ||
                g_vm_test_request!=1U || !App_CanJogReady() || !App_HeadingPowerReady(now))
                App_HeadingRuntimeTrip(3U);
            else if(now-s_heading_exec_wait_ms>=5000U)App_HeadingRuntimeTrip(4U);
            else if(status.session!=expected_session || status.active || status.result || status.cut_vm)
                App_HeadingRuntimeTrip(5U);
            else if(!prepared_ok)App_HeadingRuntimeTrip(6U);
            else
            {
                s_heading_exec=s_heading_prepared;
                prepare_cycle=HWT101_TimingBegin();
                accepted=ChassisAuto_BeginPreparedHeading(&s_heading_backend_prepared,expected_session);
                HWT101_TimingEnd(1U,prepare_cycle);
                HeadingSession_ReportBegin(&s_heading_exec, accepted, &output);
                if (accepted != 1U) App_HeadingRuntimeTrip(6U);
                else
                {
                    (void)ChassisAuto_ReadStatus(&status);
                    g_heading_exec_backend_session = status.session;
                    g_heading_exec_phase = HR_RUN;
                    s_heading_exec_wait_ms = now;
                    g_heading_stop_trace.target_deg = output.target_deg;
                    App_HeadingStopCapture(&g_heading_stop_trace.start, now);
                    g_heading_stop_trace.started = 1U;
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
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            if (output.action == HS_ACTION_UPDATE)
            {
                record_zero = ((output.command_rpm == 0) &&
                    (!s_heading_exec.turn_trial || s_heading_exec.brake.moved) &&
                    (g_heading_stop_trace.zero_request.captured == 0U)) ? 1U : 0U;
                if (record_zero != 0U)
                    App_HeadingStopCapture(&g_heading_stop_trace.zero_request, now);
                accepted = ChassisAuto_UpdateHeading(output.command_rpm);
                if (record_zero != 0U)
                    g_heading_stop_trace.zero_submit_accepted = accepted;
                HeadingSession_ReportUpdate(&s_heading_exec, accepted, &output);
            }
            else if (output.action == HS_ACTION_STOP)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.reached, now);
                accepted = ChassisAuto_RequestStop();
                HeadingSession_ReportStop(&s_heading_exec, accepted, &output);
            }
            if (output.action == HS_ACTION_ABORT) App_HeadingRuntimeTrip(6U);
            else if (output.action == HS_ACTION_COMPLETE)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.completed, now);
                g_heading_exec_phase = HR_DONE;
                g_vm_test_request = App_StageDoneVm();
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
    HWT101_TimingEnd(0U,critical_cycle);
    taskEXIT_CRITICAL();

    /* 真正的CAN交易放在临界区外，仍只有这一个CAN任务。 */
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
        ChassisAuto_CanStep();
}

/* 前进保持开发入口；request1一次开始，2取消，其余字段只观察。 */
typedef struct
{
    uint32_t request, phase, trip, backend_session;
    uint32_t run_started, stopping, elapsed_ms, final_valid;
    float correction_rpm;
    float start_deg, target_deg, error_deg, final_deg;
    float rate_dps, peak_error_deg, peak_rate_dps;
    uint32_t sample_count, max_sample_gap_ms;
    /* 已接受的非零纠偏提交样本统计，停车后保留，不代表CAN执行次数。 */
    uint32_t motion_mode; /* 编译选择，仅观察，不用Watch切换方向。 */
    uint32_t correction_nonzero_samples;
    float correction_peak_abs_rpm;
    float correction_last_nonzero_rpm;
} ForwardHoldDebug;
volatile ForwardHoldDebug g_forward_hold = {.motion_mode = APP_TRANSLATION_HOLD_MODE};
volatile uint32_t g_translation_hold_selftest = 0U;
#if APP_TEST_MODE == 3U
/* request 1启动整组一次，2取消；state 0空闲1运行2间隔3完成4失败5取消。 */
typedef struct
{
    uint32_t request, state, index, cause, completed;
    uint32_t phase[MOTION_ACTION_CAPACITY], session[MOTION_ACTION_CAPACITY], elapsed_ms[MOTION_ACTION_CAPACITY];
    float final_yaw[MOTION_ACTION_CAPACITY], final_error[MOTION_ACTION_CAPACITY];
} CombinedTestDebug;
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count;
static RuntimeRoute s_runtime_route;
static RuntimeRoutePrepared s_runtime_prepared;
static uint32_t s_route_ack_pending,s_route_ack_id,s_route_ack_op,s_route_ack_result,s_route_ack_generation;

volatile uint32_t g_debug_plan_id = DEBUG_PLAN_DEFAULT;
#define MOTION_ACTION_COUNT (s_motion_action_count)

typedef struct {uint32_t captured,tick_ms;ChassisOdometryStatus pose;} PerimeterRecord;
volatile PerimeterRecord g_perimeter_records[9];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
static float s_perimeter_targets[9][3]={
{2100.0f,2100.0f,-90.0f},
{2100.0f,300.0f,-90.0f},
{300.0f,300.0f,-90.0f},
{300.0f,300.0f,-180.0f},
{300.0f,300.0f,90.0f},
{300.0f,2100.0f,90.0f},
{300.0f,2100.0f,-180.0f},
{300.0f,2100.0f,-90.0f},
{2100.0f,2100.0f,-90.0f}
};

/* 动态预设仍受1..8项及组总时限校验；不是运行中可改路线。 */
volatile uint32_t g_motion_plan_valid = 0U;
volatile CombinedTestDebug g_combined_test = {0};
static uint32_t s_combined_turn, s_combined_mode;
static uint32_t s_combined_begin_ms, s_combined_stage_ms, s_combined_gap_ms;
static volatile uint32_t s_combined_end_ms; /* 终态时刻保留供本地诊断。 */
#define App_TestTranslationMode() (s_combined_mode)
#else
#define App_TestTranslationMode() (APP_TRANSLATION_HOLD_MODE)
#endif
static uint32_t s_hold_wait_ms, s_hold_begin_ms, s_hold_run_ms, s_hold_stop_ms;
static uint32_t s_hold_sequence, s_hold_new_ms;
static float s_hold_target;
static HeadingHoldController s_hold_controller = {0};
static uint32_t s_serial_start_permit; /* 仅调试串口可授予一次启动许可。 */
#if APP_SERIAL_MOTION_CONTROL
volatile uint32_t g_serial_motion_state = 0U, g_serial_motion_result = 0U;
volatile uint32_t g_serial_motion_rejected = 0U, g_serial_motion_accepted = 0U;
volatile uint32_t g_serial_motion_link_lost = 0U;
static uint32_t s_serial_token_seq, s_serial_token_tick, s_serial_arm_ms;
static uint32_t s_serial_keepalive_ms;
static uint32_t s_serial_tx_seq, s_serial_dropped, s_serial_tx_tick;
static uint32_t s_serial_tx_valid; /* 至少一条当前状态已排入串口 */
static uint32_t s_serial_proto_ok;
static void App_SerialMotionSafety(void);
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t earliest_ms);
#endif

/* 每100ms记录运行期提交值，非实际轮速；容量覆盖8秒，满后不覆盖。 */
typedef struct
{
    uint32_t elapsed_ms;
    float error_deg, rate_dps;
    float correction_rpm;
} ForwardHoldTraceSample;
volatile ForwardHoldTraceSample g_forward_hold_trace[96];
volatile uint32_t g_forward_hold_trace_count = 0U;
static uint32_t s_hold_trace_ms;
/* 首个平移trip的38字段快照；取消/后续STOP均不覆盖，HF2仅供诊断。 */
#define MOTION_FAULT_DIAG_FIELDS 43U
volatile uint32_t g_motion_fault_diag[MOTION_FAULT_DIAG_FIELDS];
volatile uint32_t g_motion_fault_diag_valid;
static uint32_t s_motion_fault_timing[HWT_TIMING_FIELDS];
#if APP_SERIAL_MOTION_CONTROL
/* 成功段计时先留存，整组正常结束且VM关闭后再发送，不增加运行期串口流量。 */
#define MOTION_TIMING_RECORD_FIELDS (HWT_TIMING_FIELDS + 3U)
#define MOTION_TIMING_RECORD_CAPACITY 12U
static uint32_t s_motion_timing_records[MOTION_TIMING_RECORD_CAPACITY][MOTION_TIMING_RECORD_FIELDS];
static uint32_t s_motion_timing_count, s_motion_timing_cursor;
static void App_TimingRecordCompleted(uint32_t index, uint32_t session, uint32_t now)
{
    uint32_t *record;
    if (s_motion_timing_count >= MOTION_TIMING_RECORD_CAPACITY) return;
    record = s_motion_timing_records[s_motion_timing_count];
    record[0] = 1U; record[1] = index; record[2] = session;
    HWT101_ReadTiming(record + 3U, now);
    s_motion_timing_count++;
}
#endif

static void App_MotionFaultCapture(uint32_t cause)
{
    HwtFaultDiagnostic hwt;
    ChassisAutoFaultDiagnostic backend;
    ChassisAutoStatus status;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if(g_motion_fault_diag_valid)return;
    HWT101_ReadDiagnostic(&hwt); (void)ChassisAuto_ReadStatus(&status);
    g_motion_fault_diag[0] = now;
    g_motion_fault_diag[1] = hwt.arm_sequence;
    g_motion_fault_diag[2] = cause;
    g_motion_fault_diag[3] = (g_heading_exec_phase==HR_WAIT || g_heading_exec_phase==HR_RUN) ?
        g_heading_exec_phase : g_forward_hold.phase;
    g_motion_fault_diag[4] = status.state;
    g_motion_fault_diag[5] = status.cause;
    g_motion_fault_diag[6] = status.session;
    g_motion_fault_diag[7] = App_HeadingRuntimeSource(now);
    g_motion_fault_diag[8] = App_HeadingFrameAge(&s_heading_source,now,1U);
    g_motion_fault_diag[9] = App_HeadingFrameAge(&s_heading_source,now,2U);
    g_motion_fault_diag[10] = g_hwt_uart_errors;
    g_motion_fault_diag[11] = g_hwt_overflows;
    g_motion_fault_diag[12] = g_hwt_checksum_errors;
    g_motion_fault_diag[13] = hwt.base_uart_errors;
    g_motion_fault_diag[14] = hwt.base_overflows;
    g_motion_fault_diag[15] = hwt.base_checksum_errors;
    g_motion_fault_diag[16] = hwt.uart_ore;
    g_motion_fault_diag[17] = hwt.uart_fe;
    g_motion_fault_diag[18] = hwt.uart_ne;
    g_motion_fault_diag[19] = hwt.uart_pe;
    g_motion_fault_diag[20] = hwt.last_uart_sr;
    g_motion_fault_diag[21] = hwt.event_tick;
    g_motion_fault_diag[22] = hwt.event_reason;
    g_motion_fault_diag[23] = hwt.event_sr;
    g_motion_fault_diag[24] = hwt.event_uart_errors;
    g_motion_fault_diag[25] = hwt.event_overflows;
    g_motion_fault_diag[26] = hwt.event_checksum_errors;
    g_motion_fault_diag[27] = hwt.event_seen;
    g_motion_fault_diag[28] = now - s_heading_source.sample_ms;
    ChassisAuto_ReadFaultDiagnostic(&backend);
    g_motion_fault_diag[29] = backend.reason;
    g_motion_fault_diag[30] = backend.state;
    g_motion_fault_diag[31] = backend.io_result;
    g_motion_fault_diag[32] = backend.address;
    g_motion_fault_diag[33] = backend.command_age_ms;
    g_motion_fault_diag[34] = backend.status_mask;
    g_motion_fault_diag[35] = backend.speed_mask;
    g_motion_fault_diag[36] = backend.feedback_age_ms;
    g_motion_fault_diag[37] = backend.tick_ms;
    g_motion_fault_diag[38] = (uint32_t)backend.target_decirpm;
    g_motion_fault_diag[39] = (uint32_t)backend.measured_decirpm;
    g_motion_fault_diag[40] = backend.target_ack_ms;
    g_motion_fault_diag[41] = backend.speed_sample_ms;
    g_motion_fault_diag[42] = backend.reverse_samples;
    HWT101_ReadTiming(s_motion_fault_timing, now);
    g_motion_fault_diag_valid = 1U;
}


/* trip:1取消、2源失效、3平台、4上电等待、5执行器、6偏角、7截止、8提交。 */
static void App_ForwardHoldTrip(uint32_t cause)
{
    taskENTER_CRITICAL();
    if (g_forward_hold.trip == 0U)
    {
        App_MotionFaultCapture(cause);
        g_forward_hold.trip = cause;
    }
    g_forward_hold.phase = (g_forward_hold.trip == 1U) ? HR_CANCELLED : HR_FAULT;
    g_forward_hold.correction_rpm = 0;
    g_forward_hold.final_deg = s_heading_source.yaw_deg;
    g_forward_hold.final_valid = App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount());
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    taskEXIT_CRITICAL();
}

static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *status)
{
    return ((status->session == g_forward_hold.backend_session) &&
        (status->heading_mode == 1U) && (status->forward_hold_mode == 1U) &&
        (status->translation_mode == App_TestTranslationMode()) &&
        (status->segment == 2U) && (status->dry_run == 0U)) ? 1U : 0U;
}

/* 独立Safety监督，既有100ms源/提交失效和有界等待均不放宽。 */
static void App_ForwardHoldSafety(void)
{
    ChassisAutoStatus status;
    uint32_t now, normal_done;
    float correction;
    float error;
    taskENTER_CRITICAL();
    if ((g_forward_hold.phase != HR_WAIT) && (g_forward_hold.phase != HR_RUN))
    { taskEXIT_CRITICAL(); return; }
    now = (uint32_t)xTaskGetTickCount();
    (void)ChassisAuto_ReadStatus(&status);
    /* 等待新段时后端仍保留上一段到距结果，只有当前已归属会话才能触发停止。 */
    if ((g_forward_hold.phase == HR_RUN) && App_ForwardHoldOwns(&status) &&
        status.distance_mode && status.distance_reached && !g_forward_hold.stopping)
    {
        g_forward_hold.stopping = 1U; g_forward_hold.correction_rpm = 0;
        s_hold_stop_ms = now;
    }
    if (g_forward_hold.request == 2U) App_ForwardHoldTrip(1U);
    else if (!App_HeadingRuntimeSource(now) ||
        ((g_forward_hold.phase == HR_RUN) && ((uint32_t)(now - s_hold_new_ms) >= HS_SOURCE_MS)))
        App_ForwardHoldTrip(2U);
    else if ((g_translation_hold_selftest != 1U) ||
        !App_HeadingRuntimeExclusive() || (g_vm_test_request != 1U))
        App_ForwardHoldTrip(3U);
    else if (g_forward_hold.phase == HR_WAIT)
    {
        if ((uint32_t)(now - s_hold_wait_ms) >= 5000U) App_ForwardHoldTrip(4U);
        else if (status.active != 0U || status.result != 0U) App_ForwardHoldTrip(5U);
    }
    else
    {
        normal_done = ((g_forward_hold.stopping == 1U) &&
            (status.active == 0U) && (status.result == 1U) &&
            (status.rotation_stop_mask == 0x0FU)) ? 1U : 0U;
        if (!App_ForwardHoldOwns(&status)) App_ForwardHoldTrip(5U);
        else if ((uint32_t)(now - s_hold_begin_ms) >=
            ((App_StageDistanceMm()>0.0f) ? App_StageRunMs() : APP_HEADING_RUN_MS))
            App_ForwardHoldTrip(7U);
        else if (g_forward_hold.stopping &&
            ((uint32_t)(now - s_hold_stop_ms) >= HS_STOP_MS)) App_ForwardHoldTrip(7U);
        else if (!normal_done && ((status.active != 1U) || (status.result != 0U) ||
            (g_safety_ready != 1U) || (g_can_guard_ready != 1U))) App_ForwardHoldTrip(5U);
        else if (HeadingController_HoldCorrection(s_hold_target, s_heading_source.yaw_deg,
            s_heading_source.rate_dps, &correction, &error) != 1U) App_ForwardHoldTrip(6U);
    }
    taskEXIT_CRITICAL();
}

/* 仅CAN任务执行；本模式清除旧转角请求，避免误触发原地转动。 */
static void App_ForwardHoldStep(void)
{
    ChassisAutoStatus status;
    uint32_t now, request, delta, accepted, critical_cycle, begin_cycle;
#if APP_DISTANCE_SCURVE_ACTIVE
    /* 唯一CAN任务准备，避免新增轨迹对象占用CAN任务栈；不供其他上下文复用。 */
    static ChassisAutoPreparedTranslation prepared;
#else
    ChassisAutoPreparedTranslation prepared;
#endif
    uint32_t prepared_ok, expected_session, mode, base_rpm, run_ms;
    float route_speed;
    float distance_mm;
    float correction = 0;
    float error = 0.0f, desired = 0.0f;
    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    critical_cycle = HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    request = g_forward_hold.request;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_forward_hold.phase >= HR_DONE)
    { g_vm_test_request = (g_forward_hold.phase == HR_DONE) ? App_StageDoneVm() : 0U; HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
    if ((request == 2U) && (g_forward_hold.phase != HR_IDLE)) App_ForwardHoldTrip(1U);
    if ((request == 1U) && (g_forward_hold.phase == HR_IDLE))
    {
#if APP_SERIAL_MOTION_CONTROL && (APP_TEST_MODE != 3U)
        if (s_serial_start_permit != 1U)
        { App_ForwardHoldTrip(3U); HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
#endif
        if ((g_translation_hold_selftest != 1U) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != App_StageStartVm()) || (status.active != 0U) ||
            (status.result != 0U) || (status.cut_vm != 0U) ||
            !(s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f))
            App_ForwardHoldTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
#endif
            s_hold_wait_ms = now;
            g_forward_hold.phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    App_ForwardHoldSafety();
    if ((g_forward_hold.phase == HR_WAIT) && (App_CanJogReady() == 1U) &&
        (App_HeadingPowerReady(now) == 1U))
    {
        /* 只携带参数和旧会话号出锁，不携带过期的运动许可。 */
        mode = App_TestTranslationMode(); base_rpm = App_StageBaseRpm();
        run_ms = App_StageRunMs(); distance_mm = App_StageDistanceMm();
#if APP_TEST_MODE == 3U
        route_speed=s_combined_reference_mm_s;
#else
        route_speed=0.0f;
#endif
        (void)ChassisAuto_ReadStatus(&status); expected_session = status.session;
        HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
        prepared_ok = route_speed!=0.0f ?
            ChassisAuto_PrepareRouteTranslation(mode,base_rpm,run_ms,distance_mm,route_speed,&prepared) :
            ChassisAuto_PrepareTranslation(mode,base_rpm,run_ms,distance_mm,&prepared);
        taskENTER_CRITICAL();
        critical_cycle = HWT101_TimingBegin();
        now = (uint32_t)xTaskGetTickCount();
        App_HeadingSourceObserve();
        App_ForwardHoldSafety();
        if (g_forward_hold.phase != HR_WAIT || g_forward_hold.trip)
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        (void)ChassisAuto_ReadStatus(&status);
        if (!prepared_ok || status.session != expected_session ||
            mode != App_TestTranslationMode() || base_rpm != App_StageBaseRpm() ||
            run_ms != App_StageRunMs() || distance_mm != App_StageDistanceMm()
#if APP_TEST_MODE == 3U
            || route_speed!=s_combined_reference_mm_s
#endif
            )
        {
            App_ForwardHoldTrip(5U);
            HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return;
        }
        /* 源/故障/取消已重查；准备条件变化则继续有界等待，不能用旧许可提交。 */
        if (!App_CanJogReady() || !App_HeadingPowerReady(now))
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        /* 提交运动前验证固定参考，异常目标不获得电机启动许可。 */
        if (!s_route_heading_reference.valid ||
            !HeadingController_HoldCorrection(s_route_heading_target,s_heading_source.yaw_deg,
                s_heading_source.rate_dps,&desired,&error))
        {
            App_ForwardHoldTrip(6U);
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();return;
        }
        begin_cycle = HWT101_TimingBegin();
        accepted = (VEHICLE_STAGE1_BLEND_ENABLE && g_vehicle_stage==1U && g_vehicle_scan_search_state==0U) ?
            ChassisAuto_BeginPreparedBlend(&prepared,expected_session) : ChassisAuto_BeginPreparedTranslation(&prepared,expected_session);
        HWT101_TimingEnd(1U, begin_cycle);
        if (accepted != 1U) App_ForwardHoldTrip(5U);
        else
        {
            (void)ChassisAuto_ReadStatus(&status);
            g_forward_hold.backend_session = status.session;
            s_hold_target = s_route_heading_target;
            g_forward_hold.start_deg = s_heading_source.yaw_deg;
            g_forward_hold.target_deg = s_hold_target;
            s_hold_sequence = s_heading_source.sequence;
            s_hold_begin_ms = s_hold_new_ms = now;
            s_hold_controller = (HeadingHoldController){0};
            g_forward_hold.phase = HR_RUN;
        }
    }
    if (g_forward_hold.phase == HR_RUN)
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_ForwardHoldOwns(&status)) App_ForwardHoldTrip(5U);
        else if (HeadingController_HoldCorrection(s_hold_target, s_heading_source.yaw_deg,
            s_heading_source.rate_dps, &desired, &error) != 1U) App_ForwardHoldTrip(6U);
        else
        {
            g_forward_hold.error_deg = error;
            /* 有效任务码提前结束本段；沿用唯一CAN任务有序停车及零速核验。 */
            if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
                !g_forward_hold.stopping && status.active && status.state>=CHASSIS_AUTO_START &&
                status.state<=CHASSIS_AUTO_SETTLE)
            {
                if (ChassisAuto_RequestStop()!=1U) App_ForwardHoldTrip(8U);
                else {g_forward_hold.stopping=1U;g_forward_hold.correction_rpm=0;s_hold_stop_ms=now;}
            }
            if (status.distance_mode && status.distance_reached && !g_forward_hold.stopping)
            {
                g_forward_hold.stopping = 1U; g_forward_hold.correction_rpm = 0;
                s_hold_stop_ms = now;
            }
            if ((g_forward_hold.run_started == 0U) && (status.state == CHASSIS_AUTO_RUN) &&
                (g_chassis_auto_start_mask[2] == 0x0FU))
            {
                s_hold_run_ms = now;
                g_forward_hold.run_started = 1U;
            }
            if ((g_forward_hold.run_started != 0U) && (g_forward_hold.stopping == 0U))
                g_forward_hold.elapsed_ms = now - s_hold_run_ms;
            if ((g_forward_hold.stopping == 0U) && g_forward_hold.run_started &&
                (g_forward_hold.elapsed_ms >= App_StageRunMs()))
            {
                if (App_StageDistanceMm() > 0.0f) App_ForwardHoldTrip(2U);
                else if (ChassisAuto_RequestStop() != 1U) App_ForwardHoldTrip(8U);
                else
                {
                    g_forward_hold.stopping = 1U;
                    g_forward_hold.correction_rpm = 0;
                    s_hold_stop_ms = now;
                }
            }
            delta = s_heading_source.sequence - s_hold_sequence;
            if (delta > 0x7FFFFFFFU) App_ForwardHoldTrip(2U);
            else if (delta != 0U)
            {
                if (now - s_hold_new_ms > g_forward_hold.max_sample_gap_ms)
                    g_forward_hold.max_sample_gap_ms = now - s_hold_new_ms;
                s_hold_sequence = s_heading_source.sequence;
                s_hold_new_ms = now;
                g_forward_hold.sample_count++;
                g_forward_hold.rate_dps = s_heading_source.rate_dps;
                if (((error < 0.0f) ? -error : error) > g_forward_hold.peak_error_deg)
                    g_forward_hold.peak_error_deg = ((error < 0.0f) ? -error : error);
                if (((s_heading_source.rate_dps < 0.0f) ? -s_heading_source.rate_dps : s_heading_source.rate_dps) > g_forward_hold.peak_rate_dps)
                    g_forward_hold.peak_rate_dps = ((s_heading_source.rate_dps < 0.0f) ? -s_heading_source.rate_dps : s_heading_source.rate_dps);
                if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 0U))
                {
                    if (HeadingController_HoldStep(&s_hold_controller, s_hold_target,
                        s_heading_source.yaw_deg, s_heading_source.rate_dps, now,
                        &correction, &error) != 1U) App_ForwardHoldTrip(2U);
                    else if (ChassisAuto_UpdateForwardHold(correction) != 1U) App_ForwardHoldTrip(8U);
                    else
                    {
                        g_forward_hold.correction_rpm = correction;
                        if (correction != 0)
                        {
                            float magnitude = (correction < 0.0f) ? -correction : correction;
                            g_forward_hold.correction_nonzero_samples++;
                            if (magnitude > g_forward_hold.correction_peak_abs_rpm)
                                g_forward_hold.correction_peak_abs_rpm = magnitude;
                            g_forward_hold.correction_last_nonzero_rpm = correction;
                        }
                        if ((g_forward_hold.run_started == 1U) &&
                            (g_forward_hold_trace_count < 96U) &&
                            ((g_forward_hold_trace_count == 0U) ||
                             ((uint32_t)(now - s_hold_trace_ms) >= 100U)))
                        {
                            uint32_t index = g_forward_hold_trace_count;
                            g_forward_hold_trace[index].elapsed_ms = g_forward_hold.elapsed_ms;
                            g_forward_hold_trace[index].error_deg = error;
                            g_forward_hold_trace[index].rate_dps = s_heading_source.rate_dps;
                            g_forward_hold_trace[index].correction_rpm = correction;
                            s_hold_trace_ms = now;
                            g_forward_hold_trace_count = index + 1U;
                        }
                    }
                }
            }
            if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 1U) &&
                (status.active == 0U) && (status.result == 1U) &&
                (status.rotation_stop_mask == 0x0FU))
            {
                g_forward_hold.final_deg = s_heading_source.yaw_deg;
                g_forward_hold.final_valid = App_HeadingRuntimeSource(now);
                g_forward_hold.correction_rpm = 0;
                g_forward_hold.phase = HR_DONE;
                g_vm_test_request = App_StageDoneVm();
            }
        }
    }
    HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
    if ((g_forward_hold.phase == HR_RUN) && (g_forward_hold.trip == 0U)) ChassisAuto_CanStep();
}

#if APP_TEST_MODE == 3U
/* 独立组合测试，唯一CAN任务每轮调用一次，不阻塞、不增加任务。
 * 固定顺序：前进8秒、左移8秒、相对转向+90度、前进8秒。
 * 段间停车核验后保持VM，额外间隔由统一参数配置；仅整组完成或异常断VM。 */
/* 只校验编译期表；不发硬件命令，不清故障。总时限保持120秒。 */
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)
    {
        const MotionAction *action = &s_motion_actions[i];
        if(action->reference_mm_s!=0.0f) {
            if(g_debug_plan_id!=DEBUG_PLAN_DEFAULT || !APP_ROUTE_SPEED_TRIAL_ACTIVE ||
                action->kind<MOTION_FORWARD_DISTANCE || action->kind>MOTION_BACKWARD_DISTANCE ||
                action->relative_deg!=0.0f || !MotionRoute_ShapeAllowed(action->kind-MOTION_FORWARD_DISTANCE,
                    action->base_rpm,action->run_ms,action->distance_mm,action->reference_mm_s)) return 0U;
            budget+=action->run_ms+6000U;continue;
        }
        if (action->kind <= MOTION_RIGHT)
        {
            if ((action->base_rpm < 3U) || (action->base_rpm > 15U) ||
                (action->run_ms == 0U) || (action->run_ms > 8000U) ||
                (action->relative_deg != 0.0f) || action->distance_mm != 0.0f) return 0U;
            budget += action->run_ms + 3500U;
        }
        else if (action->kind == MOTION_TURN)
        {
            if ((action->base_rpm > 5U && !(APP_TURN_BRAKE_TRIAL_ACTIVE &&
                (MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY) && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
                action->base_rpm==APP_TURN_BRAKE_MAX_RPM &&
                (action->relative_deg==90.0f || action->relative_deg==-90.0f))) || action->run_ms || action->distance_mm != 0.0f ||
                !(((action->relative_deg >= 5.0f) && (action->relative_deg <= 90.0f)) ||
                  ((action->relative_deg <= -5.0f) && (action->relative_deg >= -90.0f)))) return 0U;
            budget += APP_HEADING_RUN_MS + 3500U;
        }
        else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        {
            if (!APP_POSITION_DIAGNOSTIC || !Chassis_DistanceSelfTest() ||
                action->base_rpm < 3U || action->run_ms ||
                action->relative_deg != 0.0f ||
                !MotionDistanceAllowed(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)) return 0U;
            if (action->base_rpm>15U &&
                (MOTION_ACTION_COUNT!=1U || g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20)) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= APP_PERIMETER_GROUP_MS) ? 1U : 0U;
}
#if APP_IS_DEBUG
static uint32_t App_DebugSelectPlan(uint32_t id)
{
    MotionAction actions[MOTION_ACTION_CAPACITY];
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t i,count;
    if (!AppDebugPlan_Load(id,actions,&count)) return 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (g_combined_test.state || g_combined_test.request || s_motion_session_open ||
        s_motion_has_action || g_forward_hold.phase != HR_IDLE ||
        g_heading_exec_phase != HR_IDLE || g_forward_hold.request || g_heading_exec_request ||
        g_vm_test_request || power.active || status.active || status.result || status.cut_vm)
    { taskEXIT_CRITICAL(); return 0U; }
    for(i=0U;i<count;i++) s_motion_actions[i]=actions[i];
    s_motion_action_count=count; g_debug_plan_id=id;
    g_motion_plan_valid=App_MotionPlanValid();
    taskEXIT_CRITICAL(); return g_motion_plan_valid;
}
#endif
/* 公共接口只提交动作，CAN任务继续执行原单段闭环；忙时绝不覆盖。 */
uint32_t MotionGetState(void)
{
    uint32_t phase, state;
    taskENTER_CRITICAL();
    phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
    if (!s_motion_has_action) state = MOTION_IDLE;
    else if ((s_combined_turn ? g_heading_exec_request : g_forward_hold.request) == 1U ||
             phase == HR_WAIT || phase == HR_RUN) state = MOTION_BUSY;
    else if (phase == HR_DONE) state = MOTION_DONE;
    else state = (phase == HR_CANCELLED) ? MOTION_CANCELLED : MOTION_FAULT;
    taskEXIT_CRITICAL();
    return state;
}
static uint32_t App_MotionAccept(void)
{
#if APP_FINE_RPM_OFFLINE_ONLY
    return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t state = MotionGetState();
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed()) return 0U;
    if ((state != MOTION_IDLE && state != MOTION_DONE) ||
        (g_combined_test.state >= 3U) ||
        ((g_combined_test.state != 0U) && !s_motion_list_submit) ||
        !App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount()) ||
        !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
        (g_translation_hold_selftest != 1U)) return 0U;
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (!s_motion_session_open)
    {
        if (s_motion_has_action || g_vm_test_request || (power.active && !StartupAxes_Holding()) ||
            !ChassisAuto_BeginSequence()) return 0U;
#if APP_SERIAL_MOTION_CONTROL
        s_motion_timing_count = s_motion_timing_cursor = 0U;
#endif
        s_motion_session_open = 1U;
        if (!s_motion_list_submit) s_combined_begin_ms = (uint32_t)xTaskGetTickCount();
        /* 在同一临界区发布首段VM请求，避免Safety把新组当作断电重置。 */
        g_vm_test_request = 1U;
        /* 联合机构已持VM时不再等待上电；同批发布后端请求，避免先提交后被旧0拒绝。
         * 仅同步本次已通过准入的请求，不写健康标志，不清故障，不在此发CAN。 */
        ChassisAuto_SafetyUpdate(g_vm_test_request);
        s_combined_powered_segment = 1U;
    }
    else
    {
        if (g_vm_test_request != 1U || power.active != 1U ||
            !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)) ||
            !ChassisAuto_ReleaseCompleted(status.session)) return 0U;
        s_combined_powered_segment = 1U;
    }
    s_combined_keep_vm = 1U;
    s_combined_stage_ms = (uint32_t)xTaskGetTickCount();
    return 1U;
#endif
}
static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms, float distance_mm,float route_speed)
{
    ForwardHoldDebug empty = {0};
    uint32_t accepted = 0U;
    if(route_speed!=0.0f) {
        if(!APP_POSITION_DIAGNOSTIC || !s_motion_list_submit || g_debug_plan_id!=DEBUG_PLAN_DEFAULT ||
            !MotionRoute_ShapeAllowed(direction,rpm,run_ms,distance_mm,route_speed) ||
            !Chassis_DistanceSelfTest()) return 0U;
    }
    else if (distance_mm!=0.0f)
    {
        if (!APP_POSITION_DIAGNOSTIC || !MotionDistanceAllowed(direction,rpm,distance_mm) ||
            run_ms!=MotionDistanceRunMsFor(direction,rpm,distance_mm) || !Chassis_DistanceSelfTest()) return 0U;
        if (rpm>15U && (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 || !s_motion_list_submit)) return 0U;
    }
    else if (direction > MOVE_RIGHT || rpm < MOTION_MOVE_MIN_RPM ||
        rpm > MOTION_MOVE_MAX_RPM || !run_ms || run_ms > MOTION_MOVE_MAX_MS) return 0U;
    taskENTER_CRITICAL();
    if (!g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())
    {
        s_combined_turn = 0U; s_combined_mode = direction;
        s_combined_base_rpm = rpm; s_combined_run_ms = run_ms; s_combined_distance_mm = distance_mm;
        s_combined_reference_mm_s=route_speed;
        empty.motion_mode = direction; g_forward_hold = empty;
        HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;
        s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ return App_MoveSubmit(direction,rpm,run_ms,0.0f,0.0f); }
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm)
{
    float mm=distance_cm*10.0f;
    return App_MoveSubmit(direction,max_rpm,MotionDistanceRunMsFor(direction,max_rpm,mm),mm,0.0f);
}
uint32_t MoveForwardDistanceStart(uint32_t max_rpm, float distance_cm)
{ return MoveDistanceStart(MOVE_FORWARD,max_rpm,distance_cm); }
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT==1U && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        HeadingSession_Init(&s_heading_exec);
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t MotionFinish(void)
{
    ChassisAutoStatus status;
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status);
    if (s_motion_session_open && MotionGetState() == MOTION_DONE && status.result == 1U &&
        !status.active && !status.cause && status.rotation_stop_mask == 15U &&
        ((g_combined_test.state == 0U) || s_motion_list_submit))
    {
        s_combined_end_ms = (uint32_t)xTaskGetTickCount();
        s_motion_session_open = s_combined_keep_vm = 0U;
        g_vm_test_request = 0U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
/* 动作表仅安排顺序，同样使用公共单动作入口。 */
static uint32_t App_PerimeterTurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        HeadingSession_Init(&s_heading_exec);
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static uint32_t App_CombinedLoadAction(uint32_t index)
{
    const MotionAction *action;
    uint32_t accepted, load_cycle, i;
    float planned_offset=0.0f;
    if (index >= MOTION_ACTION_COUNT || !MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY ||
        g_debug_plan_id != DEBUG_PLAN_DEFAULT) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset=HeadingController_WrapDeg(planned_offset+s_motion_actions[i].relative_deg);
    if(!RouteHeading_Target(&s_route_heading_reference,planned_offset,&s_route_heading_target)) return 0U;
    action = &s_motion_actions[index];
    s_route_heading_turn_active = action->kind == MOTION_TURN;
    g_route_heading_targets[index]=s_route_heading_target;
    g_route_heading_start_yaw[index]=s_heading_source.yaw_deg;
    s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
    else if (action->kind == MOTION_TURN)
        accepted = App_PerimeterTurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm ? action->base_rpm : 5U);
    else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        accepted = MoveDistanceStart(action->kind - MOTION_FORWARD_DISTANCE, action->base_rpm, action->distance_mm / 10.0f);
    else accepted = MoveStart(action->kind, action->base_rpm, action->run_ms);
    s_motion_list_submit = 0U;
    HWT101_TimingEnd(2U, load_cycle);
    return accepted;
}

static void App_CombinedTestStop(uint32_t cause)
{
    /* STOP是幂等的；已完成/失败不覆盖首因，不清原单段trip。 */
    if (g_combined_test.state >= 3U)
    { g_vm_test_request = 0U; Power_Disable(); return; }
    s_motion_session_open = s_combined_keep_vm = 0U;
    s_combined_end_ms = (uint32_t)xTaskGetTickCount();
    g_combined_test.cause = cause;
    g_combined_test.state = (cause == 1U) ? 5U : 4U;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    if (s_combined_turn) App_HeadingRuntimeTrip(1U);
    else App_ForwardHoldTrip(1U);
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    Power_Disable();
}
/* 独立安全任务也检查整组取消与时限，防止CAN任务等待时漏掉取消。 */
void MotionStop(void)
{
    taskENTER_CRITICAL();
    App_CombinedTestStop(1U);
    taskEXIT_CRITICAL();
}

static void App_CombinedTestSafety(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if ((g_combined_test.state == 1U) || (g_combined_test.state == 2U))
    {
        if (g_combined_test.request == 2U) App_CombinedTestStop(1U);
        else if ((uint32_t)(now - s_combined_begin_ms) >= APP_PERIMETER_GROUP_MS)
            App_CombinedTestStop(2U);
        else if (s_runtime_route.committed.id && s_runtime_route.consumed &&
                 g_combined_test.state==1U && g_combined_test.index<s_runtime_route.committed.count &&
                 (uint32_t)(now-s_combined_stage_ms)>=s_runtime_route.committed.rows[g_combined_test.index].timeout_ms)
            App_CombinedTestStop(2U);
        else if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
                 g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if ((g_combined_test.state == 2U) &&
                 ((g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
                  (g_can_guard_ready != 1U))) App_CombinedTestStop(3U);
        else if (g_combined_test.state == 1U)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety();
            else App_ForwardHoldSafety();
        }
    }
    else if (s_motion_session_open)
    {
        if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if (MotionGetState() == MOTION_BUSY)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety();
            else App_ForwardHoldSafety();
        }
        else if (g_vm_test_request != 1U || g_can_guard_ready != 1U)
            App_CombinedTestStop(3U);
    }
    else { g_vm_test_request = 0U; }
    taskEXIT_CRITICAL();
}
static void App_CombinedTestStep(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t now = (uint32_t)xTaskGetTickCount(), request, phase, index;
    taskENTER_CRITICAL();
    request = g_combined_test.request;
    g_combined_test.request = 0U;
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed())
    {
        s_serial_start_permit=0U;
        g_forward_hold.request=g_heading_exec_request=0U;
        g_chassis_auto_request=g_can_jog_request=0U;
        g_vm_test_request=0U;
        taskEXIT_CRITICAL(); return;
    }
    if (request == 2U && (g_combined_test.state < 3U))
    { App_CombinedTestStop(1U); taskEXIT_CRITICAL(); return; }
    /* 启动请求尚未发给执行器时已取得码：撤销排队，不制造成功路程。 */
    if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
        g_combined_test.state==0U && request==1U && !s_motion_session_open)
    {
        s_serial_start_permit=0U;g_stage1_motion_skipped=1U;
        g_combined_test.state=3U;s_combined_end_ms=now;
        taskEXIT_CRITICAL();return;
    }
    if (g_combined_test.state == 0U && s_motion_session_open)
    {
        /* 独立API不启动动作表，仍由唯一CAN任务服务。 */
        if (MotionGetState() == MOTION_BUSY)
        {
            if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
            { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
            taskEXIT_CRITICAL();
            if (s_combined_turn) App_HeadingRuntimeStep();
            else App_ForwardHoldStep();
            return;
        }
        if (MotionGetState() == MOTION_FAULT || MotionGetState() == MOTION_CANCELLED)
            App_CombinedTestStop(5U);
        taskEXIT_CRITICAL(); return;
    }
    if (g_combined_test.state == 0U)
    {
        /* 其他单段Watch入口在本模式不参与调度。整组每次上电只运行一次。 */
        g_forward_hold.request = 0U; g_heading_exec_request = 0U;
        if (request != 1U) { taskEXIT_CRITICAL(); return; }
        if (s_serial_start_permit != 1U)
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
        (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
        if (status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||
            g_vm_test_request || !App_HeadingRuntimeSource(now) ||
            !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
            (g_translation_hold_selftest != 1U) || (g_motion_plan_valid != 1U) ||
            (g_forward_hold.phase != HR_IDLE) || (g_heading_exec_phase != HR_IDLE))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        if(!RouteHeading_Init(&s_route_heading_reference,s_heading_source.yaw_deg))
        { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        s_combined_powered_segment = 0U;
        s_combined_begin_ms = s_combined_stage_ms = now;
        g_combined_test.index = 0U;
        if (!App_CombinedLoadAction(0U))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if ((g_combined_test.state < 3U) && (g_combined_test.index >= MOTION_ACTION_COUNT))
    { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state >= 3U)
    { g_forward_hold.request = g_heading_exec_request = 0U;
      g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state == 2U)
    {
        /* 段间取得码后直接释放已停稳会话，不再装载下一段。 */
        if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested)
        {
            s_motion_list_submit=1U;(void)MotionFinish();s_motion_list_submit=0U;
            Power_ReadCommandStatus(&power);
            if(!g_vm_test_request && (!power.active || StartupAxes_Holding()))
            {g_combined_test.state=3U;s_combined_end_ms=now;}
            taskEXIT_CRITICAL();return;
        }
        /* 保留上一段诊断，仅本组成功且仍上电的停稳会话允许释放。 */
        if ((VEHICLE_BETWEEN_LEGS_MS && (uint32_t)(now - s_combined_gap_ms) < VEHICLE_BETWEEN_LEGS_MS))
        { taskEXIT_CRITICAL(); return; }
        Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
        if ((g_vm_test_request != 1U) || (power.active != 1U) ||
            status.session != g_combined_test.session[g_combined_test.index])
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.index++;
        index = g_combined_test.index;
        s_combined_powered_segment = 1U;
        s_combined_stage_ms = now;
        if (!App_CombinedLoadAction(index))
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if (g_combined_test.state == 1U)
    {
        if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
        { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
        /* 单段执行可能等待CAN，必须先退出临界区；Safety仍可抢占取消。 */
        taskEXIT_CRITICAL();
        if (s_combined_turn) App_HeadingRuntimeStep();
        else App_ForwardHoldStep();
        taskENTER_CRITICAL();
        if (g_combined_test.state >= 3U)
        { taskEXIT_CRITICAL(); return; }
        phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
        if (phase == HR_FAULT || phase == HR_CANCELLED)
        { App_CombinedTestStop(5U); taskEXIT_CRITICAL(); return; }
        /* 首段实际开始运行后，只展开PA1锁定的一套，不在上电或ARM时动作。 */
        if (g_combined_test.index==0U && phase==HR_RUN && !g_radar_servo_departure_done &&
            (s_combined_turn || g_forward_hold.run_started))
        {
            if (!RadarServo_OnDeparture(RadarZoneSwitch_Lock()))
            { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        }
        if (phase == HR_DONE)
        {
            (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
            /* 正常DONE也会置cut_vm请求断电，此处等待关电而非判故障。 */
            if (status.active || status.state != CHASSIS_AUTO_DONE ||
                status.result != 1U || status.cause || status.rotation_stop_mask != 15U ||
                !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)))
            { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
            if (g_stage1_stop_requested || (g_combined_test.index + 1U) == MOTION_ACTION_COUNT)
            {
                s_motion_list_submit = 1U;
                (void)MotionFinish();
                s_motion_list_submit = 0U;
            }
            /* 等安全任务确实发出关闭命令，不把VM请求0当作PE2已关。 */
            if ((!g_stage1_stop_requested && (g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?
                ((g_vm_test_request == 1U) && (power.active == 1U) && !status.cut_vm) :
                (!g_vm_test_request && (!power.active || StartupAxes_Holding())))
            {
                index = g_combined_test.index;
                g_combined_test.phase[index] = phase;
                g_combined_test.session[index] = status.session;
                g_combined_test.elapsed_ms[index] = now - s_combined_stage_ms;
                g_combined_test.final_yaw[index] = s_heading_source.yaw_deg;
                g_combined_test.final_error[index] = s_combined_turn ?
                    g_heading_exec_error_deg : g_forward_hold.error_deg;
#if APP_SERIAL_MOTION_CONTROL
                if (!s_combined_turn) App_TimingRecordCompleted(index, status.session, now);
#endif
                if(!g_stage1_stop_requested || status.distance_reached) g_combined_test.completed++;
                s_combined_gap_ms = now;
                g_combined_test.state = (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;
                if (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;
            }
        }
    }
    taskEXIT_CRITICAL();
}
#endif

#if APP_SERIAL_MOTION_CONTROL
/* RC1/ TM1：CRC16-CCITT，ARM/START两步，只启动既有单次平移会话。 */
static uint16_t App_SerialCrc(const char *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1));
    }
    return crc;
}
static int32_t App_SerialMilli(float value)
{
    if (!(value >= -2000.0f && value <= 2000.0f)) return (-2147483647L - 1L);
    return (int32_t)(value * 1000.0f);
}
/* 距离独立量纲范围，支持2100mm及原50mm越界诊断；不扩大角度范围。 */
static int32_t App_SerialDistanceMilli(float value)
{
    if (!(value>=-5.0f && value<=2500.0f)) return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static uint32_t App_SerialUInt(const char **text, uint32_t *out)
{
    uint32_t n = 0U, digit;
    const char *p = *text;
    if (*p < '0' || *p > '9') return 0U;
    while (*p >= '0' && *p <= '9')
    {
        digit = (uint32_t)(*p - '0');
        if (n > (0xFFFFFFFFU - digit) / 10U) return 0U;
        n = n * 10U + digit; p++;
    }
    if (*p != ',') return 0U;
    *text = p + 1; *out = n; return 1U;
}
static uint32_t App_SerialIdleReady(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)now;return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    if(!CAN1_EmmSettingsConfirmed())return 0U;
    (void)ChassisAuto_ReadStatus(&status);
    Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if ((g_combined_test.state != 0U) || (g_combined_test.request != 0U) ||
        (g_motion_plan_valid != 1U)) return 0U;
#endif
    return ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 0U) &&
        (g_vm_test_request == 0U) && ((power.active == 0U) || StartupAxes_Holding()) &&
        (status.active == 0U) && (status.result == 0U) && (status.cut_vm == 0U) &&
        (g_translation_hold_selftest == 1U) && (s_serial_proto_ok == 1U) &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        (g_can_ctrl_fault_latched == 0U) &&
        (s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f)) ? 1U : 0U;
#endif
}
/* 调用者持锁；只容忍成功终态的晚到KEEP，不续期、不重新发车。 */
static uint32_t App_SerialCompletedStopped(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    if (g_serial_motion_state != 3U || g_serial_motion_result != 2U) return 0U;
#if APP_TEST_MODE == 3U
    if (g_combined_test.state != 3U || g_combined_test.cause ||
        g_combined_test.completed != MOTION_ACTION_COUNT) return 0U;
#else
    if (g_forward_hold.phase != HR_DONE || g_forward_hold.trip) return 0U;
#endif
    Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
    return (!g_vm_test_request && !power.active && !status.active &&
        status.result == 1U && !status.cause && status.rotation_stop_mask == 15U &&
        !g_can_ctrl_fault_latched && !g_rtos_fault && !g_safety_fault) ? 1U : 0U;
}
static void App_SerialReject(uint32_t reason)
{ g_serial_motion_rejected++; g_serial_motion_result = reason; }

/* 路线上传不启动电机；只能初始待机或正常完成且断VM后修改。 */
static uint32_t App_RuntimeRouteEditable(uint32_t now)
{
    if(g_serial_motion_state==0U && App_SerialIdleReady(now))return 1U;
    return App_SerialCompletedStopped() && !s_motion_session_open &&
        !g_heading_exec_trip && !g_forward_hold.trip &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        !g_serial_motion_link_lost && !g_can_ctrl_fault_latched;
}
static uint32_t App_RuntimeRouteInstall(const RuntimeRoutePrepared *p,uint32_t now)
{
    uint32_t i;ChassisAutoStatus status;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!App_RuntimeRouteEditable(now))return 0U;
    if(g_combined_test.state==3U) {
        (void)ChassisAuto_ReadStatus(&status);
        if(!ChassisAuto_RearmCompleted(status.session))return 0U;
        g_combined_test=combined;g_forward_hold=forward;
        g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
        HeadingSession_Init(&s_heading_exec);
        s_motion_has_action=s_combined_turn=s_combined_powered_segment=0U;
        s_combined_keep_vm=s_serial_start_permit=0U;
        s_serial_token_seq=s_serial_token_tick=s_serial_arm_ms=s_serial_keepalive_ms=0U;
        g_serial_motion_state=g_serial_motion_result=0U;
    }
    for(i=0U;i<p->count;i++)s_motion_actions[i]=p->actions[i];
    s_motion_action_count=p->count;g_debug_plan_id=DEBUG_PLAN_DEFAULT;
    /* 转换器已校验类型、速度、距离、时限和总预算，此处只做有界复制。 */
    g_motion_plan_valid=1U;
    s_route_heading_reference=(RouteHeadingReference){0};
    s_route_heading_turn_active=0U;
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
    memcpy(s_perimeter_targets,p->local_targets,sizeof(s_perimeter_targets));
    RuntimeRoute_Install(&s_runtime_route,p);
    return 1U;
}
static void App_RuntimeRouteCommand(const char *line)
{
    RuntimeRouteMessage m;uint32_t result,now=(uint32_t)xTaskGetTickCount();
    result=RuntimeRoute_Decode(line,&m);
    if(result==RQ_OK) {
        taskENTER_CRITICAL();
        if(m.op==RQ_GO) {
            if(m.seq!=s_serial_token_seq || m.tick!=s_serial_token_tick)result=RQ_STALE;
            else if((g_serial_motion_state==2U || App_SerialCompletedStopped()) &&
                    s_runtime_route.consumed && s_runtime_route.committed.id==m.id &&
                    s_runtime_route.committed.digest==m.digest)result=RQ_DUPLICATE;
            else if(g_serial_motion_state!=1U || (uint32_t)(now-s_serial_arm_ms)>=2000U ||
                    !App_SerialIdleReady(now))result=RQ_BUSY;
            else {
                result=RuntimeRoute_Start(&s_runtime_route,m.id,m.digest);
                if(result==RQ_OK) {
                    s_serial_start_permit=1U;s_serial_keepalive_ms=now;
                    g_combined_test.request=1U;
                    g_serial_motion_state=2U;g_serial_motion_result=2U;
                    g_serial_motion_accepted++;
                }
            }
        } else if(!s_serial_tx_valid || !s_serial_tx_seq ||
                  (uint32_t)(s_serial_tx_seq-1U-m.seq)>5U || (uint32_t)(now-m.tick)>=500U)
            result=RQ_STALE;
        else if(!App_RuntimeRouteEditable(now))result=RQ_BUSY;
        else if(m.op!=RQ_COMMIT)result=RuntimeRoute_Stage(&s_runtime_route,&m,now);
        taskEXIT_CRITICAL();
        if(m.op==RQ_COMMIT && result==RQ_OK) {
            /* 浮点轨迹转换在临界区外完成，不阻塞独立安全任务。 */
            result=RuntimeRoute_Prepare(&s_runtime_route,&m,now,&s_runtime_prepared);
            if(result==RQ_OK) {
                taskENTER_CRITICAL();now=(uint32_t)xTaskGetTickCount();
                if(!s_runtime_route.staging || m.id!=s_runtime_route.stage_id ||
                   (uint32_t)(now-s_runtime_route.stage_ms)>=RQ_UPLOAD_MS)result=RQ_EXPIRED;
                else if(!App_RuntimeRouteInstall(&s_runtime_prepared,now))result=RQ_BUSY;
                taskEXIT_CRITICAL();
            }
        }
    }
    taskENTER_CRITICAL();
    s_runtime_route.last_result=result;
    s_route_ack_id=m.id;s_route_ack_op=m.op;s_route_ack_result=result;s_route_ack_pending=1U;s_route_ack_generation++;
    taskEXIT_CRITICAL();
}
/* RA1/RS1只反馈路线上传和执行，不充当ARM/GO/KEEP健康令牌。 */
static void App_RuntimeRouteTelemetry(void)
{
    static uint32_t last;static char frame[256];
    uint32_t now=(uint32_t)xTaskGetTickCount(),used,values[12],ack,generation,i;int count;
    int32_t progress=0;uint16_t crc;ChassisAutoStatus status;
    taskENTER_CRITICAL();RuntimeRoute_Tick(&s_runtime_route,now);taskEXIT_CRITICAL();
    if(!App_SerialDiagnosticWindow(now,20U))return;
    taskENTER_CRITICAL();
    ack=s_route_ack_pending;generation=s_route_ack_generation;
    if(ack) {
        values[0]=s_route_ack_id;values[1]=s_route_ack_op;values[2]=s_route_ack_result;
        values[3]=s_runtime_route.stage_mask;values[4]=s_runtime_route.committed.id;
        values[5]=s_runtime_route.committed.digest;values[6]=s_runtime_route.committed.count;
        values[7]=s_runtime_route.ready;values[8]=s_runtime_route.consumed;
    } else {
        if((uint32_t)(now-last)<200U || !s_runtime_route.committed.id) {
            taskEXIT_CRITICAL();return;
        }
        values[0]=s_runtime_route.committed.id;values[1]=s_runtime_route.committed.digest;
        values[2]=s_runtime_route.staging?6U:(g_combined_test.state?g_combined_test.state:
            (s_runtime_route.ready?1U:0U));
        if(g_combined_test.state==1U || g_combined_test.state==2U)values[2]=2U;
        values[3]=g_combined_test.index;values[4]=g_combined_test.completed;
        values[5]=s_runtime_route.committed.count;
        values[6]=(g_combined_test.state==1U)?now-s_combined_stage_ms:0U;
        (void)ChassisAuto_ReadStatus(&status);
        if(!s_combined_turn && g_combined_test.state && status.distance_mode)
            progress=App_SerialDistanceMilli(status.distance_progress_mm);
        values[7]=0U;values[8]=g_combined_test.cause;values[9]=s_runtime_route.stage_id;
        values[10]=s_runtime_route.stage_mask;values[11]=s_runtime_route.last_result;
    }
    taskEXIT_CRITICAL();
    /* Format and enqueue outside the control critical section. */
    count=snprintf(frame,sizeof(frame),ack?"RA1,":"RS1,");
    if(count<0)return;
    used=(uint32_t)count;
    for(i=0U;i<(ack?9U:12U);i++) {
        if(!ack && i==7U)count=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)progress);
        else count=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)values[i]);
        if(count<0 || (uint32_t)count>=sizeof(frame)-used-6U)return;
        used+=(uint32_t)count;
    }
    crc=App_SerialCrc(frame,used);
    (void)snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U) {
        if(ack) {
            taskENTER_CRITICAL();
            if(s_route_ack_generation==generation)s_route_ack_pending=0U;
            taskEXIT_CRITICAL();
        } else last=now;
    }
}

static void App_SerialMotionCommand(char *line)
{
    static const volatile uint32_t source_parameter_entry=VEHICLE_INTEGRATED_ENABLE;
    if(source_parameter_entry)return;

    if(FieldMap_LocalMode()) { App_SerialReject(9U);return; }
    if(strncmp(line,"RQ1,",4U)==0) { App_RuntimeRouteCommand(line);return; }
    char *last = strrchr(line, ',');
    const char *p;
    uint32_t seq, tick, crc = 0U, i, digit, now;
    if ((strncmp(line, "RC1,", 4U) != 0) || (last == NULL) || strlen(last + 1) != 4U)
    { App_SerialReject(4U); return; }
    for (i = 1U; i <= 4U; i++)
    {
        char c = last[i];
        if (c >= '0' && c <= '9') digit = (uint32_t)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (uint32_t)(c - 'A') + 10U;
        else { App_SerialReject(4U); return; }
        crc = (crc << 4) | digit;
    }
    if ((uint32_t)App_SerialCrc(line, (uint32_t)(last - line + 1)) != crc)
    { App_SerialReject(4U); return; }
    *last = '\0'; p = line + 4;
    if (!App_SerialUInt(&p, &seq) || !App_SerialUInt(&p, &tick))
    { App_SerialReject(4U); return; }
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if (strcmp(p, "STOP") == 0)
    {
        s_serial_start_permit = 0U;
#if APP_TEST_MODE == 3U
        App_CombinedTestStop(1U);
#else
        if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
            (g_forward_hold.request == 1U)) g_forward_hold.request = 2U;
#endif
        g_serial_motion_state = 3U; g_serial_motion_result = 3U;
        g_serial_motion_accepted++;
    }
#if APP_TEST_MODE == 3U
    else if (strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<=('0'+DEBUG_PLAN_MAX) && p[5]=='\0')
    {
        if (!s_serial_tx_seq || (uint32_t)(s_serial_tx_seq-1U-seq)>5U ||
            (uint32_t)(now-tick)>=500U) App_SerialReject(5U);
        else if (g_serial_motion_state || s_runtime_route.staging || s_runtime_route.committed.id || !App_SerialIdleReady(now) ||
                 !App_DebugSelectPlan((uint32_t)(p[4]-'0'))) App_SerialReject(6U);
        else g_serial_motion_accepted++;
    }
#endif
    else if (strcmp(p, "ARM") == 0)
    {
        if ((s_serial_tx_seq == 0U) || ((uint32_t)(s_serial_tx_seq - 1U - seq) > 5U) ||
            ((uint32_t)(now - tick) >= 500U)) App_SerialReject(5U);
        else if ((g_serial_motion_state != 0U) || !App_SerialIdleReady(now) ||
                 !s_runtime_route.ready || s_runtime_route.staging || s_runtime_route.consumed) App_SerialReject(6U);
        else
        {
            s_serial_token_seq = seq; s_serial_token_tick = tick; s_serial_arm_ms = now;
            g_serial_motion_state = 1U; g_serial_motion_result = 1U;
            g_serial_motion_accepted++;
        }
    }
    else if ((seq != s_serial_token_seq) || (tick != s_serial_token_tick)) App_SerialReject(5U);
    else if (strcmp(p, "START") == 0)
    {
#if APP_TEST_MODE == 3U
        App_SerialReject(6U);
#else
        if (g_serial_motion_state == 2U) { /* 重复帧只确认，不重新启动或续期。 */ }
        else if ((g_serial_motion_state != 1U) || ((uint32_t)(now - s_serial_arm_ms) >= 2000U) ||
            !App_SerialIdleReady(now)) App_SerialReject(6U);
        else
        {
            s_serial_start_permit = 1U; s_serial_keepalive_ms = now;
#if APP_TEST_MODE == 3U
            g_combined_test.request = 1U;
#else
            g_forward_hold.request = 1U;
#endif
            g_serial_motion_state = 2U; g_serial_motion_result = 2U;
            g_serial_motion_accepted++;
        }
#endif
    }
    else if (strcmp(p, "KEEP") == 0)
    {
        if (g_serial_motion_state == 2U)
        { s_serial_keepalive_ms = now; g_serial_motion_accepted++; }
        else if (App_SerialCompletedStopped())
        { g_serial_motion_accepted++; } /* 同令牌已在上方核对；终态只确认，不续期。 */
        else App_SerialReject(6U);
    }
    else App_SerialReject(4U);
    taskEXIT_CRITICAL();
}
static void App_SerialMotionByte(uint8_t data)
{
    static char line[RQ_LINE_MAX];
    static uint32_t used, invalid, last_ms;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - last_ms) >= 200U) { used = 0U; invalid = 0U; }
    last_ms = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if (!invalid && used) { line[used] = '\0'; App_SerialMotionCommand(line); }
        used = invalid = 0U; return;
    }
    if (invalid) return;
    if (data < 32U || data > 126U || used >= sizeof(line) - 1U)
    { invalid = 1U; App_SerialReject(4U); return; }
    line[used++] = (char)data;
}
static void App_SerialMotionSafety(void)
{
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if ((g_serial_motion_state == 1U) && ((uint32_t)(now - s_serial_arm_ms) >= 2000U))
    { g_serial_motion_state = 0U; g_serial_motion_result = 7U; s_serial_start_permit = 0U; }
    if (g_serial_motion_state == 2U)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state >= 3U) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            s_serial_start_permit = 0U; g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_CombinedTestStop(1U);
        }
#else
        if ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 2U))
        {
            s_serial_start_permit = 0U;
            g_serial_motion_state = 3U; g_serial_motion_result = 3U;
            App_ForwardHoldTrip(1U);
        }
        else if (g_forward_hold.phase >= HR_DONE) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            s_serial_start_permit = 0U; g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_ForwardHoldTrip(1U);
        }
#endif
    }
    taskEXIT_CRITICAL();
}
/* 当前状态优先：忙时下一轮重试，只有排队成功才推进100ms周期。
 * 历史诊断不得在状态已到期时抢占单缓冲；不改变主机超时或运动保护。 */
#define APP_SERIAL_STATUS_PERIOD_MS 100U
static uint32_t App_SerialStatusDue(uint32_t now)
{
    return !s_serial_tx_valid ||
        (uint32_t)(now - s_serial_tx_tick) >= APP_SERIAL_STATUS_PERIOD_MS;
}
static uint32_t App_SerialDiagnosticWindow(uint32_t now, uint32_t earliest_ms)
{
    return s_serial_tx_valid && !App_SerialStatusDue(now) &&
        (uint32_t)(now - s_serial_tx_tick) >= earliest_ms;
}
static void App_SerialStatusSend(const char *frame, uint32_t now)
{
    if (App_DebugSendString(frame) != 1U) { s_serial_dropped++; return; }
    s_serial_tx_tick = now;
    s_serial_tx_valid = 1U;
}
#if APP_POSITION_DIAGNOSTIC
/* PE2是带方向的完整历史起止记录，按索引只排队成功发送一次；不当作实时有效位。 */
static void App_PositionEndpointTelemetry(void)
{
    static uint32_t index;
    static char frame[512];
    ChassisPositionEndpoints endpoint;
    uint32_t i, used, phase;
    int count;
    uint16_t crc;
    if (!App_SerialDiagnosticWindow((uint32_t)xTaskGetTickCount(), 40U)) return;
    if (!ChassisAuto_ReadPositionEndpoints(index, &endpoint)) return;
    if (endpoint.outcome == 2U) { index++; return; }
    if (endpoint.outcome != 1U || endpoint.start_mask != 15U || endpoint.end_mask != 15U) return;
    count = snprintf(frame, sizeof(frame), "PE3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)xTaskGetTickCount(), (unsigned long)endpoint.session,
        (unsigned long)endpoint.outcome, (unsigned long)endpoint.start_mask,
        (unsigned long)endpoint.end_mask, (unsigned long)endpoint.translation_mode,
        (unsigned long)endpoint.distance_mode);
    if (count < 0 || (uint32_t)count >= sizeof(frame)) return;
    used = (uint32_t)count;
    for (phase = 0U; phase < 2U; phase++)
        for (i = 0U; i < 4U; i++)
        {
            count = snprintf(frame + used, sizeof(frame) - used, "%lu,%lu,%lu,",
                (unsigned long)(phase ? endpoint.end_sign[i] : endpoint.start_sign[i]),
                (unsigned long)(phase ? endpoint.end_raw[i] : endpoint.start_raw[i]),
                (unsigned long)(phase ? endpoint.end_ms[i] : endpoint.start_ms[i]));
            if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
            used += (uint32_t)count;
        }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    if (App_DebugSendString(frame) == 1U) index++;
}

#if APP_HEADING_SESSION_TEST
/* OD1是只读诊断，1Hz尽力发送；忙则跳过，不参与启动/KEEP判定。 */
static void App_PerimeterTelemetry(void)
{
    static uint32_t cursor,last;static char frame[256];
    PerimeterRecord record;uint32_t now=(uint32_t)xTaskGetTickCount();int n;uint16_t crc;
    if(g_combined_test.state<3U || cursor>8U || now-last<100U ||
       !App_SerialDiagnosticWindow(now,80U)) return;
    taskENTER_CRITICAL();record=g_perimeter_records[cursor];taskEXIT_CRITICAL();
    n=snprintf(frame,sizeof(frame),"OP1,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%ld,%ld,%ld,",
       (unsigned long)cursor,(unsigned long)record.captured,(unsigned long)record.tick_ms,
       (long)record.pose.x_mm,(long)record.pose.y_mm,(long)(record.pose.heading_deg*1000.0f),
       (unsigned long)record.pose.valid,(unsigned long)record.pose.reasons,
       (unsigned long)record.pose.continuity_lost,(unsigned long)record.pose.age_ms,
       (long)s_perimeter_targets[cursor][0],(long)s_perimeter_targets[cursor][1],
       (long)(s_perimeter_targets[cursor][2]*1000.0f));
    if(n<0 || (uint32_t)n>=sizeof(frame)-7U) return;
    crc=App_SerialCrc(frame,(uint32_t)n);
    (void)snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U){cursor++;last=now;}
}

static void App_OdometryTelemetry(void)
{
    static char frame[192];
    static uint32_t last;
    ChassisOdometryStatus status;
    uint32_t now=(uint32_t)xTaskGetTickCount();
    int count;
    uint16_t crc;
    if(now-last<1000U || !App_SerialDiagnosticWindow(now,80U)) return;
    taskENTER_CRITICAL(); status=g_chassis_odometry; taskEXIT_CRITICAL();
    count=snprintf(frame,sizeof(frame),"OD1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%lu,",
        (unsigned long)now,(unsigned long)status.sample_ms,(unsigned long)status.updates,
        (unsigned long)status.valid,(unsigned long)status.reasons,
        (unsigned long)status.continuity_lost,(unsigned long)status.local_origin,
        (long)status.x_mm,(long)status.y_mm,(long)(status.heading_deg*1000.0f),
        (unsigned long)status.age_ms);
    if(count<0 || (uint32_t)count>=sizeof(frame)-7U) return;
    crc=App_SerialCrc(frame,(uint32_t)count);
    (void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U) last=now;
}
#endif
/* PM1与TM1错开至少40ms，静态缓冲不叠加TaskA栈；忙时丢诊断帧。 */
#include "stage1_runtime.inc"
/* WF1：补齐系统辨识需要的应答目标/真实轮速/样本时刻；低优先级观察帧。 */
static void App_WheelLogTelemetry(void)
{
#if VEHICLE_WHEEL_LOG_ENABLE
    static char frame[512];static ChassisWheelLog sample;
    static uint32_t last,last_active,seen_active,sequence,dropped;
    uint32_t now=(uint32_t)xTaskGetTickCount(),i,used;int count;uint16_t crc;
    if (g_chassis_auto_active) {last_active=now;seen_active=1U;}
    if (!seen_active || (!g_chassis_auto_active && (uint32_t)(now-last_active)>1000U) ||
        (uint32_t)(now-last)<VEHICLE_WHEEL_LOG_PERIOD_MS ||
        !App_SerialDiagnosticWindow(now,40U)) return;
    last=now;(void)ChassisAuto_ReadWheelLog(&sample);
    count=snprintf(frame,sizeof(frame),"WF1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)sample.tick_ms,(unsigned long)sample.session,
        (unsigned long)sample.active,(unsigned long)sample.state,(unsigned long)sample.dry_run,
        (unsigned long)sample.speed_valid_mask,(unsigned long)sample.ack_valid_mask,(unsigned long)dropped);
    if(count<0 || (uint32_t)count>=sizeof(frame)) {dropped++;return;}
    used=(uint32_t)count;
    for(i=0U;i<4U;i++) {
        count=snprintf(frame+used,sizeof(frame)-used,"%ld,%ld,%lu,%lu,%lu,",
            (long)sample.acknowledged_decirpm[i],(long)sample.measured_decirpm[i],
            (unsigned long)sample.ack_ms[i],(unsigned long)sample.sample_ms[i],(unsigned long)sample.age_ms[i]);
        if(count<0 || (uint32_t)count>=sizeof(frame)-used) {dropped++;return;}
        used+=(uint32_t)count;
    }
    crc=App_SerialCrc(frame,used);count=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(count<0 || (uint32_t)count>=sizeof(frame)-used || !App_DebugSendString(frame)) dropped++;
#endif
}

static void App_PositionTelemetry(void)
{
    static char frame[512];
    static uint32_t last;
    ChassisPositionSnapshot position;
    uint32_t now = (uint32_t)xTaskGetTickCount(), used, i;
    int count;
    uint16_t crc;
    if ((uint32_t)(now - last) < 200U ||
        !App_SerialDiagnosticWindow(now, 40U)) return;
    last = now;
    (void)ChassisAuto_ReadPositions(&position);
    count = snprintf(frame, sizeof(frame), "PM1,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)now, (unsigned long)position.session,
        (unsigned long)position.sequence, (unsigned long)position.valid_mask,
        (unsigned long)position.vm_active);
    if (count < 0 || (uint32_t)count >= sizeof(frame)) return;
    used = (uint32_t)count;
    for (i = 0U; i < 4U; i++)
    {
        count = snprintf(frame + used, sizeof(frame) - used, "%lu,%lu,%lu,%lu,%lu,",
            (unsigned long)position.result[i], (unsigned long)position.sign[i],
            (unsigned long)position.raw[i], (unsigned long)position.sample_ms[i],
            (unsigned long)position.age_ms[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    (void)App_DebugSendString(frame);
}
#endif

/* 故障后每秒重发，便于另开只读监听；运行中不新增串口负载。 */
static void App_MotionFaultTelemetry(void)
{
    static char frame[512];
    static uint32_t values[MOTION_FAULT_DIAG_FIELDS], last;
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    if (!g_motion_fault_diag_valid || (uint32_t)(now - last) < 1000U ||
        !App_SerialDiagnosticWindow(now, 60U)) return;
    taskENTER_CRITICAL();
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++) values[i] = g_motion_fault_diag[i];
    taskEXIT_CRITICAL();
    memcpy(frame, "HF3,", 4U);
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++)
    {
        count = snprintf(frame + used, sizeof(frame) - used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    if (App_DebugSendString(frame) == 1U) last = now;
}

/* 故障保留CT1；正常完成后发送带段号/会话的CT2，并定期重发成功历史。 */
static void App_MotionTimingTelemetry(void)
{
    static char frame[384];
    static uint32_t values[MOTION_TIMING_RECORD_FIELDS], last;
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U, fields, normal;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    ChassisAutoStatus status;
    if (!App_SerialDiagnosticWindow(now, 70U)) return;
    /* 运动中快速返回，避免为了诊断反复进入临界区复制后端状态。 */
    if (!g_motion_fault_diag_valid)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state != 3U) return;
#else
        if (g_forward_hold.phase != HR_DONE) return;
#endif
    }
    taskENTER_CRITICAL();
    normal = 0U;
    if (g_motion_fault_diag_valid)
    {
        if ((uint32_t)(now-last) < 1000U) { taskEXIT_CRITICAL(); return; }
        fields = HWT_TIMING_FIELDS;
        for (i = 0U; i < fields; i++) values[i] = s_motion_fault_timing[i];
    }
    else
    {
        Power_ReadCommandStatus(&power);
        (void)ChassisAuto_ReadStatus(&status);
#if APP_TEST_MODE == 3U
        if (g_combined_test.state != 3U ||
            g_combined_test.completed != MOTION_ACTION_COUNT)
        { taskEXIT_CRITICAL(); return; }
#else
        if (g_forward_hold.phase != HR_DONE || g_forward_hold.trip)
        { taskEXIT_CRITICAL(); return; }
#endif
        if (g_vm_test_request || power.active || status.active || status.cause ||
            status.result != 1U || status.rotation_stop_mask != 15U)
        { taskEXIT_CRITICAL(); return; }
#if APP_TEST_MODE != 3U
        if (!s_motion_timing_count) App_TimingRecordCompleted(0U, status.session, now);
#endif
        if (!s_motion_timing_count) { taskEXIT_CRITICAL(); return; }
        if (s_motion_timing_cursor >= s_motion_timing_count)
        {
            if ((uint32_t)(now-last) < 1000U) { taskEXIT_CRITICAL(); return; }
            s_motion_timing_cursor = 0U;
        }
        else if ((uint32_t)(now-last) < 100U) { taskEXIT_CRITICAL(); return; }
        fields = MOTION_TIMING_RECORD_FIELDS; normal = 1U;
        for (i = 0U; i < fields; i++) values[i] = s_motion_timing_records[s_motion_timing_cursor][i];
    }
    taskEXIT_CRITICAL();
    memcpy(frame, normal ? "CT2," : "CT1,", 4U);
    for (i = 0U; i < fields; i++)
    {
        count = snprintf(frame+used, sizeof(frame)-used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame+used, sizeof(frame)-used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
    if (App_DebugSendString(frame) == 1U)
    {
        last = now;
        if (normal) s_motion_timing_cursor++;
    }
}

/* BT1仅在VM请求/输出关闭且后端不活动时逐条导出，不在运动中格式化。 */
static void App_BatchDiagnosticTelemetry(void)
{
#if APP_IS_DEBUG
    static char frame[448];
    static ChassisBatchDiagnostic record;
    static ChassisBatchStatistics statistics;
    static uint32_t cursor, last, values[35];
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    if (g_vm_test_request || g_chassis_auto_active ||
        (uint32_t)(now - last) < 100U || !App_SerialDiagnosticWindow(now, 70U)) return;
    Power_ReadCommandStatus(&power);
    if (power.active) return;
    if (!ChassisAuto_ReadBatchDiagnostic(cursor, &record, &statistics))
    {
        if ((uint32_t)(now-last) >= 1000U) cursor = 0U;
        return;
    }
    values[0] = (uint32_t)record.session;
    values[1] = (uint32_t)record.batch_id;
    values[2] = (uint32_t)record.segment;
    values[3] = (uint32_t)record.dry_run;
    values[4] = (uint32_t)record.generation_source;
    values[5] = (uint32_t)record.generated_ms;
    values[6] = (uint32_t)record.start_ms;
    values[7] = (uint32_t)record.end_ms;
    values[8] = (uint32_t)record.outcome;
    values[9] = (uint32_t)record.sent_mask;
    values[10] = (uint32_t)record.ack_mask;
    values[11] = (uint32_t)record.failed_address;
    values[12] = (uint32_t)record.io_result;
    values[13] = (uint32_t)FineRpm_Deci(record.correction_rpm);
    values[14] = (uint32_t)record.base_rpm;
    values[15] = (uint32_t)record.pending_changes;
    values[16] = (uint32_t)FineRpm_Deci(record.pending_rpm);
    values[17] = (uint32_t)record.send_ms[0];
    values[18] = (uint32_t)record.send_ms[1];
    values[19] = (uint32_t)record.send_ms[2];
    values[20] = (uint32_t)record.send_ms[3];
    values[21] = (uint32_t)record.reply_ms[0];
    values[22] = (uint32_t)record.reply_ms[1];
    values[23] = (uint32_t)record.reply_ms[2];
    values[24] = (uint32_t)record.reply_ms[3];
    values[25] = (uint32_t)record.result[0];
    values[26] = (uint32_t)record.result[1];
    values[27] = (uint32_t)record.result[2];
    values[28] = (uint32_t)record.result[3];
    values[29] = statistics.overwritten;
    values[30] = statistics.max_queue_ms;
    values[31] = statistics.max_batch_ms;
    values[32] = statistics.max_call_ms;
    values[33] = statistics.total_success;
    values[34] = statistics.total_interrupted;
    memcpy(frame, "BTQ,", 4U);
    for (i = 0U; i < 35U; i++)
    {
        if (i == 13U || i == 16U)
            count = snprintf(frame+used, sizeof(frame)-used, "%ld,", (long)(int32_t)values[i]);
        else count = snprintf(frame+used, sizeof(frame)-used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame+used, sizeof(frame)-used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
    if (App_DebugSendString(frame) == 1U) { cursor++; last = now; }
#endif
}


static void App_ChassisLoadTelemetry(void)
{
    static uint32_t cursor=0U,session=0U,last=0U;
    static char frame[512];
    ChassisLoadRecord record;ChassisLoadStatistics stats;PowerCommandStatus power;
    uint32_t now=xTaskGetTickCount(),n,i,j,used;int count;uint16_t crc;
    const uint8_t *bytes;static const char hex[]="0123456789ABCDEF";
    Power_ReadCommandStatus(&power);
    if(g_chassis_auto_active || power.active || g_vm_test_request || (uint32_t)(now-last)<70U ||
       !App_SerialDiagnosticWindow(now,40U))return;
    if(!ChassisLoad_Read(0U,&record,&stats))return;
    if(session!=stats.session){session=stats.session;cursor=0U;}
    if(cursor>=stats.count)return;
    n=stats.count-cursor;if(n>8U)n=8U;
    count=snprintf(frame,sizeof(frame),"XD1,%lu,%lu,%lu,%lu,%lu,",(unsigned long)session,
        (unsigned long)cursor,(unsigned long)n,(unsigned long)stats.count,(unsigned long)stats.dropped);
    if(count<0 || (uint32_t)count>=sizeof(frame))return;used=(uint32_t)count;
    for(i=0U;i<n;i++){
        if(!ChassisLoad_Read(cursor+i,&record,&stats))return;
        bytes=(const uint8_t*)&record;
        for(j=0U;j<sizeof(record);j++){if(used+2U>=sizeof(frame)-8U)return;frame[used++]=hex[bytes[j]>>4];frame[used++]=hex[bytes[j]&15U];}
    }
    frame[used++]=',';crc=App_SerialCrc(frame,used);
    count=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(count<0 || (uint32_t)count>=sizeof(frame)-used)return;
    if(App_DebugSendString(frame)==1U){cursor+=n;last=now;}
}

static void App_SerialMotionTelemetry(void)
{
    static int32_t values[(APP_TEST_MODE == 3U) ? 66U : 49U];
    static char frame[768];
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    ChassisAutoStatus status;
    if (!App_SerialStatusDue(now)) return;
    taskENTER_CRITICAL();
    Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
    values[0] = (int32_t)(s_serial_tx_seq++); /* seq */
    values[1] = (int32_t)(now); /* tick_ms */
    values[2] = (int32_t)(g_forward_hold.phase); /* phase */
    values[3] = (int32_t)(g_forward_hold.trip); /* trip */
    values[4] = (int32_t)(g_forward_hold.elapsed_ms); /* elapsed_ms */
    values[5] = (int32_t)(App_HeadingRuntimeSource(now)); /* source_valid */
    values[6] = (int32_t)(App_SerialMilli(s_heading_source.yaw_deg)); /* yaw_mdeg */
    values[7] = (int32_t)(App_SerialMilli(g_forward_hold.target_deg)); /* target_mdeg */
    values[8] = (int32_t)(App_SerialMilli(g_forward_hold.error_deg)); /* error_mdeg */
    values[9] = (int32_t)(App_SerialMilli(s_heading_source.rate_dps)); /* rate_mdps */
    values[10] = (int32_t)(App_HeadingFrameAge(&s_heading_source,now,1U)); /* angle_age_ms */
    values[11] = (int32_t)(App_HeadingFrameAge(&s_heading_source,now,2U)); /* rate_age_ms */
    values[12] = FineRpm_Deci(g_forward_hold.correction_rpm); /* correction_rpm */
    values[13] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[0]); /* rpm1 */
    values[14] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[1]); /* rpm2 */
    values[15] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[2]); /* rpm3 */
    values[16] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[3]); /* rpm4 */
    values[17] = (int32_t)(g_vm_test_request); /* vm_request */
    values[18] = (int32_t)(power.active); /* pe2_active */
    values[19] = (int32_t)(status.state); /* backend_state */
    values[20] = (int32_t)(status.active); /* backend_active */
    values[21] = (int32_t)(status.result); /* backend_result */
    values[22] = (int32_t)(status.cause); /* backend_cause */
    values[23] = (int32_t)(status.rotation_stop_mask); /* stop_mask */
    values[24] = (int32_t)(g_can_ctrl_fault_latched); /* can_fault */
    values[25] = (int32_t)(g_rtos_fault); /* rtos_fault */
    values[26] = (int32_t)(g_safety_fault); /* safety_fault */
    values[27] = (int32_t)(g_forward_hold.motion_mode); /* motion_mode */
    values[28] = (int32_t)(g_forward_hold.correction_nonzero_samples); /* nonzero_samples */
    values[29] = FineRpm_Deci(g_forward_hold.correction_peak_abs_rpm); /* peak_abs_rpm */
    values[30] = FineRpm_Deci(g_forward_hold.correction_last_nonzero_rpm); /* last_nonzero_rpm */
    values[31] = (int32_t)(g_serial_motion_state); /* remote_state */
    values[32] = (int32_t)(g_serial_motion_result); /* remote_result */
    values[33] = (int32_t)(s_serial_token_seq); /* token_seq */
    values[34] = (int32_t)(s_serial_token_tick); /* token_tick */
    values[35] = (int32_t)(s_serial_dropped); /* telemetry_dropped */
    values[36] = (int32_t)(g_serial_motion_rejected); /* remote_rejected */
    values[37] = (int32_t)(g_translation_hold_selftest); /* selftest */
    values[38] = (int32_t)((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM); /* base_rpm */
    values[39] = (int32_t)((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM : APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM); /* feedback_limit_rpm */
    values[40] = (int32_t)(App_SerialMilli(g_forward_hold.peak_error_deg)); /* peak_error_mdeg */
    values[41] = (int32_t)(g_forward_hold.max_sample_gap_ms); /* max_sample_gap_ms */
    values[42] = (int32_t)(g_uart1_rx_errors); /* uart_rx_errors */
    values[43] = (int32_t)(g_rx_stack_words); /* rx_stack_words */
    values[44] = (int32_t)(g_heap_min_bytes); /* heap_min_bytes */
    values[45] = (int32_t)(g_serial_motion_link_lost); /* link_lost */
    values[46] = (int32_t)(s_serial_proto_ok); /* protocol_selftest */
    values[47] = (int32_t)(g_serial_motion_accepted); /* remote_accepted */
#if APP_TEST_MODE == 3U
    /* 整组阶段独立于单段DONE，避免电脑在第一段完成时误判整组完成。 */
    values[2] = (g_combined_test.state == 0U) ? 0 :
        ((g_combined_test.state >= 3U) ? (int32_t)g_combined_test.state : 2);
    values[3] = (int32_t)g_combined_test.cause;
    values[4] = (g_combined_test.state == 0U) ? 0 : (int32_t)(((g_combined_test.state >= 3U) ? s_combined_end_ms : now) - s_combined_begin_ms);
    values[7] = App_SerialMilli(s_combined_turn ? g_heading_exec_target_deg : g_forward_hold.target_deg);
    values[8] = App_SerialMilli(s_combined_turn ? g_heading_exec_error_deg : g_forward_hold.error_deg);
    values[12] = FineRpm_Deci(s_combined_turn ? g_heading_exec_rpm : g_forward_hold.correction_rpm);
    values[27] = 3; /* 编译为组合测试，主机必须显式--mode 3 */
    values[38] = (int32_t)((g_combined_test.state == 0U) ?
        ((s_motion_actions[0].kind == MOTION_TURN) ? 15U : s_motion_actions[0].base_rpm) :
        (s_combined_turn ? 15U : s_combined_base_rpm));
    if (APP_SPEED20_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_20)
        values[39]=(int32_t)APP_SPEED20_FEEDBACK_RPM;
    if(APP_ROUTE_SPEED_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT)
        values[39]=(int32_t)ROUTE_TRIAL_FEEDBACK_RPM;
    if(APP_TURN_BRAKE_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
       ((g_combined_test.state==0U && s_motion_actions[0].kind==MOTION_TURN) || s_combined_turn)) {
        values[38]=(int32_t)APP_TURN_BRAKE_MAX_RPM;
        values[39]=(int32_t)APP_TURN_BRAKE_FEEDBACK_RPM;
    }
    values[48] = (int32_t)g_combined_test.state;
    values[49] = (int32_t)g_combined_test.index;
    values[50] = (int32_t)g_combined_test.completed;
    values[51] = (int32_t)g_can_guard_cause;
    values[52] = (int32_t)g_can_guard_bad_nodes;
    values[53] = (int32_t)g_heading_power_wait_state;
    values[54] = (int32_t)g_heading_power_post_settle_mask;
    values[55] = (int32_t)MOTION_ACTION_COUNT;
    values[58] = (int32_t)status.distance_mode;
    values[59] = App_SerialDistanceMilli(status.distance_target_mm);
    values[60] = App_SerialDistanceMilli(status.distance_progress_mm);
    values[61] = (int32_t)status.distance_base_rpm;
    values[62] = (int32_t)status.distance_reached;
    values[63] = (int32_t)status.distance_fault_reason;

    values[56] = (int32_t)(s_combined_turn ? MOTION_TURN :
        ((App_StageDistanceMm() > 0.0f) ? (MOTION_FORWARD_DISTANCE + App_TestTranslationMode()) : App_TestTranslationMode()));
    if (status.distance_mode && (status.distance_base_rpm >= 3U || status.distance_profile_mode))
        values[38] = (int32_t)status.distance_base_rpm;
    values[57] = s_combined_turn ? (-2147483647L - 1L) :
        App_SerialMilli(MotionRpmToCmPerSec((float)values[38]));
    /* 新候选字段显示参考速度上限，不能把含2RPM余量的基速当成参考速度。 */
    if(APP_ROUTE_SPEED_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT && !s_combined_turn)
        values[57]=App_SerialMilli(((g_combined_test.state==0U) ?
            s_motion_actions[0].reference_mm_s : s_combined_reference_mm_s)/10.0f);
    if (g_motion_plan_valid != 1U) values[37] = 2;
    if (g_combined_test.state == 0U && s_motion_has_action)
    {
        uint32_t state = MotionGetState();
        values[2] = (state == MOTION_BUSY) ? 2 :
            ((state == MOTION_DONE) ? 3 : ((state == MOTION_CANCELLED) ? 5 : 4));
        values[3] = s_combined_turn ? (int32_t)g_heading_exec_trip : (int32_t)g_forward_hold.trip;
        values[4] = (int32_t)((s_motion_session_open ? now : s_combined_end_ms) - s_combined_begin_ms);
        values[48] = values[2]; values[49] = 0;
        values[50] = (state == MOTION_DONE) ? 1 : 0;
        values[55] = 1;
        if (g_translation_hold_selftest == 1U) values[37] = 1;
    }
#endif
    values[(APP_TEST_MODE == 3U) ? 64U : 48U] = (int32_t)g_can_task_stack_words; /* CAN生命周期最低剩余words */
#if APP_TEST_MODE == 3U
    values[65]=(int32_t)g_debug_plan_id;
#endif
    taskEXIT_CRITICAL();
    frame[0] = 'T'; frame[1] = 'M'; frame[2] = 'Q'; frame[3] = ',';
    for (i = 0U; i < ((APP_TEST_MODE == 3U) ? 66U : 49U); i++)
    {
        count = snprintf(&frame[used], sizeof(frame) - used, "%ld,", (long)values[i]);
        if ((count < 0) || ((uint32_t)count >= sizeof(frame) - used)) { s_serial_dropped++; return; }
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(&frame[used], sizeof(frame) - used, "%04X\r\n", (unsigned int)crc);
    if ((count < 0) || ((uint32_t)count >= sizeof(frame) - used)) { s_serial_dropped++; return; }
    App_SerialStatusSend(frame, now);
}
#endif

#endif

#if APP_TEST_MODE != 3U
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ (void)direction; (void)rpm; (void)run_ms; return 0U; }
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{ (void)direction; (void)angle_deg; (void)max_rpm; return 0U; }
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm)
{ (void)direction; (void)max_rpm; (void)distance_cm; return 0U; }
uint32_t MoveForwardDistanceStart(uint32_t max_rpm, float distance_cm)
{ (void)max_rpm; (void)distance_cm; return 0U; }
uint32_t MotionGetState(void) { return MOTION_UNAVAILABLE; }
void MotionStop(void) { g_vm_test_request = 0U; Power_Disable(); }
uint32_t MotionFinish(void) { return 0U; }
#endif
/* 上层读取同一时刻的动作/电源/故障快照，不等待总线。 */
uint32_t MotionReadStatus(MotionStatus *status)
{
    MotionStatus snapshot = {0};
    ChassisAutoStatus backend;
    PowerCommandStatus power;
    if (status == NULL) return 0U;
    taskENTER_CRITICAL();
    snapshot.state = MotionGetState();
#if APP_TEST_MODE == 3U
    snapshot.session_open = s_motion_session_open;
#else
    snapshot.session_open = 0U;
#endif
    (void)ChassisAuto_ReadStatus(&backend);
    Power_ReadCommandStatus(&power);
    snapshot.vm_requested = g_vm_test_request;
    snapshot.vm_output_active = power.active;
    snapshot.backend_active = backend.active;
    snapshot.backend_result = backend.result;
    snapshot.backend_cause = backend.cause;
    snapshot.stop_mask = backend.rotation_stop_mask;
    snapshot.can_fault = g_can_ctrl_fault_latched;
    snapshot.rtos_fault = g_rtos_fault;
    snapshot.safety_fault = g_safety_fault;
#if APP_TEST_MODE == 3U
    snapshot.distance_mode = (s_motion_has_action && !s_combined_turn &&
        s_combined_distance_mm > 0.0f) ? 1U : 0U;
    if (snapshot.distance_mode)
    {
        snapshot.distance_direction = s_combined_mode;
        snapshot.distance_target_cm = s_combined_distance_mm / 10.0f;
        /* 排队时底层可能仍有上一段结果，不能嫁接为当前进度。 */
        if (backend.distance_mode && App_ForwardHoldOwns(&backend))
        {
            snapshot.distance_progress_cm = backend.distance_progress_mm / 10.0f;
            snapshot.distance_reached = backend.distance_reached;
            snapshot.distance_base_rpm = backend.distance_base_rpm;
            snapshot.distance_fault_reason = backend.distance_fault_reason;
        }
    }
#endif
    taskEXIT_CRITICAL();
    *status = snapshot;
    return 1U;
}
float MotionRpmToCmPerSec(float rpm)
{
    if (!(rpm >= -65535.0f && rpm <= 65535.0f)) return 0.0f;
    return rpm * 3.14159265358979323846f * g_chassis_geometry.wheel_diameter_mm / 600.0f;
}

#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
/* CAN任务独占计算状态；TaskA只复制小型只读结果。 */
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
/* 调试Watch：先填三个目标，最后写request=1；仅VM关闭且无运动时受理。
 * result:1接受，2条件不足；CAN任务消费一次，不等待条件后自动执行。 */
#if APP_IS_DEBUG
volatile uint32_t g_odometry_pose_request=0U, g_odometry_pose_result=0U;
volatile float g_odometry_pose_x_mm=0.0f, g_odometry_pose_y_mm=0.0f;
volatile float g_odometry_pose_heading_deg=0.0f;
#endif
static void App_OdometryStep(void)
{
    AppHeadingSource source;
    ChassisOdometrySamples samples;
    uint32_t i,now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    taskENTER_CRITICAL(); source=s_heading_source; taskEXIT_CRITICAL();
#if APP_IS_DEBUG
    if(g_odometry_pose_request==1U) {
        PowerCommandStatus power;
        float x,y,heading;
        taskENTER_CRITICAL();
        x=g_odometry_pose_x_mm; y=g_odometry_pose_y_mm;
        heading=g_odometry_pose_heading_deg; g_odometry_pose_request=0U;
        taskEXIT_CRITICAL();
        Power_ReadCommandStatus(&power);
        g_odometry_pose_result=2U;
        if(!g_vm_test_request && !power.active && !g_chassis_auto_active &&
           !g_wheel_test_active && !s_can_jog_preparing && source.healthy &&
           App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS &&
           !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)
            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
                g_odometry_pose_result=1U;
    }
#endif
    if(!s_perimeter_odom_started && g_combined_test.state==1U && source.healthy) {
        if(s_route_heading_reference.valid && ChassisOdometry_SetPose(&s_odometry,
            (VEHICLE_STAGE1_ENABLE || s_runtime_route.committed.id)?0.0f:2100.0f,(VEHICLE_STAGE1_ENABLE || s_runtime_route.committed.id)?0.0f:2100.0f,
            (VEHICLE_STAGE1_ENABLE || s_runtime_route.committed.id)?0.0f:-90.0f,s_route_heading_reference.origin_deg))
            s_perimeter_odom_started=1U;
    }
    ChassisAuto_ReadOdometrySamples(&samples);
    if(source.healthy && App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS)
        ChassisOdometry_Yaw(&s_odometry,source.yaw_deg,source.angle_ms);
    else ChassisOdometry_Invalidate(&s_odometry);
    for(i=0U;i<4U;i++)
        if(samples.mask & (1UL<<i))
            ChassisOdometry_Wheel(&s_odometry,i,samples.sign[i],samples.raw[i],samples.ms[i]);
    if(samples.mask!=15U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)
        ChassisOdometry_Invalidate(&s_odometry);
    ChassisOdometry_Update(&s_odometry,now);
    taskENTER_CRITICAL();
    g_chassis_odometry=s_odometry.status;
    if(s_perimeter_odom_started && !g_perimeter_records[0].captured && s_odometry.status.valid &&
       g_combined_test.completed==0U) {
        g_perimeter_records[0].captured=1U;g_perimeter_records[0].tick_ms=now;
        g_perimeter_records[0].pose=s_odometry.status;
    }
    if(g_combined_test.completed>s_perimeter_completed_seen && g_combined_test.completed<=8U) {
        uint32_t slot=g_combined_test.completed;
        g_perimeter_records[slot].captured=1U;g_perimeter_records[slot].tick_ms=now;
        g_perimeter_records[slot].pose=s_odometry.status;
        s_perimeter_completed_seen=slot;
    }
    taskEXIT_CRITICAL();
}
#endif

static void App_ComboIdlePoll(void)
{
    static uint32_t position_turn;
    uint32_t combined = (g_combined_test.state==1U || g_combined_test.state==2U);
    if(combined && position_turn) {
        position_turn=0U;
        ChassisAuto_OdometryPoll();
    } else {
        position_turn=combined;
        App_CanFeedbackStep();
    }
}

static void App_CanTask(void *argument)
{
    (void)argument;
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
    ChassisOdometry_Init(&s_odometry);
#endif
    g_can_test_result = CAN1_LoopbackTest();
    while (1)
    {
        g_can_task_count++;
        if (APP_UART_LINK_TEST != 0U)
        {
            g_vm_test_request = 0U;
            Power_Disable();
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
            Safety_MotionFeedbackReport(0U, 0U, 0U, NULL, NULL);
        }
        else
        {
#if APP_SERIAL_MOTION_CONTROL
        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();
#endif
        #if APP_HEADING_SESSION_TEST
                #if APP_TEST_MODE == 3U
                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
                if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && (VehicleArm_Busy() || g_vehicle_arm_stop_pending))VehicleArm_Service();
                if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_DM_STARTUP_ONLY && !VEHICLE_STARTUP_GROUP_ONLY && !VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();
#else
                if (APP_FORWARD_HOLD_TEST != 0U) App_ForwardHoldStep();
                else App_HeadingRuntimeStep();
#endif
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
                if (g_chassis_auto_active == 0U) {
                    App_ComboIdlePoll();
                    if(!VEHICLE_EXTENSION_HOME_ONLY && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                }
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
        } /* 串口测试之外才允许原CAN交易路径 */
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
        App_OdometryStep();
#endif
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_UART_LINK_TEST ? 50U : (APP_HEADING_SESSION_TEST ? 10U : 50U)));
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
    return ((g_hwt_version_result == 1U) && (value.valid_mask == 3U) &&
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

#if VEHICLE_INTEGRATED_ENABLE
#include "vehicle_mission.inc"
#endif
/* 任务A：每100毫秒计数一次 */
/* 独立串口测试任务：替代业务任务A，不执行扫码显示或运动门控。 */
static void App_UartLinkTestTask(void *argument)
{
    (void)argument;
    if (APP_IS_DEBUG) USART1_EnableReceive();
    HWT101_EnableReceive();
    Buzzer_Start(APP_INIT_BEEP_COUNT);
    while (1)
    {
        HWT101_Update();
        Buzzer_Update();
        g_test_count_a++;
        g_test_tick = xTaskGetTickCount();
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 完成真实接收解析后报告进展，原安全监督和看门狗不变。 */
        Safety_Report(SAFETY_TASK_APP);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    if (APP_IS_DEBUG) USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    SAOMA_EnableReceive();
    HWT101_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
    /* 已验收的基础平台就绪后，只请求一次提示音 */
    Buzzer_Start(APP_INIT_BEEP_COUNT);
        /* 一次性虚拟角度自检，不驱动小车。 */
    App_HeadingDryRun();
#if APP_SERIAL_MOTION_CONTROL
    s_serial_proto_ok = (App_SerialCrc("123456789", 9U) == 0x29B1U) ? 1U : 0U;
    RuntimeRoute_Init(&s_runtime_route);
#endif
#if APP_HEADING_SESSION_TEST && APP_FORWARD_HOLD_TEST
    /* 只运行数学自检，失败禁止平移启动，默认不发动作。 */
    g_translation_hold_selftest = Chassis_TranslationHoldSelfTest() ? 1U : 2U;
#if APP_TEST_MODE == 3U
    if (APP_IS_DEBUG)
    {
        (void)AppDebugPlan_Load(DEBUG_PLAN_DEFAULT,s_motion_actions,&s_motion_action_count);
        g_motion_plan_valid = App_MotionPlanValid();
    }
#endif
#endif
    if(!VEHICLE_INTEGRATED_ENABLE && VEHICLE_STAGE1_ENABLE && !FieldMap_Stage1(&s_stage1_plan))
        {g_stage1_state=4U;g_stage1_cause=10U;}
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
        if(VEHICLE_INTEGRATED_ENABLE){VehicleMission_Service();VehicleMission_Telemetry();}
        else if(VEHICLE_STAGE1_ENABLE)
        {
            App_Stage1Service();
            App_Stage1Display((TickType_t)(xTaskGetTickCount()-screen_start_tick)>=pdMS_TO_TICKS(1000U));
        }
        else
        {
        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */
        if (g_start_request != 0U)
        {
            SAOMA_DataReady = 0U;
        }
        if (SAOMA_DataReady != 0U &&
            1U &&
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
        }

        App_StartGateUpdate();
#if APP_SERIAL_MOTION_CONTROL
        /* 先尝试当前状态；诊断只用成功状态之后的空档。 */
        App_SerialMotionTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_Stage1Telemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_Stage1CodeTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_RuntimeRouteTelemetry();
#if APP_POSITION_DIAGNOSTIC
        App_PositionEndpointTelemetry();
#endif
        App_MotionFaultTelemetry();
        App_MotionTimingTelemetry();
        App_BatchDiagnosticTelemetry();
        App_ChassisLoadTelemetry();
#if APP_POSITION_DIAGNOSTIC
        App_WheelLogTelemetry();
        App_PositionTelemetry();
#if APP_HEADING_SESSION_TEST
        App_OdometryTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_PerimeterTelemetry();
#endif
#endif
#endif
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

#if APP_UART_LINK_TEST
/* 仅测试数据：无运动命令、无参数写入、无VM允许。 */
volatile uint32_t g_uart_link_test_value = 0U;
volatile uint32_t g_uart_link_rx_lines = 0U, g_uart_link_tx_replies = 0U;
volatile uint32_t g_uart_link_errors = 0U, g_uart_link_dropped = 0U;
volatile uint32_t g_uart_link_hb_count = 0U;
static char s_link_line[48], s_link_reply[64];
static uint32_t s_link_used, s_link_overflow, s_link_pending;
static uint32_t s_link_last_byte, s_link_hb_tick, s_link_yaw_pending;

static void App_UartLinkCommand(void)
{
    const char *digit;
    uint32_t value = 0U, valid = 1U;
    g_uart_link_rx_lines++;
    if (s_link_pending != 0U) { g_uart_link_dropped++; return; }
    if ((strncmp(s_link_line, "PING,", 5U) == 0) &&
        (strlen(s_link_line + 5U) > 0U) && (strlen(s_link_line + 5U) <= 32U))
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "PONG,%s\r\n", s_link_line + 5U);
    else if (strncmp(s_link_line, "ECHO,", 5U) == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "ECHO,%s\r\n", s_link_line + 5U);
    else if (strcmp(s_link_line, "INFO") == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "INFO,UART1,115200,8N1,VM0\r\n");
    else if (strcmp(s_link_line, "GET") == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "VALUE,%lu\r\n", (unsigned long)g_uart_link_test_value);
    else if (strncmp(s_link_line, "SET,", 4U) == 0)
    {
        digit = s_link_line + 4U;
        if (*digit == '\0') valid = 0U;
        while (*digit != '\0')
        {
            if ((*digit < '0') || (*digit > '9')) { valid = 0U; break; }
            value = value * 10U + (uint32_t)(*digit - '0');
            if (value > 1000U) { valid = 0U; break; }
            digit++;
        }
        if (valid != 0U)
        {
            g_uart_link_test_value = value;
            (void)snprintf(s_link_reply, sizeof(s_link_reply), "VALUE,%lu\r\n", (unsigned long)value);
        }
        else
        {
            g_uart_link_errors++;
            (void)snprintf(s_link_reply, sizeof(s_link_reply), "ERR,RANGE\r\n");
        }
    }
    else
    {
        g_uart_link_errors++;
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "ERR,COMMAND\r\n");
    }
    s_link_pending = 1U;
}

static void App_UartLinkByte(uint8_t data)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - s_link_last_byte) >= 500U)
    { s_link_used = 0U; s_link_overflow = 0U; }
    s_link_last_byte = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if ((s_link_overflow == 0U) && (s_link_used > 0U))
        {
            s_link_line[s_link_used] = '\0';
            App_UartLinkCommand();
        }
        s_link_used = 0U; s_link_overflow = 0U;
        return;
    }
    if (s_link_overflow != 0U) return;
    if ((data < 32U) || (data > 126U) || (s_link_used >= sizeof(s_link_line) - 1U))
    { g_uart_link_errors++; s_link_overflow = 1U; return; }
    s_link_line[s_link_used++] = (char)data;
}

static void App_UartLinkPoll(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    char heartbeat[64];
    if (s_link_pending != 0U)
    {
        if (App_DebugSendString(s_link_reply) == 1U)
        { s_link_pending = 0U; g_uart_link_tx_replies++; }
        return;
    }
    /* 与命令回复共用同一个发送入口，避免两个任务竞争串口。 */
    if (s_link_yaw_pending != 0U)
    {
        float yaw;
        uint32_t valid, age;
        taskENTER_CRITICAL();
        yaw = g_hwt_yaw_deg;
        valid = g_hwt_valid_mask;
        age = now - g_hwt_update_tick;
        taskEXIT_CRITICAL();
        /* 定点毫度，避免依赖浮点printf；无效值用哨兵表示。 */
        (void)snprintf(heartbeat, sizeof(heartbeat), "LINK,YAW,%ld,%lu,%lu\r\n",
            (((yaw >= -180.0f) && (yaw <= 180.0f)) ?
                (long)(yaw * 1000.0f) : (-2147483647L - 1L)),
            (unsigned long)valid, (unsigned long)age);
        if (App_DebugSendString(heartbeat) == 1U) s_link_yaw_pending = 0U;
        return;
    }
    if ((uint32_t)(now - s_link_hb_tick) >= 1000U)
    {
        (void)snprintf(heartbeat, sizeof(heartbeat), "LINK,HB,%lu,%lu,VM0\r\n",
            (unsigned long)g_uart_link_hb_count, (unsigned long)now);
        if (App_DebugSendString(heartbeat) == 1U)
        { g_uart_link_hb_count++; s_link_hb_tick = now; s_link_yaw_pending = 1U; }
    }
}
#endif

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
#if APP_UART_LINK_TEST
            App_UartLinkByte(data);
#elif APP_SERIAL_MOTION_CONTROL
            App_SerialMotionByte(data);
#endif
        }
        else
        {
            /* 空闲时记录历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
#if APP_UART_LINK_TEST
        App_UartLinkPoll();
#endif
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
        RadarZoneSwitch_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));
        RadarLift_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS),
            (g_vm_test_request==0U && g_chassis_auto_active==0U &&
             g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
        RadarServo_DebugService((g_vm_test_request==0U && g_chassis_auto_active==0U &&
            g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
/* Armed servo test consumes PC13 without starting the chassis. */
        if(VEHICLE_INTEGRATED_ENABLE)
        {if(Key_GetEvent()!=0U){taskENTER_CRITICAL();g_vehicle_key=1U;taskEXIT_CRITICAL();}}
        else if (VEHICLE_STAGE1_ENABLE)
        {
            if(Key_GetEvent()!=0U) {
                g_start_key_count++;taskENTER_CRITICAL();
                if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U) {
                    g_stage1_key_request=2U;
                    if(g_stage1_state==7U) StartupAxes_Cancel();
                    else if(g_stage1_state==1U || g_stage1_state==6U) g_combined_test.request=2U;
                } else g_stage1_key_request=1U;
                taskEXIT_CRITICAL();
            }
        }
        else if (g_radar_servo_key_test_enable != 0U)
        {
            if (Key_GetEvent() != 0U)
            {
                g_start_key_count++;
                taskENTER_CRITICAL();
                (void)RadarServo_KeyTest(g_radar_zone_ready?g_radar_zone_selected:0U,
                    (g_vm_test_request==0U && g_chassis_auto_active==0U &&
                     g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
                taskEXIT_CRITICAL();
            }
        }
        else if (APP_UART_LINK_TEST != 0U)
        { if (Key_GetEvent() != 0U) g_start_key_count++; }
        else
        {
#if APP_IS_COMPETITION
        if (Key_GetEvent()!=0U)
        {
            g_start_key_count++;
            taskENTER_CRITICAL();
            if (g_start_gate_ready && !g_start_request &&
                g_competition_entry.state==COMP_ENTRY_IDLE)
            { s_start_pending_revision=s_start_gate_revision; s_start_pending=1U; }
            else g_start_deny_count++;
            taskEXIT_CRITICAL();
        }
#elif APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* PC13只取消，不用按键启动本次开发试验。 */
            taskENTER_CRITICAL();
#if APP_TEST_MODE == 3U
            g_combined_test.request = 2U;
#else
            if (APP_FORWARD_HOLD_TEST != 0U)
            {
                if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
                    (APP_SERIAL_MOTION_CONTROL && (g_forward_hold.request == 1U)))
                    g_forward_hold.request = 2U;
            }
            else if ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN))
                g_heading_exec_request = 2U;
#endif
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
        }
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
if (APP_UART_LINK_TEST != 0U)
        { g_vm_test_request = 0U; Power_Disable(); }
        else
        {
#if APP_SERIAL_MOTION_CONTROL
        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();
#endif
#if APP_HEADING_SESSION_TEST
        #if APP_TEST_MODE == 3U
        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);
        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();
        App_CombinedTestSafety();
#else
        if (APP_FORWARD_HOLD_TEST != 0U) App_ForwardHoldSafety();
        else App_HeadingRuntimeSafety();
#endif
#elif APP_HWT_STATIC_TEST
        g_vm_test_request = 0U;
#elif APP_HWT_TURN_TEST
        App_HwtTurnSafety();
#endif
        }
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
    StartupPose_Init();
    RadarZoneSwitch_Init();
    (void)RadarServo_Init();
    Safety_Init();
    SAOMA_Init();
    HWT101_Init();
    g_watchdog_init_result = Watchdog_Init();
    configASSERT(g_watchdog_init_result == 1U);
    
    /* 每个任务栈为256个字，即1024字节 */
    /* 编译常量选择任务，只创建一方；关闭测试即可恢复业务。 */
    if (APP_UART_LINK_TEST != 0U)
        g_test_create_a = xTaskCreate(App_UartLinkTestTask, "UartTest",
                                     256U, NULL, 2U, NULL);
    else
        g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                     (APP_SERIAL_MOTION_CONTROL ? 512U : 256U), NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);
    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);
    /* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   (APP_SERIAL_MOTION_CONTROL ? 384U : 256U), NULL, 3U, NULL);
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
