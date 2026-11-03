#include "safety.h"
#include "chassis_auto_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include "power.h"
#include "watchdog.h"
#include "freertos_hooks.h"

/* 仅用于当前静态平台，后续运动控制需另行冻结更快的限值 */
#define SAFETY_STARTUP_MS 1000U
#define SAFETY_APP_MS     1000U
#define SAFETY_RX_MS       300U
#define SAFETY_KEY_MS      100U
/* 静态接入候选：先等待驱动器上电，已有健康后不重新宽限 */
#define SAFETY_CAN_BOOT_MS  3000U
#define SAFETY_CAN_FRESH_MS 1000U
#define SAFETY_FAULT_CAN_DATA    0x20U
#define SAFETY_FAULT_CAN_PROTECT 0x40U

volatile uint32_t g_safety_fault = 0U;
volatile uint32_t g_safety_ready = 0U;
volatile uint32_t g_safety_count = 0U;
/* 运动许可诊断不参与现有VM供电决策；本阶段固定禁止运动 */
volatile uint32_t g_motion_allowed = 0U;
volatile uint32_t g_motion_block_reason = MOTION_BLOCK_VM_OFF |
    MOTION_BLOCK_PLATFORM | MOTION_BLOCK_CAN | MOTION_BLOCK_POLICY_PENDING |
    MOTION_BLOCK_SPEED_FEEDBACK | MOTION_BLOCK_POSITION_FEEDBACK;

volatile uint32_t g_motion_speed_fresh_mask = 0U;
volatile uint32_t g_motion_position_fresh_mask = 0U;
/* 仅诊断：恢复新鲜不会清除此记录；复位会清除RAM记录 */
volatile uint32_t g_motion_diag_fault_latched = 0U;
volatile uint32_t g_motion_diag_monitor_armed = 0U;
volatile uint32_t g_motion_diag_ack_request = 0U;
volatile uint32_t g_motion_diag_ack_result = 0U;
volatile uint32_t g_motion_auto_stop_state = MOTION_STOP_IDLE;
volatile uint32_t g_motion_auto_stop_ack_mask = 0U;
volatile uint32_t g_motion_auto_stop_fail_mask = 0U;
volatile uint32_t g_motion_auto_stop_result[4] = {0U};
volatile uint32_t g_motion_auto_stop_cut_vm = 0U;
volatile uint32_t g_motion_auto_stop_session = 0U;
volatile uint32_t g_motion_auto_stop_cause = 0U;
volatile uint32_t g_wheel_test_state = WHEEL_TEST_IDLE;
volatile uint32_t g_wheel_test_active = 0U;
volatile uint32_t g_wheel_test_cut_vm = 0U;
volatile uint32_t g_wheel_test_session = 0U;
volatile uint32_t g_wheel_test_cause = 0U;
static uint32_t s_wheel_test_used = 0U;
static uint32_t s_wheel_test_vm_requested = 0U;
static TickType_t s_wheel_test_start_tick = 0U;

static uint32_t s_auto_stop_pending = 0U;
static uint32_t s_auto_stop_inflight = 0U;
static uint32_t s_auto_stop_seen_latch = 0U;
static uint32_t s_auto_stop_vm_requested = 0U;
static TickType_t s_auto_stop_start_tick = 0U;
static volatile uint32_t s_motion_feedback_enabled = 0U;
static volatile uint32_t s_motion_speed_valid = 0U;
static volatile uint32_t s_motion_position_valid = 0U;
static volatile TickType_t s_motion_speed_ticks[4];
static volatile TickType_t s_motion_position_ticks[4];

static TickType_t s_last_tick[3];
static uint32_t s_seen_mask = 0U;
static TickType_t s_start_tick = 0U;
static uint32_t s_started = 0U;
static uint32_t s_was_ready = 0U;

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

/* 调度前调用，默认不允许VM */
void Safety_Init(void)
{
    uint32_t i;

    ChassisAuto_Init();
    g_safety_fault = 0U;
    g_safety_ready = 0U;
    g_safety_count = 0U;
    g_motion_allowed = 0U;
    g_motion_block_reason = MOTION_BLOCK_VM_OFF | MOTION_BLOCK_PLATFORM |
                            MOTION_BLOCK_CAN | MOTION_BLOCK_POLICY_PENDING |
                            MOTION_BLOCK_SPEED_FEEDBACK |
                            MOTION_BLOCK_POSITION_FEEDBACK;
    g_motion_speed_fresh_mask = 0U;
    g_motion_position_fresh_mask = 0U;
    g_motion_diag_fault_latched = 0U;
    g_motion_diag_monitor_armed = 0U;
    g_motion_diag_ack_request = 0U;
    g_motion_diag_ack_result = 0U;
    g_motion_auto_stop_state = MOTION_STOP_IDLE;
    g_motion_auto_stop_ack_mask = 0U;
    g_motion_auto_stop_fail_mask = 0U;
    g_motion_auto_stop_cut_vm = 0U;
    g_motion_auto_stop_session = 0U;
    g_motion_auto_stop_cause = 0U;
    g_wheel_test_state = WHEEL_TEST_IDLE;
    g_wheel_test_active = 0U;
    g_wheel_test_cut_vm = 0U;
    g_wheel_test_session = 0U;
    g_wheel_test_cause = 0U;
    s_wheel_test_used = 0U;
    s_wheel_test_vm_requested = 0U;
    s_wheel_test_start_tick = 0U;
    s_auto_stop_pending = 0U;
    s_auto_stop_inflight = 0U;
    s_auto_stop_seen_latch = 0U;
    s_auto_stop_vm_requested = 0U;
    s_auto_stop_start_tick = 0U;
    for (i = 0U; i < 4U; i++) g_motion_auto_stop_result[i] = 0U;
    s_motion_feedback_enabled = 0U;
    s_motion_speed_valid = 0U;
    s_motion_position_valid = 0U;
    s_seen_mask = 0U;
    s_start_tick = 0U;
    s_started = 0U;
    s_was_ready = 0U;
    for (i = 0U; i < 3U; i++)
    {
        s_last_tick[i] = 0U;
    }
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
        s_motion_speed_ticks[i] = 0U;
        s_motion_position_ticks[i] = 0U;
    }
    Power_Disable();
}

/* 由相应任务在一轮实际工作结束后报告，不由其他任务代报 */
void Safety_Report(uint32_t task_id)
{
    if (task_id < 3U)
    {
        taskENTER_CRITICAL();
        s_last_tick[task_id] = xTaskGetTickCount();
        s_seen_mask |= 1UL << task_id;
        taskEXIT_CRITICAL();
    }
}

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

/* 只发布快照，不改变原状态保护、VM及喂狗决策 */
void Safety_MotionFeedbackReport(uint32_t enabled,
    uint32_t speed_valid_mask, uint32_t position_valid_mask,
    const volatile TickType_t *speed_ticks,
    const volatile TickType_t *position_ticks)
{
    uint32_t i;
    taskENTER_CRITICAL();
    s_motion_feedback_enabled = (enabled == 1U) ? 1U : 0U;
    s_motion_speed_valid = (speed_ticks != NULL) ?
                          (speed_valid_mask & 0x0FU) : 0U;
    s_motion_position_valid = (position_ticks != NULL) ?
                             (position_valid_mask & 0x0FU) : 0U;
    for (i = 0U; i < 4U; i++)
    {
        s_motion_speed_ticks[i] = (speed_ticks != NULL) ? speed_ticks[i] : 0U;
        s_motion_position_ticks[i] = (position_ticks != NULL) ? position_ticks[i] : 0U;
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
    uint32_t fresh = 0U, ready_fresh = 0U;
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
            /* 就绪只计本次上电稳定后采到的状态；原保护位仍按fresh检查。 */
            if (((TickType_t)(now - s_can_power_tick) >=
                 pdMS_TO_TICKS(APP_HEADING_POWER_SETTLE_MS)) &&
                ((TickType_t)(now - ticks[i]) <=
                 (TickType_t)(now - s_can_power_tick) -
                 pdMS_TO_TICKS(APP_HEADING_POWER_SETTLE_MS)))
                ready_fresh |= 1UL << i;
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
    if ((enabled == 1U) && (ready_fresh == 0x0FU))
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
            (enabled == 1U) ? (0x0FU & ~ready_fresh) : 0x0FU;
        g_can_guard_cause = (enabled == 1U) ? 2U : 1U;
        g_safety_fault |= SAFETY_FAULT_CAN_DATA;
    }
}

/* 已在短临界区内调用；只发布关VM标志 */
static void Safety_MotionStopCut(uint32_t cause)
{
    g_motion_auto_stop_state = MOTION_STOP_VM_CUT;
    g_motion_auto_stop_cut_vm = 1U;
    g_motion_auto_stop_cause = cause;
    s_auto_stop_pending = 0U;
}

/* Safety独立推进总等待时间，不等待CAN任务 */
static void Safety_MotionStopUpdate(uint32_t vm_request)
{
    TickType_t now;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    s_auto_stop_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (g_motion_diag_fault_latched == 0U)
    {
        if (vm_request == 0U)
        {
            /* 仅初始或撤VM后明确确认才重置，不靠反馈恢复重置 */
            g_motion_auto_stop_state = MOTION_STOP_IDLE;
            g_motion_auto_stop_cut_vm = 0U;
            g_motion_auto_stop_cause = 0U;
            s_auto_stop_pending = 0U;
            s_auto_stop_inflight = 0U;
            s_auto_stop_seen_latch = 0U;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (vm_request != 1U)
    {
        if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
        {
            g_motion_auto_stop_state = MOTION_STOP_CANCELLED;
            s_auto_stop_pending = 0U;
        }
        taskEXIT_CRITICAL();
        return;
    }
    if (s_auto_stop_seen_latch == 0U)
    {
        /* 每个锁存会话只请求一次，禁止无限重试 */
        s_auto_stop_seen_latch = 1U;
        g_motion_auto_stop_session++;
        g_motion_auto_stop_state = MOTION_STOP_PENDING;
        g_motion_auto_stop_ack_mask = 0U;
        g_motion_auto_stop_fail_mask = 0U;
        g_motion_auto_stop_result[0] = 0U;
        g_motion_auto_stop_result[1] = 0U;
        g_motion_auto_stop_result[2] = 0U;
        g_motion_auto_stop_result[3] = 0U;
        g_motion_auto_stop_cause = 0U;
        s_auto_stop_pending = 0x0FU;
        s_auto_stop_inflight = 0U;
        s_auto_stop_start_tick = now;
    }
    if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
    {
        if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_safety_ready != 1U)) Safety_MotionStopCut(3U);
        else if ((TickType_t)(now - s_auto_stop_start_tick) >=
                 pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS))
            Safety_MotionStopCut(2U);
    }
    taskEXIT_CRITICAL();
}

uint32_t Safety_MotionStopTake(uint8_t *address, uint32_t *session)
{
    uint32_t i;
    uint32_t bit;
    uint32_t taken = 0U;
    if ((address == NULL) || (session == NULL)) return 0U;
    taskENTER_CRITICAL();
    if ((g_motion_auto_stop_state == MOTION_STOP_PENDING) &&
        (s_auto_stop_vm_requested == 1U) && (s_auto_stop_inflight == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U))
    {
        if ((TickType_t)(xTaskGetTickCount() - s_auto_stop_start_tick) >=
            pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS)) Safety_MotionStopCut(2U);
        else
        {
            for (i = 0U; i < 4U; i++)
            {
                bit = 1UL << i;
                if ((s_auto_stop_pending & bit) != 0U)
                {
                    s_auto_stop_pending &= ~bit;
                    s_auto_stop_inflight = bit;
                    *address = (uint8_t)(i + 1U);
                    *session = g_motion_auto_stop_session;
                    taken = 1U;
                    break;
                }
            }
        }
    }
    taskEXIT_CRITICAL();
    return taken;
}

void Safety_MotionStopReport(uint8_t address, uint32_t session,
                             uint32_t result)
{
    uint32_t bit;
    if ((address < 1U) || (address > 4U)) return;
    bit = 1UL << (address - 1U);
    taskENTER_CRITICAL();
    if ((session == g_motion_auto_stop_session) &&
        (s_auto_stop_inflight == bit))
    {
        s_auto_stop_inflight = 0U;
        g_motion_auto_stop_result[address - 1U] = result;
        if (result == 1U) g_motion_auto_stop_ack_mask |= bit;
        else g_motion_auto_stop_fail_mask |= bit;
        /* 超时或撤VM后的晚回报只记录，不得恢复完成态 */
        if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
        {
            if (result != 1U) Safety_MotionStopCut(1U);
            else if ((TickType_t)(xTaskGetTickCount() - s_auto_stop_start_tick) >=
                     pdMS_TO_TICKS(MOTION_STOP_ACK_WAIT_MS))
                Safety_MotionStopCut(2U);
            else if (g_motion_auto_stop_ack_mask == 0x0FU)
                g_motion_auto_stop_state = MOTION_STOP_ACKED;
        }
    }
    taskEXIT_CRITICAL();
}

/* 调用者持短临界区；不等待CAN，关电在本轮或下一轮Safety输出执行 */
static void Safety_WheelTestCut(uint32_t cause)
{
    if (cause != 0U) g_wheel_test_state = WHEEL_TEST_CUT;
    g_wheel_test_active = 0U;
    g_wheel_test_cut_vm = 1U;
    g_wheel_test_cause = cause;
}

/* 每20ms由Safety推进；即使CAN任务停止推进，截止时间仍生效 */
static void Safety_WheelTestUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_wheel_test_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (vm_request != 1U)
    {
        if (g_wheel_test_active == 1U)
            g_wheel_test_state = WHEEL_TEST_CANCELLED;
        g_wheel_test_active = 0U;
        g_wheel_test_cut_vm = 0U;
        s_wheel_test_used = 0U;
    }
    else if (g_wheel_test_active == 1U)
    {
        if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_safety_ready != 1U) || (g_can_guard_ready != 1U) ||
            (g_motion_diag_fault_latched != 0U) ||
            (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
            (g_motion_auto_stop_cut_vm != 0U))
            Safety_WheelTestCut(4U);
        else if (g_wheel_test_state == WHEEL_TEST_PREPARING)
        {
            if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                pdMS_TO_TICKS(WHEEL_TEST_PREP_MS)) Safety_WheelTestCut(6U);
        }
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS))
            Safety_WheelTestCut((g_wheel_test_state == WHEEL_TEST_DONE) ? 0U : 3U);
    }
    taskEXIT_CRITICAL();
}

/* 先占用本次供电会话，准备超时也由Safety独立关VM */
uint32_t Safety_WheelTestPrepare(void)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((s_wheel_test_vm_requested == 1U) && (s_wheel_test_used == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U))
    {
        s_wheel_test_used = 1U;
        s_wheel_test_start_tick = xTaskGetTickCount();
        g_wheel_test_session++;
        g_wheel_test_state = WHEEL_TEST_PREPARING;
        g_wheel_test_active = 1U;
        g_wheel_test_cause = 0U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

void Safety_WheelTestPrepareFailed(void)
{
    taskENTER_CRITICAL();
    if ((g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_PREPARING)) Safety_WheelTestCut(6U);
    taskEXIT_CRITICAL();
}

uint32_t Safety_WheelTestArm(uint32_t *session)
{
    uint32_t accepted = 0U;
    if (session == NULL) return 0U;
    taskENTER_CRITICAL();
    if ((s_wheel_test_vm_requested == 1U) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_PREPARING) &&
        (g_wheel_test_cut_vm == 0U) && (g_rtos_fault == 0U) &&
        (g_safety_fault == 0U) && (g_safety_ready == 1U) &&
        (g_can_guard_ready == 1U) && (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U))
    {
        if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
            pdMS_TO_TICKS(WHEEL_TEST_PREP_MS)) Safety_WheelTestCut(6U);
        else
        {
            /* 转动时间从本次速度发送前另起算，不包含准备阶段 */
            s_wheel_test_start_tick = xTaskGetTickCount();
            *session = g_wheel_test_session;
            g_wheel_test_state = WHEEL_TEST_STARTING;
            accepted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}

/* 四轮分笔启动前复核本会话，过期或撤VM后不得补发启动 */
uint32_t Safety_WheelTestStartAllowed(uint32_t session)
{
    uint32_t allowed = 0U;
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STARTING))
    {
        if ((s_wheel_test_vm_requested != 1U) || (g_rtos_fault != 0U) ||
            (g_safety_fault != 0U) || (g_safety_ready != 1U) ||
            (g_can_guard_ready != 1U)) Safety_WheelTestCut(4U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_RUN_MS)) Safety_WheelTestCut(3U);
        else allowed = 1U;
    }
    taskEXIT_CRITICAL();
    return allowed;
}

void Safety_WheelTestStartReport(uint32_t session, uint32_t result)
{
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STARTING))
    {
        if (result != 1U) Safety_WheelTestCut(1U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else g_wheel_test_state = WHEEL_TEST_RUNNING;
    }
    taskEXIT_CRITICAL();
}

uint32_t Safety_WheelTestTakeStop(uint32_t *session)
{
    uint32_t taken = 0U;
    if (session == NULL) return 0U;
    taskENTER_CRITICAL();
    if ((g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_RUNNING))
    {
        TickType_t age = (TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick);
        if (age >= pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else if (age >= pdMS_TO_TICKS(WHEEL_TEST_RUN_MS))
        {
            g_wheel_test_state = WHEEL_TEST_STOPPING;
            *session = g_wheel_test_session;
            taken = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return taken;
}

void Safety_WheelTestStopReport(uint32_t session, uint32_t result)
{
    taskENTER_CRITICAL();
    if ((session == g_wheel_test_session) && (g_wheel_test_active == 1U) &&
        (g_wheel_test_state == WHEEL_TEST_STOPPING))
    {
        if (result != 1U) Safety_WheelTestCut(2U);
        else if ((TickType_t)(xTaskGetTickCount() - s_wheel_test_start_tick) >=
                 pdMS_TO_TICKS(WHEEL_TEST_CUT_MS)) Safety_WheelTestCut(3U);
        else g_wheel_test_state = WHEEL_TEST_DONE;
    }
    taskEXIT_CRITICAL();
}

void Safety_WheelTestAbort(void)
{
    taskENTER_CRITICAL();
    if (g_wheel_test_active == 1U) Safety_WheelTestCut(5U);
    taskEXIT_CRITICAL();
}

/* 反馈诊断锁存只记账，不改原故障、供电、喂狗或发送CAN */
static void Safety_MotionDiagLatchUpdate(uint32_t vm_request,
                                        uint32_t speed_fresh,
                                        uint32_t position_fresh)
{
    if (vm_request != 1U)
    {
        /* 正常撤VM不作为新反馈故障；已有锁存仍保留 */
        g_motion_diag_monitor_armed = 0U;
    }
    else if ((speed_fresh == 0x0FU) && (position_fresh == 0x0FU))
    {
        /* 首次采集未完成时不误锁存；恢复新鲜也不清旧记录 */
        g_motion_diag_monitor_armed = 1U;
    }
    else if (g_motion_diag_monitor_armed == 1U)
    {
        if (speed_fresh != 0x0FU)
        {
            g_motion_diag_fault_latched |= MOTION_BLOCK_SPEED_FEEDBACK;
        }
        if (position_fresh != 0x0FU)
        {
            g_motion_diag_fault_latched |= MOTION_BLOCK_POSITION_FEEDBACK;
        }
    }

    if (g_motion_diag_ack_request == 1U)
    {
        /* 本次立即处理，失败不排队到条件恢复时自动确认 */
        g_motion_diag_ack_request = 0U;
        if ((vm_request == 0U) && (g_rtos_fault == 0U) &&
            (g_safety_fault == 0U) && (g_safety_ready == 1U))
        {
            /* 只确认这层诊断，不代表修复硬件或重新允许运动 */
            g_motion_diag_fault_latched = 0U;
            g_motion_diag_monitor_armed = 0U;
            g_motion_diag_ack_result = 1U;
        }
        else
        {
            g_motion_diag_ack_result = 9U;
        }
    }
}

/* 只由监督任务调用；这一步只显示禁止原因，不发送CAN */
static void Safety_MotionPermissionUpdate(uint32_t vm_request)
{
    uint32_t blocked = MOTION_BLOCK_POLICY_PENDING;
    uint32_t enabled;
    uint32_t speed_valid;
    uint32_t position_valid;
    uint32_t speed_fresh = 0U;
    uint32_t position_fresh = 0U;
    uint32_t i;
    TickType_t now;
    TickType_t speed_ticks[4];
    TickType_t position_ticks[4];

    /* 一次取出一致快照；CAN任务停住时，本任务仍会继续算年龄 */
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    enabled = s_motion_feedback_enabled;
    speed_valid = s_motion_speed_valid;
    position_valid = s_motion_position_valid;
    for (i = 0U; i < 4U; i++)
    {
        speed_ticks[i] = s_motion_speed_ticks[i];
        position_ticks[i] = s_motion_position_ticks[i];
    }
    taskEXIT_CRITICAL();

    if ((vm_request == 1U) && (enabled == 1U))
    {
        for (i = 0U; i < 4U; i++)
        {
            /* 有效位确认本会话成功采样；无符号相减支持tick回绕 */
            if (((speed_valid & (1UL << i)) != 0U) &&
                ((TickType_t)(now - speed_ticks[i]) <=
                 pdMS_TO_TICKS(SAFETY_MOTION_DIAG_FRESH_MS)))
            {
                speed_fresh |= 1UL << i;
            }
            if (((position_valid & (1UL << i)) != 0U) &&
                ((TickType_t)(now - position_ticks[i]) <=
                 pdMS_TO_TICKS(SAFETY_MOTION_DIAG_FRESH_MS)))
            {
                position_fresh |= 1UL << i;
            }
        }
    }
    g_motion_speed_fresh_mask = speed_fresh;
    g_motion_position_fresh_mask = position_fresh;
    Safety_MotionDiagLatchUpdate(vm_request, speed_fresh, position_fresh);
    if (g_motion_diag_fault_latched != 0U)
    {
        blocked |= MOTION_BLOCK_DIAG_LATCHED;
    }
    if (speed_fresh != 0x0FU)
    {
        blocked |= MOTION_BLOCK_SPEED_FEEDBACK;
    }
    if (position_fresh != 0x0FU)
    {
        blocked |= MOTION_BLOCK_POSITION_FEEDBACK;
    }
    if (vm_request != 1U)
    {
        blocked |= MOTION_BLOCK_VM_OFF;
    }
    if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
        (g_safety_ready != 1U))
    {
        blocked |= MOTION_BLOCK_PLATFORM;
    }
    if (g_can_guard_ready != 1U)
    {
        blocked |= MOTION_BLOCK_CAN;
    }

    /* 平台/CAN就绪也不放行，不能用静态1秒反馈阈值允许运动 */
    g_motion_block_reason = blocked;
    g_motion_allowed = 0U;
}

/* 只有监督任务调用；故障锁定后，正常心跳恢复也不能自动开VM */
void Safety_Update(uint32_t vm_request, uint32_t can_test_result)
{
    TickType_t now;
    TickType_t last[3];
    uint32_t seen;
    uint32_t bad = 0U;
    uint32_t i;

    g_safety_count++;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    seen = s_seen_mask;
    for (i = 0U; i < 3U; i++)
    {
        last[i] = s_last_tick[i];
    }
    taskEXIT_CRITICAL();

    if (s_started == 0U)
    {
        s_start_tick = now;
        s_started = 1U;
    }

    if (((seen & 0x01U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_APP]) >
         pdMS_TO_TICKS(SAFETY_APP_MS)))
    {
        bad |= 0x01U;
    }
    if (((seen & 0x02U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_RX]) >
         pdMS_TO_TICKS(SAFETY_RX_MS)))
    {
        bad |= 0x02U;
    }
    if (((seen & 0x04U) == 0U) ||
        ((TickType_t)(now - last[SAFETY_TASK_KEY]) >
         pdMS_TO_TICKS(SAFETY_KEY_MS)))
    {
        bad |= 0x04U;
    }

    /* 回环结果0表示还在启动，其他非1结果表示已失败 */
    if ((can_test_result != 0U) && (can_test_result != 1U))
    {
        g_safety_fault |= 0x10U;
    }
    if ((s_was_ready != 0U) ||
        ((TickType_t)(now - s_start_tick) >=
         pdMS_TO_TICKS(SAFETY_STARTUP_MS)))
    {
        g_safety_fault |= bad;
        if (can_test_result == 0U)
        {
            g_safety_fault |= 0x08U;
        }
    }
    /* 在最后决定VM允许之前，先纳入CAN故障判断 */
    /* 短时测试主动关电后不把随后的掉线记成原CAN故障 */
    Safety_CanGuardUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);
    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    /* 更新诊断并发布停止请求，失败或超时追加关VM条件 */
    Safety_WheelTestUpdate(vm_request);
    ChassisAuto_SafetyUpdate(vm_request);
    Safety_MotionPermissionUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);
    Safety_MotionStopUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? 0U : vm_request);

    if ((g_safety_ready == 1U) && (vm_request == 1U) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_cut_vm == 0U) &&
        (g_chassis_auto_cut_vm == 0U))
    {
        Power_Enable();
    }
    else
    {
        Power_Disable();
    }
    /* 本轮任务/CAN检查与输出决策完成，只有健康才继续喂狗 */
    if ((g_safety_ready == 1U) && (g_rtos_fault == 0U))
    {
        Watchdog_Feed();
    }
}
