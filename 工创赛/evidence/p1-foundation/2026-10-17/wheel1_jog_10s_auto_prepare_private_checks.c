#include <stdio.h>
#include <string.h>
typedef unsigned int uint32_t;typedef unsigned short uint16_t;typedef unsigned char uint8_t;typedef uint32_t TickType_t;
typedef struct {uint32_t ExtId;uint8_t DLC,Data[8];} CanRxMsg;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
static TickType_t test_tick;
static TickType_t xTaskGetTickCount(void){return test_tick;}
static unsigned int power_enabled,feeds;
static void Power_Enable(void){power_enabled=1;}
static void Power_Disable(void){power_enabled=0;}
static void Watchdog_Feed(void){feeds++;}
volatile uint32_t g_rtos_fault;
#define SAFETY_TASK_APP 0U
#define SAFETY_TASK_RX  1U
#define SAFETY_TASK_KEY 2U
#define MOTION_BLOCK_VM_OFF         0x01U
#define MOTION_BLOCK_PLATFORM       0x02U
#define MOTION_BLOCK_CAN            0x04U
#define MOTION_BLOCK_POLICY_PENDING 0x08U
#define MOTION_BLOCK_SPEED_FEEDBACK 0x10U
#define MOTION_BLOCK_POSITION_FEEDBACK 0x20U
#define MOTION_BLOCK_DIAG_LATCHED   0x40U
#define SAFETY_MOTION_DIAG_FRESH_MS 1000U
#define MOTION_STOP_IDLE 0U
#define MOTION_STOP_PENDING 1U
#define MOTION_STOP_ACKED 2U
#define MOTION_STOP_VM_CUT 3U
#define MOTION_STOP_CANCELLED 4U
#define MOTION_STOP_ACK_WAIT_MS 500U
#define WHEEL_TEST_RUN_MS 10000U
#define WHEEL_TEST_CUT_MS 10500U
#define WHEEL_TEST_PREP_MS 5000U
#define WHEEL_TEST_IDLE 0U
#define WHEEL_TEST_STARTING 1U
#define WHEEL_TEST_RUNNING 2U
#define WHEEL_TEST_STOPPING 3U
#define WHEEL_TEST_DONE 4U
#define WHEEL_TEST_CUT 5U
#define WHEEL_TEST_CANCELLED 6U
#define WHEEL_TEST_PREPARING 7U

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
    Safety_CanGuardUpdate((g_wheel_test_cut_vm == 1U) ? 0U : vm_request);
    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    /* 更新诊断并发布停止请求，失败或超时追加关VM条件 */
    Safety_WheelTestUpdate(vm_request);
    Safety_MotionPermissionUpdate((g_wheel_test_cut_vm == 1U) ? 0U : vm_request);
    Safety_MotionStopUpdate((g_wheel_test_cut_vm == 1U) ? 0U : vm_request);

    if ((g_safety_ready == 1U) && (vm_request == 1U) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_cut_vm == 0U))
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
volatile uint32_t g_can_disable_request;
volatile uint32_t g_can_enable_request;
volatile uint32_t g_can_feedback_enable;
volatile uint32_t g_can_jog_check_result;
volatile uint32_t g_can_jog_dry_run;
volatile uint32_t g_can_jog_esr;
volatile uint32_t g_can_jog_mode;
volatile uint32_t g_can_jog_prepare_result[4];
volatile uint32_t g_can_jog_prepare_stage;
CanRxMsg g_can_jog_reply;
volatile uint32_t g_can_jog_request;
volatile uint32_t g_can_jog_result;
volatile uint32_t g_can_jog_start_result;
CanRxMsg g_can_jog_stop_reply;
volatile uint32_t g_can_jog_stop_result;
volatile uint32_t g_can_poll_count[4];
volatile uint32_t g_can_poll_enable;
volatile uint32_t g_can_poll_flags[4];
volatile uint32_t g_can_poll_result[4];
volatile uint32_t g_can_poll_tick[4];
volatile uint32_t g_can_position_request;
volatile uint32_t g_can_speed_request;
volatile uint32_t g_can_status_request;
volatile uint32_t g_can_stop_request;
volatile uint32_t g_can_version_request;
volatile uint32_t g_can_zero_request;
volatile uint32_t g_motion_diag_fail_stop_once;
volatile uint32_t g_motion_diag_hold_report;
volatile uint32_t g_motion_diag_hold_stop_service;
volatile uint32_t g_vm_test_request;
static uint32_t s_can_jog_preparing,s_can_jog_check_index,s_can_jog_dry_run,s_can_jog_poll_between;
struct {uint32_t ESR;} fake_can;
#define CAN1 (&fake_can)
static unsigned int status_calls,speed_calls,stop_calls,poll_calls,bad_call,checks,disable_calls,enable_calls;
static uint32_t status_flags[4],status_codes[4],speed_code,stop_code,disable_codes[4],enable_code;
static unsigned int ignore_disable,ignore_enable,extra_enable,events[32],event_count;
static uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg *reply){status_calls++;reply->Data[0]=0x3A;reply->Data[1]=(uint8_t)status_flags[a-1];return status_codes[a-1];}
static uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t en,CanRxMsg *reply){uint32_t r;
 if(a<1||a>4||en>1)bad_call=1;reply->Data[0]=0xF3;
 if(en==0){disable_calls++;events[event_count++]=a;r=disable_codes[a-1];if(r==1&&ignore_disable!=a)status_flags[a-1]&=~1U;}
 else{enable_calls++;events[event_count++]=10+a;r=enable_code;if(a!=1)bad_call=1;if(r==1&&!ignore_enable)status_flags[a-1]|=1;if(extra_enable)status_flags[2]|=1;}
 return r;}
static uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t d,uint16_t v,uint8_t acc,CanRxMsg *reply){speed_calls++;events[event_count++]=50;if(a!=1||d!=0||v!=1||acc!=0)bad_call=1;reply->Data[0]=0xF6;reply->Data[1]=(speed_code==1)?2:0xE2;return speed_code;}
static uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg *reply){stop_calls++;if(a!=1)bad_call=1;reply->Data[0]=0xFE;return stop_code;}
static void App_CanPollStep(void){poll_calls++;}
static uint32_t App_CanEnableTestReady(void)
{
    return ((g_vm_test_request == 1U) &&
            (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
            (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
            (g_can_poll_enable == 1U) &&
            (g_wheel_test_cut_vm == 0U) &&
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

/* 每轮至多一笔交易；自动全失能、确认、使能1号、再确认后才启动 */
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
            g_can_jog_check_result = CAN1_SetMotorEnable(1U, 1U, &reply);
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
            s_can_jog_check_index = 0U;
            g_can_jog_prepare_stage = 4U;
            return;
        }
        if ((g_can_jog_prepare_stage == 2U) || (g_can_jog_prepare_stage == 4U))
        {
            uint32_t expected = ((g_can_jog_prepare_stage == 4U) &&
                                 (index == 0U)) ? 1U : 0U;
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
        /* 固定原始速度1，不读取Watch中的地址/方向/速度 */
        if (s_can_jog_dry_run == 1U)
            g_can_jog_start_result = 1U; /* 演练模拟，不是电机应答 */
        else
            g_can_jog_start_result = CAN1_SetEmmSpeedRaw(1U, 0U, 1U, 0U, &reply);
        g_can_jog_reply = reply;
        g_can_jog_esr = CAN1->ESR;
        g_can_jog_result = g_can_jog_start_result;
        Safety_WheelTestStartReport(session, g_can_jog_start_result);
        return;
    }
    if (Safety_WheelTestTakeStop(&session) == 1U)
    {
        if (s_can_jog_dry_run == 1U)
            g_can_jog_stop_result = 1U; /* 演练模拟，不发FE */
        else
            g_can_jog_stop_result = CAN1_StopMotor(1U, &reply);
        g_can_jog_stop_reply = reply;
        g_can_jog_esr = CAN1->ESR;
        Safety_WheelTestStopReport(session, g_can_jog_stop_result);
    }
    else if (g_wheel_test_active == 1U)
    {
        /* 运动窗口只保留必要状态轮询，不插入其他人工查询 */
        App_CanPollStep();
    }
}


static void step(void){
        if ((s_can_jog_preparing == 1U) || (g_wheel_test_active == 1U))
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
                    for (i = 0U; i < 4U; i++) g_can_jog_prepare_result[i] = 0U;
                    s_can_jog_check_index = 0U;
                    s_can_jog_poll_between = 0U;
                    g_can_jog_prepare_stage = 1U;
                    s_can_jog_preparing = 1U;
                }
                else g_can_jog_result = 9U;
            }
            else g_can_jog_result = 9U;
        }
}
static void healthy(TickType_t t){TickType_t ticks[4];unsigned int i;test_tick=t;for(i=0;i<4;i++)ticks[i]=t;Safety_Report(0);Safety_Report(1);Safety_Report(2);Safety_CanReport(1,15,0,ticks);Safety_Update(g_vm_test_request,1);}
static void base(TickType_t t){unsigned int i;Safety_Init();g_rtos_fault=0;
g_can_disable_request=0;
g_can_enable_request=0;
g_can_feedback_enable=0;
g_can_jog_check_result=0;
g_can_jog_dry_run=0;
g_can_jog_esr=0;
g_can_jog_mode=0;
for(i=0;i<4;i++)g_can_jog_prepare_result[i]=0;
g_can_jog_prepare_stage=0;
g_can_jog_request=0;
g_can_jog_result=0;
g_can_jog_start_result=0;
g_can_jog_stop_result=0;
for(i=0;i<4;i++)g_can_poll_count[i]=0;
g_can_poll_enable=0;
for(i=0;i<4;i++)g_can_poll_flags[i]=0;
for(i=0;i<4;i++)g_can_poll_result[i]=0;
for(i=0;i<4;i++)g_can_poll_tick[i]=0;
g_can_position_request=0;
g_can_speed_request=0;
g_can_status_request=0;
g_can_stop_request=0;
g_can_version_request=0;
g_can_zero_request=0;
g_motion_diag_fail_stop_once=0;
g_motion_diag_hold_report=0;
g_motion_diag_hold_stop_service=0;
g_vm_test_request=0;
g_vm_test_request=g_can_poll_enable=1;s_can_jog_preparing=s_can_jog_check_index=s_can_jog_dry_run=s_can_jog_poll_between=0;
status_calls=speed_calls=stop_calls=poll_calls=bad_call=disable_calls=enable_calls=event_count=0;ignore_disable=ignore_enable=extra_enable=0;speed_code=stop_code=enable_code=1;
for(i=0;i<4;i++){status_flags[i]=1;status_codes[i]=disable_codes[i]=1;}healthy(t);}
static void start_request(void){g_can_jog_request=1;step();}
static void prepare(void){unsigned int i;start_request();for(i=0;i<100&&s_can_jog_preparing&&g_wheel_test_active;i++){healthy(test_tick+50);step();}}
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d state%u cause%u stage%u tick%u\n",__LINE__,g_wheel_test_state,g_wheel_test_cause,g_can_jog_prepare_stage,test_tick);return 1;}}while(0)
int main(void){unsigned int i;uint32_t session,oldsession;TickType_t start;
volatile uint32_t *other[]={&g_can_stop_request,&g_can_disable_request,&g_can_enable_request,&g_can_zero_request,&g_can_speed_request,&g_can_position_request,&g_can_version_request,&g_can_status_request};
CHECK(WHEEL_TEST_RUN_MS==10000&&WHEEL_TEST_CUT_MS==10500&&WHEEL_TEST_PREP_MS==5000);
base(100);step();CHECK(!disable_calls&&!enable_calls&&!speed_calls&&!g_wheel_test_active);
base(100);g_vm_test_request=0;healthy(120);start_request();CHECK(g_can_jog_result==9&&!disable_calls&&!g_wheel_test_active);
base(100);g_can_feedback_enable=1;start_request();CHECK(g_can_jog_result==9&&!disable_calls);
for(i=0;i<8;i++){base(100);*other[i]=1;start_request();CHECK(g_can_jog_result==9&&!disable_calls);}
/* Starting with ALL motors enabled now automatically prepares just motor1. */
base(100);prepare();CHECK(disable_calls==4&&enable_calls==1&&status_calls==8&&poll_calls==5&&speed_calls==1&&!bad_call);
CHECK(event_count==6&&events[0]==1&&events[1]==2&&events[2]==3&&events[3]==4&&events[4]==11&&events[5]==50);
CHECK(status_flags[0]==1&&status_flags[1]==0&&status_flags[2]==0&&status_flags[3]==0);
CHECK(g_wheel_test_state==WHEEL_TEST_RUNNING&&g_can_jog_prepare_stage==5&&g_can_jog_start_result==1);
start=test_tick;session=g_wheel_test_session;CHECK(Safety_WheelTestPrepare()==0);CHECK(Safety_WheelTestArm(&oldsession)==0);
healthy(start+9999);step();CHECK(!stop_calls&&g_wheel_test_active&&power_enabled);
g_can_jog_request=1;healthy(start+10000);step();CHECK(stop_calls==1&&g_wheel_test_state==WHEEL_TEST_DONE&&!g_can_jog_request);
healthy(start+10499);step();CHECK(stop_calls==1&&power_enabled);healthy(start+10500);CHECK(!power_enabled&&g_wheel_test_cut_vm==1&&g_wheel_test_cause==0&&g_wheel_test_state==WHEEL_TEST_DONE&&g_safety_fault==0);
Safety_WheelTestStopReport(session,1);CHECK(g_wheel_test_cut_vm==1);start_request();CHECK(g_can_jog_result==9&&speed_calls==1);
g_vm_test_request=0;healthy(test_tick+20);CHECK(!g_wheel_test_cut_vm&&!s_wheel_test_used);g_vm_test_request=1;healthy(test_tick+20);prepare();CHECK(speed_calls==2&&g_wheel_test_session!=session);Safety_WheelTestStartReport(session,5);CHECK(g_wheel_test_state==WHEEL_TEST_RUNNING);
/* Every initial disable must succeed before any enable/speed command. */
for(i=0;i<4;i++){base(100);disable_codes[i]=4;prepare();CHECK(g_can_jog_prepare_result[i]==4&&disable_calls==i+1&&!enable_calls&&!speed_calls);CHECK(g_can_jog_result==10&&g_wheel_test_cut_vm==1&&g_wheel_test_cause==6);healthy(test_tick+20);CHECK(!power_enabled);}
base(100);ignore_disable=3;prepare();CHECK(disable_calls==4&&!enable_calls&&!speed_calls&&g_wheel_test_cause==6);
base(100);enable_code=5;prepare();CHECK(disable_calls==4&&enable_calls==1&&!speed_calls&&g_can_jog_check_result==5&&g_wheel_test_cause==6);
base(100);ignore_enable=1;prepare();CHECK(enable_calls==1&&!speed_calls&&g_can_jog_result==10&&g_wheel_test_cause==6);
base(100);extra_enable=1;prepare();CHECK(enable_calls==1&&!speed_calls&&g_wheel_test_cause==6);
for(i=0;i<4;i++){base(100);status_codes[i]=5;prepare();CHECK(!enable_calls&&!speed_calls&&g_wheel_test_cause==6);}
base(100);status_flags[0]=0x81;prepare();CHECK(speed_calls==1&&!bad_call); /* bit7 ignored */
/* Preparation cannot hang indefinitely, including stalled CAN service. */
base(100);start_request();CHECK(g_wheel_test_state==WHEEL_TEST_PREPARING);healthy(5099);CHECK(power_enabled);healthy(5100);CHECK(!power_enabled&&g_wheel_test_cause==6&&!speed_calls);
base(100);start_request();test_tick=5100;CHECK(Safety_WheelTestArm(&session)==0&&g_wheel_test_cause==6);
base(100);CHECK(Safety_WheelTestArm(&session)==0);CHECK(Safety_WheelTestArm(0)==0);
/* Dry run still prepares F3 states but sends no F6 or FE; mode is latched. */
base(100);g_can_jog_dry_run=1;start_request();g_can_jog_dry_run=0;for(i=0;i<100&&s_can_jog_preparing&&g_wheel_test_active;i++){healthy(test_tick+50);step();}
CHECK(g_can_jog_mode==1&&disable_calls==4&&enable_calls==1&&!speed_calls&&g_wheel_test_state==WHEEL_TEST_RUNNING);start=test_tick;
healthy(start+10000);step();CHECK(!stop_calls&&g_wheel_test_state==WHEEL_TEST_DONE&&g_can_jog_stop_reply.Data[0]==0);healthy(start+10500);CHECK(!power_enabled&&g_wheel_test_cause==0);
base(100);g_can_jog_dry_run=2;start_request();CHECK(g_can_jog_result==9&&!disable_calls);
base(100);speed_code=8;prepare();CHECK(g_can_jog_start_result==8&&g_wheel_test_cause==1&&g_wheel_test_cut_vm==1);healthy(test_tick+20);CHECK(!power_enabled);
base(100);prepare();start=test_tick;stop_code=5;healthy(start+10000);step();CHECK(stop_calls==1&&g_wheel_test_cause==2);healthy(test_tick+20);CHECK(!power_enabled);
base(100);prepare();start=test_tick;healthy(start+10500);CHECK(!power_enabled&&g_wheel_test_cause==3);Safety_WheelTestStartReport(g_wheel_test_session,1);CHECK(!g_wheel_test_active&&g_wheel_test_cut_vm);
base(100);prepare();oldsession=g_wheel_test_session;g_vm_test_request=0;healthy(test_tick+20);CHECK(g_wheel_test_state==WHEEL_TEST_CANCELLED&&!power_enabled);Safety_WheelTestStartReport(oldsession,1);CHECK(!g_wheel_test_active);
for(i=0;i<8;i++){base(100);prepare();*other[i]=1;step();CHECK(!*other[i]&&g_wheel_test_cause==5);healthy(test_tick+20);CHECK(!power_enabled);}
base(100);start_request();g_can_enable_request=1;step();CHECK(!disable_calls&&!g_can_enable_request&&g_wheel_test_cut_vm==1);
base(100);prepare();g_rtos_fault=1;healthy(test_tick+20);CHECK(!power_enabled&&g_wheel_test_cause==4);
base(100);prepare();g_safety_fault=1;healthy(test_tick+20);CHECK(!power_enabled&&g_wheel_test_cause==4);
base(0xFFFFF000U);prepare();start=test_tick;healthy(start+10000);step();CHECK(stop_calls==1&&g_wheel_test_state==WHEEL_TEST_DONE);healthy(start+10500);CHECK(!power_enabled&&g_wheel_test_cut_vm==1);
base(100);prepare();start=test_tick;test_tick=start+10000;CHECK(Safety_WheelTestTakeStop(&session)==1);CHECK(Safety_WheelTestTakeStop(&session)==0);Safety_WheelTestStopReport(session-1,1);CHECK(g_wheel_test_state==WHEEL_TEST_STOPPING);test_tick=start+10500;Safety_WheelTestStopReport(session,1);CHECK(g_wheel_test_cause==3&&g_wheel_test_state==WHEEL_TEST_CUT);
base(100);g_motion_diag_fault_latched=0x10;healthy(120);CHECK(g_motion_auto_stop_state==MOTION_STOP_PENDING);CHECK(Safety_MotionStopTake((uint8_t*)&i,&session)==1);Safety_MotionStopReport(1,session,4);healthy(140);CHECK(!power_enabled&&g_motion_auto_stop_cut_vm==1);
printf("PASS %u checks (actual Safety and automatic 10s jog code; mocked CAN/power/tick)\n",checks);return 0;}
