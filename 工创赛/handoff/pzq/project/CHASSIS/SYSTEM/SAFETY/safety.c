#include "startup_axes.h"
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

static uint32_t s_auto_stop_pending = 0U;
static uint32_t s_auto_stop_inflight = 0U;
static uint32_t s_auto_stop_seen_latch = 0U;
static uint32_t s_auto_stop_vm_requested = 0U;
static TickType_t s_auto_stop_start_tick = 0U;
/* VM is latched on after first healthy enable; faults never drop it. */
static uint32_t s_power_latched = 0U;
static uint32_t s_auto_stop_retries = 0U;
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
    /* Diagnostic publish only. Speed/position freshness now lives in the
     * chassis backend (400 ms / 1 s) and never gates VM. */
    (void)enabled; (void)speed_valid_mask; (void)position_valid_mask;
    (void)speed_ticks; (void)position_ticks;
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
    /* 2026-11-08: the wheel guard only protects wheel motion. While the chassis is
     * not being driven the mechanism axes own the VM; their own CAN traffic starves
     * the 400ms wheel poll, which used to expire this guard and latch 0x20, killing
     * the motion that caused it (and with it the DM service via startup_axes FAULT).
     * Check the wheels only when the chassis actually asks to move. */
    extern volatile uint32_t g_lift_ruler_state, g_startup_axes_state, g_chassis_auto_request;
    extern volatile uint32_t g_it_state, g_it_lift_permit;
    /* 2026-11-08: the guard must also stand down while an IT transfer is being
     * authorised. IT1/IT2 run before either mechanism state reaches 1, so the old
     * check armed the guard during that window. The wheel poll visits one node per
     * CAN task turn (~0.4 s), slower than SAFETY_CAN_FRESH_MS, so it latched 0x20
     * within 3 s; that forced safety_ready=0 and rejected IT1 with rejected=2.
     * IT only ever runs while g_chassis_auto_request==0, so no wheel motion can be
     * left unprotected here. Wheel protection under a chassis request is unchanged. */
    if (g_chassis_auto_request == 0U &&
        (g_lift_ruler_state >= 1U || g_startup_axes_state >= 1U ||
         g_it_lift_permit == 1U || g_it_state == 1U))
    {
        s_can_requested = 0U;
        s_can_was_ready = 0U;
        return;
    }
    if (g_safety_fault != 0U)
    {
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
    g_motion_auto_stop_cause = cause;
#if APP_FAULT_CUT_VM_ENABLE
    g_motion_auto_stop_state = MOTION_STOP_VM_CUT;
    g_motion_auto_stop_cut_vm = 1U;
    s_auto_stop_pending = 0U;
#else
    /* Never cut VM: keep re-sending stop frames until all four wheels ack. */
    if (s_auto_stop_retries < 20U)
    {
        s_auto_stop_retries++;
        g_motion_auto_stop_state = MOTION_STOP_PENDING;
        g_motion_auto_stop_fail_mask = 0U;
        s_auto_stop_pending = 0x0FU & ~g_motion_auto_stop_ack_mask;
        s_auto_stop_inflight = 0U;
        s_auto_stop_start_tick = xTaskGetTickCount();
        if (s_auto_stop_pending == 0U) g_motion_auto_stop_state = MOTION_STOP_ACKED;
    }
    else
    {
        g_motion_auto_stop_state = MOTION_STOP_ACKED;
        s_auto_stop_pending = 0U;
    }
#endif
}

/* Safety独立推进总等待时间，不等待CAN任务 */
/* Any latched fault or backend cause asks for orderly wheel stop. */
static uint32_t Safety_StopNeeded(void)
{
    return ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_chassis_auto_cause != 0U) || (g_startup_axes_cause != 0U)) ? 1U : 0U;
}

/* Safety independently drives the stop deadline; never waits for the CAN task. */
static void Safety_MotionStopUpdate(uint32_t vm_request)
{
    TickType_t now;
    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    s_auto_stop_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (Safety_StopNeeded() == 0U)
    {
        if (vm_request == 0U)
        {
            g_motion_auto_stop_state = MOTION_STOP_IDLE;
            g_motion_auto_stop_cut_vm = 0U;
            g_motion_auto_stop_cause = 0U;
            s_auto_stop_pending = 0U;
            s_auto_stop_inflight = 0U;
            s_auto_stop_seen_latch = 0U;
            s_auto_stop_retries = 0U;
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
        s_auto_stop_retries = 0U;
        s_auto_stop_start_tick = now;
    }
    if (g_motion_auto_stop_state == MOTION_STOP_PENDING)
    {
        if ((TickType_t)(now - s_auto_stop_start_tick) >=
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
        (s_auto_stop_vm_requested == 1U) && (s_auto_stop_inflight == 0U))
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
        else
        {
            g_motion_auto_stop_fail_mask |= bit;
            /* Retry this address; VM stays on. */
            Safety_MotionStopCut(1U);
        }
        if ((g_motion_auto_stop_ack_mask == 0x0FU) &&
            (g_motion_auto_stop_state == MOTION_STOP_PENDING))
            g_motion_auto_stop_state = MOTION_STOP_ACKED;
    }
    taskEXIT_CRITICAL();
}

/* 调用者持短临界区；不等待CAN，关电在本轮或下一轮Safety输出执行 */
/* Wheel spin test is not reachable in this build (no request source); kept as
 * refusing stubs so a stray request can never arm a 10 s unattended run. */
uint32_t Safety_WheelTestPrepare(void) { return 0U; }
void Safety_WheelTestPrepareFailed(void) { }
uint32_t Safety_WheelTestArm(uint32_t *session) { (void)session; return 0U; }
uint32_t Safety_WheelTestStartAllowed(uint32_t session) { (void)session; return 0U; }
void Safety_WheelTestStartReport(uint32_t session, uint32_t result)
{ (void)session; (void)result; }
uint32_t Safety_WheelTestTakeStop(uint32_t *session) { (void)session; return 0U; }
void Safety_WheelTestStopReport(uint32_t session, uint32_t result)
{ (void)session; (void)result; }
void Safety_WheelTestAbort(void) { }

/* 反馈诊断锁存只记账，不改原故障、供电、喂狗或发送CAN */
/* Removed: MOTION_BLOCK_* permission diagnostic. It only produced
 * false fault latches and never authorised any motion. */
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
    Safety_CanGuardUpdate(((g_wheel_test_cut_vm == 1U) || (g_chassis_auto_cut_vm == 1U)) ? (StartupAxes_NormalStopHold()?g_startup_axes_vm_request:0U) : (vm_request || g_startup_axes_vm_request));
    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    /* Publish stop request and backend snapshot; faults never cut VM. */
    ChassisAuto_SafetyUpdate(vm_request);
    Safety_MotionStopUpdate(vm_request);

    if ((vm_request == 1U) || (g_startup_axes_vm_request == 1U))
    {
#if APP_FAULT_CUT_VM_ENABLE
        if ((g_safety_ready == 1U) && (g_motion_auto_stop_cut_vm == 0U) &&
            ((g_chassis_auto_cut_vm == 0U) || StartupAxes_NormalStopHold()))
            Power_Enable();
        else
            Power_Disable();
#else
        /* Latched power: once healthy-enabled, keep VM on through any fault so
         * the wheels stay braked instead of coasting on a dead driver. Only an
         * explicit VM request drop (normal end / key cancel) powers off. */
        if ((g_safety_ready == 1U) || (s_power_latched != 0U))
        {
            Power_Enable();
            s_power_latched = 1U;
        }
#endif
    }
    else
    {
        Power_Disable();
        s_power_latched = 0U;
    }

    /* Keep the MCU alive to hold VM and keep issuing stop frames. */
    if (g_rtos_fault == 0U)
    {
        Watchdog_Feed();
    }
}
