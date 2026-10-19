#include "safety.h"
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
    Safety_CanGuardUpdate(vm_request);
    g_safety_ready = 0U;
    if ((g_safety_fault == 0U) && (bad == 0U) &&
        (can_test_result == 1U))
    {
        s_was_ready = 1U;
        g_safety_ready = 1U;
    }

    if ((g_safety_ready == 1U) && (vm_request == 1U))
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
