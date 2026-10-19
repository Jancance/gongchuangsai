#include "chassis_auto_test.h"
#include "chassis_kinematics.h"
#include "can.h"
#include "safety.h"
#include "freertos_hooks.h"
#include "task.h"
#include <stddef.h>

/* 将新读到的3A供原CAN监督使用，不伪造采样时间 */
extern volatile uint32_t g_can_poll_result[4];
extern volatile uint32_t g_can_poll_flags[4];
extern volatile TickType_t g_can_poll_tick[4];
extern volatile uint32_t g_can_poll_count[4];

volatile uint32_t g_chassis_auto_request = 0U;
volatile uint32_t g_chassis_auto_dry_run = 1U;
volatile uint32_t g_chassis_auto_mode = 1U;
volatile uint32_t g_chassis_auto_state = CHASSIS_AUTO_IDLE;
volatile uint32_t g_chassis_auto_segment = 0U;
volatile uint32_t g_chassis_auto_active = 0U;
volatile uint32_t g_chassis_auto_cut_vm = 0U;
volatile uint32_t g_chassis_auto_session = 0U;
volatile uint32_t g_chassis_auto_result = 0U;
volatile uint32_t g_chassis_auto_cause = 0U;
volatile uint32_t g_chassis_auto_io_result = 0U;
volatile uint32_t g_chassis_auto_last_address = 0U;
volatile uint32_t g_chassis_auto_start_mask[3] = {0U};
volatile uint32_t g_chassis_auto_stop_mask[3] = {0U};
volatile int32_t g_chassis_auto_driver_rpm[4] = {0};

static uint32_t s_vm_requested, s_used, s_mode;
static uint32_t s_motor_index, s_monitor_cursor, s_monitor_between;
static uint32_t s_status_valid, s_speed_valid, s_run_started, s_stop_due;
static uint32_t s_status[4];
static TickType_t s_status_tick[4], s_speed_tick[4];
static TickType_t s_total_tick, s_phase_tick, s_run_tick;
static ChassisMotorCommand s_commands[3][4];

/* 以下带Locked的函数只在短临界区内操作内存，不等待CAN */
static void Auto_CutLocked(uint32_t cause)
{
    g_chassis_auto_active = 0U;
    g_chassis_auto_cut_vm = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_FAULT;
    g_chassis_auto_cause = cause;
    g_chassis_auto_result = 2U;
}

static void Auto_EnterLocked(uint32_t state, TickType_t now)
{
    g_chassis_auto_state = state;
    s_phase_tick = now;
    s_motor_index = 0U;
    s_monitor_between = 0U;
}

static uint32_t Auto_BaseHealthy(void)
{
    return ((g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U)) ? 1U : 0U;
}

static uint32_t Auto_AllStatusSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_status_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_status_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_AllSpeedSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_speed_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_ZeroSpeedLocked(void)
{
    uint32_t i;
    for (i = 0U; i < 4U; i++) if (g_chassis_auto_driver_rpm[i] != 0) return 0U;
    return 1U;
}

/* 独立时限和反馈检查，CAN停推进时由Safety继续调用 */
static void Auto_CheckLocked(TickType_t now)
{
    uint32_t i, state = g_chassis_auto_state;
    TickType_t run_age;
    if (g_chassis_auto_active != 1U) return;
    if (!Auto_BaseHealthy()) { Auto_CutLocked(2U); return; }
    if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_TOTAL_MS))
    { Auto_CutLocked(6U); return; }
    if (state <= CHASSIS_AUTO_VERIFY_ON)
    {
        if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_PREP_MS))
            Auto_CutLocked(6U);
        return;
    }
    for (i = 0U; i < 4U; i++)
    {
        if (((s_status_valid & (1UL << i)) == 0U) ||
            ((s_speed_valid & (1UL << i)) == 0U) ||
            ((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)))
        { Auto_CutLocked(3U); return; }
        if ((s_status[i] & 0x0DU) != 1U) { Auto_CutLocked(2U); return; }
    }
    if ((state == CHASSIS_AUTO_SETTLE) &&
        ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_SETTLE_MS)))
    { Auto_CutLocked(7U); return; }
    if (!s_run_started || (state == CHASSIS_AUTO_SETTLE)) return;
    run_age = (TickType_t)(now - s_run_tick);
    if (run_age >= pdMS_TO_TICKS(CHASSIS_AUTO_RUN_MS)) s_stop_due = 1U;
    if ((run_age >= pdMS_TO_TICKS(CHASSIS_AUTO_RUN_MS + CHASSIS_AUTO_STOP_MS)) &&
        (g_chassis_auto_stop_mask[g_chassis_auto_segment] != 0x0FU))
    { Auto_CutLocked(6U); return; }
    if ((s_mode == 0U) && (state == CHASSIS_AUTO_RUN) &&
        (run_age >= pdMS_TO_TICKS(500U)))
    {
        for (i = 0U; i < 4U; i++)
        {
            int32_t raw = g_chassis_auto_driver_rpm[i];
            /* 不把1RPM量化为0误判为停转；只拒绝明显超速和新采到的反向 */
            if ((raw < -(int32_t)CHASSIS_AUTO_SPEED_LIMIT_RPM) ||
                (raw > (int32_t)CHASSIS_AUTO_SPEED_LIMIT_RPM)) { Auto_CutLocked(8U); return; }
            if (((TickType_t)(now - s_speed_tick[i]) <= run_age - pdMS_TO_TICKS(500U)) &&
                (((s_commands[g_chassis_auto_segment][i].direction == 0U) && (raw < 0)) ||
                 ((s_commands[g_chassis_auto_segment][i].direction == 1U) && (raw > 0))))
            { Auto_CutLocked(8U); return; }
        }
    }
}

void ChassisAuto_Init(void)
{
    uint32_t i;
    g_chassis_auto_request = 0U;
    g_chassis_auto_dry_run = g_chassis_auto_mode = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_IDLE;
    g_chassis_auto_segment = g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
    g_chassis_auto_session = g_chassis_auto_result = g_chassis_auto_cause = 0U;
    g_chassis_auto_io_result = g_chassis_auto_last_address = 0U;
    s_vm_requested = s_used = s_mode = 0U;
    s_motor_index = s_monitor_cursor = s_monitor_between = 0U;
    s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
    s_total_tick = s_phase_tick = s_run_tick = 0U;
    for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
    for (i = 0U; i < 4U; i++)
    { s_status[i] = 0U; s_status_tick[i] = s_speed_tick[i] = 0U; g_chassis_auto_driver_rpm[i] = 0; }
}

uint32_t ChassisAuto_Begin(uint32_t dry_run)
{
    uint32_t i, j, accepted = 0U;
    taskENTER_CRITICAL();
    if ((dry_run <= 1U) && s_vm_requested && !s_used && !g_chassis_auto_active &&
        !g_chassis_auto_cut_vm && Auto_BaseHealthy())
    {
        for (i = 0U; i < 3U; i++)
        {
            if (!Chassis_BasicTestCommands(&g_chassis_geometry, i, s_commands[i])) break;
            /* 三个基本动作每轮原为1RPM，保持其方向组合，统一放大到5RPM。 */
            for (j = 0U; j < 4U; j++)
                s_commands[i][j].speed_raw = (uint16_t)CHASSIS_AUTO_TARGET_RPM;
#if APP_HWT_TURN_TEST && APP_HWT_TURN_CLOCKWISE
            /* 仅反转单段转动的四轮方向；原运动学与三段模式不改。 */
            if (i == 2U)
                for (j = 0U; j < 4U; j++) s_commands[i][j].direction ^= 1U;
#endif
        }
        if (i == 3U)
        {
            s_used = 1U; s_mode = dry_run; g_chassis_auto_mode = dry_run;
            g_chassis_auto_session++;
            g_chassis_auto_segment = CHASSIS_AUTO_FIRST_SEGMENT;
            g_chassis_auto_result = g_chassis_auto_cause = 0U;
            s_total_tick = xTaskGetTickCount();
            s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
            s_monitor_cursor = 0U;
            for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
            g_chassis_auto_active = 1U;
            Auto_EnterLocked(CHASSIS_AUTO_DISABLE, s_total_tick);
            accepted = 1U;
        }
    }
    if (!accepted) g_chassis_auto_result = 9U;
    taskEXIT_CRITICAL();
    return accepted;
}

void ChassisAuto_SafetyUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (!s_vm_requested)
    {
        if (g_chassis_auto_active)
        { g_chassis_auto_state = CHASSIS_AUTO_CANCELLED; g_chassis_auto_result = 3U; }
        g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
        s_used = 0U;
    }
    else Auto_CheckLocked(xTaskGetTickCount());
    taskEXIT_CRITICAL();
}

void ChassisAuto_Abort(void)
{
    taskENTER_CRITICAL();
    if (g_chassis_auto_active) Auto_CutLocked(9U);
    taskEXIT_CRITICAL();
}

static void Auto_Monitor(uint32_t session)
{
    uint32_t cursor = s_monitor_cursor, i = cursor / 2U, result;
    uint32_t is_speed = cursor & 1U;
    int32_t rpm = 0;
    CanRxMsg reply = {0};
    if (is_speed) result = CAN1_ReadSpeed((uint8_t)(i + 1U), &reply, &rpm);
    else result = CAN1_ReadStatus((uint8_t)(i + 1U), &reply);
    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active)
    {
        g_chassis_auto_last_address = i + 1U; g_chassis_auto_io_result = result;
        if (result != 1U)
        {
            if (is_speed) s_speed_valid &= ~(1UL << i);
            else { s_status_valid &= ~(1UL << i); g_can_poll_result[i] = result; }
            Auto_CutLocked(3U);
        }
        else if (is_speed)
        {
            g_chassis_auto_driver_rpm[i] = rpm;
            s_speed_tick[i] = xTaskGetTickCount(); s_speed_valid |= 1UL << i;
        }
        else
        {
            s_status[i] = reply.Data[1]; s_status_tick[i] = xTaskGetTickCount();
            s_status_valid |= 1UL << i;
            g_can_poll_result[i] = 1U; g_can_poll_flags[i] = reply.Data[1];
            g_can_poll_tick[i] = s_status_tick[i]; g_can_poll_count[i]++;
            if ((reply.Data[1] & 0x0CU) != 0U) Auto_CutLocked(2U);
        }
        s_monitor_cursor = (cursor + 1U) % 8U;
    }
    taskEXIT_CRITICAL();
}

/* 仅CAN任务调用：一轮至多一笔有界交易，不在临界区内等待 */
void ChassisAuto_CanStep(void)
{
    uint32_t state, session, index, result, expected, i;
    TickType_t now;
    CanRxMsg reply = {0};
    taskENTER_CRITICAL();
    now = xTaskGetTickCount(); Auto_CheckLocked(now);
    if (!g_chassis_auto_active) { taskEXIT_CRITICAL(); return; }
    state = g_chassis_auto_state; session = g_chassis_auto_session;
    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)
    { Auto_EnterLocked(CHASSIS_AUTO_STOP, now); state = CHASSIS_AUTO_STOP; }
    if ((state == CHASSIS_AUTO_DISABLE) || (state == CHASSIS_AUTO_ENABLE) ||
        (state == CHASSIS_AUTO_START))
    {
        if (s_monitor_between)
        { s_monitor_between = 0U; taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    }
    if ((state == CHASSIS_AUTO_VERIFY_OFF) || (state == CHASSIS_AUTO_VERIFY_ON))
    {
        expected = (state == CHASSIS_AUTO_VERIFY_ON) ? 1U : 0U;
        if (Auto_AllStatusSinceLocked(now) &&
            ((state == CHASSIS_AUTO_VERIFY_OFF) || Auto_AllSpeedSinceLocked(now)))
        {
            for (i = 0U; i < 4U; i++) if ((s_status[i] & 0x0DU) != expected) break;
            if (i != 4U) Auto_CutLocked(1U);
            else if ((state == CHASSIS_AUTO_VERIFY_ON) && !Auto_ZeroSpeedLocked())
                Auto_CutLocked(7U);
            else if (state == CHASSIS_AUTO_VERIFY_OFF) Auto_EnterLocked(CHASSIS_AUTO_ENABLE, now);
            else { s_run_started = s_stop_due = 0U; Auto_EnterLocked(CHASSIS_AUTO_START, now); }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_SETTLE)
    {
        if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&
            ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS)))
        {
            if (g_chassis_auto_segment == 2U)
            {
                g_chassis_auto_state = CHASSIS_AUTO_DONE;
                g_chassis_auto_result = 1U; g_chassis_auto_cause = 0U;
                g_chassis_auto_active = 0U; g_chassis_auto_cut_vm = 1U;
            }
            else
            {
                g_chassis_auto_segment++; s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_RUN)
    { taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    index = s_motor_index;
    if (index >= 4U) { Auto_CutLocked(6U); taskEXIT_CRITICAL(); return; }
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    { s_run_tick = now; s_run_started = 1U; }
    taskEXIT_CRITICAL();

    if (state == CHASSIS_AUTO_DISABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 0U, &reply);
    else if (state == CHASSIS_AUTO_ENABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 1U, &reply);
    else if (state == CHASSIS_AUTO_START)
    {
        ChassisMotorCommand command = s_commands[g_chassis_auto_segment][index];
        result = (s_mode == 1U) ? 1U : CAN1_SetEmmSpeedRaw(command.address,
            command.direction, command.speed_raw, 0U, &reply);
    }
    else if (state == CHASSIS_AUTO_STOP)
        result = (s_mode == 1U) ? 1U : CAN1_StopMotor((uint8_t)(index + 1U), &reply);
    else return;

    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active &&
        (g_chassis_auto_state == state))
    {
        g_chassis_auto_last_address = index + 1U; g_chassis_auto_io_result = result;
        if (result != 1U)
            Auto_CutLocked((state == CHASSIS_AUTO_START) ? 4U :
                           ((state == CHASSIS_AUTO_STOP) ? 5U : 1U));
        else
        {
            if (state == CHASSIS_AUTO_START)
                g_chassis_auto_start_mask[g_chassis_auto_segment] |= 1UL << index;
            if (state == CHASSIS_AUTO_STOP)
                g_chassis_auto_stop_mask[g_chassis_auto_segment] |= 1UL << index;
            s_motor_index++;
            if (s_motor_index == 4U)
            {
                if (state == CHASSIS_AUTO_DISABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_OFF, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_ENABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_ON, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_START) Auto_EnterLocked(CHASSIS_AUTO_RUN, xTaskGetTickCount());
                else Auto_EnterLocked(CHASSIS_AUTO_SETTLE, xTaskGetTickCount());
            }
            else if (state != CHASSIS_AUTO_STOP) s_monitor_between = 1U;
        }
    }
    taskEXIT_CRITICAL();
}
