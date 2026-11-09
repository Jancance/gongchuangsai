#include "dm_can2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

volatile uint32_t g_dm_jog_request = 0U;
/* 只供人工看护空载调试；上电默认不允许点位运动。 */
volatile uint32_t g_dm_point_enable = 0U;
volatile float g_dm_posvel_speed_deg_s = DM_CAN2_POSVEL_SPEED_DEG_S;
volatile float g_dm_posvel_latched_speed_deg_s = 0.0F;
volatile uint32_t g_dm_mode_change_state = 0U;
static uint32_t s_mode_pending = 0U, s_mode_attempted = 0U;
static TickType_t s_mode_tick;
static float s_speed;
volatile float g_dm_pick_target_deg = 177.833679F;
/* 用户明确存在偏差：仅作粗调初值，后续可独立调整，不当最终放料标定。 */
volatile float g_dm_place_target_deg = 33.8401489F;
volatile float g_dm_point_latched_deg = 0.0F;
volatile uint32_t g_dm_jog_state = 0U;
volatile uint32_t g_dm_jog_result = 0U;
volatile uint32_t g_dm_jog_stop_result = 0U;
volatile uint32_t g_dm_jog_tx_count = 0U;
volatile uint32_t g_dm_jog_rx_count = 0U;
volatile uint32_t g_dm_jog_elapsed_ms = 0U;
volatile float g_dm_jog_start_deg = 0.0F;
volatile float g_dm_jog_target_deg = 0.0F;
volatile float g_dm_jog_current_deg = 0.0F;
volatile float g_dm_jog_reference_deg = 0.0F;
volatile float g_dm_jog_tracking_error_deg = 0.0F;

#define JOG_RAD_PER_DEG 0.01745329252F
#define JOG_DEG_PER_RAD 57.2957795F
static TickType_t s_started, s_step, s_move_started, s_last_service;
static uint32_t s_cycle, s_pending = 0U, s_tx_done = 0U;
static uint32_t s_enable_possible = 0U, s_settled = 0U, s_clean_done = 0U;
static uint8_t s_mailbox = CAN_TxStatus_NoMailBox;
static int32_t s_sign = 1;
static uint32_t s_point_mode = 0U;
static float s_point_goal;
static float DM_CAN2_DisplayRelativeDeg(float raw_deg);
static float s_pmax, s_vmax, s_tmax, s_start, s_target, s_reference;
static DM_CAN2_Feedback s_feedback;

static float JogAbs(float value) { return value < 0.0F ? -value : value; }

uint32_t DM_CAN2_JogIsBusy(void)
{
    return (g_dm_jog_state >= 1U && g_dm_jog_state <= 6U) ? 1U : 0U;
}

static uint32_t JogSend(CanTxMsg *tx, TickType_t now)
{
    CanRxMsg discard;
    uint32_t i;
    for (i = 0U; i < 3U && CAN_MessagePending(CAN2, CAN_FIFO0); i++)
        CAN_Receive(CAN2, CAN_FIFO0, &discard);
    CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
    s_mailbox = CAN_Transmit(CAN2, tx);
    if (s_mailbox > 2U) return 0U;
    s_pending = 1U;
    s_tx_done = 0U;
    s_step = now;
    g_dm_jog_tx_count++;
    return 1U;
}

static void JogStop(uint32_t reason, TickType_t now)
{
    CanTxMsg tx;
    if (s_pending && s_mailbox <= 2U) CAN_CancelTransmit(CAN2, s_mailbox);
    s_pending = 0U;
    s_mode_pending = 0U;
    DM_CAN2_CancelRead();
    g_dm_jog_result = reason;
    if (!s_enable_possible)
    { g_dm_jog_state = 8U; return; }
    g_dm_jog_state = 6U;
    g_dm_jog_stop_result = 0U;
    s_step = now;
    if (!DM_CAN2_BuildSwitch(0xFDU, &tx) || !JogSend(&tx, now))
    { g_dm_jog_stop_result = 2U; g_dm_jog_state = 8U; }
}

/* 返回0继续等、1收到新状态、5发送失败、6回复超时。 */
static uint32_t JogPoll(TickType_t now)
{
    uint32_t status, i;
    if (!s_pending) return 0U;
    if (!s_tx_done)
    {
        status = CAN_TransmitStatus(CAN2, s_mailbox);
        if (status == CAN_TxStatus_Ok) s_tx_done = 1U;
        else if (status == CAN_TxStatus_Failed ||
                 (TickType_t)(now - s_step) >= pdMS_TO_TICKS(30U)) return 5U;
    }
    if (!s_tx_done) return 0U;
    if (CAN_GetFlagStatus(CAN2, CAN_FLAG_FOV0) != RESET) return 6U;
    for (i = 0U; i < 3U && CAN_MessagePending(CAN2, CAN_FIFO0); i++)
    {
        CAN_Receive(CAN2, CAN_FIFO0, &g_dm_can2_last_rx);
        g_dm_jog_rx_count++;
        if (!DM_CAN2_ParseStatus(&g_dm_can2_last_rx, s_pmax, s_vmax, s_tmax, &s_feedback))
            continue;
        if (g_dm_jog_state == 6U && s_feedback.motor_state != 0U) continue;
        s_pending = 0U;
        s_mailbox = CAN_TxStatus_NoMailBox;
        g_dm_can2_position_rad = s_feedback.position_rad;
        g_dm_can2_position_deg = s_feedback.position_rad * JOG_DEG_PER_RAD;
        g_dm_can2_velocity_rad_s = s_feedback.velocity_rad_s;
        g_dm_can2_torque_nm = s_feedback.torque_nm;
        g_dm_can2_motor_state = s_feedback.motor_state;
        g_dm_can2_position_raw = s_feedback.position_raw;
        g_dm_can2_velocity_raw = s_feedback.velocity_raw;
        g_dm_can2_torque_raw = s_feedback.torque_raw;
        g_dm_can2_feedback_valid = 1U;
        g_dm_can2_feedback_tick = (uint32_t)now;
        g_dm_jog_current_deg = g_dm_can2_position_deg;
        return 1U;
    }
    if ((TickType_t)(now - s_step) >= pdMS_TO_TICKS(100U)) return 6U;
    return 0U;
}

static uint32_t JogWithinLimits(void)
{
    float low = (s_start < s_target ? s_start : s_target) - 2.0F * JOG_RAD_PER_DEG;
    float high = (s_start > s_target ? s_start : s_target) + 2.0F * JOG_RAD_PER_DEG;
    g_dm_jog_tracking_error_deg = (s_reference - s_feedback.position_rad) * JOG_DEG_PER_RAD;
    if (s_feedback.motor_state > 1U) return 7U;
    /* 位置速度模式直接发最终目标，不能再套MIT参考跟随8度判停。
     * 用户指定参考速度300度/秒，不沿用旧低速0.5rad/s阈值。 */
    if (s_feedback.position_rad < low || s_feedback.position_rad > high) return 8U;
    return 0U;
}

static void JogReference(float reference, TickType_t now)
{
    CanTxMsg tx;
    s_reference = reference;
    g_dm_jog_reference_deg = reference * JOG_DEG_PER_RAD;
    if (!DM_CAN2_BuildPosVel(reference, s_speed, &tx) || !JogSend(&tx, now)) JogStop(5U, now);
}

static void JogService(uint32_t ready, uint32_t busy)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t request = g_dm_jog_request, reply, reason;
    float current_work_deg, delta_deg;
    CanTxMsg tx;
    g_dm_jog_request = 0U;
    if (request == 3U && (DM_CAN2_JogIsBusy() || s_enable_possible))
    { s_clean_done = 0U; JogStop(2U, now); return; }
    if (!DM_CAN2_JogIsBusy())
    {
        if (request != 1U && request != 2U && request != 4U && request != 5U) return;
        if (!ready || busy || s_enable_possible)
        { g_dm_jog_state = 8U; g_dm_jog_result = 3U; return; }
        s_mode_pending = 0U; s_mode_attempted = 0U; g_dm_mode_change_state = 0U;
        if (!(g_dm_posvel_speed_deg_s > 0.0F && g_dm_posvel_speed_deg_s <= DM_CAN2_POSVEL_SPEED_DEG_S))
        { g_dm_jog_state = 8U; g_dm_jog_result = 13U; return; }
        g_dm_posvel_latched_speed_deg_s = g_dm_posvel_speed_deg_s;
        s_speed = g_dm_posvel_latched_speed_deg_s * JOG_RAD_PER_DEG;
        s_point_mode = (request == 4U || request == 5U) ? request : 0U;
        if (s_point_mode)
        {
            if (g_dm_point_enable != 1U)
            { g_dm_jog_state = 8U; g_dm_jog_result = 3U; return; }
            s_point_goal = request == 4U ? g_dm_pick_target_deg : g_dm_place_target_deg;
            /* 0到180度仅为本轮调试候选区，不是实测安全机械限位。含NaN/Inf拒绝。 */
            if (!(s_point_goal >= 0.0F && s_point_goal <= 180.0F))
            { g_dm_jog_state = 8U; g_dm_jog_result = 13U; return; }
            g_dm_point_latched_deg = s_point_goal;
        }
        s_sign = request == 1U ? 1 : -1;
        s_started = now;
        s_last_service = now;
        s_cycle = g_dm_can2_read_cycle;
        s_clean_done = 0U;
        s_settled = 0U;
        s_pending = 0U;
        g_dm_jog_result = 0U;
        g_dm_jog_stop_result = 0U;
        g_dm_jog_tx_count = 0U;
        g_dm_jog_rx_count = 0U;
        g_dm_jog_elapsed_ms = 0U;
        g_dm_jog_state = 1U;
        g_dm_can2_read_request = 2U;
        return;
    }
    /* 忙时新启动请求直接消费，不排队、不延长动作时间；点位参数已锁存。 */
    if (g_dm_jog_state != 6U)
    {
        if (!ready || busy || (s_point_mode && g_dm_point_enable != 1U))
        { JogStop(3U, now); return; }
        /* 用户指定：动作/到位等待不设总期限，显式停止仍可中止。 */
        if (g_dm_jog_state > 1U &&
            (TickType_t)(now - s_last_service) > pdMS_TO_TICKS(50U))
        { JogStop(12U, now); return; }
        if (g_dm_jog_state > 1U && (CAN2->ESR & CAN_ESR_BOFF))
        { JogStop(5U, now); return; }
    }
    s_last_service = now;

    if (s_mode_pending)
    {
        if (s_mode_pending == 1U)
        {
            reply = CAN_TransmitStatus(CAN2, s_mailbox);
            if (reply == CAN_TxStatus_Failed ||
                (reply != CAN_TxStatus_Ok && (TickType_t)(now - s_step) >= pdMS_TO_TICKS(30U)))
            { s_mode_pending = 0U; JogStop(5U, now); return; }
            if (reply != CAN_TxStatus_Ok) return;
            s_pending = 0U; s_mailbox = CAN_TxStatus_NoMailBox;
            s_mode_pending = 2U; s_mode_tick = now;
        }
        if ((TickType_t)(now - s_mode_tick) < pdMS_TO_TICKS(50U)) return;
        s_mode_pending = 0U; s_cycle = g_dm_can2_read_cycle;
        g_dm_can2_read_request = 2U;
        g_dm_mode_change_state = 2U;
        return;
    }
    if (g_dm_jog_state == 1U)
    {
        if (g_dm_can2_read_cycle == s_cycle || g_dm_can2_state != 4U)
        {
            if (g_dm_can2_state == 5U ||
                (TickType_t)(now - s_started) > pdMS_TO_TICKS(2500U)) JogStop(4U, now);
            return;
        }
        if (g_dm_can2_result != 1U || g_dm_can2_valid_mask != 7U ||
            g_dm_can2_range_mask != 7U || !g_dm_can2_feedback_valid ||
            (TickType_t)(now - (TickType_t)g_dm_can2_feedback_tick) > pdMS_TO_TICKS(100U) ||
            g_dm_can2_esc_id != 1U || g_dm_can2_master_id != 0U ||
            (g_dm_can2_ctrl_mode != 1U && g_dm_can2_ctrl_mode != DM_CAN2_MODE) || g_dm_can2_motor_state != 0U ||
            JogAbs(g_dm_can2_pmax - 12.5F) > 0.001F ||
            JogAbs(g_dm_can2_vmax - 30.0F) > 0.001F ||
            JogAbs(g_dm_can2_tmax - 10.0F) > 0.001F ||
            JogAbs(g_dm_can2_velocity_rad_s) > 0.035F)
        { JogStop(11U, now); return; }
        if (g_dm_can2_ctrl_mode != DM_CAN2_MODE)
        {
            if (s_mode_attempted) { g_dm_mode_change_state = 4U; JogStop(15U, now); return; }
            /* 当前新鲜状态已确认失能且零速，只切RAM模式，不保存Flash/零点。 */
            s_mode_attempted = 1U; g_dm_mode_change_state = 1U;
            DM_CAN2_CancelRead();
            if (!DM_CAN2_BuildMode2(&tx) || !JogSend(&tx, now)) { JogStop(5U, now); return; }
            s_mode_pending = 1U; return;
        }
        g_dm_mode_change_state = 3U;
        s_pmax = g_dm_can2_pmax;
        s_vmax = g_dm_can2_vmax;
        s_tmax = g_dm_can2_tmax;
        s_start = g_dm_can2_position_rad;
        if (s_point_mode)
        {
            /* 从本次新鲜原始反馈选同一工作分支，沿本轮人工确认的半圈区域计算。
             * 不能用保存的raw角直接下发，也不能据此推断掉电转轴后的线束圈数。
             * 起点留2度调试余量，4仅逆时针/近同点，5仅顺时针/近同点。 */
            current_work_deg = DM_CAN2_DisplayRelativeDeg(s_start * JOG_DEG_PER_RAD);
            delta_deg = s_point_goal - current_work_deg;
            if (!(current_work_deg >= -2.0F && current_work_deg <= 182.0F) ||
                (s_point_mode == 4U && delta_deg < -0.1F) ||
                (s_point_mode == 5U && delta_deg > 0.1F))
            { JogStop(14U, now); return; }
            s_target = s_start + delta_deg * JOG_RAD_PER_DEG;
        }
        else s_target = s_start + (float)s_sign * DM_CAN2_JOG_DEG * JOG_RAD_PER_DEG;
        if (!(s_start > -s_pmax && s_start < s_pmax && s_target > -s_pmax && s_target < s_pmax))
        { JogStop(11U, now); return; }
        g_dm_jog_start_deg = s_start * JOG_DEG_PER_RAD;
        g_dm_jog_target_deg = s_target * JOG_DEG_PER_RAD;
        g_dm_jog_current_deg = g_dm_jog_start_deg;
        /* 先预置新读到的当前位置，再使能；不发送0度目标。 */
        s_enable_possible = 1U;
        g_dm_jog_state = 2U;
        s_reference = s_start;
        g_dm_jog_reference_deg = g_dm_jog_start_deg;
        if (!DM_CAN2_BuildPosVel(s_start, 0.0F, &tx) ||
            !JogSend(&tx, now)) JogStop(5U, now);
        return;
    }

    reply = JogPoll(now);
    if (g_dm_jog_state == 6U)
    {
        if (reply == 1U)
        {
            s_enable_possible = 0U;
            g_dm_jog_stop_result = 1U;
            g_dm_jog_state = (s_clean_done && g_dm_jog_result == 1U) ? 7U : 8U;
        }
        else if (reply >= 5U || (TickType_t)(now - s_step) > pdMS_TO_TICKS(200U))
        {
            if (s_pending && s_mailbox <= 2U) CAN_CancelTransmit(CAN2, s_mailbox);
            s_pending = 0U;
            g_dm_jog_stop_result = 2U;
            if (g_dm_jog_result == 1U) g_dm_jog_result = (reply == 5U) ? 5U : 6U;
            g_dm_jog_state = 8U;
        }
        return;
    }
    if (reply >= 5U) { JogStop(reply, now); return; }
    if (reply != 1U) return;
    if ((g_dm_jog_state == 4U || g_dm_jog_state == 5U) &&
        s_feedback.motor_state != 1U) { JogStop(7U, now); return; }
    reason = JogWithinLimits();
    if (reason) { JogStop(reason, now); return; }
    if (g_dm_jog_state == 2U)
    {
        if (s_feedback.motor_state != 0U) { JogStop(7U, now); return; }
        g_dm_jog_state = 3U;
        if (!DM_CAN2_BuildSwitch(0xFCU, &tx) || !JogSend(&tx, now)) JogStop(5U, now);
        return;
    }
    if (g_dm_jog_state == 3U)
    {
        if (s_feedback.motor_state != 1U) { JogStop(7U, now); return; }
        s_move_started = now;
        g_dm_jog_state = 4U;
    }
    if (g_dm_jog_state == 4U || g_dm_jog_state == 5U)
    {
        g_dm_jog_elapsed_ms = (uint32_t)((now - s_move_started) * portTICK_PERIOD_MS);
        if (JogAbs(s_target - s_feedback.position_rad) <= 3.0F * JOG_RAD_PER_DEG &&
            JogAbs(s_feedback.velocity_rad_s) <= 0.035F) s_settled++;
        else s_settled = 0U;
        if (s_settled >= 15U) { s_clean_done = 1U; JogStop(1U, now); return; }
        /* 与参考代码一致持续发最终位置和速度上限，未到位持续等待。 */
        JogReference(s_target, now);
    }
}

/* 同圈工作分支：显示使用；人工准入的点位调试再结合新鲜原始角及范围/方向检查使用。 */
static float DM_CAN2_DisplayRelativeDeg(float raw_deg)
{
    float relative = fmodf(raw_deg - DM_CAN2_LEFT_ZERO_DEG + 90.0F, 360.0F);
    if (relative < 0.0F) relative += 360.0F;
    return relative - 90.0F;
}

void DM_CAN2_ModuleService(uint32_t ready, uint32_t busy)
{
    if (g_dm_jog_state >= 2U && g_dm_jog_state <= 6U)
        g_dm_can2_read_request = 0U;
    else DM_CAN2_Service(ready, busy);
    JogService(ready, busy);
    /* 与只读/试转的最新反馈统一更新；仅显示，不进入运动目标或限幅。 */
    if (g_dm_can2_feedback_valid == 1U)
        g_dm_can2_logical_position_deg = DM_CAN2_DisplayRelativeDeg(g_dm_can2_position_deg);
    else
        g_dm_can2_logical_position_deg = 0.0F;
}
