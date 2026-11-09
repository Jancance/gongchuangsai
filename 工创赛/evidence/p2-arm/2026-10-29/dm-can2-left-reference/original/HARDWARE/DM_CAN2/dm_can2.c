#include "dm_can2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include <string.h>

volatile uint32_t g_dm_can2_read_request = 0U;
volatile uint32_t g_dm_can2_read_kind = 0U;
volatile uint32_t g_dm_can2_read_cycle = 0U;
volatile uint32_t g_dm_can2_feedback_tick = 0U;
volatile uint32_t g_dm_can2_state = 0U;
volatile uint32_t g_dm_can2_result = 0U;
volatile uint32_t g_dm_can2_init_result = 0U;
volatile uint32_t g_dm_can2_esc_id = 0U;
volatile uint32_t g_dm_can2_master_id = 0U;
volatile uint32_t g_dm_can2_ctrl_mode = 0U;
volatile uint32_t g_dm_can2_valid_mask = 0U;
volatile uint32_t g_dm_can2_range_mask = 0U;
volatile uint32_t g_dm_can2_feedback_valid = 0U;
volatile float g_dm_can2_pmax = 0.0F;
volatile float g_dm_can2_vmax = 0.0F;
volatile float g_dm_can2_tmax = 0.0F;
volatile float g_dm_can2_position_rad = 0.0F;
volatile float g_dm_can2_position_deg = 0.0F;
volatile float g_dm_can2_velocity_rad_s = 0.0F;
volatile float g_dm_can2_torque_nm = 0.0F;
volatile uint32_t g_dm_can2_motor_state = 0U;
volatile uint32_t g_dm_can2_position_raw = 0U;
volatile uint32_t g_dm_can2_velocity_raw = 0U;
volatile uint32_t g_dm_can2_torque_raw = 0U;
volatile uint32_t g_dm_can2_tx_count = 0U;
volatile uint32_t g_dm_can2_rx_count = 0U;
volatile uint32_t g_dm_can2_ignored_count = 0U;
volatile uint32_t g_dm_can2_last_rid = 0U;
volatile uint32_t g_dm_can2_esr = 0U;
volatile uint32_t g_dm_can2_pclk1_hz = 0U;
volatile uint32_t g_dm_can2_filter_start = 0U;
volatile uint32_t g_dm_can2_can1_btr_before = 0U;
volatile uint32_t g_dm_can2_can1_btr_after = 0U;
CanRxMsg g_dm_can2_replies[7];
CanRxMsg g_dm_can2_last_rx;

static const uint8_t s_rids[6] = {0x08U, 0x07U, 0x0AU, 0x15U, 0x16U, 0x17U};
static const uint32_t s_expected[3] = {DM_CAN2_NODE_ID, DM_CAN2_MASTER_ID, DM_CAN2_MODE};
static uint32_t s_index = 0U;
static uint32_t s_count = 3U;
static uint8_t s_mailbox = CAN_TxStatus_NoMailBox;
static TickType_t s_started = 0U;
static TickType_t s_step_started = 0U;

static uint32_t DM_CAN2_IsBusy(void)
{
    return (g_dm_can2_state >= 1U && g_dm_can2_state <= 3U) ? 1U : 0U;
}

static void DM_CAN2_Fail(uint32_t reason)
{
    if (s_mailbox <= 2U) CAN_CancelTransmit(CAN2, s_mailbox);
    s_mailbox = CAN_TxStatus_NoMailBox;
    g_dm_can2_esr = CAN2->ESR;
    g_dm_can2_valid_mask = 0U;
    g_dm_can2_range_mask = 0U;
    g_dm_can2_feedback_valid = 0U;
    g_dm_can2_result = reason;
    g_dm_can2_state = 5U;
}

static uint32_t DM_CAN2_Init(void)
{
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;

    RCC_GetClocksFreq(&clocks);
    g_dm_can2_pclk1_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != 42000000U) return 2U;
    /* CAN2依赖CAN1时钟；仅开时钟，不复位或重配CAN1。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
    g_dm_can2_can1_btr_before = CAN1->BTR;
    g_dm_can2_filter_start = (CAN1->FMR >> 8) & 0x3FU;
    /* 保持当前默认14/14分配，不擅自改变其他控制器的过滤器归属。 */
    if (g_dm_can2_filter_start != 14U) return 9U;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_11tq;
    can.CAN_BS2 = CAN_BS2_2tq;
    /* 42MHz / 3 / (1+11+2) = 1Mbps；CAN1原500kbps不动。 */
    can.CAN_Prescaler = 3U;
    can.CAN_NART = ENABLE;
    if (CAN_Init(CAN2, &can) != CAN_InitStatus_Success) return 3U;

    memset(&filter, 0, sizeof(filter));
    filter.CAN_FilterNumber = 14U;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
    filter.CAN_FilterIdHigh = 0U;
    filter.CAN_FilterIdLow = 0U;
    filter.CAN_FilterMaskIdHigh = 0xFFE0U;
    /* 精确匹配标准ID0的数据帧，屏蔽扩展帧及远程帧。 */
    filter.CAN_FilterMaskIdLow = 0x0006U;
    filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);
    g_dm_can2_can1_btr_after = CAN1->BTR;
    if (g_dm_can2_can1_btr_after != g_dm_can2_can1_btr_before) return 9U;
    return 1U;
}

void DM_CAN2_CancelRead(void)
{
    g_dm_can2_read_request = 0U;
    if (DM_CAN2_IsBusy()) DM_CAN2_Fail(8U);
}

static void DM_CAN2_BeginRead(TickType_t now)
{
    CanTxMsg tx;
    CanRxMsg discard;
    uint32_t i;
    /* 每次查询前丢弃旧FIFO，最终仍核帧ID、长度、操作码和寄存器。 */
    for (i = 0U; i < 3U && CAN_MessagePending(CAN2, CAN_FIFO0) != 0U; i++)
        CAN_Receive(CAN2, CAN_FIFO0, &discard);
    CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
    g_dm_can2_last_rid = (s_index == 6U) ? 0xCCU : s_rids[s_index];
    if (s_index == 6U)
    {
        if (!DM_CAN2_BuildStatus(&tx)) { DM_CAN2_Fail(6U); return; }
    }
    else if (!DM_CAN2_BuildRead(s_rids[s_index], &tx))
    { DM_CAN2_Fail(6U); return; }
    s_mailbox = CAN_Transmit(CAN2, &tx);
    if (s_mailbox > 2U) { DM_CAN2_Fail(4U); return; }
    g_dm_can2_tx_count++;
    s_step_started = now;
    g_dm_can2_state = 1U;
}

void DM_CAN2_Service(uint32_t ready, uint32_t busy)
{
    uint32_t request = g_dm_can2_read_request;
    uint32_t tx_status, i, value;
    float range;
    DM_CAN2_Feedback feedback;
    TickType_t now = xTaskGetTickCount();
    g_dm_can2_read_request = 0U;

    if (ready != 1U || busy != 0U)
    {
        if (request != 0U || DM_CAN2_IsBusy()) DM_CAN2_Fail(8U);
        else if (ready != 1U)
        {
            g_dm_can2_valid_mask = 0U;
            g_dm_can2_range_mask = 0U;
            g_dm_can2_feedback_valid = 0U;
            g_dm_can2_feedback_tick = 0U;
            /* 失败保留诊断；只把失电后的成功结果撤销。 */
            if (g_dm_can2_state != 5U)
            { g_dm_can2_state = 0U; g_dm_can2_result = 0U; }
        }
        return;
    }
    if ((request == 1U || request == 2U) && !DM_CAN2_IsBusy())
    {
        g_dm_can2_read_kind = request;
        g_dm_can2_read_cycle++;
        s_count = (request == 2U) ? 7U : 3U;
        g_dm_can2_result = 0U;
        g_dm_can2_valid_mask = 0U;
        g_dm_can2_range_mask = 0U;
        g_dm_can2_feedback_valid = 0U;
        g_dm_can2_feedback_tick = 0U;
        g_dm_can2_pmax = 0.0F;
        g_dm_can2_vmax = 0.0F;
        g_dm_can2_tmax = 0.0F;
        g_dm_can2_position_rad = 0.0F;
        g_dm_can2_position_deg = 0.0F;
        g_dm_can2_velocity_rad_s = 0.0F;
        g_dm_can2_torque_nm = 0.0F;
        g_dm_can2_motor_state = 0U;
        g_dm_can2_position_raw = 0U;
        g_dm_can2_velocity_raw = 0U;
        g_dm_can2_torque_raw = 0U;
        g_dm_can2_tx_count = 0U;
        g_dm_can2_rx_count = 0U;
        g_dm_can2_ignored_count = 0U;
        g_dm_can2_esc_id = 0U;
        g_dm_can2_master_id = 0U;
        g_dm_can2_ctrl_mode = 0U;
        memset(g_dm_can2_replies, 0, sizeof(g_dm_can2_replies));
        memset(&g_dm_can2_last_rx, 0, sizeof(g_dm_can2_last_rx));
        s_index = 0U;
        s_mailbox = CAN_TxStatus_NoMailBox;
        g_dm_can2_init_result = DM_CAN2_Init();
        if (g_dm_can2_init_result != 1U)
        { DM_CAN2_Fail(g_dm_can2_init_result); return; }
        s_started = now;
        DM_CAN2_BeginRead(now);
        return;
    }
    if (!DM_CAN2_IsBusy()) return;
    g_dm_can2_esr = CAN2->ESR;
    if ((CAN2->ESR & CAN_ESR_BOFF) != 0U) { DM_CAN2_Fail(7U); return; }
    if (((CAN1->FMR >> 8) & 0x3FU) != 14U || (CAN1->FA1R & (1UL << 14)) == 0U)
    { DM_CAN2_Fail(9U); return; }
    if ((TickType_t)(now - s_started) >= pdMS_TO_TICKS(2000U))
    { DM_CAN2_Fail(5U); return; }

    if (g_dm_can2_state == 3U) { DM_CAN2_BeginRead(now); return; }
    if (g_dm_can2_state == 1U)
    {
        tx_status = CAN_TransmitStatus(CAN2, s_mailbox);
        if (tx_status == CAN_TxStatus_Ok)
        {
            s_mailbox = CAN_TxStatus_NoMailBox;
            s_step_started = now;
            g_dm_can2_state = 2U;
        }
        else if (tx_status == CAN_TxStatus_Failed ||
                 (TickType_t)(now - s_step_started) >= pdMS_TO_TICKS(30U))
        { DM_CAN2_Fail(4U); return; }
    }
    if (g_dm_can2_state != 2U) return;
    if (CAN_GetFlagStatus(CAN2, CAN_FLAG_FOV0) != RESET)
    { DM_CAN2_Fail(6U); return; }
    for (i = 0U; i < 3U && CAN_MessagePending(CAN2, CAN_FIFO0) != 0U; i++)
    {
        CAN_Receive(CAN2, CAN_FIFO0, &g_dm_can2_last_rx);
        g_dm_can2_rx_count++;
        if (s_index == 6U)
        {
            if (!DM_CAN2_ParseStatus(&g_dm_can2_last_rx, g_dm_can2_pmax,
                                    g_dm_can2_vmax, g_dm_can2_tmax, &feedback))
            { g_dm_can2_ignored_count++; continue; }
            g_dm_can2_position_raw = feedback.position_raw;
            g_dm_can2_velocity_raw = feedback.velocity_raw;
            g_dm_can2_torque_raw = feedback.torque_raw;
            g_dm_can2_position_rad = feedback.position_rad;
            g_dm_can2_position_deg = feedback.position_rad * 57.2957795F;
            g_dm_can2_velocity_rad_s = feedback.velocity_rad_s;
            g_dm_can2_torque_nm = feedback.torque_nm;
            g_dm_can2_motor_state = feedback.motor_state;
            g_dm_can2_feedback_valid = 1U;
            g_dm_can2_feedback_tick = (uint32_t)now;
        }
        else
        {
            if (!DM_CAN2_ParseRead(&g_dm_can2_last_rx, s_rids[s_index], &value))
            { g_dm_can2_ignored_count++; continue; }
            if (s_index < 3U)
            {
                if (s_index == 0U) g_dm_can2_esc_id = value;
                else if (s_index == 1U) g_dm_can2_master_id = value;
                else g_dm_can2_ctrl_mode = value;
                if (value != s_expected[s_index]) { DM_CAN2_Fail(10U); return; }
                g_dm_can2_valid_mask |= (1UL << s_index);
            }
            else
            {
                if (!DM_CAN2_DecodeRange(value, &range))
                { DM_CAN2_Fail(11U); return; }
                if (s_index == 3U) g_dm_can2_pmax = range;
                else if (s_index == 4U) g_dm_can2_vmax = range;
                else g_dm_can2_tmax = range;
                g_dm_can2_range_mask |= (1UL << (s_index - 3U));
            }
        }
        g_dm_can2_replies[s_index] = g_dm_can2_last_rx;
        s_index++;
        if (s_index == s_count)
        { g_dm_can2_state = 4U; g_dm_can2_result = 1U; }
        else g_dm_can2_state = 3U;
        return;
    }
    if ((TickType_t)(now - s_step_started) >= pdMS_TO_TICKS(150U))
        DM_CAN2_Fail(5U);
}
