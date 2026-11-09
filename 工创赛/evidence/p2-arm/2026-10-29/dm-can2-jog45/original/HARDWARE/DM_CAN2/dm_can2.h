#ifndef DM_CAN2_H
#define DM_CAN2_H

#include "stm32f4xx_can.h"

/* 本机已实读：接收ID1、反馈ID0、MIT模式1、经典CAN 1Mbps。 */
#define DM_CAN2_NODE_ID       1U
#define DM_CAN2_MASTER_ID     0U
#define DM_CAN2_MODE          1U
#define DM_CAN2_PARAMETER_ID  0x7FFU

/* 默认0；写1只读身份；写2读身份、反馈量程及一次状态，均不运动。 */
extern volatile uint32_t g_dm_can2_read_request;
extern volatile uint32_t g_dm_can2_read_kind;
/* 0空闲，1等发送，2等回复，3准备下一项，4完成，5失败。 */
extern volatile uint32_t g_dm_can2_state;
/* 0未完成，1全部匹配；2时钟，3初始化，4发送，5回复超时，
 * 6错误帧/溢出，7总线关闭，8互锁，9过滤器分配，10参数不符，11量程非法。 */
extern volatile uint32_t g_dm_can2_result;
extern volatile uint32_t g_dm_can2_init_result;
extern volatile uint32_t g_dm_can2_esc_id;
extern volatile uint32_t g_dm_can2_master_id;
extern volatile uint32_t g_dm_can2_ctrl_mode;
/* 位0/1/2对应ESC_ID/MST_ID/CTRL_MODE，本轮全部有效时为7。 */
extern volatile uint32_t g_dm_can2_valid_mask;
/* 量程位0/1/2为P/V/T，本次状态回复独立用feedback_valid。 */
extern volatile uint32_t g_dm_can2_range_mask;
extern volatile uint32_t g_dm_can2_feedback_valid;
extern volatile float g_dm_can2_pmax;
extern volatile float g_dm_can2_vmax;
extern volatile float g_dm_can2_tmax;
extern volatile float g_dm_can2_position_rad;
extern volatile float g_dm_can2_position_deg;
extern volatile float g_dm_can2_velocity_rad_s;
extern volatile float g_dm_can2_torque_nm;
extern volatile uint32_t g_dm_can2_motor_state;
extern volatile uint32_t g_dm_can2_position_raw;
extern volatile uint32_t g_dm_can2_velocity_raw;
extern volatile uint32_t g_dm_can2_torque_raw;
extern volatile uint32_t g_dm_can2_tx_count;
extern volatile uint32_t g_dm_can2_rx_count;
extern volatile uint32_t g_dm_can2_ignored_count;
extern volatile uint32_t g_dm_can2_last_rid;
extern volatile uint32_t g_dm_can2_esr;
extern volatile uint32_t g_dm_can2_pclk1_hz;
extern volatile uint32_t g_dm_can2_filter_start;
extern volatile uint32_t g_dm_can2_can1_btr_before;
extern volatile uint32_t g_dm_can2_can1_btr_after;
/* 顺序：ESC_ID、MST_ID、CTRL_MODE、PMAX、VMAX、TMAX、状态。 */
extern CanRxMsg g_dm_can2_replies[7];
extern CanRxMsg g_dm_can2_last_rx;

typedef struct
{
    uint32_t position_raw, velocity_raw, torque_raw, motor_state;
    float position_rad, velocity_rad_s, torque_nm;
} DM_CAN2_Feedback;

/* 仅供唯一CAN任务调用。ready为现有供电/平台允许，busy为机构忙。 */
void DM_CAN2_Service(uint32_t ready, uint32_t busy);
/* 纯报文层：仅能生成0x33读取；不提供使能/写参数/运动编码入口。 */
uint32_t DM_CAN2_BuildRead(uint8_t rid, CanTxMsg *tx);
uint32_t DM_CAN2_ParseRead(const CanRxMsg *rx, uint8_t rid, uint32_t *value);
uint32_t DM_CAN2_BuildStatus(CanTxMsg *tx);
uint32_t DM_CAN2_DecodeRange(uint32_t bits, float *value);
uint32_t DM_CAN2_ParseStatus(const CanRxMsg *rx, float pmax, float vmax,
                            float tmax, DM_CAN2_Feedback *feedback);

#endif
