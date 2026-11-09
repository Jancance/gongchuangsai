#ifndef DM_CAN2_H
#define DM_CAN2_H
#include "../../USER/vehicle_parameters.h"

#include "stm32f4xx_can.h"

/* 已实读ID1/反馈0、CAN1Mbps；本版目标为位置速度模式2，待板端读回。 */
#define DM_CAN2_NODE_ID       1U
#define DM_CAN2_MASTER_ID     0U
#define DM_CAN2_MODE          2U
#define DM_CAN2_PARAMETER_ID  0x7FFU

/* 默认0；写1只读身份；写2读身份、反馈量程及一次状态，均不运动。 */
extern volatile uint32_t g_dm_can2_read_request;
extern volatile uint32_t g_dm_can2_read_kind;
extern volatile uint32_t g_dm_can2_read_cycle;
extern volatile uint32_t g_dm_can2_feedback_tick;
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

/* 左侧人工基准候选：2026-10-29只读反馈；掉电恢复尚待实物验证。
 * 仅换算显示，不修改电机零点，不作为自动回零或行程放行依据。
 * 逻辑显示为原始角减左侧基准后的同圈分支，逆时针为正。
 * 显示范围[-90,270)度，覆盖预计0到180度工作区并保留零点负误差。
 * 单圈显示不恢复多圈数；点位调试另需人工准入、新鲜状态及下述调试范围检查。
 * 仅在feedback_valid为1且本次反馈新鲜时解释逻辑角；0值本身不代表已回零。 */
#define DM_CAN2_LEFT_ZERO_DEG VEHICLE_DM_ZERO_RAW_DEG
extern volatile float g_dm_can2_logical_position_deg;

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

/* 用户指定45度；本次低速工程试验参数，不是机械额定值或限位。 */
#define DM_CAN2_JOG_DEG       45.0F
#define DM_CAN2_JOG_TIME_MS   6000U
#define DM_CAN2_JOG_KP        5.0F
#define DM_CAN2_JOG_KD        0.5F
/* 默认0；1正向45度，2负向45度，3中止/失能；4去取料候选，5去放料候选；6按键启动专用逆时针回显示0度。
 * 4/5仅用于人工看护的空载调试，须先明确当前姿态及全路径避让后置point_enable=1。
 * 点位默认禁用，不自动往返，不用于比赛自主动作组；不得按模一圈显示猜线束圈数。 */
#define DM_CAN2_POINT_TIME_MS 15000U
extern volatile uint32_t g_dm_point_enable;
/* 参考Tower_task的G6220_Position(...,300)，单位度/秒；接单锁存。 */
#define DM_CAN2_POSVEL_SPEED_DEG_S VEHICLE_DM_SPEED_DEG_S
extern volatile float g_dm_posvel_speed_deg_s;
extern volatile float g_dm_posvel_latched_speed_deg_s;
extern volatile uint32_t g_dm_mode_change_state;
/* 1表示到位后仍使能/重发目标；request3显式停止，停止应答另看stop_result。 */
extern volatile uint32_t g_dm_posvel_holding;
uint32_t DM_CAN2_BuildPosVel(float position, float speed, CanTxMsg *tx);
uint32_t DM_CAN2_BuildMode2(CanTxMsg *tx);
/* 单位为度；仅在空闲时调整，接单锁存，重启恢复下列源码初值。 */
extern volatile float g_dm_pick_target_deg;
extern volatile float g_dm_place_target_deg;
extern volatile float g_dm_point_latched_deg;
extern volatile uint32_t g_dm_jog_request;
/* 首次真实故障锁存，不被后续失能/人工取消覆盖。 */
extern volatile uint32_t g_dm_jog_first_fault;
/* 0空闲，1读新状态，2预置当前位置，3使能，4驱动器位置速度运动，
 * 5持续等待末端停稳，6失能退出，7软件到位并保持，8失败/中止。 */
extern volatile uint32_t g_dm_jog_state;
/* 1软件完成，2人工中止，3互锁，4准备读取失败，5发送失败，
 * 6回复超时，7驱动状态异常，8位置越界，9保留，
 * 10动作超时，11配置不匹配，12服务间隔过大，13点位参数非法，14点位范围/方向不符，15模式切换未确认。 */
extern volatile uint32_t g_dm_jog_result;
/* 1收到失能状态回复；2未能确认。不是实物停车验收。 */
extern volatile uint32_t g_dm_jog_stop_result;
extern volatile uint32_t g_dm_jog_tx_count;
extern volatile uint32_t g_dm_jog_rx_count;
extern volatile uint32_t g_dm_jog_elapsed_ms;
extern volatile float g_dm_jog_start_deg;
extern volatile float g_dm_jog_target_deg;
extern volatile float g_dm_jog_current_deg;
extern volatile float g_dm_jog_reference_deg;
extern volatile float g_dm_jog_tracking_error_deg;
uint32_t DM_CAN2_SourceTransitionPending(void);
extern volatile uint32_t g_dm_source_smooth,g_dm_source_go;
extern volatile float g_dm_source_accel_deg_s2;
void DM_CAN2_ModuleService(uint32_t ready, uint32_t busy);
uint32_t DM_CAN2_JogIsBusy(void);
void DM_CAN2_CancelRead(void);
uint32_t DM_CAN2_BuildSwitch(uint8_t command, CanTxMsg *tx);
uint32_t DM_CAN2_BuildMitHold(float position, float pmax, float vmax,
                             float tmax, CanTxMsg *tx);
uint32_t DM_CAN2_BuildMitPreload(float position, float pmax, float vmax,
                                float tmax, CanTxMsg *tx);
float DM_CAN2_SmoothStep(float ratio);
/* 五次位置曲线的一阶导数；单位为归一化时间的倒数。 */
float DM_CAN2_SmoothVelocity(float ratio);
uint32_t DM_CAN2_BuildMitTrajectory(float position, float velocity, float pmax,
                                   float vmax, float tmax, CanTxMsg *tx);

#endif
