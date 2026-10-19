#ifndef HWT101_H
#define HWT101_H
#include <stdint.h>
void HWT101_Init(void);
void HWT101_EnableReceive(void);
void HWT101_Update(void);
extern volatile uint32_t g_hwt_baud;
extern volatile uint32_t g_hwt_rx_bytes, g_hwt_uart_errors, g_hwt_overflows;
extern volatile uint32_t g_hwt_angle_count, g_hwt_rate_count, g_hwt_checksum_errors;
extern volatile uint32_t g_hwt_valid_mask, g_hwt_angle_age_ms, g_hwt_rate_age_ms;
extern volatile float g_hwt_yaw_deg, g_hwt_wz_dps;
extern volatile uint8_t g_hwt_last_frame[11];
/* 默认启动读取一次版本，只读2E；自动启动不发送A6/A7/归零/保存命令。
 * request=1重新读；result 1收到，2发送超时，3回复超时，4串口/溢出错误。
 * result1仅表示本次读到校验有效5F，不表示零偏校准成功。 */
extern volatile uint32_t g_hwt_version_request, g_hwt_version_result;
extern volatile uint32_t g_hwt_version_raw, g_hwt_version_low16;
extern volatile uint32_t g_hwt_version_state, g_hwt_version_reply_count;
extern volatile uint32_t g_hwt_tx_bytes;
extern volatile uint8_t g_hwt_version_reply[11];
/* request1为A6，3确认静止开始0x48采20s，2取消；默认0不自动采集。
 * result1流程完成而非设备确认；2失败；9拒绝。cause见操作说明。 */
extern volatile uint32_t g_hwt_cal_request, g_hwt_cal_active;
extern volatile uint32_t g_hwt_cal_state, g_hwt_cal_result, g_hwt_cal_cause;
extern volatile uint32_t g_hwt_cal_start_tx_result, g_hwt_cal_end_tx_result;
extern volatile uint32_t g_hwt_cal_elapsed_ms;
extern volatile uint32_t g_hwt_cal_mode; /* 1=A6，2=0x48 */
extern volatile float g_hwt_cal_before_deg, g_hwt_cal_after_deg;
/* A7 request默认0，置1读一次。result0初始/进行中、1有效5F回复、
 * 2发送超时、3回复超时、4串口错误、9忙拒绝。raw仅0/1有定义。
 * 5F无地址回显，有效回复不等于已证明固件支持A7。不写A7配置。 */
extern volatile uint32_t g_hwt_a7_request, g_hwt_a7_result;
extern volatile uint32_t g_hwt_a7_raw, g_hwt_a7_reply_count;
extern volatile uint8_t g_hwt_a7_reply[11];
/* cause4瞬间快照：kind位0为角速度越限，位1为角度差越限。
 * 下次获准试验时清零，退出和后续接收不会覆盖；不证明物理运动。 */
extern volatile uint32_t g_hwt_cal_trip_kind, g_hwt_cal_trip_state, g_hwt_cal_trip_ms;
extern volatile float g_hwt_cal_trip_wz_dps, g_hwt_cal_trip_delta_deg;
extern volatile float g_hwt_cal_trip_yaw_deg, g_hwt_cal_trip_prev_deg;
/* 0x48采集中接受一次静止归零的记录，不代表零偏效果已通过。 */
extern volatile uint32_t g_hwt_cal_rezero_count, g_hwt_cal_rezero_ms;
extern volatile float g_hwt_cal_rezero_prev_deg;
extern volatile uint32_t g_hwt_cal_resume_count; /* 与采集归零配对的退出恢复次数 */
extern volatile uint32_t g_hwt_update_tick; /* 采样任务最后推进时刻 */
#endif
