#ifndef LIFT_H
#define LIFT_H
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"

extern const uint32_t g_lift_address;
/* 0等供电，1等驱动启动，2读版本，3读状态，4读取完成，5读取失败。 */
extern volatile uint32_t g_lift_state;
extern volatile uint32_t g_lift_read_request;
extern volatile uint32_t g_lift_read_attempts;
extern volatile uint32_t g_lift_version_result;
extern volatile uint32_t g_lift_status_result;
extern CanRxMsg g_lift_version_reply;
extern CanRxMsg g_lift_status_reply;

/* 运动请求：1方向0小步，2方向1小步，3立即停止，不松轴。 */
extern volatile uint32_t g_lift_motion_request;
/* 0闲置，1~6准备/运行，7反馈完成，8用户停止，9失败。 */
extern volatile uint32_t g_lift_motion_state;
/* 1反馈完成，2用户停，3超时，4通讯/格式，5驱动保护，6未就绪。 */
extern volatile uint32_t g_lift_motion_result;
extern volatile uint32_t g_lift_enable_result;
extern volatile uint32_t g_lift_move_result;
extern volatile uint32_t g_lift_stop_result;
extern volatile uint32_t g_lift_flags;
extern volatile int32_t g_lift_speed_rpm;
extern volatile int64_t g_lift_delta_raw;
extern volatile uint32_t g_lift_can_esr;
extern volatile uint32_t g_lift_options_result;
extern CanRxMsg g_lift_control_reply;
/* 1A reply: command, options high byte, options low byte, checksum. */
extern CanRxMsg g_lift_options_reply;
/* 只由现有CAN任务串行调用。 */
void Lift_Service(uint32_t power_ready);

/* 只读回零参数：请求1查询一次，结果1成功、6格式错误、其他同CAN结果。 */
typedef struct
{
    uint32_t mode;
    uint32_t direction;
    uint32_t rpm;
    uint32_t timeout_ms;
    uint32_t detect_rpm;
    uint32_t detect_current_ma;
    uint32_t detect_time_ms;
    uint32_t auto_home;
} LiftHomeParameters;
extern volatile uint32_t g_lift_home_read_request;
extern volatile uint32_t g_lift_home_read_result;
extern LiftHomeParameters g_lift_home_params;
extern uint8_t g_lift_home_raw[17];
extern CanRxMsg g_lift_home_replies[3];

/* 仅由同一CAN任务内的升降/回零模块调用。 */
uint32_t Lift_ReadHomeParametersNow(void);
#endif
