#ifndef EXTENSION_H
#define EXTENSION_H
#include "stm32f4xx_can.h"

/* 28伸缩轴，地址6；只读识别及人工单次低速小步，均不写驱动参数。 */
extern volatile uint32_t g_extension_read_request;
/* 0空闲，1读版本，2读状态，3读选项，4读取完成，5失败。 */
extern volatile uint32_t g_extension_state;
extern volatile uint32_t g_extension_version_result;
extern volatile uint32_t g_extension_status_result;
extern volatile uint32_t g_extension_options_result;
/* 0无错误，7供电未就绪/升降忙，其他值同CAN查询结果。 */
extern volatile uint32_t g_extension_error;
extern CanRxMsg g_extension_version_reply;
extern CanRxMsg g_extension_status_reply;
extern CanRxMsg g_extension_options_reply;
/* 只由现有CAN任务串行调用；升降运动或回零时拒绝开始。 */
void Extension_Service(uint32_t ready, uint32_t lift_busy);
/* 请求默认0；1方向0小步，2方向1小步，3停止；伸/缩方向待实物确认。 */
extern volatile uint32_t g_extension_motion_request;
/* 0空闲，1至6准备/运行，7反馈完成，8人工停止，9失败锁存。 */
extern volatile uint32_t g_extension_motion_state;
/* 1完成，2人工停止，3超时，4通信/选项，5驱动状态，6互锁，7服务间隔超限。 */
extern volatile uint32_t g_extension_motion_result;
extern volatile uint32_t g_extension_enable_result;
extern volatile uint32_t g_extension_move_result;
extern volatile uint32_t g_extension_stop_result;
extern volatile uint32_t g_extension_motion_options_result;
extern volatile uint32_t g_extension_flags;
extern volatile int32_t g_extension_speed_rpm;
extern volatile int64_t g_extension_delta_raw;
extern volatile uint32_t g_extension_motion_elapsed_ms;
extern volatile uint32_t g_extension_motion_direction;
extern CanRxMsg g_extension_motion_reply;
extern CanRxMsg g_extension_control_reply;
uint32_t Extension_MotionIsBusy(void);
void Extension_MotionService(uint32_t ready, uint32_t other_busy);
#endif
