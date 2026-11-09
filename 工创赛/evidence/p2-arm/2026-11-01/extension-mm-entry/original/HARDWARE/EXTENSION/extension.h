#ifndef EXTENSION_H
#define EXTENSION_H
#include "stm32f4xx_can.h"
#include "extension_home.h"

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
/* 请求默认0；1方向0小步，2方向1小步，3停止；已确认1缩回、2伸出。 */
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
/* 普通运行瞬时堵转观察；保护位或失能仍立即停，以下值仅用于诊断。 */
extern volatile uint32_t g_extension_stall_pending;
extern volatile uint32_t g_extension_stall_elapsed_ms;
extern volatile uint32_t g_extension_stall_max_ms;
extern volatile uint32_t g_extension_stall_events;
extern volatile uint32_t g_extension_motion_direction;
extern CanRxMsg g_extension_motion_reply;
extern CanRxMsg g_extension_control_reply;
uint32_t Extension_MotionIsBusy(void);
void Extension_MotionService(uint32_t ready, uint32_t other_busy);
/* 只读6号回零参数，默认请求0，写1读取一次，不触发回零或写参数。 */
typedef struct
{
    uint32_t mode, direction, rpm, timeout_ms;
    uint32_t detect_rpm, detect_current_ma, detect_time_ms, auto_home;
} ExtensionHomeParameters;
extern volatile uint32_t g_extension_home_read_request;
/* 1读出并解释成功，6格式/数据范围错误，7未就绪/机构忙，其他同CAN结果。 */
extern volatile uint32_t g_extension_home_read_result;
extern ExtensionHomeParameters g_extension_home_params;
extern uint8_t g_extension_home_raw[17];
extern CanRxMsg g_extension_home_replies[3];
void Extension_HomeReadService(uint32_t ready, uint32_t other_busy);
#endif
