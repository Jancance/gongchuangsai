#ifndef CHASSIS_AUTO_TEST_H
#define CHASSIS_AUTO_TEST_H
#include <stdint.h>
#include "app_config.h"
#define CHASSIS_AUTO_FIRST_SEGMENT (APP_HWT_TURN_TEST ? 2U : 0U)

/* 开发测试：配置选择三段或只逆时针；阈值不作为正式运动限值。 */
/* 目标仍5RPM；本轮无线转动超10RPM中止，原三段保留7RPM门槛。 */
#define CHASSIS_AUTO_TARGET_RPM 5U
#define CHASSIS_AUTO_SPEED_LIMIT_RPM (APP_HWT_TURN_TEST ? 10U : (CHASSIS_AUTO_TARGET_RPM + 2U))
#define CHASSIS_AUTO_PREP_MS 10000U
#define CHASSIS_AUTO_RUN_MS 10000U
/* 航向模式单独兜底，晚于会话截止；原定时测试仍为10秒。 */
#define CHASSIS_HEADING_RUN_MS (APP_HEADING_RUN_MS + 1000U)
/* 前进保持另有较短兜底，避免沿用原地转角的21秒运行窗口。 */
#define CHASSIS_FORWARD_HOLD_RUN_MS (APP_FORWARD_HOLD_RUN_MS + 1000U)
#define CHASSIS_LATERAL_HOLD_RUN_MS (APP_LATERAL_HOLD_RUN_MS + 1000U)
#define CHASSIS_AUTO_STOP_MS 500U
#define CHASSIS_AUTO_GAP_MS 1000U
#define CHASSIS_AUTO_SETTLE_MS 2000U
#define CHASSIS_AUTO_FRESH_MS 1000U
#define CHASSIS_AUTO_TOTAL_MS 45000U
#define CHASSIS_AUTO_IDLE 0U
#define CHASSIS_AUTO_DISABLE 1U
#define CHASSIS_AUTO_VERIFY_OFF 2U
#define CHASSIS_AUTO_ENABLE 3U
#define CHASSIS_AUTO_VERIFY_ON 4U
#define CHASSIS_AUTO_START 5U
#define CHASSIS_AUTO_RUN 6U
#define CHASSIS_AUTO_STOP 7U
#define CHASSIS_AUTO_SETTLE 8U
#define CHASSIS_AUTO_DONE 9U
#define CHASSIS_AUTO_FAULT 10U
#define CHASSIS_AUTO_CANCELLED 11U

extern volatile uint32_t g_chassis_auto_request;
extern volatile uint32_t g_chassis_auto_dry_run;
extern volatile uint32_t g_chassis_auto_mode;
extern volatile uint32_t g_chassis_auto_state;
extern volatile uint32_t g_chassis_auto_segment; /* 0前进、1左移、2配置选定的转动段 */
extern volatile uint32_t g_chassis_auto_active;
extern volatile uint32_t g_chassis_auto_cut_vm;
extern volatile uint32_t g_chassis_auto_session;
/* result：0进行中/初始，1所选测试完成，2异常，3撤VM取消，9请求拒绝 */
extern volatile uint32_t g_chassis_auto_result;
/* cause：1准备，2健康，3反馈失效，4启动，5停止，6时限，7未停稳，8速度/符号异常，9人工中止 */
extern volatile uint32_t g_chassis_auto_cause;
extern volatile uint32_t g_chassis_auto_io_result;
extern volatile uint32_t g_chassis_auto_last_address;
extern volatile uint32_t g_chassis_auto_start_mask[3];
extern volatile uint32_t g_chassis_auto_stop_mask[3];
extern volatile int32_t g_chassis_auto_driver_rpm[4];

void ChassisAuto_Init(void);
uint32_t ChassisAuto_Begin(uint32_t dry_run);
void ChassisAuto_CanStep(void);
void ChassisAuto_Abort(void);
/* 1接受正常停车请求，0当前阶段不接受；并非物理停车完成。 */
uint32_t ChassisAuto_RequestStop(void);
/* 单次旋转：1接受，0拒绝；不等于目标角已到位。 */
/* 前进保持模式：3RPM基速，修正0表示直行而非停车；停车用RequestStop。 */
uint32_t ChassisAuto_BeginForwardHold(uint32_t dry_run);
uint32_t ChassisAuto_BeginTranslationHold(uint32_t dry_run, uint32_t motion_mode);
uint32_t ChassisAuto_UpdateForwardHold(int32_t correction_rpm);
uint32_t ChassisAuto_BeginHeading(uint32_t dry_run, int32_t command_rpm);
/* 同一转向的下一组轮速更新；1接受排队，0拒绝，不表示已发完。 */
uint32_t ChassisAuto_UpdateHeading(int32_t command_rpm);
/* Safety任务独立调用，不发送/等待CAN */
void ChassisAuto_SafetyUpdate(uint32_t vm_request);
/* TaskA到CAN任务的纯数据交接；1有效，0无效/过期。 */
#define CHASSIS_HEADING_MAILBOX_MAX_AGE_MS 100U
typedef struct
{
    int32_t command_rpm;
    uint32_t stop_required;
    uint32_t sequence;
} ChassisHeadingRequest;

uint32_t ChassisAuto_HeadingPublish(int32_t command_rpm, uint32_t stop_required);
uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *request);
/* 执行器同一时刻的状态，仅供任务读取；不是运动许可或物理停车证明。 */
typedef struct
{
    uint32_t state, active, result, cause;
    uint32_t session, segment, rotation_stop_mask;
    uint32_t heading_mode, dry_run, cut_vm, forward_hold_mode;
    uint32_t translation_mode; /* 0前进、1左移、2右移；只读会话归属。 */
} ChassisAutoStatus;

/* 1仅表示复制成功；0为指针无效。不发CAN，不修改状态。 */
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *status);


#endif
