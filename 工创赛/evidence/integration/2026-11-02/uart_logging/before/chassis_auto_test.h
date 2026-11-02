#ifndef CHASSIS_AUTO_TEST_H
#define CHASSIS_AUTO_TEST_H
#include <stdint.h>
#include "app_config.h"
#include "chassis_kinematics.h"
#include "motion_profile.h"
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
/* X反馈统一为物理RPM，保留一位小数。 */
extern volatile float g_chassis_auto_driver_rpm[4];

/* 后端首故障诊断；reason细分原因3，读取不改变故障或控制状态。 */
typedef struct
{
    uint32_t reason, state, io_result, address, command_age_ms;
    uint32_t status_mask, speed_mask, feedback_age_ms, tick_ms;
    int32_t target_decirpm, measured_decirpm;
    uint32_t target_ack_ms, speed_sample_ms, reverse_samples;
} ChassisAutoFaultDiagnostic;
extern volatile ChassisAutoFaultDiagnostic g_chassis_auto_fault_diagnostic;
void ChassisAuto_ReadFaultDiagnostic(ChassisAutoFaultDiagnostic *snapshot);
/* 调试批次历史，只观察速度指令通信，不证明机械响应；全部时间为uint32毫秒。
 * outcome:1四轮确认、2故障中断、3停止/撤VM中断；dry_run=1无真实CAN。
 * generation_source:0首次发送选择、1邮箱发布、2直接更新API、3距离减速。
 * 仅保留最近16批，覆盖量与累计最大延迟另行返回；初始化后从头计数。 */
#define CHASSIS_BATCH_DIAGNOSTIC_CAPACITY 16U
typedef struct
{
    uint32_t session, batch_id, segment, dry_run, generation_source;
    uint32_t generated_ms, start_ms, end_ms, outcome, sent_mask, ack_mask;
    uint32_t failed_address, io_result;
    float correction_rpm;
    uint32_t base_rpm, pending_changes;
    float pending_rpm;
    uint32_t send_ms[4], reply_ms[4], result[4];
} ChassisBatchDiagnostic;
typedef struct
{
    uint32_t overwritten, max_queue_ms, max_batch_ms, max_call_ms;
    uint32_t total_success, total_interrupted;
} ChassisBatchStatistics;
/* index按当前保留历史从旧到新读取；失败不修改运动状态，比赛入口返回0。 */
uint32_t ChassisAuto_ReadBatchDiagnostic(uint32_t index, ChassisBatchDiagnostic *record,
    ChassisBatchStatistics *statistics);
/* CAN任务两阶段平移启动：Prepare仅算局部命令；Begin在短锁内复核并提交。 */
typedef struct
{
    uint32_t valid, motion_mode, base_rpm, run_ms;
    float distance_mm, route_reference_mm_s;
#if APP_DISTANCE_SCURVE_ACTIVE
    MotionProfile profile;
#endif
    ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT];
} ChassisAutoPreparedTranslation;
uint32_t ChassisAuto_PrepareTranslation(uint32_t motion_mode, uint32_t base_rpm,
    uint32_t run_ms, float distance_mm, ChassisAutoPreparedTranslation *prepared);
uint32_t ChassisAuto_PrepareRouteTranslation(uint32_t motion_mode,uint32_t base_rpm,
    uint32_t run_ms,float distance_mm,float reference_mm_s,ChassisAutoPreparedTranslation *prepared);
uint32_t ChassisAuto_BeginPreparedTranslation(const ChassisAutoPreparedTranslation *prepared,
    uint32_t expected_session);
void ChassisAuto_Init(void);
uint32_t ChassisAuto_Begin(uint32_t dry_run);
void ChassisAuto_CanStep(void);
void ChassisAuto_Abort(void);
#if APP_TEST_MODE == 3U
uint32_t ChassisAuto_BeginSequence(void);
uint32_t ChassisAuto_RearmCompleted(uint32_t expected_session);
uint32_t ChassisAuto_ReleaseCompleted(uint32_t session);
#endif
/* 1接受正常停车请求，0当前阶段不接受；并非物理停车完成。 */
uint32_t ChassisAuto_RequestStop(void);
/* 单次旋转：1接受，0拒绝；不等于目标角已到位。 */
/* 前进保持模式：3RPM基速，修正0表示直行而非停车；停车用RequestStop。 */
uint32_t ChassisAuto_BeginForwardHold(uint32_t dry_run);
uint32_t ChassisAuto_BeginTranslationTimed(uint32_t dry_run, uint32_t motion_mode,
    uint32_t base_rpm, uint32_t run_ms);
uint32_t ChassisAuto_BeginTranslationHold(uint32_t dry_run, uint32_t motion_mode);
uint32_t ChassisAuto_UpdateForwardHold(float correction_rpm);
/* 平移mode0前/1左/2右、100..1000mm、最大3..15RPM，30秒运行截止；必须开启位置诊断。 */
uint32_t ChassisAuto_BeginTranslationDistance(uint32_t motion_mode, uint32_t max_rpm, float target_mm);
uint32_t ChassisAuto_BeginForwardDistance(uint32_t max_rpm, float target_mm);
typedef struct {
    uint32_t valid, dry_run;
    float command_rpm;
    int32_t trial_sign;
    ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT];
} ChassisAutoPreparedHeading;
uint32_t ChassisAuto_PrepareHeading(uint32_t dry_run,float command_rpm,
    int32_t trial_sign,ChassisAutoPreparedHeading *prepared);
uint32_t ChassisAuto_BeginPreparedHeading(const ChassisAutoPreparedHeading *prepared,
    uint32_t expected_session);
uint32_t ChassisAuto_BeginHeading(uint32_t dry_run, float command_rpm);
/* 同一转向的下一组轮速更新；1接受排队，0拒绝，不表示已发完。 */
uint32_t ChassisAuto_UpdateHeading(float command_rpm);
/* Safety任务独立调用，不发送/等待CAN */
void ChassisAuto_SafetyUpdate(uint32_t vm_request);
/* TaskA到CAN任务的纯数据交接；1有效，0无效/过期。 */
#define CHASSIS_HEADING_MAILBOX_MAX_AGE_MS 100U
typedef struct
{
    float command_rpm;
    uint32_t stop_required;
    uint32_t sequence;
} ChassisHeadingRequest;

uint32_t ChassisAuto_HeadingPublish(float command_rpm, uint32_t stop_required);
uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *request);
/* 执行器同一时刻的状态，仅供任务读取；不是运动许可或物理停车证明。 */
typedef struct
{
    uint32_t state, active, result, cause;
    uint32_t session, segment, rotation_stop_mask;
    uint32_t heading_mode, dry_run, cut_vm, forward_hold_mode;
    uint32_t distance_mode, distance_reached, distance_base_rpm, distance_fault_reason;
    float distance_target_mm, distance_progress_mm;
    /* 候选曲线状态，仅本地快照；不改变TM1字段布局。 */
    uint32_t distance_profile_mode, distance_profile_done;
    float distance_reference_mm, distance_reference_rpm;
    uint32_t translation_mode; /* 0前进、1左移、2右移；只读会话归属。 */
} ChassisAutoStatus;

/* 1仅表示复制成功；0为指针无效。不发CAN，不修改状态。 */
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *status);


/* 位置诊断：地址1..4，符号/32位幅值独立，各轮非同步采样。
 * valid_mask同时检查会话、供电及1000ms时效，不代表里程精度。 */
typedef struct
{
    uint32_t enabled, session, sequence, vm_active, valid_mask;
    uint32_t result[4], sign[4], raw[4], sample_ms[4], age_ms[4];
} ChassisPositionSnapshot;
uint32_t ChassisAuto_ReadPositions(ChassisPositionSnapshot *snapshot);
/* 独立观察快照不随动作重置；仅保存已有真实CAN查询，不额外查询。
 * CAN任务读取后交里程计；mask仅表示最近一次交易成功，仍须核对帧龄。 */
typedef struct {
    uint32_t mask, sign[4], raw[4], ms[4];
} ChassisOdometrySamples;
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *samples);
void ChassisAuto_OdometryPoll(void);
/* 历史完整起止记录，诊断开启时每次复位最多保存前8个动作。
 * outcome：0未完成，1正常完成，2失败/取消；不是实时位置快照。
 * 只在四轮起止均齐、正常DONE后才输出完整记录；断VM后仍可读取历史。 */
#define CHASSIS_POSITION_ENDPOINT_CAPACITY 8U
typedef struct
{
    uint32_t session, outcome, start_mask, end_mask;
    uint32_t translation_mode, distance_mode; /* 0前/1左/2右/3非平移，是否距离动作 */
    uint32_t start_sign[4], start_raw[4], start_ms[4];
    uint32_t end_sign[4], end_raw[4], end_ms[4];
} ChassisPositionEndpoints;
uint32_t ChassisAuto_ReadPositionEndpoints(uint32_t index, ChassisPositionEndpoints *record);
uint32_t ChassisAuto_BeginTurnBrake(int32_t direction);
#endif
