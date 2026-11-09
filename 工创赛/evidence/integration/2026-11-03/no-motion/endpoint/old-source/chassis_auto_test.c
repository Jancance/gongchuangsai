#include "chassis_auto_test.h"
#include "startup_axes.h"
#include "chassis_direction_guard.h"
#include "zdt_emm_protocol.h"
#include "motion_distance_limits.h"
#include "chassis_kinematics.h"
#include "can.h"
#include "safety.h"
#include "freertos_hooks.h"
#include "task.h"
#include <stddef.h>
#include "power.h"
#if APP_DISTANCE_SCURVE_ACTIVE
#include <math.h>
#endif

/* 将新读到的3A供原CAN监督使用，不伪造采样时间 */
extern volatile uint32_t g_can_poll_result[4];
extern volatile uint32_t g_can_poll_flags[4];
extern volatile TickType_t g_can_poll_tick[4];
extern volatile uint32_t g_can_poll_count[4];

volatile uint32_t g_chassis_auto_request = 0U;
volatile uint32_t g_chassis_auto_dry_run = 1U;
volatile uint32_t g_chassis_auto_mode = 1U;
volatile uint32_t g_chassis_auto_state = CHASSIS_AUTO_IDLE;
volatile uint32_t g_chassis_auto_segment = 0U;
volatile uint32_t g_chassis_auto_active = 0U;
volatile uint32_t g_chassis_auto_cut_vm = 0U;
volatile uint32_t g_chassis_auto_session = 0U;
volatile uint32_t g_chassis_auto_result = 0U;
volatile uint32_t g_chassis_auto_cause = 0U;
volatile uint32_t g_chassis_auto_io_result = 0U;
volatile uint32_t g_chassis_auto_last_address = 0U;
volatile uint32_t g_chassis_auto_start_mask[3] = {0U};
volatile uint32_t g_chassis_auto_stop_mask[3] = {0U};
volatile float g_chassis_auto_driver_rpm[4] = {0};

#if APP_POSITION_DIAGNOSTIC
static ChassisPositionSnapshot s_position;
static ChassisOdometrySamples s_odometry_samples;
/* 仅在原锁内记录整数，不在串口中断屏蔽期间进行位置计算。 */
static void Auto_OdometrySample(uint32_t wheel,uint32_t result,uint32_t sign,uint32_t raw,uint32_t ms)
{
    if(result==1U) {
        s_odometry_samples.sign[wheel]=sign; s_odometry_samples.raw[wheel]=raw;
        s_odometry_samples.ms[wheel]=ms; s_odometry_samples.mask|=1UL<<wheel;
    } else s_odometry_samples.mask &= ~(1UL<<wheel);
}
static TickType_t s_position_tick[4];
static uint32_t s_position_valid, s_position_due;
static ChassisPositionEndpoints s_endpoints[CHASSIS_POSITION_ENDPOINT_CAPACITY];
static uint32_t s_endpoint_count, s_endpoint_index;
/* 有界历史仅供同一时刻的位置比较，不外推，不延长400ms时效。 */
#define AUTO_POSITION_HISTORY 16U
typedef struct { uint32_t ms; float mm; } AutoPositionPoint;
static AutoPositionPoint s_position_history[4][AUTO_POSITION_HISTORY];
static uint32_t s_position_history_count[4], s_position_history_head[4];
static uint32_t s_alignment_seen, s_alignment_ms;
static uint32_t s_left_trim_tick, s_left_trim_valid, s_left_trim_rpm;
static float s_left_trim_mm, s_position_trim_mm[4];
static void Auto_LeftTrimAdvance(uint32_t now_ms)
{
    if (s_left_trim_valid)
        s_left_trim_mm-=(float)(now_ms-s_left_trim_tick)*0.001f*(float)s_left_trim_rpm*
            (3.14159265358979323846f*77.0f/60.0f);
    s_left_trim_tick=now_ms; s_left_trim_valid=1U;
}
static void Auto_PositionRemember(uint32_t wheel, uint32_t ms, float mm)
{
    uint32_t n = s_position_history_head[wheel];
    s_position_history[wheel][n].ms = ms;
    s_position_history[wheel][n].mm = mm;
    s_position_history_head[wheel] = (n+1U)%AUTO_POSITION_HISTORY;
    if (s_position_history_count[wheel] < AUTO_POSITION_HISTORY)
        s_position_history_count[wheel]++;
}
static uint32_t Auto_PositionAt(uint32_t wheel, uint32_t ms, float *mm)
{
    uint32_t j, count = s_position_history_count[wheel];
    AutoPositionPoint upper = {0U,0.0f};
    uint32_t have_upper = 0U;
    for (j=0U; j<count; j++)
    {
        AutoPositionPoint point = s_position_history[wheel]
            [(s_position_history_head[wheel]+AUTO_POSITION_HISTORY-1U-j)%AUTO_POSITION_HISTORY];
        int32_t offset = (int32_t)(point.ms-ms);
        if (!offset) { *mm=point.mm; return 1U; }
        if (offset>0) { upper=point; have_upper=1U; }
        else
        {
            uint32_t span=upper.ms-point.ms;
            if (!have_upper || !span || span>=400U) return 0U;
            *mm=point.mm+(upper.mm-point.mm)*(float)(ms-point.ms)/(float)span;
            return 1U;
        }
    }
    return 0U;
}
static void Auto_PositionReset(uint32_t session)
{
    ChassisPositionSnapshot empty = {0};
    uint32_t i;
    s_position = empty; s_position.session = session;
    s_alignment_seen = s_alignment_ms = 0U;
    s_left_trim_tick=s_left_trim_valid=s_left_trim_rpm=0U;s_left_trim_mm=0.0f;
    for(i=0U;i<4U;i++) s_position_trim_mm[i]=0.0f;
    for (i=0U;i<4U;i++) s_position_history_count[i]=s_position_history_head[i]=0U;
    if (session == 0U) s_endpoint_count = 0U;
    s_endpoint_index = CHASSIS_POSITION_ENDPOINT_CAPACITY;
    if (session && s_endpoint_count < CHASSIS_POSITION_ENDPOINT_CAPACITY)
    {
        ChassisPositionEndpoints endpoint = {0};
        endpoint.session = session;
        endpoint.translation_mode = 4U; /* 3表示非平移/尚未指定。 */
        s_endpoint_index = s_endpoint_count++;
        s_endpoints[s_endpoint_index] = endpoint;
    }
    s_position_valid = s_position_due = 0U;
    for (i = 0U; i < 4U; i++) s_position_tick[i] = 0U;
}
#endif
static uint32_t s_vm_requested, s_used, s_mode;
static uint32_t s_motor_index, s_monitor_cursor, s_monitor_between;
static uint32_t s_status_valid, s_speed_valid, s_run_started, s_stop_due;
static uint32_t s_status[4];
static TickType_t s_status_tick[4], s_speed_tick[4];
static TickType_t s_total_tick, s_phase_tick, s_run_tick;
static ChassisMotorCommand s_commands[3][4];
static ChassisDirectionGuard s_direction_guard[4];
/* 正常停车后结束本次序列，记录首次请求时刻。 */
static uint32_t s_finish_requested = 0U;
static TickType_t s_finish_tick = 0U;
/* 仅BeginHeading会话使用，旧三段测试不走动态更新。 */
static uint32_t s_heading_session = 0U;
static uint32_t s_turn_brake_trial;
static uint32_t s_forward_hold = 0U;
static uint32_t s_translation_mode = 0U;
static uint32_t s_translation_base_rpm, s_translation_run_ms;
static uint32_t s_route_trial;
static uint32_t s_distance_mode, s_distance_reached, s_distance_fault_reason;
static uint32_t s_translation_applied_base;
static float s_distance_target_mm, s_distance_progress_mm;
#if APP_DISTANCE_SCURVE_ACTIVE
static MotionProfile s_distance_profile;
static MotionProfileSample s_distance_reference;
static uint32_t s_profile_active, s_profile_done;
static float s_profile_mm_per_rpm_s;
static float s_profile_follow_rpm, s_profile_follow_rate;
static TickType_t s_profile_follow_tick;

#endif
#if APP_POSITION_DIAGNOSTIC
static uint32_t s_distance_max_rpm;
static uint32_t s_distance_seen_ms[4], s_distance_seen_raw[4], s_distance_seen_sign[4];
#endif
static float s_heading_initial_rpm = 0;
static float s_heading_pending_rpm = 0, s_heading_applied_rpm = 0;
static TickType_t s_heading_update_tick = 0U;
/* 最近一次合法轮速提交时间，与实际四轮更新起点分开。 */
static TickType_t s_heading_command_tick = 0U;
static ChassisHeadingRequest s_heading_request = {0, 1U, 0U};
static TickType_t s_heading_request_tick = 0U;
static uint32_t s_heading_request_valid = 0U;
#if APP_TEST_MODE == 3U
static uint32_t s_sequence_hold, s_sequence_continue;
#define AUTO_SEQUENCE_HOLD() (s_sequence_hold)
#else
#define AUTO_SEQUENCE_HOLD() (0U)
#endif

/* 以下带Locked的函数只在短临界区内操作内存，不等待CAN */
/* detail：1命令过期，2状态/速度过期，3端点批次过期，4端点读失败，
 * 5运行位置读失败，6状态/速度读失败，7缺起点，8距离保护；0其它原因。 */
/* 批次诊断只读写有界内存，不发串口、不等待CAN，不改变控制判断。 */
#if APP_IS_DEBUG
static ChassisBatchDiagnostic s_batch_history[CHASSIS_BATCH_DIAGNOSTIC_CAPACITY];
static ChassisBatchDiagnostic s_batch_current;
static ChassisBatchStatistics s_batch_statistics;
static uint32_t s_batch_count, s_batch_head, s_batch_id, s_batch_open;
static TickType_t s_batch_generated_tick;
static uint32_t s_batch_generation_source;
static void Auto_BatchChangedLocked(TickType_t generated, uint32_t source)
{
    s_batch_generated_tick = generated;
    s_batch_generation_source = source;
    if (s_batch_open) s_batch_current.pending_changes++;
}
static void Auto_BatchEndLocked(uint32_t outcome, TickType_t now)
{
    uint32_t slot, elapsed;
    if (!s_batch_open) return;
    s_batch_current.end_ms = (uint32_t)now * portTICK_PERIOD_MS;
    s_batch_current.outcome = outcome;
    s_batch_current.pending_rpm = s_heading_pending_rpm;
    elapsed = s_batch_current.end_ms - s_batch_current.start_ms;
    if (elapsed > s_batch_statistics.max_batch_ms) s_batch_statistics.max_batch_ms = elapsed;
    if (outcome == 1U) s_batch_statistics.total_success++;
    else s_batch_statistics.total_interrupted++;
    if (s_batch_count < CHASSIS_BATCH_DIAGNOSTIC_CAPACITY)
    {
        slot = (s_batch_head + s_batch_count++) % CHASSIS_BATCH_DIAGNOSTIC_CAPACITY;
    }
    else
    {
        slot = s_batch_head;
        s_batch_head = (s_batch_head + 1U) % CHASSIS_BATCH_DIAGNOSTIC_CAPACITY;
        s_batch_statistics.overwritten++;
    }
    s_batch_history[slot] = s_batch_current;
    s_batch_open = 0U;
}
static void Auto_BatchSendLocked(uint32_t index, TickType_t now)
{
    uint32_t elapsed;
    if (!s_batch_open)
    {
        ChassisBatchDiagnostic empty = {0};
        s_batch_current = empty;
        s_batch_current.session = g_chassis_auto_session;
        s_batch_current.batch_id = ++s_batch_id;
        s_batch_current.segment = g_chassis_auto_segment;
        s_batch_current.dry_run = s_mode;
        s_batch_current.generation_source = s_batch_generation_source;
        s_batch_current.generated_ms = (uint32_t)s_batch_generated_tick * portTICK_PERIOD_MS;
        s_batch_current.start_ms = (uint32_t)now * portTICK_PERIOD_MS;
        s_batch_current.correction_rpm = s_heading_applied_rpm;
        s_batch_current.base_rpm = s_forward_hold ? s_translation_applied_base : 0U;
        elapsed = s_batch_current.start_ms - s_batch_current.generated_ms;
        if (elapsed > s_batch_statistics.max_queue_ms) s_batch_statistics.max_queue_ms = elapsed;
        s_batch_open = 1U;
    }
    s_batch_current.send_ms[index] = (uint32_t)now * portTICK_PERIOD_MS;
    s_batch_current.sent_mask |= 1UL << index;
}
static void Auto_BatchReplyLocked(uint32_t session, uint32_t index, uint32_t result, TickType_t now)
{
    uint32_t elapsed;
    if (!s_batch_open || s_batch_current.session != session) return;
    s_batch_current.reply_ms[index] = (uint32_t)now * portTICK_PERIOD_MS;
    s_batch_current.result[index] = result;
    elapsed = s_batch_current.reply_ms[index] - s_batch_current.send_ms[index];
    if (elapsed > s_batch_statistics.max_call_ms) s_batch_statistics.max_call_ms = elapsed;
    if (result == 1U) s_batch_current.ack_mask |= 1UL << index;
    else
    {
        s_batch_current.failed_address = index + 1U;
        s_batch_current.io_result = result;
    }
    if (s_batch_current.ack_mask == 15U) Auto_BatchEndLocked(1U, now);
}
#else
#define Auto_BatchChangedLocked(t,s) ((void)0)
#define Auto_BatchEndLocked(o,t) ((void)0)
#define Auto_BatchSendLocked(i,t) ((void)0)
#define Auto_BatchReplyLocked(s,i,r,t) ((void)0)
#endif
uint32_t ChassisAuto_ReadBatchDiagnostic(uint32_t index, ChassisBatchDiagnostic *record,
    ChassisBatchStatistics *statistics)
{
#if APP_IS_DEBUG
    uint32_t found = 0U;
    if (!record || !statistics) return 0U;
    taskENTER_CRITICAL();
    if (index < s_batch_count)
    {
        *record = s_batch_history[(s_batch_head + index) % CHASSIS_BATCH_DIAGNOSTIC_CAPACITY];
        *statistics = s_batch_statistics;
        found = 1U;
    }
    taskEXIT_CRITICAL();
    return found;
#else
    (void)index; (void)record; (void)statistics; return 0U;
#endif
}

volatile ChassisAutoFaultDiagnostic g_chassis_auto_fault_diagnostic;
static uint32_t s_fault_diagnostic_valid;
static void Auto_CutDetailLocked(uint32_t cause, uint32_t detail, uint32_t io, uint32_t address)
{
    if (!s_fault_diagnostic_valid)
    {
        uint32_t i, age, oldest = 0U;
        TickType_t now = xTaskGetTickCount();
        g_chassis_auto_fault_diagnostic.reason = detail;
        g_chassis_auto_fault_diagnostic.state = g_chassis_auto_state;
        g_chassis_auto_fault_diagnostic.io_result = io;
        g_chassis_auto_fault_diagnostic.address = address;
        if(address>=1U && address<=4U) {
            const ChassisDirectionGuard *d=&s_direction_guard[address-1U];
            g_chassis_auto_fault_diagnostic.target_decirpm=d->target_deci;
            g_chassis_auto_fault_diagnostic.measured_decirpm=ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[address-1U]);
            g_chassis_auto_fault_diagnostic.target_ack_ms=d->ack_ms;
            g_chassis_auto_fault_diagnostic.speed_sample_ms=(uint32_t)s_speed_tick[address-1U]*portTICK_PERIOD_MS;
            g_chassis_auto_fault_diagnostic.reverse_samples=d->reverse_samples;
        }
        g_chassis_auto_fault_diagnostic.command_age_ms = (uint32_t)(now - s_heading_command_tick) * portTICK_PERIOD_MS;
        g_chassis_auto_fault_diagnostic.status_mask = s_status_valid;
        g_chassis_auto_fault_diagnostic.speed_mask = s_speed_valid;
        for (i = 0U; i < 4U; i++)
        {
            age = (uint32_t)(now - s_status_tick[i]) * portTICK_PERIOD_MS;
            if (age > oldest) oldest = age;
            age = (uint32_t)(now - s_speed_tick[i]) * portTICK_PERIOD_MS;
            if (age > oldest) oldest = age;
        }
        g_chassis_auto_fault_diagnostic.feedback_age_ms = oldest;
        g_chassis_auto_fault_diagnostic.tick_ms = (uint32_t)now * portTICK_PERIOD_MS;
        s_fault_diagnostic_valid = 1U;
    }
#if APP_POSITION_DIAGNOSTIC
    if (s_endpoint_index < s_endpoint_count)
        s_endpoints[s_endpoint_index].outcome = 2U;
#endif
    Auto_BatchEndLocked(2U, xTaskGetTickCount());
    g_chassis_auto_active = 0U;
    g_chassis_auto_cut_vm = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_FAULT;
    g_chassis_auto_cause = cause;
    g_chassis_auto_result = 2U;
}

static void Auto_CutLocked(uint32_t cause)
{ Auto_CutDetailLocked(cause, 0U, 0U, 0U); }
void ChassisAuto_ReadFaultDiagnostic(ChassisAutoFaultDiagnostic *snapshot)
{
    if (!snapshot) return;
    taskENTER_CRITICAL();
    *snapshot = g_chassis_auto_fault_diagnostic;
    taskEXIT_CRITICAL();
}

static void Auto_EnterLocked(uint32_t state, TickType_t now)
{
    if (state == CHASSIS_AUTO_STOP) Auto_BatchEndLocked(3U, now);
    g_chassis_auto_state = state;
    s_phase_tick = now;
    /* 普通首次批次没有前端发布时间，以选择START时刻为生成边界。 */
    if (state == CHASSIS_AUTO_START && !s_heading_session)
        Auto_BatchChangedLocked(now, 0U);
    s_motor_index = 0U;
    s_monitor_between = 0U;
}

static uint32_t Auto_BaseHealthy(void)
{
    return ((g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_safety_ready == 1U) && (g_can_guard_ready == 1U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U)) ? 1U : 0U;
}

static uint32_t Auto_AllStatusSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_status_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_status_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_AllSpeedSinceLocked(TickType_t now)
{
    uint32_t i;
    if (s_speed_valid != 0x0FU) return 0U;
    for (i = 0U; i < 4U; i++)
        if (((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > (TickType_t)(now - s_phase_tick)))
            return 0U;
    return 1U;
}

static uint32_t Auto_ZeroSpeedLocked(void)
{
    uint32_t i;
    for (i = 0U; i < 4U; i++) if (g_chassis_auto_driver_rpm[i] != 0) return 0U;
    return 1U;
}

#if APP_DISTANCE_SCURVE_ACTIVE
/* P目标限幅后，经精确离散的二阶临界阻尼滤波；每次最多更新一次。
 * 对|u|<=C，连续滤波的保守界为|c|<=C、|c'|<=2*C*w、|c''|<=6*C*w*w。
 * prepare在原速度/加速度/jerk总预算中扣除这些余量；整数命令/到距停止另验。 */
static uint32_t Auto_ProfileFollowRpmLocked(float elapsed, float progress, TickType_t now, float *rpm)
{
    float c = s_route_trial ? ROUTE_TRIAL_FOLLOW_RPM : MotionDistanceFollowRpm(s_distance_max_rpm), w = APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
    float desired = (s_route_trial ? ROUTE_TRIAL_FOLLOW_KP : MotionDistanceFollowKp(s_distance_max_rpm)) *
        (s_distance_reference.position_mm-progress) / s_profile_mm_per_rpm_s;
    TickType_t ticks = (TickType_t)(now-s_profile_follow_tick);
    float value;
    if (!(desired >= -1.0e6f && desired <= 1.0e6f) ||
        !(s_profile_follow_rpm >= -c-0.001f && s_profile_follow_rpm <= c+0.001f) ||
        !(s_profile_follow_rate >= -2.0f*c*w-0.001f && s_profile_follow_rate <= 2.0f*c*w+0.001f)) return 0U;
    if (desired > c) desired = c;
    if (desired < -c) desired = -c;
    if (ticks >= pdMS_TO_TICKS(10U))
    {
        float dt = (float)ticks * portTICK_PERIOD_MS / 1000.0f;
        float y, b, decay;
        /* 调度迟到时不回放多步；较长缺口仍由原源/位置保护处理。 */
        if (dt > 0.1f) dt = 0.1f;
        y = s_profile_follow_rpm-desired;
        b = s_profile_follow_rate+w*y;
        decay = expf(-w*dt);
        value = desired+(y+b*dt)*decay;
        s_profile_follow_rate = (s_profile_follow_rate-w*b*dt)*decay;
        s_profile_follow_rpm = value;
        s_profile_follow_tick = now;
    }
    value = s_distance_reference.speed_mm_s/s_profile_mm_per_rpm_s+s_profile_follow_rpm;
    if (!(value >= -c-0.001f && value <= (float)s_distance_max_rpm+0.001f)) return 0U;
    if (value < 0.0f) value = 0.0f;
    if (value > (float)s_distance_max_rpm) value = (float)s_distance_max_rpm;
    /* 从减速段连续接近，防止取整先变0后再补一脚。仍受原duration+2s截止约束。 */
    if (elapsed >= s_distance_profile.boundary_s[4] &&
        value < (float)APP_DISTANCE_SCURVE_APPROACH_RPM)
        value = (float)APP_DISTANCE_SCURVE_APPROACH_RPM;
    *rpm = value;
    return 1U;
}
#endif

/* 独立时限和反馈检查，CAN停推进时由Safety继续调用 */
#if APP_POSITION_DIAGNOSTIC
static void Auto_DistanceFaultLocked(uint32_t reason)
{ s_distance_fault_reason = reason; Auto_CutDetailLocked(3U,8U,0U,0U); }
static void Auto_DistanceCheckLocked(TickType_t now)
{
    uint32_t i, rpm;
    float mean, spread, low=0.0f, high=0.0f;
    uint32_t common_ms=0U, oldest_age=0U;
    ChassisPositionEndpoints *base;
    if (!s_distance_mode || s_finish_requested || s_stop_due ||
        (g_chassis_auto_state != CHASSIS_AUTO_RUN
#if APP_DISTANCE_SCURVE_ACTIVE
         && !(s_profile_active && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START)
#endif
        )) return;
    if (s_endpoint_index >= s_endpoint_count) { Auto_DistanceFaultLocked(1U); return; }
    base = &s_endpoints[s_endpoint_index];
    if (base->start_mask != 15U) { Auto_DistanceFaultLocked(1U); return; }
    if (s_position_valid != 15U)
    {
        if ((TickType_t)(now - s_run_tick) >= pdMS_TO_TICKS(500U)) Auto_DistanceFaultLocked(2U);
        return;
    }
    for (i = 0U; i < 4U; i++)
    {
        uint32_t ms = (uint32_t)s_position_tick[i] * portTICK_PERIOD_MS;
        if ((TickType_t)(now - s_position_tick[i]) >= pdMS_TO_TICKS(400U))
        { Auto_DistanceFaultLocked(3U); return; }
        if (!s_position_history_count[i])
            Auto_PositionRemember(i,base->start_ms[i],0.0f);
        if (ms != s_distance_seen_ms[i])
        {
            int64_t old = (int64_t)s_distance_seen_raw[i], current = (int64_t)s_position.raw[i];
            uint32_t dt = ms - s_distance_seen_ms[i];
            int64_t change;
            if (s_distance_seen_sign[i]) old = -old;
            if (s_position.sign[i]) current = -current;
            change = current - old; if (change < 0) change = -change;
            /* 60RPM等效上限加量化余量：仅跳变诊断，原30RPM保护不变。 */
            if (dt > 2000U || change > (int64_t)dt * (s_route_trial ? ((ROUTE_TRIAL_FEEDBACK_RPM+30U)*65536U+59999U)/60000U :
                    MotionDistanceJumpCountsPerMs(s_translation_mode,s_distance_max_rpm,s_distance_target_mm)) + 512)
            { Auto_DistanceFaultLocked(4U); return; }
            {
                static const int8_t projection[4][4]={{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1},{-1,-1,-1,-1}};
                int64_t start=(int64_t)base->start_raw[i];
                float mm;
                if (s_translation_mode>3U || base->start_sign[i]>1U || s_position.sign[i]>1U)
                { Auto_DistanceFaultLocked(5U); return; }
                if (base->start_sign[i]) start=-start;
                mm=(float)(current-start)*3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/65536.0f;
                if (g_chassis_forward_direction[i]) mm=-mm;
                mm*=projection[s_translation_mode][i];
                if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE)
                    mm-=((i==0U || i==3U)?1.0f:-1.0f)*s_position_trim_mm[i];
                Auto_PositionRemember(i,ms,mm);
            }
            s_distance_seen_ms[i] = ms; s_distance_seen_raw[i] = s_position.raw[i];
            s_distance_seen_sign[i] = s_position.sign[i];
        }
    }
    /* 最旧的最新样本是共同时间；其它轮只在历史包围区间内插值。 */
    for (i=0U;i<4U;i++)
    {
        uint32_t age=(uint32_t)(now-s_position_tick[i])*portTICK_PERIOD_MS;
        if (!i || age>oldest_age) { oldest_age=age; common_ms=(uint32_t)s_position_tick[i]*portTICK_PERIOD_MS; }
    }
    mean=0.0f;
    for (i=0U;i<4U;i++)
    {
        float mm;
        if (!Auto_PositionAt(i,common_ms,&mm))
        {
            /* 数据持续更新但历史不相交也不能无限等待。 */
            uint32_t since=s_alignment_seen ? (uint32_t)now*portTICK_PERIOD_MS-s_alignment_ms :
                (uint32_t)(now-s_run_tick)*portTICK_PERIOD_MS;
            if (since>=400U) Auto_DistanceFaultLocked(3U);
            return;
        }
        if (!i || mm<low) low=mm;
        if (!i || mm>high) high=mm;
        mean+=mm*0.25f;
    }
    spread=high-low;
    if (!Chassis_CalibrateDistanceMm(s_translation_mode,mean,&mean) ||
        mean < -5.0f || spread > 50.0f || mean < s_distance_progress_mm-5.0f ||
        mean > s_distance_target_mm+50.0f)
    { Auto_DistanceFaultLocked(5U); return; }
    s_alignment_seen=1U; s_alignment_ms=common_ms;
    s_distance_progress_mm = mean;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (s_profile_active)
    {
        uint32_t result;
        float elapsed = (float)(TickType_t)(now-s_run_tick) * portTICK_PERIOD_MS / 1000.0f;
        float ref_rpm;
        result = MotionProfile_Sample(&s_distance_profile,elapsed,&s_distance_reference);
        if (!result) { Auto_DistanceFaultLocked(8U); return; }
        s_profile_done = (result == MOTION_PROFILE_DONE);
        /* 到实际编码目标优先停；参考结束不能冒充实际到距。 */
        if (mean >= s_distance_target_mm ||
            (s_profile_done && mean >= s_distance_target_mm-APP_DISTANCE_SCURVE_ARRIVAL_MM))
        {
            s_distance_reached = 1U; s_finish_requested = s_stop_due = 1U;
            s_finish_tick = now; return;
        }
        {
            MotionProfileSample observed_reference;
            float observed_elapsed=(float)(uint32_t)(common_ms-(uint32_t)s_run_tick*portTICK_PERIOD_MS)/1000.0f;
            if ((int32_t)(common_ms-(uint32_t)s_run_tick*portTICK_PERIOD_MS)<0) observed_elapsed=0.0f;
            if (!MotionProfile_Sample(&s_distance_profile,observed_elapsed,&observed_reference))
            { Auto_DistanceFaultLocked(8U); return; }
            if (mean-observed_reference.position_mm > APP_DISTANCE_SCURVE_TRACK_MM ||
                observed_reference.position_mm-mean > APP_DISTANCE_SCURVE_TRACK_MM)
            { Auto_DistanceFaultLocked(7U); return; }
            /* 跟随修正使用同一时刻的误差，速度前馈仍按当前时刻推进。 */
            float current_reference=s_distance_reference.position_mm;
            float equivalent_progress=mean+current_reference-observed_reference.position_mm;
            if (!Auto_ProfileFollowRpmLocked(elapsed,equivalent_progress,now,&ref_rpm))
            { Auto_DistanceFaultLocked(8U); return; }
        }
        if (s_profile_done && elapsed >= s_distance_profile.duration_s +
                (float)APP_DISTANCE_SCURVE_FINAL_WAIT_MS/1000.0f)
        { Auto_DistanceFaultLocked(9U); return; }
        if (!(ref_rpm >= -0.001f && ref_rpm <= (float)s_distance_max_rpm+0.001f))
        { Auto_DistanceFaultLocked(8U); return; }
        rpm = (ref_rpm <= 0.0f) ? 0U : (uint32_t)(ref_rpm+0.5f);
        if (rpm > s_distance_max_rpm) rpm = s_distance_max_rpm;
        if (rpm != s_translation_base_rpm) Auto_BatchChangedLocked(now,3U);
        s_translation_base_rpm = rpm;
        return; /* 末端有限接近；到距、源失效、跟随误差及原截止仍优先。 */
    }
#endif
    rpm = Chassis_DistanceRpm(s_distance_target_mm - mean, s_distance_max_rpm);
    if (!rpm)
    {
        s_distance_reached = 1U;
        s_finish_requested = s_stop_due = 1U;
        s_finish_tick = now;
    }
    else
    {
        if (rpm != s_translation_base_rpm) Auto_BatchChangedLocked(now, 3U);
        s_translation_base_rpm = rpm;
    }
}
#endif

static void Auto_CheckWheelSpeedLocked(uint32_t i,TickType_t now)
{
    TickType_t run_age=(TickType_t)(now-s_run_tick);
    if(!g_chassis_auto_active || s_mode!=0U || !s_run_started || run_age<pdMS_TO_TICKS(500U) ||
       (g_chassis_auto_state!=CHASSIS_AUTO_START && g_chassis_auto_state!=CHASSIS_AUTO_RUN))return;
            float raw = g_chassis_auto_driver_rpm[i];
            uint32_t speed_limit = (s_forward_hold == 1U) ?
                ((s_translation_mode == 0U) ? APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM :
                 APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM) : CHASSIS_AUTO_SPEED_LIMIT_RPM;
#if APP_POSITION_DIAGNOSTIC
            if (s_forward_hold && s_distance_mode)
                speed_limit=s_route_trial ? ROUTE_TRIAL_FEEDBACK_RPM :
                    MotionDistanceFeedbackLimit(s_translation_mode,s_distance_max_rpm,s_distance_target_mm);
#endif
            if (s_turn_brake_trial) speed_limit=APP_TURN_BRAKE_FEEDBACK_RPM;
            TickType_t direction_age = ((s_heading_session == 1U) && (s_forward_hold == 0U)) ?
                (TickType_t)(now - s_heading_update_tick) : run_age;
            /* 不把1RPM量化为0误判为停转；只拒绝明显超速和新采到的反向 */
            if (!(raw >= -(float)speed_limit && raw <= (float)speed_limit))
            { Auto_CutDetailLocked(8U,9U,0U,i+1U); return; }
            if ((direction_age >= pdMS_TO_TICKS(500U)) &&
                ((TickType_t)(now-s_speed_tick[i]) <= direction_age-pdMS_TO_TICKS(500U))) {
                uint32_t endpoint_window=0U,reason;
#if APP_DISTANCE_SCURVE_ACTIVE
                endpoint_window=s_forward_hold && s_distance_mode && s_route_trial && s_profile_active &&
                    !s_finish_requested && !s_stop_due && !s_distance_reached &&
                    s_translation_base_rpm<=2U && s_distance_target_mm>=100.0f &&
                    s_distance_progress_mm>=s_distance_target_mm-10.0f &&
                    s_distance_progress_mm<=s_distance_target_mm &&
                    s_distance_reference.position_mm>=s_distance_target_mm*0.5f &&
                    s_distance_reference.acceleration_mm_s2<=0.0f;
#endif
                reason=DirectionGuard_Check(&s_direction_guard[i],(uint32_t)now*portTICK_PERIOD_MS,
                    (uint32_t)s_speed_tick[i]*portTICK_PERIOD_MS,ZdtEmm_RpmDeci(raw),endpoint_window);
                if(reason) { Auto_CutDetailLocked(8U,reason,0U,i+1U); return; }
            }
}

static void Auto_CheckLocked(TickType_t now)
{
    uint32_t i, state = g_chassis_auto_state;
    TickType_t run_age, run_limit;
    if (g_chassis_auto_active != 1U) return;
    if (!Auto_BaseHealthy()) { Auto_CutLocked(2U); return; }
    /* 任务提交中断时撤销会话；正常停车继续由原截止和反馈监督。 */
    if ((s_heading_session == 1U) && (s_finish_requested == 0U) &&
        (s_stop_due == 0U) && (state <= CHASSIS_AUTO_RUN) &&
        ((TickType_t)(now - s_heading_command_tick) >=
         pdMS_TO_TICKS(CHASSIS_HEADING_MAILBOX_MAX_AGE_MS)))
    { Auto_CutDetailLocked(3U,1U,0U,0U); return; }
    if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS((s_distance_mode && s_translation_run_ms>30000U) ? 75000U : CHASSIS_AUTO_TOTAL_MS))
    { Auto_CutLocked(6U); return; }
    if (state <= CHASSIS_AUTO_VERIFY_ON)
    {
        if ((TickType_t)(now - s_total_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_PREP_MS))
            Auto_CutLocked(6U);
        return;
    }
    for (i = 0U; i < 4U; i++)
    {
        if (((s_status_valid & (1UL << i)) == 0U) ||
            ((s_speed_valid & (1UL << i)) == 0U) ||
            ((TickType_t)(now - s_status_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)) ||
            ((TickType_t)(now - s_speed_tick[i]) > pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS)))
        { Auto_CutDetailLocked(3U,2U,0U,i+1U); return; }
        if ((s_status[i] & 0x0DU) != 1U) { Auto_CutLocked(2U); return; }
    }
    if ((state == CHASSIS_AUTO_SETTLE) &&
        ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_SETTLE_MS)))
    { Auto_CutLocked(7U); return; }
    /* 从首次正常停车请求计时；停帧未齐则走原VM兜底。 */
    if ((s_finish_requested != 0U) &&
        ((TickType_t)(now - s_finish_tick) >=
         pdMS_TO_TICKS(CHASSIS_AUTO_STOP_MS)) &&
        (g_chassis_auto_stop_mask[g_chassis_auto_segment] != 0x0FU))
    { Auto_CutLocked(6U); return; }
    if (!s_run_started || (state == CHASSIS_AUTO_SETTLE)) return;
    /* Pending low-speed reverse remains bounded even while a new batch sends.
     * STOP has its independent, shorter 500ms stop-ACK deadline. */
    if(state==CHASSIS_AUTO_START || state==CHASSIS_AUTO_RUN) {
        for(i=0U;i<4U;i++)if(DirectionGuard_Expired(&s_direction_guard[i],(uint32_t)now*portTICK_PERIOD_MS))
        { Auto_CutDetailLocked(8U,DIRECTION_PERSISTENT,0U,i+1U);return; }
    }
    run_age = (TickType_t)(now - s_run_tick);
    run_limit = pdMS_TO_TICKS((s_forward_hold == 1U) ?
        (s_translation_run_ms + 1000U) :
        ((s_heading_session == 1U) ? CHASSIS_HEADING_RUN_MS : CHASSIS_AUTO_RUN_MS));
    if (run_age >= run_limit)
    {
        if (s_distance_mode) { s_distance_fault_reason = 6U; Auto_CutLocked(6U); return; }
        s_stop_due = 1U;
    }
    if ((run_age >= (TickType_t)(run_limit + pdMS_TO_TICKS(CHASSIS_AUTO_STOP_MS))) &&
        (g_chassis_auto_stop_mask[g_chassis_auto_segment] != 0x0FU))
    { Auto_CutLocked(6U); return; }
    if ((s_mode == 0U) && (state == CHASSIS_AUTO_RUN) &&
        (run_age >= pdMS_TO_TICKS(500U)))
    {
        for (i = 0U; i < 4U; i++)
        {
            Auto_CheckWheelSpeedLocked(i,now);
            if(!g_chassis_auto_active)return;
        }
    }
#if APP_POSITION_DIAGNOSTIC
    Auto_DistanceCheckLocked(now);
#endif
}

void ChassisAuto_Init(void)
{
    uint32_t i;
    ChassisAutoFaultDiagnostic empty_fault = {0};
    g_chassis_auto_fault_diagnostic = empty_fault; s_fault_diagnostic_valid = 0U;
    g_chassis_auto_request = 0U;
    g_chassis_auto_dry_run = g_chassis_auto_mode = 1U;
    g_chassis_auto_state = CHASSIS_AUTO_IDLE;
    g_chassis_auto_segment = g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
    g_chassis_auto_session = g_chassis_auto_result = g_chassis_auto_cause = 0U;
    g_chassis_auto_io_result = g_chassis_auto_last_address = 0U;
    s_vm_requested = s_used = s_mode = 0U;
    for(i=0U;i<4U;i++)DirectionGuard_Reset(&s_direction_guard[i]);
#if APP_POSITION_DIAGNOSTIC
    {
        ChassisOdometrySamples empty={0}; s_odometry_samples=empty;
    }
    Auto_PositionReset(0U);
#endif
#if APP_TEST_MODE == 3U
    s_sequence_hold = s_sequence_continue = 0U;
#endif
    s_finish_requested = 0U;
    s_finish_tick = 0U;
    s_heading_session = 0U;
    s_turn_brake_trial = 0U;
    s_forward_hold = 0U;s_route_trial=0U; s_distance_mode = s_distance_reached = s_distance_fault_reason = 0U;
    s_translation_mode = 0U;
    s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
    s_heading_update_tick = 0U;
    s_heading_command_tick = 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    s_profile_active = s_profile_done = 0U;
    s_profile_follow_rpm = s_profile_follow_rate = 0.0f; s_profile_follow_tick = 0U;
#endif
#if APP_IS_DEBUG
    {
        ChassisBatchStatistics empty = {0};
        s_batch_statistics = empty;
        s_batch_count = s_batch_head = s_batch_id = s_batch_open = 0U;
        s_batch_generated_tick = 0U; s_batch_generation_source = 0U;
    }
#endif
    s_motor_index = s_monitor_cursor = s_monitor_between = 0U;
    s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
    s_total_tick = s_phase_tick = s_run_tick = 0U;
    for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
    for (i = 0U; i < 4U; i++)
    { s_status[i] = 0U; s_status_tick[i] = s_speed_tick[i] = 0U; g_chassis_auto_driver_rpm[i] = 0; }
}

/* 基础命令只写局部缓冲，允许在中断开启时预计算。 */
static uint32_t Auto_PrepareBasicCommands(ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT])
{
    uint32_t i, j;
    for (i = 0U; i < 3U; i++)
    {
        if (!Chassis_BasicTestCommands(&g_chassis_geometry, i, commands[i])) return 0U;
        for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++)
            commands[i][j].speed_raw = (uint16_t)(10U*CHASSIS_AUTO_TARGET_RPM);
#if APP_HWT_TURN_TEST && APP_HWT_TURN_CLOCKWISE
        if (i == 2U)
            for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++) commands[i][j].direction ^= 1U;
#endif
    }
    return 1U;
}
/* 保留原启动门禁和初始化，仅在短锁内复制预计算结果。 */
static uint32_t Auto_BeginPreparedBasicLocked(uint32_t dry_run,
    const ChassisMotorCommand *commands)
{
    uint32_t i, j, accepted = 0U;
    if ((dry_run <= 1U) && s_vm_requested && !s_used && !g_chassis_auto_active &&
        !g_chassis_auto_cut_vm && Auto_BaseHealthy())
    {
        for (i = 0U; i < 3U; i++)
            for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++) s_commands[i][j] = commands[i * CHASSIS_WHEEL_COUNT + j];
        ChassisAutoFaultDiagnostic empty_fault = {0};
        g_chassis_auto_fault_diagnostic = empty_fault; s_fault_diagnostic_valid = 0U;
        s_used = 1U; s_mode = dry_run; g_chassis_auto_mode = dry_run;
        g_chassis_auto_session++;
        for(i=0U;i<4U;i++)DirectionGuard_Reset(&s_direction_guard[i]);
        Auto_BatchChangedLocked(xTaskGetTickCount(), 0U);
#if APP_POSITION_DIAGNOSTIC
        Auto_PositionReset(g_chassis_auto_session);
#endif
        g_chassis_auto_segment = CHASSIS_AUTO_FIRST_SEGMENT;
        g_chassis_auto_result = g_chassis_auto_cause = 0U;
        s_total_tick = xTaskGetTickCount();
        s_finish_requested = 0U;
        s_finish_tick = 0U;
        s_heading_session = 0U;
    s_turn_brake_trial = 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
        s_profile_active = s_profile_done = 0U;
        s_profile_follow_rpm = s_profile_follow_rate = 0.0f; s_profile_follow_tick = 0U;
#endif
        s_forward_hold = 0U;s_route_trial=0U; s_distance_mode = s_distance_reached = s_distance_fault_reason = 0U;
        s_translation_mode = 0U;
        s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
        s_heading_update_tick = 0U;
        s_heading_command_tick = 0U;
        s_status_valid = s_speed_valid = s_run_started = s_stop_due = 0U;
        s_monitor_cursor = 0U;
        for (i = 0U; i < 3U; i++) g_chassis_auto_start_mask[i] = g_chassis_auto_stop_mask[i] = 0U;
        g_chassis_auto_active = 1U;
#if APP_TEST_MODE == 3U
        /* 后续段保持使能，只重新核对零速/状态，不重复失能和使能。 */
        Auto_EnterLocked(s_sequence_continue ? CHASSIS_AUTO_VERIFY_ON :
                         CHASSIS_AUTO_DISABLE, s_total_tick);
        s_sequence_continue = 0U;
#else
        Auto_EnterLocked(CHASSIS_AUTO_DISABLE, s_total_tick);
#endif
        accepted = 1U;
    }
    if (!accepted) g_chassis_auto_result = 9U;
    return accepted;
}
uint32_t ChassisAuto_Begin(uint32_t dry_run)
{
    ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT];
    uint32_t prepared = Auto_PrepareBasicCommands(commands), accepted;
    taskENTER_CRITICAL();
    if (prepared) accepted = Auto_BeginPreparedBasicLocked(dry_run, &commands[0][0]);
    else { g_chassis_auto_result = 9U; accepted = 0U; }
    taskEXIT_CRITICAL();
    return accepted;
}

void ChassisAuto_SafetyUpdate(uint32_t vm_request)
{
    taskENTER_CRITICAL();
    s_vm_requested = (vm_request == 1U) ? 1U : 0U;
    if (!s_vm_requested)
    {
        Auto_BatchEndLocked(3U, xTaskGetTickCount());
#if APP_POSITION_DIAGNOSTIC
        s_position_valid = s_position_due = 0U;
#endif
        if (g_chassis_auto_active)
        {
#if APP_POSITION_DIAGNOSTIC
            if (s_endpoint_index < s_endpoint_count) s_endpoints[s_endpoint_index].outcome = 2U;
#endif
            g_chassis_auto_state = CHASSIS_AUTO_CANCELLED; g_chassis_auto_result = 3U;
        }
        g_chassis_auto_active = g_chassis_auto_cut_vm = 0U;
        s_used = 0U;
#if APP_TEST_MODE == 3U
        s_sequence_hold = s_sequence_continue = 0U;
#endif
    }
    else Auto_CheckLocked(xTaskGetTickCount());
    taskEXIT_CRITICAL();
}

#if APP_TEST_MODE == 3U
/* 整组许可只在首次闲置申请；故障/取消后的组不能从这里重开。 */
/* Only a normally completed, zero-speed, powered-off sequence may be reused.
 * This never clears a fault/cancelled result or any global fault latch. */
void ChassisAuto_ReportIdleSpeed(uint32_t index,float rpm,uint32_t tick)
{
    if(index>=4U || g_chassis_auto_active || !StartupAxes_Holding())return;
    taskENTER_CRITICAL();
    g_chassis_auto_driver_rpm[index]=rpm;s_speed_tick[index]=(TickType_t)tick;s_speed_valid|=1UL<<index;
    taskEXIT_CRITICAL();
}
uint32_t ChassisAuto_RearmCompleted(uint32_t expected_session)
{
    uint32_t accepted=0U,i,fresh=1U;PowerCommandStatus power;
    taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
    for(i=0U;i<4U;i++)if(!(s_speed_valid&(1UL<<i)) || xTaskGetTickCount()-s_speed_tick[i]>pdMS_TO_TICKS(1000U))fresh=0U;
    if(fresh && expected_session==g_chassis_auto_session && !s_vm_requested && !s_used &&
       !s_sequence_hold && !g_chassis_auto_active && !g_chassis_auto_cut_vm && (!power.active || StartupAxes_Holding()) &&
       g_chassis_auto_state==CHASSIS_AUTO_DONE && g_chassis_auto_result==1U &&
       !g_chassis_auto_cause && g_chassis_auto_segment<3U &&
       g_chassis_auto_stop_mask[g_chassis_auto_segment]==15U &&
       Auto_ZeroSpeedLocked() && !s_fault_diagnostic_valid &&
       !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched &&
       !g_motion_diag_fault_latched && !g_motion_auto_stop_cut_vm &&
       !g_wheel_test_active && !g_wheel_test_cut_vm) {
        g_chassis_auto_state=CHASSIS_AUTO_IDLE;g_chassis_auto_result=0U;
        s_sequence_continue=0U;accepted=1U;
    }
    taskEXIT_CRITICAL();return accepted;
}
uint32_t ChassisAuto_BeginSequence(void)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if (!s_sequence_hold && !s_vm_requested && !s_used &&
        !g_chassis_auto_active && !g_chassis_auto_cut_vm &&
        (g_chassis_auto_state == CHASSIS_AUTO_IDLE) &&
        !g_chassis_auto_result && !g_rtos_fault && !g_safety_fault &&
        !g_can_ctrl_fault_latched)
    { s_sequence_hold = 1U; s_sequence_continue = 0U; accepted = 1U; }
    taskEXIT_CRITICAL();
    return accepted;
}
/* 仅释放已成功停稳且仍上电的本组会话，不清故障，不重新初始化。 */
uint32_t ChassisAuto_ReleaseCompleted(uint32_t session)
{
    uint32_t accepted = 0U;
    PowerCommandStatus power;
    taskENTER_CRITICAL();
    Power_ReadCommandStatus(&power);
    if (s_sequence_hold && (session == g_chassis_auto_session) &&
        s_vm_requested && s_used && !g_chassis_auto_active &&
        !g_chassis_auto_cut_vm && (power.active == 1U) && Auto_BaseHealthy() &&
        (g_chassis_auto_state == CHASSIS_AUTO_DONE) &&
        (g_chassis_auto_result == 1U) && (g_chassis_auto_cause == 0U) &&
        (g_chassis_auto_stop_mask[2] == 0x0FU) && Auto_ZeroSpeedLocked() &&
        !g_can_ctrl_fault_latched)
    {
        s_sequence_continue = 1U;
        s_used = 0U;
        g_chassis_auto_result = 0U;
        g_chassis_auto_state = CHASSIS_AUTO_IDLE;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
#endif

void ChassisAuto_Abort(void)
{
    taskENTER_CRITICAL();
    if (g_chassis_auto_active) Auto_CutLocked(9U);
    taskEXIT_CRITICAL();
}

#if APP_POSITION_DIAGNOSTIC
/* 仅从原零速准备/停稳条件成立的分支调用，每次最多一轮位置事务。
 * 下一轮仍重新检查原状态、零速与截止，不延长原有保护。 */
static uint32_t Auto_EndpointStep(uint32_t session, uint32_t end)
{
    uint32_t i, mask, result, state, index;
    uint8_t sign = 0U;
    uint32_t raw = 0U;
    CanRxMsg reply = {0};
    taskENTER_CRITICAL();
    index = s_endpoint_index;
    if (index >= s_endpoint_count || !s_heading_session)
    { taskEXIT_CRITICAL(); return 0U; }
    state = end ? CHASSIS_AUTO_SETTLE : CHASSIS_AUTO_VERIFY_ON;
    if (session != g_chassis_auto_session || !g_chassis_auto_active ||
        g_chassis_auto_state != state || !s_vm_requested)
    { taskEXIT_CRITICAL(); return 0U; }
    mask = end ? s_endpoints[index].end_mask : s_endpoints[index].start_mask;
    if (mask == 15U)
    {
        TickType_t now = xTaskGetTickCount();
        uint32_t first = end ? s_endpoints[index].end_ms[0] : s_endpoints[index].start_ms[0];
        if ((uint32_t)(now * portTICK_PERIOD_MS - first) >= CHASSIS_AUTO_FRESH_MS)
        { Auto_CutDetailLocked(3U,3U,0U,0U); taskEXIT_CRITICAL(); return 1U; }
        taskEXIT_CRITICAL(); return 0U;
    }
    for (i = 0U; i < 4U; i++) if (!(mask & (1UL << i))) break;
    taskEXIT_CRITICAL();
    result = CAN1_ReadPosition((uint8_t)(i + 1U), &reply, &sign, &raw);
    taskENTER_CRITICAL();
    if (session == g_chassis_auto_session && g_chassis_auto_active &&
        g_chassis_auto_state == state && s_vm_requested)
    {
        uint32_t ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
        Auto_OdometrySample(i,result,sign,raw,ms);
        if (result != 1U) Auto_CutDetailLocked(3U,4U,result,i+1U);
        else if (end)
        {
            s_endpoints[index].end_sign[i] = sign;
            s_endpoints[index].end_raw[i] = raw;
            s_endpoints[index].end_ms[i] = ms;
            s_endpoints[index].end_mask |= 1UL << i;
        }
        else
        {
            s_endpoints[index].start_sign[i] = sign;
            s_endpoints[index].start_raw[i] = raw;
            s_endpoints[index].start_ms[i] = ms;
            s_endpoints[index].start_mask |= 1UL << i;
        }
    }
    taskEXIT_CRITICAL(); return 1U;
}
#endif

uint32_t ChassisAuto_ReadPositionEndpoints(uint32_t index, ChassisPositionEndpoints *record)
{
    if (record == NULL) return 0U;
#if APP_POSITION_DIAGNOSTIC
    taskENTER_CRITICAL();
    if (index >= s_endpoint_count)
    { taskEXIT_CRITICAL(); return 0U; }
    *record = s_endpoints[index];
    taskEXIT_CRITICAL(); return 1U;
#else
    (void)index;
    return 0U;
#endif
}

#if APP_POSITION_DIAGNOSTIC
/* 仅RUN插入一次有界只读事务；准备、改速、STOP和SETTLE不插入。
 * 状态/速度未齐或任一超过500ms时跳过，原1000ms保护不放宽。 */
static uint32_t Auto_PositionMonitor(uint32_t session)
{
    uint32_t i, result, moving, oldest;
    uint8_t sign = 0U;
    uint32_t raw = 0U;
    TickType_t now;
    CanRxMsg reply = {0};
    taskENTER_CRITICAL();
    i = s_position_due;
    s_position_due = 0U;
    now = xTaskGetTickCount();
    moving = (g_chassis_auto_state == CHASSIS_AUTO_RUN) ||
        g_chassis_auto_state==CHASSIS_AUTO_VERIFY_ON || g_chassis_auto_state==CHASSIS_AUTO_VERIFY_OFF ||
        g_chassis_auto_state==CHASSIS_AUTO_SETTLE ||
        ((s_distance_mode || s_heading_session) && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START);
    /* 调速串行下发期间仍在运动：在原monitor槽优先补最旧的位置，
     * 避免反复纠偏让某轮饿死；400ms时效不放宽。 */
    oldest = 0U;
    if (moving)
        for (result = 0U; result < 4U; result++)
        {
            uint32_t age = (uint32_t)(TickType_t)(now - s_position_tick[result]);
            if (!(s_position_valid & (1UL << result))) { i = result + 1U; break; }
            if (age >= pdMS_TO_TICKS(100U) && age > oldest)
            { oldest = age; i = result + 1U; }
        }
    if (!i || session != g_chassis_auto_session || !g_chassis_auto_active ||
        !moving || (g_chassis_auto_state!=CHASSIS_AUTO_SETTLE && (s_stop_due || s_finish_requested)) ||
        s_status_valid != 15U || s_speed_valid != 15U || !Auto_BaseHealthy())
    { taskEXIT_CRITICAL(); return 0U; }
    for (result = 0U; result < 4U; result++)
        if ((TickType_t)(now - s_status_tick[result]) >= pdMS_TO_TICKS(500U) ||
            (TickType_t)(now - s_speed_tick[result]) >= pdMS_TO_TICKS(500U))
        { taskEXIT_CRITICAL(); return 0U; }
    i--;
    taskEXIT_CRITICAL();
    result = CAN1_ReadPosition((uint8_t)(i + 1U), &reply, &sign, &raw);
    taskENTER_CRITICAL();
    if (session == g_chassis_auto_session && g_chassis_auto_active &&
        ((g_chassis_auto_state == CHASSIS_AUTO_RUN) ||
         ((s_distance_mode || s_heading_session) && s_run_started && g_chassis_auto_state == CHASSIS_AUTO_START) ||
         g_chassis_auto_state==CHASSIS_AUTO_VERIFY_ON || g_chassis_auto_state==CHASSIS_AUTO_VERIFY_OFF ||
         g_chassis_auto_state==CHASSIS_AUTO_SETTLE) && s_vm_requested)
    {
        Auto_OdometrySample(i,result,sign,raw,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
        s_position.result[i] = result;
        s_position.sequence++;
        if (result == 1U)
        {
            s_position.sign[i] = sign; s_position.raw[i] = raw;
            s_position_tick[i] = xTaskGetTickCount();
            if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE)
            {
                Auto_LeftTrimAdvance((uint32_t)s_position_tick[i]*portTICK_PERIOD_MS);
                s_position_trim_mm[i]=s_left_trim_mm;
            }
            s_position_valid |= 1UL << i;
        }
        else
        {
            s_position_valid &= ~(1UL << i);
            Auto_CutDetailLocked(3U,5U,result,i+1U); /* 新增查询失败也停止，不掩盖通信异常。 */
        }
    }
    taskEXIT_CRITICAL();
    return 1U;
}
#endif

void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *samples)
{
    if(!samples) return;
    taskENTER_CRITICAL();
#if APP_POSITION_DIAGNOSTIC
    *samples=s_odometry_samples;
#else
    { ChassisOdometrySamples empty={0}; *samples=empty; }
#endif
    taskEXIT_CRITICAL();
}

/* Only inactive, powered, successful gaps/preparation; one bounded CAN transaction. */
void ChassisAuto_OdometryPoll(void)
{
    uint32_t i=0U,j,oldest=0U,result,session,now;
    uint8_t sign=0U;uint32_t raw=0U;CanRxMsg reply={0};PowerCommandStatus power;
    taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
    if(g_chassis_auto_active || !s_vm_requested || !power.active || !Auto_BaseHealthy() ||
       g_chassis_auto_cut_vm || (g_chassis_auto_result && g_chassis_auto_result!=1U))
    {taskEXIT_CRITICAL();return;}
    session=g_chassis_auto_session;now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    for(j=0U;j<4U;j++) {
        uint32_t age=now-s_odometry_samples.ms[j];
        if(!(s_odometry_samples.mask&(1UL<<j))) {i=j;oldest=400U;break;}
        if(age>oldest){oldest=age;i=j;}
    }
    if(oldest<40U){taskEXIT_CRITICAL();return;}
    taskEXIT_CRITICAL();
    result=CAN1_ReadPosition((uint8_t)(i+1U),&reply,&sign,&raw);
    taskENTER_CRITICAL();Power_ReadCommandStatus(&power);
    if(session==g_chassis_auto_session && !g_chassis_auto_active && s_vm_requested && power.active && Auto_BaseHealthy()) {
        Auto_OdometrySample(i,result,sign,raw,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
        if(result!=1U) {g_chassis_auto_cut_vm=1U;g_chassis_auto_cause=3U;}
    }
    taskEXIT_CRITICAL();
}

uint32_t ChassisAuto_ReadPositions(ChassisPositionSnapshot *snapshot)
{
    ChassisPositionSnapshot value = {0};
    uint32_t i;
    if (snapshot == NULL) return 0U;
    taskENTER_CRITICAL();
#if APP_POSITION_DIAGNOSTIC
    {
        TickType_t now = xTaskGetTickCount();
        PowerCommandStatus power;
        Power_ReadCommandStatus(&power);
        value = s_position;
        value.enabled = 1U; value.vm_active = power.active;
        value.valid_mask = 0U;
        for (i = 0U; i < 4U; i++)
        {
            value.sample_ms[i] = (uint32_t)s_position_tick[i] * portTICK_PERIOD_MS;
            value.age_ms[i] = 0xFFFFFFFFU;
            if ((s_position_valid & (1UL << i)) && s_vm_requested && power.active &&
                value.session == g_chassis_auto_session && !g_chassis_auto_cut_vm &&
                !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched)
            {
                value.age_ms[i] = (uint32_t)(TickType_t)(now - s_position_tick[i]) * portTICK_PERIOD_MS;
                if (value.age_ms[i] < CHASSIS_AUTO_FRESH_MS) value.valid_mask |= 1UL << i;
            }
        }
    }
#else
    for (i = 0U; i < 4U; i++) value.age_ms[i] = 0xFFFFFFFFU;
#endif
    taskEXIT_CRITICAL();
    *snapshot = value;
    return 1U;
}

static void Auto_Monitor(uint32_t session)
{
    uint32_t cursor = s_monitor_cursor, i = cursor / 2U, result;
    uint32_t is_speed = cursor & 1U;
    float rpm = 0;
    CanRxMsg reply = {0};
#if APP_POSITION_DIAGNOSTIC
    if (Auto_PositionMonitor(session)) return;
#endif
    if (is_speed) result = CAN1_ReadEmmSystemSnapshot((uint8_t)(i + 1U), &reply, &rpm);
    else result = CAN1_ReadStatus((uint8_t)(i + 1U), &reply);
    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active)
    {
        g_chassis_auto_last_address = i + 1U; g_chassis_auto_io_result = result;
        if (result != 1U)
        {
            if (is_speed) s_speed_valid &= ~(1UL << i);
            else { s_status_valid &= ~(1UL << i); g_can_poll_result[i] = result; }
            Auto_CutDetailLocked(3U,6U,result,i+1U);
        }
        else if (is_speed)
        {
            g_chassis_auto_driver_rpm[i] = rpm;
            s_speed_tick[i] = xTaskGetTickCount(); s_speed_valid |= 1UL << i;
            /* Validate each fresh sample immediately, including while a batch
             * is being sent; RUN-only polling could miss a short reversal. */
            Auto_CheckWheelSpeedLocked(i,s_speed_tick[i]);
            if(!g_chassis_auto_active) { taskEXIT_CRITICAL();return; }
#if APP_POSITION_DIAGNOSTIC
            s_position_due = i + 1U;
#endif
        }
        else
        {
            s_status[i] = reply.Data[1]; s_status_tick[i] = xTaskGetTickCount();
            s_status_valid |= 1UL << i;
            g_can_poll_result[i] = 1U; g_can_poll_flags[i] = reply.Data[1];
            g_can_poll_tick[i] = s_status_tick[i]; g_can_poll_count[i]++;
            if ((reply.Data[1] & 0x0CU) != 0U) Auto_CutLocked(2U);
        }
        s_monitor_cursor = (cursor + 1U) % 8U;
    }
    taskEXIT_CRITICAL();
}

/* 仅CAN任务调用：一轮至多一笔有界交易，不在临界区内等待 */
void ChassisAuto_CanStep(void)
{
    uint32_t state, session, index, result, expected, i;
    float effective_heading;
    TickType_t now;
    CanRxMsg reply = {0};
    taskENTER_CRITICAL();
    now = xTaskGetTickCount(); Auto_CheckLocked(now);
    if (!g_chassis_auto_active) { taskEXIT_CRITICAL(); return; }
    state = g_chassis_auto_state; session = g_chassis_auto_session;
    effective_heading = s_heading_pending_rpm;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (s_profile_active && !Chassis_ProfileCorrectionRpm(s_translation_base_rpm,
            s_heading_pending_rpm,&effective_heading))
    { Auto_CutLocked(8U); taskEXIT_CRITICAL(); return; }
#endif
/* 仅在整组边界接纳新目标，避免一组四轮混入两个不同轮速。 */
    if ((s_heading_session == 1U) && (s_stop_due == 0U) &&
        (s_finish_requested == 0U) &&
        ((state == CHASSIS_AUTO_RUN) ||
         ((state == CHASSIS_AUTO_START) && (s_motor_index == 0U) &&
          (s_monitor_between == 0U))) &&
        ((effective_heading != s_heading_applied_rpm) ||
         (s_distance_mode && s_translation_base_rpm != s_translation_applied_base)))
    {
        if (((s_forward_hold == 1U) ?
#if APP_DISTANCE_SCURVE_ACTIVE
            (s_profile_active ? ((s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE) ?
                Chassis_LeftTrimCommands(s_translation_base_rpm,effective_heading,s_commands[2]) :
                Chassis_TranslationProfileCommands(s_translation_mode,
                s_translation_base_rpm,effective_heading,s_commands[2])) :
#endif
            Chassis_TranslationHoldCommandsAtSpeed(s_translation_mode, s_translation_base_rpm, effective_heading, s_commands[2])
#if APP_DISTANCE_SCURVE_ACTIVE
            )
#endif
            : (s_turn_brake_trial ? Chassis_TurnBrakeCommands(effective_heading, s_commands[2]) :
                Chassis_HeadingCommands(effective_heading, s_commands[2]))) != 1U)
        {
            Auto_CutLocked(8U);
            taskEXIT_CRITICAL();
            return;
        }
        s_heading_applied_rpm = effective_heading;
        s_translation_applied_base = s_translation_base_rpm;
        s_heading_update_tick = now;
        g_chassis_auto_start_mask[2] = 0U;
        Auto_EnterLocked(CHASSIS_AUTO_START, now);
        state = CHASSIS_AUTO_START;
    }

    if ((state == CHASSIS_AUTO_START || state == CHASSIS_AUTO_RUN) && s_stop_due)
    { Auto_EnterLocked(CHASSIS_AUTO_STOP, now); state = CHASSIS_AUTO_STOP; }
    if ((state == CHASSIS_AUTO_DISABLE) || (state == CHASSIS_AUTO_ENABLE) ||
        (state == CHASSIS_AUTO_START))
    {
        if (s_monitor_between)
        { s_monitor_between = 0U; taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    }
    if ((state == CHASSIS_AUTO_VERIFY_OFF) || (state == CHASSIS_AUTO_VERIFY_ON))
    {
        expected = (state == CHASSIS_AUTO_VERIFY_ON) ? 1U : 0U;
        if (Auto_AllStatusSinceLocked(now) &&
            ((state == CHASSIS_AUTO_VERIFY_OFF) || Auto_AllSpeedSinceLocked(now)))
        {
            for (i = 0U; i < 4U; i++) if ((s_status[i] & 0x0DU) != expected) break;
            if (i != 4U) Auto_CutLocked(1U);
            else if ((state == CHASSIS_AUTO_VERIFY_ON) && !Auto_ZeroSpeedLocked())
                Auto_CutLocked(7U);
            else if (state == CHASSIS_AUTO_VERIFY_OFF) Auto_EnterLocked(CHASSIS_AUTO_ENABLE, now);
            else
            {
#if APP_POSITION_DIAGNOSTIC
                taskEXIT_CRITICAL();
                if (Auto_EndpointStep(session, 0U)) return;
                taskENTER_CRITICAL();
                now = xTaskGetTickCount(); Auto_CheckLocked(now);
                if (!g_chassis_auto_active || g_chassis_auto_session != session ||
                    g_chassis_auto_state != CHASSIS_AUTO_VERIFY_ON)
                { taskEXIT_CRITICAL(); return; }
#endif
#if APP_POSITION_DIAGNOSTIC
                if (s_distance_mode)
                {
                    if (s_endpoint_index >= s_endpoint_count || s_endpoints[s_endpoint_index].start_mask != 15U)
                    { Auto_CutDetailLocked(3U,7U,0U,0U); taskEXIT_CRITICAL(); return; }
                    for (i = 0U; i < 4U; i++)
                    {
                        s_distance_seen_ms[i] = s_endpoints[s_endpoint_index].start_ms[i];
                        s_distance_seen_raw[i] = s_endpoints[s_endpoint_index].start_raw[i];
                        s_distance_seen_sign[i] = s_endpoints[s_endpoint_index].start_sign[i];
                        s_position.sign[i] = s_distance_seen_sign[i];
                        s_position.raw[i] = s_distance_seen_raw[i]; s_position.result[i] = 1U;
                        s_position_tick[i] = (TickType_t)(s_distance_seen_ms[i] / portTICK_PERIOD_MS);
                    }
                    s_position_valid = 15U; s_position.sequence += 4U;
                }
#endif
                s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_SETTLE)
    {
        if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&
            ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS)))
        {
            if ((s_finish_requested != 0U) || (g_chassis_auto_segment == 2U))
            {
#if APP_POSITION_DIAGNOSTIC
                taskEXIT_CRITICAL();
                if (Auto_EndpointStep(session, 1U)) return;
                taskENTER_CRITICAL();
                now = xTaskGetTickCount(); Auto_CheckLocked(now);
                if (!g_chassis_auto_active || g_chassis_auto_session != session ||
                    g_chassis_auto_state != CHASSIS_AUTO_SETTLE)
                { taskEXIT_CRITICAL(); return; }
                if (s_endpoint_index < s_endpoint_count && s_heading_session)
                    s_endpoints[s_endpoint_index].outcome = 1U;
#endif
                g_chassis_auto_state = CHASSIS_AUTO_DONE;
                g_chassis_auto_result = 1U; g_chassis_auto_cause = 0U;
                g_chassis_auto_active = 0U;
                g_chassis_auto_cut_vm = AUTO_SEQUENCE_HOLD() ? 0U : 1U;
            }
            else
            {
                g_chassis_auto_segment++; s_run_started = s_stop_due = 0U;
                Auto_EnterLocked(CHASSIS_AUTO_START, now);
            }
            taskEXIT_CRITICAL(); return;
        }
        taskEXIT_CRITICAL(); Auto_Monitor(session); return;
    }
    if (state == CHASSIS_AUTO_RUN)
    { taskEXIT_CRITICAL(); Auto_Monitor(session); return; }
    index = s_motor_index;
    if (index >= 4U) { Auto_CutLocked(6U); taskEXIT_CRITICAL(); return; }
    if ((state == CHASSIS_AUTO_START) && !s_run_started)
    {
        s_run_tick = now;
#if APP_DISTANCE_SCURVE_ACTIVE
        s_profile_follow_tick = now;
#endif
        s_run_started = 1U;
        if (s_heading_session == 1U) s_heading_update_tick = now;
    }
    if (state == CHASSIS_AUTO_START) Auto_BatchSendLocked(index, xTaskGetTickCount());
    taskEXIT_CRITICAL();

    if (state == CHASSIS_AUTO_DISABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 0U, &reply);
    else if (state == CHASSIS_AUTO_ENABLE)
        result = CAN1_SetMotorEnable((uint8_t)(index + 1U), 1U, &reply);
    else if (state == CHASSIS_AUTO_START)
    {
        ChassisMotorCommand command = s_commands[g_chassis_auto_segment][index];
        result = (s_mode == 1U) ? 1U : CAN1_SetEmmSpeedRaw(command.address,
            command.direction, command.speed_raw, ZDT_EMM_MOTION_ACCELERATION, &reply); /* Emm加速度档位，外层轨迹保持。 */
    }
    else if (state == CHASSIS_AUTO_STOP)
        result = (s_mode == 1U) ? 1U : CAN1_StopMotor((uint8_t)(index + 1U), &reply);
    else return;

    taskENTER_CRITICAL();
    if ((session == g_chassis_auto_session) && g_chassis_auto_active &&
        (g_chassis_auto_state == state))
    {
        g_chassis_auto_last_address = index + 1U; g_chassis_auto_io_result = result;
        if (state == CHASSIS_AUTO_START)
            Auto_BatchReplyLocked(session, index, result, xTaskGetTickCount());
        if (result != 1U)
            Auto_CutLocked((state == CHASSIS_AUTO_START) ? 4U :
                           ((state == CHASSIS_AUTO_STOP) ? 5U : 1U));
        else
        {
            if (state == CHASSIS_AUTO_START) {
                ChassisMotorCommand command=s_commands[g_chassis_auto_segment][index];
                int32_t target=command.direction ? -(int32_t)command.speed_raw : (int32_t)command.speed_raw;
                DirectionGuard_Ack(&s_direction_guard[index],target,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                g_chassis_auto_start_mask[g_chassis_auto_segment] |= 1UL << index;
            }
            if (state == CHASSIS_AUTO_STOP) {
                DirectionGuard_Ack(&s_direction_guard[index],0,(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                g_chassis_auto_stop_mask[g_chassis_auto_segment] |= 1UL << index;
            }
            s_motor_index++;
            if (s_motor_index == 4U)
            {
#if APP_POSITION_DIAGNOSTIC
                if(s_route_trial && s_translation_mode==1U && APP_LEFT_TRIM_ACTIVE &&
                    (state==CHASSIS_AUTO_START || state==CHASSIS_AUTO_STOP))
                {
                    Auto_LeftTrimAdvance((uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);
                    s_left_trim_rpm=state==CHASSIS_AUTO_STOP ? 0U :
                        (uint32_t)((float)s_translation_applied_base*APP_LEFT_BACKWARD_RATIO+0.5f);
                }
#endif
                if (state == CHASSIS_AUTO_DISABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_OFF, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_ENABLE) Auto_EnterLocked(CHASSIS_AUTO_VERIFY_ON, xTaskGetTickCount());
                else if (state == CHASSIS_AUTO_START) Auto_EnterLocked(CHASSIS_AUTO_RUN, xTaskGetTickCount());
                else Auto_EnterLocked(CHASSIS_AUTO_SETTLE, xTaskGetTickCount());
            }
            else if (state != CHASSIS_AUTO_STOP) s_monitor_between = 1U;
        }
    }
    taskEXIT_CRITICAL();
}

/* 此处仅交接数据，不发CAN、不修改VM或原自动测试状态。 */


uint32_t ChassisAuto_HeadingPublish(float command_rpm, uint32_t stop_required)
{
    int32_t limit = s_turn_brake_trial ? (int32_t)APP_TURN_BRAKE_MAX_RPM : 5;
    uint32_t valid = ((command_rpm >= -limit) && (command_rpm <= limit) &&
        (stop_required <= 1U) &&
        ((stop_required == 0U) || (command_rpm == 0))) ? 1U : 0U;

    taskENTER_CRITICAL();
    /* 非法发布也覆盖旧指令，不能继续使用上一次非零轮速。 */
    s_heading_request.command_rpm = valid ? command_rpm : 0;
    s_heading_request.stop_required = valid ? stop_required : 1U;
    s_heading_request.sequence++;
    s_heading_request_tick = xTaskGetTickCount();
    s_heading_request_valid = valid;
    taskEXIT_CRITICAL();
    return valid;
}

uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *request)
{
    uint32_t valid;
    TickType_t now;
    if (request == NULL) return 0U;

    taskENTER_CRITICAL();
    now = xTaskGetTickCount();
    valid = s_heading_request_valid;
    if ((valid != 0U) &&
        ((TickType_t)(now - s_heading_request_tick) >=
         pdMS_TO_TICKS(CHASSIS_HEADING_MAILBOX_MAX_AGE_MS)))
    {
        s_heading_request_valid = 0U;
        valid = 0U;
    }
    *request = s_heading_request;
    if (valid == 0U)
    {
        request->command_rpm = 0;
        request->stop_required = 1U;
    }
    taskEXIT_CRITICAL();
    return valid;
}

/* 请求有序结束本次序列：只置标志，由唯一CAN任务逐台停车。 */
uint32_t ChassisAuto_RequestStop(void)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((g_chassis_auto_active == 1U) &&
        (g_chassis_auto_cut_vm == 0U) &&
        (g_chassis_auto_state >= CHASSIS_AUTO_START) &&
        (g_chassis_auto_state <= CHASSIS_AUTO_SETTLE))
    {
        /* 重复请求不能延长第一次请求的停车截止。 */
        if (s_finish_requested == 0U)
        {
            s_finish_requested = 1U;
            s_finish_tick = xTaskGetTickCount();
        }
        s_stop_due = 1U;
        accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
/* 单次旋转入口：只选旋转段，沿用原四轮准备和停车保护。 */
uint32_t ChassisAuto_PrepareHeading(uint32_t dry_run,float command_rpm,
    int32_t trial_sign,ChassisAutoPreparedHeading *p)
{
    uint32_t i;
    ChassisMotorCommand turn[CHASSIS_WHEEL_COUNT];
    if(!p)return 0U;
    p->valid=0U;
    if(dry_run>1U || (trial_sign ?
        (!APP_TURN_BRAKE_TRIAL_ACTIVE || command_rpm!=0 ||
         (trial_sign!=1 && trial_sign!=-1) || !Chassis_TurnBrakeCommands(command_rpm,turn)) :
        (command_rpm==0 || !Chassis_HeadingCommands(command_rpm,turn))))return 0U;
    if(!Auto_PrepareBasicCommands(p->commands))return 0U;
    for(i=0;i<CHASSIS_WHEEL_COUNT;i++)p->commands[2][i]=turn[i];
    p->dry_run=dry_run;p->command_rpm=command_rpm;p->trial_sign=trial_sign;
    p->valid=1U;return 1U;
}
uint32_t ChassisAuto_BeginPreparedHeading(const ChassisAutoPreparedHeading *p,
    uint32_t expected_session)
{
    uint32_t i,accepted=0U,magnitude;
    float logical;
    if(!p || p->valid!=1U || p->dry_run>1U ||
       (p->trial_sign ? (!APP_TURN_BRAKE_TRIAL_ACTIVE || p->command_rpm!=0 ||
        (p->trial_sign!=1 && p->trial_sign!=-1)) :
        (p->command_rpm==0 || !(p->command_rpm>=-5.0f && p->command_rpm<=5.0f))))return 0U;
    magnitude=(uint32_t)((p->command_rpm<0 ? -p->command_rpm : p->command_rpm)*10.0f+0.5f);
    /* Recheck the prepared rotation using integer comparisons only. */
    for(i=0;i<CHASSIS_WHEEL_COUNT;i++) {
        logical=(i==CHASSIS_LEFT_REAR || i==CHASSIS_LEFT_FRONT) ? -p->command_rpm : p->command_rpm;
        if(p->commands[2][i].address!=i+1U || p->commands[2][i].speed_raw!=magnitude ||
           p->commands[2][i].direction!=(g_chassis_forward_direction[i]^(logical<0 ? 1U : 0U)))return 0U;
    }
    taskENTER_CRITICAL();
    if(g_chassis_auto_session==expected_session && !g_chassis_auto_active)
        accepted=Auto_BeginPreparedBasicLocked(p->dry_run,&p->commands[0][0]);
    if(accepted) {
        g_chassis_auto_segment=2U;s_heading_session=1U;
        s_forward_hold=0U;s_route_trial=0U;
        s_distance_mode=s_distance_reached=s_distance_fault_reason=0U;s_translation_mode=0U;
        s_turn_brake_trial=p->trial_sign ? 1U : 0U;
        s_heading_initial_rpm=p->trial_sign ? p->trial_sign : p->command_rpm;
        s_heading_pending_rpm=s_heading_applied_rpm=p->command_rpm;
        s_heading_update_tick=xTaskGetTickCount();s_heading_command_tick=s_heading_update_tick;
    }
    taskEXIT_CRITICAL();return accepted;
}
static uint32_t Auto_BeginHeading(uint32_t dry_run,float command_rpm,int32_t trial_sign)
{
    ChassisAutoPreparedHeading p;uint32_t session;
    if(!ChassisAuto_PrepareHeading(dry_run,command_rpm,trial_sign,&p))return 0U;
    taskENTER_CRITICAL();session=g_chassis_auto_session;taskEXIT_CRITICAL();
    return ChassisAuto_BeginPreparedHeading(&p,session);
}
uint32_t ChassisAuto_BeginHeading(uint32_t dry_run, float command_rpm)
{ return Auto_BeginHeading(dry_run,command_rpm,0); }
uint32_t ChassisAuto_BeginTurnBrake(int32_t direction)
{ return Auto_BeginHeading(0U,0,direction); }
/* 仅提交下一组轮速；CAN任务在整组边界取用，不在这里发帧。 */
uint32_t ChassisAuto_UpdateHeading(float command_rpm)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if ((s_heading_session == 1U) && (g_chassis_auto_active == 1U) &&
        (g_chassis_auto_cut_vm == 0U) && (s_finish_requested == 0U) &&
        (s_stop_due == 0U) &&
        (g_chassis_auto_state >= CHASSIS_AUTO_DISABLE) &&
        (g_chassis_auto_state <= CHASSIS_AUTO_RUN))
    {
        /* 当前会话禁止反向：后续完成零速反向互锁后再扩展。 */
        if (!(command_rpm >= -(float)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U) &&
              command_rpm <= (float)(s_turn_brake_trial ? APP_TURN_BRAKE_MAX_RPM : 5U)) ||
            ((s_forward_hold == 1U) && ((command_rpm < -2) || (command_rpm > 2))) ||
            ((s_heading_initial_rpm > 0) && (command_rpm < 0)) ||
            ((s_heading_initial_rpm < 0) && (command_rpm > 0)))
            Auto_CutLocked(8U);
        else
        {
            if (command_rpm != s_heading_pending_rpm)
            {
#if APP_IS_DEBUG
                /* 邮箱同值时使用最近一次有效发布；直接API则记本次接受时刻。 */
                uint32_t from_mailbox = s_heading_request_valid &&
                    !s_heading_request.stop_required && s_heading_request.command_rpm == command_rpm &&
                    (TickType_t)(xTaskGetTickCount() - s_heading_request_tick) <
                        pdMS_TO_TICKS(CHASSIS_HEADING_MAILBOX_MAX_AGE_MS);
                Auto_BatchChangedLocked(from_mailbox ? s_heading_request_tick : xTaskGetTickCount(),
                    from_mailbox ? 1U : 2U);
#endif
            }
            s_heading_pending_rpm = command_rpm;
            s_heading_command_tick = xTaskGetTickCount();
            accepted = 1U;
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *status)
{
    if (status == 0) return 0U;
    taskENTER_CRITICAL();
    status->state = g_chassis_auto_state;
    status->active = g_chassis_auto_active;
    status->result = g_chassis_auto_result;
    status->cause = g_chassis_auto_cause;
    status->session = g_chassis_auto_session;
    status->segment = g_chassis_auto_segment;
    /* 目标航向使用旋转段2，禁止误取前进或横移的停止应答。 */
    status->rotation_stop_mask = g_chassis_auto_stop_mask[2];
    status->heading_mode = s_heading_session;
    status->forward_hold_mode = s_forward_hold;
    status->translation_mode = s_translation_mode;
    status->distance_mode = s_distance_mode;
    status->distance_fault_reason = s_distance_fault_reason;
    status->distance_reached = s_distance_reached;
    status->distance_base_rpm = s_translation_base_rpm;
    status->distance_target_mm = s_distance_target_mm;
    status->distance_progress_mm = s_distance_progress_mm;
    status->distance_profile_mode = status->distance_profile_done = 0U;
    status->distance_reference_mm = status->distance_reference_rpm = 0.0f;
#if APP_DISTANCE_SCURVE_ACTIVE
    status->distance_profile_mode = s_profile_active;
    if (s_profile_active)
    {
        status->distance_profile_done = s_profile_done;
        status->distance_reference_mm = s_distance_reference.position_mm;
        status->distance_reference_rpm = s_distance_reference.speed_mm_s/s_profile_mm_per_rpm_s;
    }
#endif
    status->dry_run = s_mode;
    status->cut_vm = g_chassis_auto_cut_vm;
    taskEXIT_CRITICAL();
    return 1U;
}

/* 只改原有执行器的命令向量；失能/使能/零速核验和停止保护共用。 */
/* 仅作局部数学计算，无CAN/VM副作用；调用者必须退出外层临界区。 */
static uint32_t Auto_PrepareTranslationEx(uint32_t motion_mode, uint32_t base_rpm,
    uint32_t run_ms, float distance_mm,float route_speed,ChassisAutoPreparedTranslation *prepared)
{
    if (!prepared) return 0U;
    prepared->valid = 0U;prepared->route_reference_mm_s=0.0f;
    if(route_speed!=0.0f && !MotionRoute_ShapeAllowed(motion_mode,base_rpm,run_ms,distance_mm,route_speed)) return 0U;
    if (__get_BASEPRI() || __get_PRIMASK() || motion_mode > 3U ||
        base_rpm < 3U || (base_rpm > 15U && route_speed==0.0f && !MotionTrial20Allowed(motion_mode,base_rpm,distance_mm))) return 0U;
    if (distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if ((!route_speed && (!MotionDistanceAllowed(motion_mode,base_rpm,distance_mm) ||
                run_ms != MotionDistanceRunMsFor(motion_mode,base_rpm,distance_mm))) ||
            !Chassis_DistanceSelfTest()) return 0U;
#else
        return 0U;
#endif
    }
    else if (!run_ms || run_ms > 10000U) return 0U;
    if (!Auto_PrepareBasicCommands(prepared->commands)) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (distance_mm>0.0f)
    { if (!Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U; }
    else
#endif
    if (!Chassis_TranslationHoldCommandsAtSpeed(motion_mode,base_rpm,0,prepared->commands[2])) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    prepared->profile.valid = 0U;
    if (distance_mm > 0.0f)
    {
        MotionProfileLimits limits;
        float factor;
        if (!Chassis_CalibrateDistanceMm(motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor)) return 0U;
        limits.speed = MotionDistanceReferenceSpeed(factor,base_rpm);
        limits.acceleration = APP_DISTANCE_SCURVE_ACCEL_MM_S2 -
            factor*2.0f*MotionDistanceFollowRpm(base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        limits.jerk = APP_DISTANCE_SCURVE_JERK_MM_S3 -
            factor*6.0f*MotionDistanceFollowRpm(base_rpm)*
                APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        if(route_speed!=0.0f) {
            uint32_t expected_rpm,expected_ms;
            if(!MotionRoute_Build(motion_mode,distance_mm,route_speed,&prepared->profile,&expected_rpm,&expected_ms) ||
                expected_rpm!=base_rpm || expected_ms!=run_ms) return 0U;
        } else if(!MotionProfile_Prepare(&prepared->profile,distance_mm,&limits)) return 0U;
        if (
            prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS >= (float)run_ms ||
            !Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U;
    }
#endif
    prepared->motion_mode = motion_mode; prepared->base_rpm = base_rpm;
    prepared->run_ms = run_ms; prepared->distance_mm = distance_mm;
    prepared->route_reference_mm_s=route_speed;
    prepared->valid = 1U;
    return 1U;
}
uint32_t ChassisAuto_PrepareTranslation(uint32_t mode,uint32_t rpm,uint32_t run_ms,
    float mm,ChassisAutoPreparedTranslation *prepared)
{ return Auto_PrepareTranslationEx(mode,rpm,run_ms,mm,0.0f,prepared); }
uint32_t ChassisAuto_PrepareRouteTranslation(uint32_t mode,uint32_t rpm,uint32_t run_ms,
    float mm,float speed,ChassisAutoPreparedTranslation *prepared)
{
    if(!(speed>0.0f)) { if(prepared) prepared->valid=0U;return 0U; }
    return Auto_PrepareTranslationEx(mode,rpm,run_ms,mm,speed,prepared);
}
uint32_t ChassisAuto_BeginPreparedTranslation(const ChassisAutoPreparedTranslation *prepared,
    uint32_t expected_session)
{
    uint32_t accepted = 0U;
    if(prepared && prepared->route_reference_mm_s!=0.0f &&
        !MotionRoute_ShapeAllowed(prepared->motion_mode,prepared->base_rpm,prepared->run_ms,
            prepared->distance_mm,prepared->route_reference_mm_s)) return 0U;
    if (!prepared || prepared->valid != 1U || prepared->motion_mode > 3U ||
        prepared->base_rpm < 3U || (prepared->base_rpm > 15U && prepared->route_reference_mm_s==0.0f &&
            !MotionTrial20Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
    if (prepared->distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if (!prepared->route_reference_mm_s &&
            (!MotionDistanceAllowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) ||
             prepared->run_ms != MotionDistanceRunMsFor(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
#else
        return 0U;
#endif
    }
    else if (!prepared->run_ms || prepared->run_ms > 10000U) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (prepared->distance_mm > 0.0f)
    {
        float factor;
        MotionProfileLimits route_limits;uint32_t route_rpm;
        if(prepared->route_reference_mm_s!=0.0f) {
            if(!MotionRoute_Limits(prepared->motion_mode,prepared->distance_mm,prepared->route_reference_mm_s,&route_limits,&route_rpm) ||
                prepared->base_rpm!=route_rpm || prepared->run_ms!=MotionRoute_Deadline(prepared->profile.duration_s) ||
                prepared->profile.limits.speed!=route_limits.speed ||
                prepared->profile.limits.acceleration!=route_limits.acceleration ||
                prepared->profile.limits.jerk!=route_limits.jerk) return 0U;
        }
        if (!Chassis_CalibrateDistanceMm(prepared->motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor) ||
            (!prepared->route_reference_mm_s && (prepared->profile.limits.speed != MotionDistanceReferenceSpeed(factor,prepared->base_rpm) ||
            prepared->profile.limits.acceleration != APP_DISTANCE_SCURVE_ACCEL_MM_S2 -
                factor*2.0f*MotionDistanceFollowRpm(prepared->base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA ||
            prepared->profile.limits.jerk != APP_DISTANCE_SCURVE_JERK_MM_S3 -
                factor*6.0f*MotionDistanceFollowRpm(prepared->base_rpm)*
                    APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA)) ||
            prepared->profile.boundary_s[0] != 0.0f || prepared->profile.position[0] != 0.0f ||
            prepared->profile.speed[0] != 0.0f || prepared->profile.acceleration[0] != 0.0f)
            return 0U;
    }
    if (prepared->distance_mm > 0.0f &&
        (prepared->profile.valid != 1U || prepared->profile.distance_mm != prepared->distance_mm ||
         !(prepared->profile.duration_s > 0.0f) ||
         !(prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS < (float)prepared->run_ms))) return 0U;
#endif
    taskENTER_CRITICAL();
    if (g_chassis_auto_session == expected_session && g_chassis_auto_state == CHASSIS_AUTO_IDLE &&
        !g_chassis_auto_active && !g_chassis_auto_cut_vm)
    {
#if APP_POSITION_DIAGNOSTIC
        if (prepared->distance_mm > 0.0f && s_endpoint_count >= CHASSIS_POSITION_ENDPOINT_CAPACITY)
        { taskEXIT_CRITICAL(); return 0U; }
#endif
        accepted = Auto_BeginPreparedBasicLocked(0U, &prepared->commands[0][0]);
        if (accepted)
        {
            g_chassis_auto_segment = 2U;
            s_heading_session = s_forward_hold = 1U;
            s_translation_mode = prepared->motion_mode;
            s_translation_base_rpm = s_translation_applied_base = prepared->base_rpm;
            s_translation_run_ms = prepared->run_ms;s_route_trial=prepared->route_reference_mm_s!=0.0f;
            s_heading_initial_rpm = s_heading_pending_rpm = s_heading_applied_rpm = 0;
            s_heading_update_tick = s_heading_command_tick = xTaskGetTickCount();
#if APP_POSITION_DIAGNOSTIC
            if (s_endpoint_index < s_endpoint_count)
                s_endpoints[s_endpoint_index].translation_mode = prepared->motion_mode;
            if (prepared->distance_mm > 0.0f)
            {
                s_distance_mode = 1U;
                s_distance_max_rpm = prepared->base_rpm;
                s_distance_target_mm = prepared->distance_mm;
                s_distance_progress_mm = 0.0f;
                s_endpoints[s_endpoint_index].distance_mode = 1U;
#if APP_DISTANCE_SCURVE_ACTIVE
                s_distance_profile = prepared->profile;
                s_profile_active = 1U; s_profile_done = 0U;
                (void)Chassis_CalibrateDistanceMm(prepared->motion_mode,
                    3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&s_profile_mm_per_rpm_s);
                s_distance_reference = (MotionProfileSample){0};
                s_profile_follow_rpm = s_profile_follow_rate = 0.0f; s_profile_follow_tick = 0U;
                s_translation_base_rpm = s_translation_applied_base = 0U;
                /* 初批必须是零速，避免错误准备向量绕过曲线起步。 */
                for (uint32_t wheel = 0U; wheel < CHASSIS_WHEEL_COUNT; wheel++)
                {
                    s_commands[2][wheel].address = (uint8_t)(wheel+1U);
                    s_commands[2][wheel].direction = g_chassis_forward_direction[wheel];
                    s_commands[2][wheel].speed_raw = 0U;
                }
#endif
            }
#endif
        }
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static uint32_t Auto_BeginTranslation(uint32_t dry_run, uint32_t motion_mode,
    uint32_t base_rpm, uint32_t run_ms, uint32_t max_run_ms)
{
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT] = {0};
    uint32_t i, accepted;
    if ((run_ms == 0U) || (run_ms > max_run_ms) ||
        !Chassis_TranslationHoldCommandsAtSpeed(motion_mode, base_rpm, 0, commands)) return 0U;
    taskENTER_CRITICAL();
    accepted = ChassisAuto_BeginHeading(dry_run, 1);
    if (accepted == 1U)
    {
        s_forward_hold = 1U;
        s_translation_mode = motion_mode;
#if APP_POSITION_DIAGNOSTIC
        if (s_endpoint_index < s_endpoint_count)
            s_endpoints[s_endpoint_index].translation_mode = motion_mode;
#endif
        s_translation_base_rpm = s_translation_applied_base = base_rpm;
        s_translation_run_ms = run_ms;
        s_heading_initial_rpm = 0;
        s_heading_pending_rpm = s_heading_applied_rpm = 0;
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++) s_commands[2][i] = commands[i];
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t ChassisAuto_BeginTranslationTimed(uint32_t dry_run, uint32_t motion_mode,
    uint32_t base_rpm, uint32_t run_ms)
{ return Auto_BeginTranslation(dry_run,motion_mode,base_rpm,run_ms,10000U); }
uint32_t ChassisAuto_BeginTranslationDistance(uint32_t motion_mode, uint32_t max_rpm, float target_mm)
{
#if APP_POSITION_DIAGNOSTIC
#if APP_DISTANCE_SCURVE_ACTIVE
    ChassisAutoPreparedTranslation prepared;
    uint32_t session;
    if (!ChassisAuto_PrepareTranslation(motion_mode,max_rpm,MotionDistanceRunMsFor(motion_mode,max_rpm,target_mm),target_mm,&prepared)) return 0U;
    taskENTER_CRITICAL(); session = g_chassis_auto_session; taskEXIT_CRITICAL();
    return ChassisAuto_BeginPreparedTranslation(&prepared,session);
#else
    uint32_t accepted;
    if (!MotionDistanceAllowed(motion_mode,max_rpm,target_mm) ||
        !Chassis_DistanceSelfTest()) return 0U;
    taskENTER_CRITICAL();
    if (s_endpoint_count >= CHASSIS_POSITION_ENDPOINT_CAPACITY)
    { taskEXIT_CRITICAL(); return 0U; }
    accepted = Auto_BeginTranslation(0U,motion_mode,max_rpm,MotionDistanceRunMs(target_mm),MotionDistanceRunMs(target_mm));
    if (accepted)
    {
        s_distance_mode = 1U; s_distance_reached = s_distance_fault_reason = 0U; s_distance_max_rpm = max_rpm;
        s_distance_target_mm = target_mm; s_distance_progress_mm = 0.0f;
        s_endpoints[s_endpoint_index].distance_mode = 1U;
    }
    taskEXIT_CRITICAL(); return accepted;
#endif
#else
    (void)motion_mode; (void)max_rpm; (void)target_mm; return 0U;
#endif
}

uint32_t ChassisAuto_BeginForwardDistance(uint32_t max_rpm, float target_mm)
{ return ChassisAuto_BeginTranslationDistance(0U,max_rpm,target_mm); }

uint32_t ChassisAuto_BeginTranslationHold(uint32_t dry_run, uint32_t motion_mode)
{
    return ChassisAuto_BeginTranslationTimed(dry_run, motion_mode,
        (motion_mode == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM,
        (motion_mode == 0U) ? APP_FORWARD_HOLD_RUN_MS : APP_LATERAL_HOLD_RUN_MS);
}

uint32_t ChassisAuto_UpdateForwardHold(float correction_rpm)
{
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    if (s_forward_hold == 1U) accepted = ChassisAuto_UpdateHeading(correction_rpm);
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t ChassisAuto_BeginForwardHold(uint32_t dry_run)
{
    return ChassisAuto_BeginTranslationHold(dry_run, 0U);
}

/* 纯复制观察快照，不查询CAN、不改状态/采样时间，也不清故障。 */
uint32_t ChassisAuto_ReadWheelLog(ChassisWheelLog *snapshot)
{
    uint32_t i;TickType_t now;
    if (!snapshot) return 0U;
    taskENTER_CRITICAL();now=xTaskGetTickCount();
    snapshot->tick_ms=(uint32_t)now*portTICK_PERIOD_MS;
    snapshot->session=g_chassis_auto_session;snapshot->active=g_chassis_auto_active;
    snapshot->state=g_chassis_auto_state;snapshot->dry_run=s_mode;
    snapshot->speed_valid_mask=snapshot->ack_valid_mask=0U;
    for(i=0U;i<4U;i++) {
        snapshot->acknowledged_decirpm[i]=s_direction_guard[i].target_deci;
        snapshot->measured_decirpm[i]=ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[i]);
        snapshot->ack_ms[i]=s_direction_guard[i].ack_ms;
        snapshot->sample_ms[i]=(uint32_t)s_speed_tick[i]*portTICK_PERIOD_MS;
        snapshot->age_ms[i]=(uint32_t)(now-s_speed_tick[i])*portTICK_PERIOD_MS;
        if (!s_mode && s_direction_guard[i].valid) snapshot->ack_valid_mask|=1UL<<i;
        if (!s_mode && g_chassis_auto_active && (s_speed_valid&(1UL<<i)) &&
            (TickType_t)(now-s_speed_tick[i])<pdMS_TO_TICKS(CHASSIS_AUTO_FRESH_MS))
            snapshot->speed_valid_mask|=1UL<<i;
    }
    taskEXIT_CRITICAL();return 1U;
}
