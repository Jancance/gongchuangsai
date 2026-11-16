#define VEHICLE_PARAMETER_TABLES
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "led.h"
#include "Buzzer.h"
#include "app_config.h"
#include "startup_pose.h"
#include "startup_axes.h"
#include "vehicle_arm.h"
#include "vehicle_mission.h"
#include "route_plan.h"
extern volatile RouteCheck g_route_check;
#include "dm_can2.h"
#include "radar_servo.h"
#include "radar_lift.h"
#include "radar_zone_switch.h"
#include "fine_rpm_telemetry.h"
#include "app_entry.h"
#include "usart.h"
#include "Key.h"
#include "power.h"
#include "can.h"
#include "chassis_kinematics.h"
#include "chassis_auto_test.h"
#include "app_odometry.h"
#include "app_can_diagnostics.h"
#include "app_heading_runtime.h"
#include "app_translation_runtime.h"
#include "app_motion_internal.h"
#include "hwt101.h"
#include "heading_source.h"
#include "safety.h"
#include "watchdog.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>
#include <string.h>
#include "heading_controller.h"
#include "heading_session.h"
#include "route_heading_reference.h"
#include "motion_action_plan.h"
#include "motion_control.h"
#include "runtime_route.h"
#include "motion_distance_limits.h"
#include "vehicle_log_config.h"
#include "field_map.h"
#include "app_motion_runtime.h"
#include "app_runtime_route.h"
#include "app_serial_internal.h"
extern volatile uint32_t g_vm_test_request,g_stage1_stop_requested,g_stage1_motion_skipped,g_scan_display_result,g_scan_display_count;
#if APP_HEADING_SESSION_TEST
#if APP_TEST_MODE == 3U
static uint32_t s_combined_keep_vm, s_combined_powered_segment;
static uint32_t s_combined_base_rpm, s_combined_run_ms;
static float s_combined_reference_mm_s;
static float s_combined_distance_mm;
static uint32_t s_motion_session_open, s_motion_has_action, s_motion_list_submit;
static uint32_t s_motion_turn_max_rpm = 5U;
#define App_StageBaseRpm() (s_combined_base_rpm)
#define App_StageRunMs() (s_combined_run_ms)
#define App_StageDistanceMm() (s_combined_distance_mm)
#define App_StageDoneVm() (s_combined_keep_vm)
#define App_StageStartVm() (s_combined_powered_segment)
#else
#define App_StageBaseRpm() ((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM)
#define App_StageRunMs() ((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_RUN_MS : APP_LATERAL_HOLD_RUN_MS)
#define App_StageDistanceMm() (0.0f)
#define App_StageDoneVm() (0U)
#define App_StageStartVm() (0U)
#endif
static RouteHeadingReference s_route_heading_reference;
static float s_route_heading_target;
static uint32_t s_route_heading_turn_active;
volatile float g_route_heading_targets[8], g_route_heading_start_yaw[8];
#if APP_TEST_MODE == 3U
static const AppHeadingRuntimeInputs s_heading_runtime_inputs = {
    &s_combined_keep_vm, &s_combined_powered_segment, &s_motion_turn_max_rpm,
    &s_route_heading_turn_active, &s_route_heading_target
};
#else
static const uint32_t s_heading_no_stage_vm = 0U;
static const uint32_t s_heading_default_max_rpm = 5U;
static const AppHeadingRuntimeInputs s_heading_runtime_inputs = {
    &s_heading_no_stage_vm, &s_heading_no_stage_vm, &s_heading_default_max_rpm,
    &s_route_heading_turn_active, &s_route_heading_target
};
#endif



/* 前进保持开发入口；request1一次开始，2取消，其余字段只观察。 */
#if APP_TEST_MODE == 3U
/* request 1启动整组一次，2取消；state 0空闲1运行2间隔3完成4失败5取消。 */
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count;

volatile uint32_t g_debug_plan_id = DEBUG_PLAN_DEFAULT;
#define MOTION_ACTION_COUNT (s_motion_action_count)

static float s_perimeter_targets[9][3]={
{2100.0f,2100.0f,-90.0f},
{2100.0f,300.0f,-90.0f},
{300.0f,300.0f,-90.0f},
{300.0f,300.0f,-180.0f},
{300.0f,300.0f,90.0f},
{300.0f,2100.0f,90.0f},
{300.0f,2100.0f,-180.0f},
{300.0f,2100.0f,-90.0f},
{2100.0f,2100.0f,-90.0f}
};

/* 动态预设仍受1..8项及组总时限校验；不是运行中可改路线。 */
volatile uint32_t g_motion_plan_valid = 0U;
volatile CombinedTestDebug g_combined_test = {0};
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
static AppOdometryInputs s_odometry_inputs = {
    &g_vm_test_request, NULL,
    &g_combined_test.state, &g_combined_test.completed,
    &s_route_heading_reference, NULL
};
#endif
static uint32_t s_combined_turn, s_combined_mode;
static uint32_t s_combined_begin_ms, s_combined_stage_ms, s_combined_gap_ms;
static volatile uint32_t s_combined_end_ms; /* 终态时刻保留供本地诊断。 */
#define App_TestTranslationMode() (s_combined_mode)
#else
#define App_TestTranslationMode() (APP_TRANSLATION_HOLD_MODE)
#endif
#if APP_TEST_MODE == 3U
static const AppTranslationRuntimeInputs s_translation_runtime_inputs = {
    &s_combined_mode, &s_combined_keep_vm, &s_combined_powered_segment,
    &s_combined_base_rpm, &s_combined_run_ms, &s_combined_distance_mm,
    &s_combined_reference_mm_s, &s_route_heading_target, &s_route_heading_reference
};
#endif
static uint32_t s_serial_start_permit; /* 仅调试串口可授予一次启动许可。 */
/* 首个平移trip的38字段快照；取消/后续STOP均不覆盖，HF2仅供诊断。 */
#define MOTION_FAULT_DIAG_FIELDS 43U
volatile uint32_t g_motion_fault_diag[MOTION_FAULT_DIAG_FIELDS];
volatile uint32_t g_motion_fault_diag_valid;
static uint32_t s_motion_fault_timing[HWT_TIMING_FIELDS];
#if APP_SERIAL_MOTION_CONTROL
/* 成功段计时先留存，整组正常结束且VM关闭后再发送，不增加运行期串口流量。 */
#define MOTION_TIMING_RECORD_FIELDS (HWT_TIMING_FIELDS + 3U)
#define MOTION_TIMING_RECORD_CAPACITY 12U
static uint32_t s_motion_timing_records[MOTION_TIMING_RECORD_CAPACITY][MOTION_TIMING_RECORD_FIELDS];
static uint32_t s_motion_timing_count, s_motion_timing_cursor;
static void App_TimingRecordCompleted(uint32_t index, uint32_t session, uint32_t now)
{
    uint32_t *record;
    if (s_motion_timing_count >= MOTION_TIMING_RECORD_CAPACITY) return;
    record = s_motion_timing_records[s_motion_timing_count];
    record[0] = 1U; record[1] = index; record[2] = session;
    HWT101_ReadTiming(record + 3U, now);
    s_motion_timing_count++;
}
#endif

void AppMotion_InternalFaultCapture(uint32_t cause)
{
    AppHeadingSource heading;
    HwtFaultDiagnostic hwt;
    ChassisAutoFaultDiagnostic backend;
    ChassisAutoStatus status;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if(g_motion_fault_diag_valid)return;
    HWT101_ReadDiagnostic(&hwt); (void)ChassisAuto_ReadStatus(&status);
    (void)App_HeadingSourceRead(&heading);
    g_motion_fault_diag[0] = now;
    g_motion_fault_diag[1] = hwt.arm_sequence;
    g_motion_fault_diag[2] = cause;
    g_motion_fault_diag[3] = (g_heading_exec_phase==HR_WAIT || g_heading_exec_phase==HR_RUN) ?
        g_heading_exec_phase : g_forward_hold.phase;
    g_motion_fault_diag[4] = status.state;
    g_motion_fault_diag[5] = status.cause;
    g_motion_fault_diag[6] = status.session;
    g_motion_fault_diag[7] = App_HeadingRuntimeSource(now);
    g_motion_fault_diag[8] = App_HeadingFrameAge(&heading,now,1U);
    g_motion_fault_diag[9] = App_HeadingFrameAge(&heading,now,2U);
    g_motion_fault_diag[10] = g_hwt_uart_errors;
    g_motion_fault_diag[11] = g_hwt_overflows;
    g_motion_fault_diag[12] = g_hwt_checksum_errors;
    g_motion_fault_diag[13] = hwt.base_uart_errors;
    g_motion_fault_diag[14] = hwt.base_overflows;
    g_motion_fault_diag[15] = hwt.base_checksum_errors;
    g_motion_fault_diag[16] = hwt.uart_ore;
    g_motion_fault_diag[17] = hwt.uart_fe;
    g_motion_fault_diag[18] = hwt.uart_ne;
    g_motion_fault_diag[19] = hwt.uart_pe;
    g_motion_fault_diag[20] = hwt.last_uart_sr;
    g_motion_fault_diag[21] = hwt.event_tick;
    g_motion_fault_diag[22] = hwt.event_reason;
    g_motion_fault_diag[23] = hwt.event_sr;
    g_motion_fault_diag[24] = hwt.event_uart_errors;
    g_motion_fault_diag[25] = hwt.event_overflows;
    g_motion_fault_diag[26] = hwt.event_checksum_errors;
    g_motion_fault_diag[27] = hwt.event_seen;
    g_motion_fault_diag[28] = now - heading.sample_ms;
    ChassisAuto_ReadFaultDiagnostic(&backend);
    g_motion_fault_diag[29] = backend.reason;
    g_motion_fault_diag[30] = backend.state;
    g_motion_fault_diag[31] = backend.io_result;
    g_motion_fault_diag[32] = backend.address;
    g_motion_fault_diag[33] = backend.command_age_ms;
    g_motion_fault_diag[34] = backend.status_mask;
    g_motion_fault_diag[35] = backend.speed_mask;
    g_motion_fault_diag[36] = backend.feedback_age_ms;
    g_motion_fault_diag[37] = backend.tick_ms;
    g_motion_fault_diag[38] = (uint32_t)backend.target_decirpm;
    g_motion_fault_diag[39] = (uint32_t)backend.measured_decirpm;
    g_motion_fault_diag[40] = backend.target_ack_ms;
    g_motion_fault_diag[41] = backend.speed_sample_ms;
    g_motion_fault_diag[42] = backend.reverse_samples;
    HWT101_ReadTiming(s_motion_fault_timing, now);
    g_motion_fault_diag_valid = 1U;
}
#if APP_TEST_MODE == 3U
/* 独立组合测试，唯一CAN任务每轮调用一次，不阻塞、不增加任务。
 * 固定顺序：前进8秒、左移8秒、相对转向+90度、前进8秒。
 * 段间停车核验后保持VM，额外间隔由统一参数配置；仅整组完成或异常断VM。 */
/* 只校验编译期表；不发硬件命令，不清故障。总时限保持120秒。 */
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)
    {
        const MotionAction *action = &s_motion_actions[i];
        if(action->reference_mm_s!=0.0f) {
            if(g_debug_plan_id!=DEBUG_PLAN_DEFAULT || !APP_ROUTE_SPEED_TRIAL_ACTIVE ||
                action->kind<MOTION_FORWARD_DISTANCE || action->kind>MOTION_BACKWARD_DISTANCE ||
                action->relative_deg!=0.0f || !MotionRoute_ShapeAllowed(action->kind-MOTION_FORWARD_DISTANCE,
                    action->base_rpm,action->run_ms,action->distance_mm,action->reference_mm_s)) return 0U;
            budget+=action->run_ms+6000U;continue;
        }
        if (action->kind <= MOTION_RIGHT)
        {
            if ((action->base_rpm < 3U) || (action->base_rpm > 15U) ||
                (action->run_ms == 0U) || (action->run_ms > 8000U) ||
                (action->relative_deg != 0.0f) || action->distance_mm != 0.0f) return 0U;
            budget += action->run_ms + 3500U;
        }
        else if (action->kind == MOTION_TURN)
        {
            if ((action->base_rpm > 5U && !(APP_TURN_BRAKE_TRIAL_ACTIVE &&
                (MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY) && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
                action->base_rpm==APP_TURN_BRAKE_MAX_RPM &&
                (action->relative_deg==90.0f || action->relative_deg==-90.0f))) || action->run_ms || action->distance_mm != 0.0f ||
                !(((action->relative_deg >= 5.0f) && (action->relative_deg <= 90.0f)) ||
                  ((action->relative_deg <= -5.0f) && (action->relative_deg >= -90.0f)))) return 0U;
            budget += APP_HEADING_RUN_MS + 3500U;
        }
        else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        {
            if (!APP_POSITION_DIAGNOSTIC || !Chassis_DistanceSelfTest() ||
                action->base_rpm < 3U || action->run_ms ||
                action->relative_deg != 0.0f ||
                !MotionDistanceAllowed(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)) return 0U;
            if (action->base_rpm>15U &&
                (MOTION_ACTION_COUNT!=1U || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&
                 !AppDebugPlan40Allowed(g_debug_plan_id,action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)))) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= APP_PERIMETER_GROUP_MS) ? 1U : 0U;
}
#if APP_IS_DEBUG
uint32_t App_DebugSelectPlan(uint32_t id)
{
    MotionAction actions[MOTION_ACTION_CAPACITY];
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t i,count;
    if (!AppDebugPlan_Load(id,actions,&count)) return 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (g_combined_test.state || g_combined_test.request || s_motion_session_open ||
        s_motion_has_action || g_forward_hold.phase != HR_IDLE ||
        g_heading_exec_phase != HR_IDLE || g_forward_hold.request || g_heading_exec_request ||
        g_vm_test_request || power.active || status.active || status.result || status.cut_vm)
    { taskEXIT_CRITICAL(); return 0U; }
    for(i=0U;i<count;i++) s_motion_actions[i]=actions[i];
    s_motion_action_count=count; g_debug_plan_id=id;
    g_motion_plan_valid=App_MotionPlanValid();
    taskEXIT_CRITICAL(); return g_motion_plan_valid;
}
#endif
/* 公共接口只提交动作，CAN任务继续执行原单段闭环；忙时绝不覆盖。 */
uint32_t MotionGetState(void)
{
    uint32_t phase, state;
    taskENTER_CRITICAL();
    phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
    if (!s_motion_has_action) state = MOTION_IDLE;
    else if ((s_combined_turn ? g_heading_exec_request : g_forward_hold.request) == 1U ||
             phase == HR_WAIT || phase == HR_RUN) state = MOTION_BUSY;
    else if (phase == HR_DONE) state = MOTION_DONE;
    else state = (phase == HR_CANCELLED) ? MOTION_CANCELLED : MOTION_FAULT;
    taskEXIT_CRITICAL();
    return state;
}
static uint32_t App_MotionAccept(void)
{
#if APP_FINE_RPM_OFFLINE_ONLY
    return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t state = MotionGetState();
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed()) return 0U;
    if ((state != MOTION_IDLE && state != MOTION_DONE) ||
        (g_combined_test.state >= 3U) ||
        ((g_combined_test.state != 0U) && !s_motion_list_submit) ||
        !App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount()) ||
        !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
        (g_translation_hold_selftest != 1U)) return 0U;
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (!s_motion_session_open)
    {
        if (s_motion_has_action || g_vm_test_request || (power.active && !StartupAxes_Holding()) ||
            !ChassisAuto_BeginSequence()) return 0U;
#if APP_SERIAL_MOTION_CONTROL
        s_motion_timing_count = s_motion_timing_cursor = 0U;
#endif
        s_motion_session_open = 1U;
        if (!s_motion_list_submit) s_combined_begin_ms = (uint32_t)xTaskGetTickCount();
        /* 在同一临界区发布首段VM请求，避免Safety把新组当作断电重置。 */
        g_vm_test_request = 1U;
        s_combined_powered_segment = 1U;
    }
    else
    {
        if (g_vm_test_request != 1U || power.active != 1U ||
            !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status,&s_translation_runtime_inputs)) ||
            !ChassisAuto_ReleaseCompleted(status.session)) return 0U;
        s_combined_powered_segment = 1U;
    }
    s_combined_keep_vm = 1U;
    s_combined_stage_ms = (uint32_t)xTaskGetTickCount();
    return 1U;
#endif
}
static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms, float distance_mm,float route_speed)
{
    ForwardHoldDebug empty = {0};
    uint32_t accepted = 0U;
    if(APP_BASELINE40_ACTIVE && !AppDebugPlan40Allowed(g_debug_plan_id,direction,rpm,distance_mm)) return 0U;
    if(route_speed!=0.0f) {
        if(!APP_POSITION_DIAGNOSTIC || !s_motion_list_submit || g_debug_plan_id!=DEBUG_PLAN_DEFAULT ||
            !MotionRoute_ShapeAllowed(direction,rpm,run_ms,distance_mm,route_speed) ||
            !Chassis_DistanceSelfTest()) return 0U;
    }
    else if (distance_mm!=0.0f)
    {
        if (!APP_POSITION_DIAGNOSTIC || !MotionDistanceAllowed(direction,rpm,distance_mm) ||
            run_ms!=MotionDistanceRunMsFor(direction,rpm,distance_mm) || !Chassis_DistanceSelfTest()) return 0U;
        if (rpm>15U && (!s_motion_list_submit || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&
            !AppDebugPlan40Allowed(g_debug_plan_id,direction,rpm,distance_mm)))) return 0U;
    }
    else if (direction > MOVE_RIGHT || rpm < MOTION_MOVE_MIN_RPM ||
        rpm > MOTION_MOVE_MAX_RPM || !run_ms || run_ms > MOTION_MOVE_MAX_MS) return 0U;
    taskENTER_CRITICAL();
    if (!g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())
    {
        s_combined_turn = 0U; s_combined_mode = direction;
        s_combined_base_rpm = rpm; s_combined_run_ms = run_ms; s_combined_distance_mm = distance_mm;
        s_combined_reference_mm_s=route_speed;
        empty.motion_mode = direction; g_forward_hold = empty;
        HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;
        s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ return App_MoveSubmit(direction,rpm,run_ms,0.0f,0.0f); }
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm)
{
    float mm=distance_cm*10.0f;
    return App_MoveSubmit(direction,max_rpm,MotionDistanceRunMsFor(direction,max_rpm,mm),mm,0.0f);
}
uint32_t MoveForwardDistanceStart(uint32_t max_rpm, float distance_cm)
{ return MoveDistanceStart(MOVE_FORWARD,max_rpm,distance_cm); }
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT==1U && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        AppHeadingRuntime_ResetSession();
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
static uint32_t App_Baseline40KeepVm(void);
uint32_t MotionFinish(void)
{
    ChassisAutoStatus status;
    uint32_t accepted = 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status);
    if (s_motion_session_open && MotionGetState() == MOTION_DONE && status.result == 1U &&
        !status.active && !status.cause && status.rotation_stop_mask == 15U &&
        ((g_combined_test.state == 0U) || s_motion_list_submit ||
         (g_combined_test.state==3U && App_Baseline40KeepVm() &&
          g_combined_test.index<MOTION_ACTION_COUNT &&
          status.session==g_combined_test.session[g_combined_test.index])))
    {
        s_combined_end_ms = (uint32_t)xTaskGetTickCount();
        s_motion_session_open = s_combined_keep_vm = 0U;
        g_vm_test_request = 0U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
/* 动作表仅安排顺序，同样使用公共单动作入口。 */
static uint32_t App_PerimeterTurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        AppHeadingRuntime_ResetSession();
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static uint32_t App_CombinedLoadAction(uint32_t index)
{
    AppHeadingSource heading;
    const MotionAction *action;
    uint32_t accepted, load_cycle, i;
    float planned_offset=0.0f;
    if (index >= MOTION_ACTION_COUNT || !MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY ||
        (g_debug_plan_id != DEBUG_PLAN_DEFAULT &&
         !(MOTION_ACTION_COUNT==1U && index==0U &&
           s_motion_actions[0].kind>=MOTION_FORWARD_DISTANCE && s_motion_actions[0].kind<=MOTION_BACKWARD_DISTANCE &&
           AppDebugPlan40Allowed(g_debug_plan_id,s_motion_actions[0].kind-MOTION_FORWARD_DISTANCE,
               s_motion_actions[0].base_rpm,s_motion_actions[0].distance_mm)))) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset=HeadingController_WrapDeg(planned_offset+s_motion_actions[i].relative_deg);
    if(!RouteHeading_Target(&s_route_heading_reference,planned_offset,&s_route_heading_target)) return 0U;
    action = &s_motion_actions[index];
    s_route_heading_turn_active = action->kind == MOTION_TURN;
    g_route_heading_targets[index]=s_route_heading_target;
    (void)App_HeadingSourceRead(&heading);
    g_route_heading_start_yaw[index]=heading.yaw_deg;
    s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
    else if (action->kind == MOTION_TURN)
        accepted = App_PerimeterTurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm ? action->base_rpm : 5U);
    else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        accepted = MoveDistanceStart(action->kind - MOTION_FORWARD_DISTANCE, action->base_rpm, action->distance_mm / 10.0f);
    else accepted = MoveStart(action->kind, action->base_rpm, action->run_ms);
    s_motion_list_submit = 0U;
    HWT101_TimingEnd(2U, load_cycle);
    return accepted;
}

void App_CombinedTestStop(uint32_t cause)
{
    /* STOP是幂等的；已完成/失败不覆盖首因，不清原单段trip。 */
    if (g_combined_test.state >= 3U)
    { g_vm_test_request = 0U; Power_Disable(); return; }
    s_motion_session_open = s_combined_keep_vm = 0U;
    s_combined_end_ms = (uint32_t)xTaskGetTickCount();
    g_combined_test.cause = cause;
    g_combined_test.state = (cause == 1U) ? 5U : 4U;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    if (s_combined_turn) App_HeadingRuntimeTrip(1U);
    else App_ForwardHoldTrip(1U);
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    Power_Disable();
}
/* 独立安全任务也检查整组取消与时限，防止CAN任务等待时漏掉取消。 */
void MotionStop(void)
{
    taskENTER_CRITICAL();
    App_CombinedTestStop(1U);
    taskEXIT_CRITICAL();
}

void App_CombinedTestSafety(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if ((g_combined_test.state == 1U) || (g_combined_test.state == 2U))
    {
        if (g_combined_test.request == 2U) App_CombinedTestStop(1U);
        else if ((uint32_t)(now - s_combined_begin_ms) >= APP_PERIMETER_GROUP_MS)
            App_CombinedTestStop(2U);
        else if (AppRuntimeRoute_Live()->committed.id && AppRuntimeRoute_Live()->consumed &&
                 g_combined_test.state==1U && g_combined_test.index<AppRuntimeRoute_Live()->committed.count &&
                 (uint32_t)(now-s_combined_stage_ms)>=AppRuntimeRoute_Live()->committed.rows[g_combined_test.index].timeout_ms)
            App_CombinedTestStop(2U);
        else if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
                 g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if ((g_combined_test.state == 2U) &&
                 ((g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
                  (g_can_guard_ready != 1U))) App_CombinedTestStop(3U);
        else if (g_combined_test.state == 1U)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety(&s_heading_runtime_inputs);
            else App_ForwardHoldSafety(&s_translation_runtime_inputs);
        }
    }
    else if (s_motion_session_open)
    {
        if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if (MotionGetState() == MOTION_BUSY)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety(&s_heading_runtime_inputs);
            else App_ForwardHoldSafety(&s_translation_runtime_inputs);
        }
        else if (g_vm_test_request != 1U || g_can_guard_ready != 1U)
            App_CombinedTestStop(3U);
    }
    else { g_vm_test_request = 0U; }
    taskEXIT_CRITICAL();
}
static uint32_t App_Baseline40KeepVm(void)
{
#if APP_BASELINE40_ACTIVE
    return (g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40) && s_motion_session_open &&
        s_combined_keep_vm && g_vm_test_request==1U;
#else
    return 0U;
#endif
}
void App_CombinedTestStep(void)
{
    AppHeadingSource heading;
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t now = (uint32_t)xTaskGetTickCount(), request, phase, index;
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    request = g_combined_test.request;
    g_combined_test.request = 0U;
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed())
    {
        s_serial_start_permit=0U;
        g_forward_hold.request=g_heading_exec_request=0U;
        g_chassis_auto_request=g_can_jog_request=0U;
        g_vm_test_request=0U;
        taskEXIT_CRITICAL(); return;
    }
    if (request == 2U && (g_combined_test.state < 3U ||
        (g_combined_test.state == 3U && App_Baseline40KeepVm())))
    { App_CombinedTestStop(1U); taskEXIT_CRITICAL(); return; }
    /* 启动请求尚未发给执行器时已取得码：撤销排队，不制造成功路程。 */
    if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
        g_combined_test.state==0U && request==1U && !s_motion_session_open)
    {
        s_serial_start_permit=0U;g_stage1_motion_skipped=1U;
        g_combined_test.state=3U;s_combined_end_ms=now;
        taskEXIT_CRITICAL();return;
    }
    if (g_combined_test.state == 0U && s_motion_session_open)
    {
        /* 独立API不启动动作表，仍由唯一CAN任务服务。 */
        if (MotionGetState() == MOTION_BUSY)
        {
            if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
            { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
            taskEXIT_CRITICAL();
            if (s_combined_turn) App_HeadingRuntimeStep(&s_heading_runtime_inputs);
            else App_ForwardHoldStep(&s_translation_runtime_inputs);
            return;
        }
        if (MotionGetState() == MOTION_FAULT || MotionGetState() == MOTION_CANCELLED)
            App_CombinedTestStop(5U);
        taskEXIT_CRITICAL(); return;
    }
    if (g_combined_test.state == 0U)
    {
        /* 其他单段Watch入口在本模式不参与调度。整组每次上电只运行一次。 */
        g_forward_hold.request = 0U; g_heading_exec_request = 0U;
        if (request != 1U) { taskEXIT_CRITICAL(); return; }
        if (s_serial_start_permit != 1U)
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
        (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
        if (status.active || status.result || status.cut_vm || (power.active && !StartupAxes_Holding()) ||
            g_vm_test_request || !App_HeadingRuntimeSource(now) ||
            !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
            (g_translation_hold_selftest != 1U) || (g_motion_plan_valid != 1U) ||
            (g_forward_hold.phase != HR_IDLE) || (g_heading_exec_phase != HR_IDLE))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        if(!RouteHeading_Init(&s_route_heading_reference,heading.yaw_deg))
        { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        s_combined_powered_segment = 0U;
        s_combined_begin_ms = s_combined_stage_ms = now;
        g_combined_test.index = 0U;
        if (!App_CombinedLoadAction(0U))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if ((g_combined_test.state < 3U) && (g_combined_test.index >= MOTION_ACTION_COUNT))
    { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state >= 3U)
    { g_forward_hold.request = g_heading_exec_request = 0U;
      if(g_combined_test.state!=3U || !App_Baseline40KeepVm()) g_vm_test_request=0U;
      taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state == 2U)
    {
        /* 段间取得码后直接释放已停稳会话，不再装载下一段。 */
        if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested)
        {
            s_motion_list_submit=1U;(void)MotionFinish();s_motion_list_submit=0U;
            Power_ReadCommandStatus(&power);
            if(!g_vm_test_request && (!power.active || StartupAxes_Holding()))
            {g_combined_test.state=3U;s_combined_end_ms=now;}
            taskEXIT_CRITICAL();return;
        }
        /* 保留上一段诊断，仅本组成功且仍上电的停稳会话允许释放。 */
        if ((VEHICLE_BETWEEN_LEGS_MS && (uint32_t)(now - s_combined_gap_ms) < VEHICLE_BETWEEN_LEGS_MS))
        { taskEXIT_CRITICAL(); return; }
        Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
        if ((g_vm_test_request != 1U) || (power.active != 1U) ||
            status.session != g_combined_test.session[g_combined_test.index])
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.index++;
        index = g_combined_test.index;
        s_combined_powered_segment = 1U;
        s_combined_stage_ms = now;
        if (!App_CombinedLoadAction(index))
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if (g_combined_test.state == 1U)
    {
        if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
        { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
        /* 单段执行可能等待CAN，必须先退出临界区；Safety仍可抢占取消。 */
        taskEXIT_CRITICAL();
        if (s_combined_turn) App_HeadingRuntimeStep(&s_heading_runtime_inputs);
        else App_ForwardHoldStep(&s_translation_runtime_inputs);
        taskENTER_CRITICAL();
        (void)App_HeadingSourceRead(&heading);
        if (g_combined_test.state >= 3U)
        { taskEXIT_CRITICAL(); return; }
        phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
        if (phase == HR_FAULT || phase == HR_CANCELLED)
        { App_CombinedTestStop(5U); taskEXIT_CRITICAL(); return; }
        /* 首段实际开始运行后，只展开PA1锁定的一套，不在上电或ARM时动作。 */
        if (!APP_BASELINE40_ACTIVE && g_combined_test.index==0U && phase==HR_RUN && !g_radar_servo_departure_done &&
            (s_combined_turn || g_forward_hold.run_started))
        {
            if (!RadarServo_OnDeparture(RadarZoneSwitch_Lock()))
            { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        }
        if (phase == HR_DONE)
        {
            (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
            /* 正常完成先核实四轮STOP，再按本趟供电策略收尾。 */
            if (status.active || status.state != CHASSIS_AUTO_DONE ||
                status.result != 1U || status.cause || status.rotation_stop_mask != 15U ||
                !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status,&s_translation_runtime_inputs)))
            { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
            if ((g_stage1_stop_requested || (g_combined_test.index + 1U) == MOTION_ACTION_COUNT) &&
                !App_Baseline40KeepVm())
            {
                s_motion_list_submit = 1U;
                (void)MotionFinish();
                s_motion_list_submit = 0U;
            }
            /* 保电完成须已实际通电；其余路径仍等待原关闭条件。 */
            if ((!g_stage1_stop_requested && (g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?
                ((g_vm_test_request == 1U) && (power.active == 1U) && !status.cut_vm) :
                (App_Baseline40KeepVm() ? (power.active==1U && !status.cut_vm) :
                 (!g_vm_test_request && (!power.active || StartupAxes_Holding()))))
            {
                index = g_combined_test.index;
                g_combined_test.phase[index] = phase;
                g_combined_test.session[index] = status.session;
                g_combined_test.elapsed_ms[index] = now - s_combined_stage_ms;
                g_combined_test.final_yaw[index] = heading.yaw_deg;
                g_combined_test.final_error[index] = s_combined_turn ?
                    g_heading_exec_error_deg : g_forward_hold.error_deg;
#if APP_SERIAL_MOTION_CONTROL
                if (!s_combined_turn) App_TimingRecordCompleted(index, status.session, now);
#endif
                if(!g_stage1_stop_requested || status.distance_reached) g_combined_test.completed++;
                s_combined_gap_ms = now;
                g_combined_test.state = (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;
                if (g_stage1_stop_requested || (index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;
            }
        }
    }
    taskEXIT_CRITICAL();
}
#endif
#if APP_SERIAL_MOTION_CONTROL
uint32_t App_RuntimeRouteInstall(const RuntimeRoutePrepared *p,uint32_t now)
{
    uint32_t i;ChassisAutoStatus status;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!App_RuntimeRouteEditable(now))return 0U;
    if(g_combined_test.state==3U) {
        (void)ChassisAuto_ReadStatus(&status);
        if(!ChassisAuto_RearmCompleted(status.session))return 0U;
        g_combined_test=combined;g_forward_hold=forward;
        g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
        AppHeadingRuntime_ResetSession();
        s_motion_has_action=s_combined_turn=s_combined_powered_segment=0U;
        s_combined_keep_vm=s_serial_start_permit=0U;
        AppSerial_ResetToken();
        g_serial_motion_state=g_serial_motion_result=0U;
    }
    for(i=0U;i<p->count;i++)s_motion_actions[i]=p->actions[i];
    s_motion_action_count=p->count;g_debug_plan_id=DEBUG_PLAN_DEFAULT;
    /* 转换器已校验类型、速度、距离、时限和总预算，此处只做有界复制。 */
    g_motion_plan_valid=1U;
    s_route_heading_reference=(RouteHeadingReference){0};
    s_route_heading_turn_active=0U;
    AppOdometry_ResetRecords();
    memcpy(s_perimeter_targets,p->local_targets,sizeof(s_perimeter_targets));
    AppRuntimeRoute_Install(p);
    return 1U;
}
/* 故障后每秒重发，便于另开只读监听；运行中不新增串口负载。 */
void App_MotionFaultTelemetry(void)
{
    static char frame[512];
    static uint32_t values[MOTION_FAULT_DIAG_FIELDS], last;
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    if (!g_motion_fault_diag_valid || (uint32_t)(now - last) < 1000U ||
        !App_SerialDiagnosticWindow(now, 60U)) return;
    taskENTER_CRITICAL();
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++) values[i] = g_motion_fault_diag[i];
    taskEXIT_CRITICAL();
    memcpy(frame, "HF3,", 4U);
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++)
    {
        count = snprintf(frame + used, sizeof(frame) - used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    if (App_DebugSendString(frame) == 1U) last = now;
}

/* 故障保留CT1；正常完成后发送带段号/会话的CT2，并定期重发成功历史。 */
void App_MotionTimingTelemetry(void)
{
    static char frame[384];
    static uint32_t values[MOTION_TIMING_RECORD_FIELDS], last;
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U, fields, normal;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    ChassisAutoStatus status;
    if (!App_SerialDiagnosticWindow(now, 70U)) return;
    /* 运动中快速返回，避免为了诊断反复进入临界区复制后端状态。 */
    if (!g_motion_fault_diag_valid)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state != 3U) return;
#else
        if (g_forward_hold.phase != HR_DONE) return;
#endif
    }
    taskENTER_CRITICAL();
    normal = 0U;
    if (g_motion_fault_diag_valid)
    {
        if ((uint32_t)(now-last) < 1000U) { taskEXIT_CRITICAL(); return; }
        fields = HWT_TIMING_FIELDS;
        for (i = 0U; i < fields; i++) values[i] = s_motion_fault_timing[i];
    }
    else
    {
        Power_ReadCommandStatus(&power);
        (void)ChassisAuto_ReadStatus(&status);
#if APP_TEST_MODE == 3U
        if (g_combined_test.state != 3U ||
            g_combined_test.completed != MOTION_ACTION_COUNT)
        { taskEXIT_CRITICAL(); return; }
#else
        if (g_forward_hold.phase != HR_DONE || g_forward_hold.trip)
        { taskEXIT_CRITICAL(); return; }
#endif
        if (g_vm_test_request || power.active || status.active || status.cause ||
            status.result != 1U || status.rotation_stop_mask != 15U)
        { taskEXIT_CRITICAL(); return; }
#if APP_TEST_MODE != 3U
        if (!s_motion_timing_count) App_TimingRecordCompleted(0U, status.session, now);
#endif
        if (!s_motion_timing_count) { taskEXIT_CRITICAL(); return; }
        if (s_motion_timing_cursor >= s_motion_timing_count)
        {
            if ((uint32_t)(now-last) < 1000U) { taskEXIT_CRITICAL(); return; }
            s_motion_timing_cursor = 0U;
        }
        else if ((uint32_t)(now-last) < 100U) { taskEXIT_CRITICAL(); return; }
        fields = MOTION_TIMING_RECORD_FIELDS; normal = 1U;
        for (i = 0U; i < fields; i++) values[i] = s_motion_timing_records[s_motion_timing_cursor][i];
    }
    taskEXIT_CRITICAL();
    memcpy(frame, normal ? "CT2," : "CT1,", 4U);
    for (i = 0U; i < fields; i++)
    {
        count = snprintf(frame+used, sizeof(frame)-used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame+used, sizeof(frame)-used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
    if (App_DebugSendString(frame) == 1U)
    {
        last = now;
        if (normal) s_motion_timing_cursor++;
    }
}
#if APP_POSITION_DIAGNOSTIC
#include "stage1_runtime.inc"
#endif
#endif
#endif
#if APP_TEST_MODE != 3U
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ (void)direction; (void)rpm; (void)run_ms; return 0U; }
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{ (void)direction; (void)angle_deg; (void)max_rpm; return 0U; }
uint32_t MoveDistanceStart(uint32_t direction, uint32_t max_rpm, float distance_cm)
{ (void)direction; (void)max_rpm; (void)distance_cm; return 0U; }
uint32_t MoveForwardDistanceStart(uint32_t max_rpm, float distance_cm)
{ (void)max_rpm; (void)distance_cm; return 0U; }
uint32_t MotionGetState(void) { return MOTION_UNAVAILABLE; }
void MotionStop(void) { g_vm_test_request = 0U; Power_Disable(); }
uint32_t MotionFinish(void) { return 0U; }
#endif
/* 上层读取同一时刻的动作/电源/故障快照，不等待总线。 */
uint32_t MotionReadStatus(MotionStatus *status)
{
    MotionStatus snapshot = {0};
    ChassisAutoStatus backend;
    PowerCommandStatus power;
    if (status == NULL) return 0U;
    taskENTER_CRITICAL();
    snapshot.state = MotionGetState();
#if APP_TEST_MODE == 3U
    snapshot.session_open = s_motion_session_open;
#else
    snapshot.session_open = 0U;
#endif
    (void)ChassisAuto_ReadStatus(&backend);
    Power_ReadCommandStatus(&power);
    snapshot.vm_requested = g_vm_test_request;
    snapshot.vm_output_active = power.active;
    snapshot.backend_active = backend.active;
    snapshot.backend_result = backend.result;
    snapshot.backend_cause = backend.cause;
    snapshot.stop_mask = backend.rotation_stop_mask;
    snapshot.can_fault = g_can_ctrl_fault_latched;
    snapshot.rtos_fault = g_rtos_fault;
    snapshot.safety_fault = g_safety_fault;
#if APP_TEST_MODE == 3U
    snapshot.distance_mode = (s_motion_has_action && !s_combined_turn &&
        s_combined_distance_mm > 0.0f) ? 1U : 0U;
    if (snapshot.distance_mode)
    {
        snapshot.distance_direction = s_combined_mode;
        snapshot.distance_target_cm = s_combined_distance_mm / 10.0f;
        /* 排队时底层可能仍有上一段结果，不能嫁接为当前进度。 */
        if (backend.distance_mode && App_ForwardHoldOwns(&backend,&s_translation_runtime_inputs))
        {
            snapshot.distance_progress_cm = backend.distance_progress_mm / 10.0f;
            snapshot.distance_reached = backend.distance_reached;
            snapshot.distance_base_rpm = backend.distance_base_rpm;
            snapshot.distance_fault_reason = backend.distance_fault_reason;
        }
    }
#endif
    taskEXIT_CRITICAL();
    *status = snapshot;
    return 1U;
}
float MotionRpmToCmPerSec(float rpm)
{
    if (!(rpm >= -65535.0f && rpm <= 65535.0f)) return 0.0f;
    return rpm * 3.14159265358979323846f * g_chassis_geometry.wheel_diameter_mm / 600.0f;
}
#if VEHICLE_INTEGRATED_ENABLE
#include "vehicle_mission.inc"
#endif
#if APP_HEADING_SESSION_TEST && APP_TEST_MODE == 3U
static const AppMotionObservation s_motion_observation = {
&s_motion_action_count, &s_motion_session_open, &s_motion_has_action, &s_combined_turn, &s_combined_mode, &s_combined_base_rpm, &s_combined_distance_mm, &s_combined_reference_mm_s, &s_combined_begin_ms, &s_combined_end_ms, &s_combined_stage_ms, s_motion_actions, (const float (*)[3])s_perimeter_targets
};
const AppMotionObservation *AppMotion_Live(void) { return &s_motion_observation; }
#endif
uint32_t AppMotion_SetStartPermit(uint32_t permit) { return s_serial_start_permit=permit; }
void AppMotion_LoadDefaultPlan(void)
{
#if APP_HEADING_SESSION_TEST && APP_FORWARD_HOLD_TEST && APP_TEST_MODE == 3U
    if (APP_IS_DEBUG)
    {
        g_debug_plan_id=APP_BASELINE40_ACTIVE ? DEBUG_PLAN_FORWARD210_40 : DEBUG_PLAN_DEFAULT;
        (void)AppDebugPlan_Load(g_debug_plan_id,s_motion_actions,&s_motion_action_count);
        g_motion_plan_valid = App_MotionPlanValid();
    }
#endif
}
void AppMotion_InitStage1Plan(void)
{
    if(!VEHICLE_INTEGRATED_ENABLE && VEHICLE_STAGE1_ENABLE && !FieldMap_Stage1(&s_stage1_plan))
        {g_stage1_state=4U;g_stage1_cause=10U;}
}
void AppMotion_InitOdometry(void)
{
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
    s_odometry_inputs.jog_preparing = AppCanDiagnostics_JogPreparing();
    s_odometry_inputs.runtime_route_id = &AppRuntimeRoute_Live()->committed.id;
    AppOdometry_Init();
#endif
}
void AppMotion_StepOdometry(void)
{
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
    AppOdometry_Step(&s_odometry_inputs);
#endif
}
void AppMotion_Stage1Service(void) { App_Stage1Service(); }
void AppMotion_Stage1Display(uint32_t ready) { App_Stage1Display(ready); }
void AppMotion_Stage1Telemetry(void) { App_Stage1Telemetry(); }
void AppMotion_Stage1CodeTelemetry(void) { App_Stage1CodeTelemetry(); }
#if VEHICLE_INTEGRATED_ENABLE
void AppMotion_MissionService(void) { VehicleMission_Service(); }
void AppMotion_MissionTelemetry(void) { VehicleMission_Telemetry(); }
#endif
