#include "app_heading_runtime.h"
#include "app_motion_internal.h"
#include "app_can_diagnostics.h"
#include "heading_source.h"
#include "freertos_hooks.h"
#include "hwt101.h"
#include "power.h"
#include "safety.h"

extern volatile uint32_t g_vm_test_request;
#if APP_HEADING_SESSION_TEST
volatile uint32_t g_heading_exec_request = 0U; /* 1开始一次，2取消 */
volatile float g_heading_exec_relative_deg = 30.0f;
volatile uint32_t g_heading_exec_phase = HR_IDLE;
volatile uint32_t g_heading_exec_trip = 0U;
volatile uint32_t g_heading_exec_state = HS_IDLE;
volatile uint32_t g_heading_exec_cause = 0U;
volatile uint32_t g_heading_exec_action = HS_ACTION_NONE;
volatile float g_heading_exec_rpm = 0.0f;
volatile float g_heading_exec_target_deg = 0.0f;
volatile float g_heading_exec_error_deg = 0.0f;
volatile uint32_t g_heading_exec_backend_session = 0U;
/* 0未检查、1等待PE2开启、2上电等待、3等待四轮新状态、4就绪。 */
volatile uint32_t g_heading_power_wait_state = 0U;
volatile uint32_t g_heading_power_on_age_ms = 0U;
volatile uint32_t g_heading_power_post_settle_mask = 0U;
/* 分阶段HWT快照；只记录，不改变运动、停车或VM控制。 */
volatile HeadingStopTrace g_heading_stop_trace = {0};
static HeadingSession s_heading_exec = {0};
/* CAN-task-only preparation buffers; keep large temporary objects off task stack. */
static HeadingSession s_heading_prepared;
static ChassisAutoPreparedHeading s_heading_backend_prepared;
extern volatile uint32_t g_motion_fault_diag_valid;
volatile uint32_t g_heading_prepare_cycles,g_heading_prepare_max_cycles;
static uint32_t s_heading_exec_wait_ms = 0U;
static float s_heading_exec_relative = 0.0f;

/* trip:1取消，2源失效/配置冲突，3平台/互斥，4等待超时，
 * 5执行器会话归属，6会话/提交失败，7运行总截止。首次原因锁存。 */
/* 首次失败快照：仅记录，不改变原有判定、截止或VM动作。 */
volatile uint32_t g_heading_fault_phase = 0U;
volatile uint32_t g_heading_fault_source_flags = 0U;
volatile uint32_t g_heading_fault_sample_age_ms = 0U;
volatile uint32_t g_heading_fault_angle_age_ms = 0U;
volatile uint32_t g_heading_fault_rate_age_ms = 0U;
volatile uint32_t g_heading_fault_hwt_update_age_ms = 0U;
volatile uint32_t g_heading_fault_tick_ms = 0U;


void App_HeadingRuntimeTrip(uint32_t cause)
{
    AppHeadingSource heading;
    uint32_t heading_ready;
    uint32_t now, flags;
    taskENTER_CRITICAL();
    heading_ready = App_HeadingSourceRead(&heading);
    if (g_heading_exec_trip == 0U)
    {
        now = (uint32_t)xTaskGetTickCount();
        AppMotion_InternalFaultCapture(cause);
        g_heading_fault_phase = g_heading_exec_phase;
        g_heading_fault_tick_ms = now;
        g_heading_fault_sample_age_ms = now - heading.sample_ms;
        g_heading_fault_angle_age_ms = App_HeadingFrameAge(&heading,now,1U);
        g_heading_fault_rate_age_ms = App_HeadingFrameAge(&heading,now,2U);
        g_heading_fault_hwt_update_age_ms = now - g_hwt_update_tick;
        flags = 0U;
        if (heading_ready != 1U) flags |= 0x001U;
        if (heading.healthy != 1U) flags |= 0x002U;
        if (g_heading_fault_sample_age_ms >= HS_SOURCE_MS) flags |= 0x004U;
        if (g_heading_fault_angle_age_ms >= HS_SOURCE_MS) flags |= 0x008U;
        if (g_heading_fault_rate_age_ms >= HS_SOURCE_MS) flags |= 0x010U;
        if (g_hwt_cal_active != 0U) flags |= 0x020U;
        if (g_hwt_cal_request != 0U) flags |= 0x040U;
        if (g_hwt_cal_end_tx_result == 2U) flags |= 0x080U;
        if (g_hwt_version_state != 0U) flags |= 0x100U;
        if (g_hwt_version_request != 0U) flags |= 0x200U;
        if (g_hwt_a7_request != 0U) flags |= 0x400U;
        g_heading_fault_source_flags = flags;
        g_heading_exec_trip = cause;
    }
    g_heading_exec_rpm = 0;
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    taskEXIT_CRITICAL();
}


uint32_t App_HeadingRuntimeExclusive(void)
{
    return ((g_chassis_auto_request == 0U) &&
        (g_can_jog_request == 0U) && ((*AppCanDiagnostics_JogPreparing()) == 0U) &&
        (g_wheel_test_active == 0U) && (g_wheel_test_cut_vm == 0U) &&
        (App_CanJogOtherRequest() == 0U) && (g_can_feedback_enable == 0U) &&
        (g_can_poll_enable == 1U) && (g_can_test_result == 1U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_motion_diag_fault_latched == 0U) &&
        (g_motion_auto_stop_state == MOTION_STOP_IDLE) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_motion_diag_hold_report == 0U) &&
        (g_motion_diag_hold_stop_service == 0U) &&
        (g_motion_diag_fail_stop_once == 0U)) ? 1U : 0U;
}

/* 由调用者在临界区内使用，独立复查真实帧年龄及配置互斥。 */
uint32_t App_HeadingRuntimeSource(uint32_t now)
{
    AppHeadingSource heading;
    uint32_t heading_ready;
    heading_ready = App_HeadingSourceRead(&heading);
    return ((heading_ready == 1U) && (heading.healthy == 1U) &&
        ((uint32_t)(now - heading.sample_ms) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&heading,now,1U)) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&heading,now,2U)) < HS_SOURCE_MS) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}

/* 仅在现有WAIT及临界区内检查；不阻塞、不请求VM、不发CAN。 */
uint32_t App_HeadingPowerReady(uint32_t now)
{
    PowerCommandStatus power;
    TickType_t on_age, sample_age, settle_ticks;
    uint32_t i, mask = 0U;
    Power_ReadCommandStatus(&power);
    g_heading_power_post_settle_mask = 0U;
    g_heading_power_on_age_ms = 0xFFFFFFFFU;
    if ((power.active != 1U) || (power.enable_seen != 1U))
    {
        g_heading_power_wait_state = 1U;
        return 0U;
    }
    on_age = (TickType_t)((TickType_t)now - (TickType_t)power.enable_tick);
    g_heading_power_on_age_ms = (uint32_t)(on_age * portTICK_PERIOD_MS);
    settle_ticks = pdMS_TO_TICKS(APP_HEADING_POWER_SETTLE_MS);
    if (on_age < settle_ticks)
    {
        g_heading_power_wait_state = 2U;
        return 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        sample_age = (TickType_t)((TickType_t)now - g_can_poll_tick[i]);
        /* 必须是本次开启并完成等待后的成功新帧，不接受等待前样本。 */
        if ((g_can_poll_result[i] == 1U) && (g_can_poll_count[i] != 0U) &&
            ((g_can_poll_flags[i] & 0x0CU) == 0U) &&
            (sample_age <= pdMS_TO_TICKS(CAN_POLL_VALID_MS)) &&
            (sample_age <= (TickType_t)(on_age - settle_ticks)))
            mask |= 1UL << i;
    }
    g_heading_power_post_settle_mask = mask;
    g_heading_power_wait_state = (mask == 0x0FU) ? 4U : 3U;
    return (mask == 0x0FU) ? 1U : 0U;
}

/* 调用者持有临界区；captured只表示事件发生，valid才表示源新鲜。 */
static void App_HeadingStopCapture(volatile HeadingStopAngleSample *sample,
    uint32_t now)
{
    AppHeadingSource heading;
    PowerCommandStatus power;
    if (sample->captured != 0U) return;
    (void)App_HeadingSourceRead(&heading);
    Power_ReadCommandStatus(&power);
    sample->valid = App_HeadingRuntimeSource(now);
    sample->event_ms = now;
    sample->angle_sample_ms = heading.angle_ms;
    sample->angle_age_ms = App_HeadingFrameAge(&heading,now,1U);
    sample->rate_age_ms = App_HeadingFrameAge(&heading,now,2U);
    sample->source_sequence = heading.sequence;
    sample->pe2_active = power.active;
    sample->yaw_deg = heading.yaw_deg;
    sample->error_deg = HeadingController_ErrorDeg(
        g_heading_stop_trace.target_deg, heading.yaw_deg);
    sample->rate_dps = heading.rate_dps;
    sample->captured = 1U;
}

/* 每轮观察包括终态；不等待、不延长VM供电，不发送任何动作。 */
static void App_HeadingStopObserve(uint32_t now)
{
    AppHeadingSource heading;
    PowerCommandStatus power;
    uint32_t off_age;
    if (g_heading_stop_trace.started == 0U) return;
    Power_ReadCommandStatus(&power);
    if ((g_heading_stop_trace.off_seen.captured == 0U) && (power.active == 0U))
    {
        g_heading_stop_trace.off_observed_ms = now;
        App_HeadingStopCapture(&g_heading_stop_trace.off_seen, now);
    }
    if ((g_heading_stop_trace.off_seen.captured == 0U) ||
        (g_heading_stop_trace.post_status != 0U)) return;
    off_age = now - g_heading_stop_trace.off_observed_ms;
    if (off_age >= 2000U)
    {
        g_heading_stop_trace.post_status = 2U;
        return;
    }
    (void)App_HeadingSourceRead(&heading);
    /* 至少一秒后采新角度帧，不能用关断前或缓存中的旧样本。 */
    if ((off_age >= 1000U) && (App_HeadingRuntimeSource(now) == 1U) &&
        ((uint32_t)(App_HeadingFrameAge(&heading,now,1U)) <= off_age - 1000U))
    {
        App_HeadingStopCapture(&g_heading_stop_trace.after_off, now);
        g_heading_stop_trace.post_status = 1U;
    }
}

uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *status)
{
    return ((status->session == g_heading_exec_backend_session) &&
        (status->heading_mode == 1U) && (status->segment == 2U) &&
        (status->forward_hold_mode == 0U) && (status->dry_run == 0U)) ? 1U : 0U;
}

/* Safety每20ms调用；不修改会话对象，不发CAN，不等待CAN任务。 */
void App_HeadingRuntimeSafety(const AppHeadingRuntimeInputs *inputs)
{
    ChassisAutoStatus status;
    uint32_t now, normal_done;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    (void)ChassisAuto_ReadStatus(&status);
    if ((g_heading_exec_phase != HR_WAIT) && (g_heading_exec_phase != HR_RUN))
    {
        if (status.active != 0U) App_HeadingRuntimeTrip(5U);
        g_vm_test_request = (g_heading_exec_phase == HR_DONE) ? (*inputs->keep_vm) : 0U;
    }
    else if (g_heading_exec_trip != 0U)
        App_HeadingRuntimeTrip(g_heading_exec_trip);
    else if (g_heading_exec_request == 2U) App_HeadingRuntimeTrip(1U);
    else if (!App_HeadingRuntimeSource(now)) App_HeadingRuntimeTrip(2U);
    else if (!App_HeadingRuntimeExclusive() || (g_vm_test_request != 1U))
        App_HeadingRuntimeTrip(3U);
    else if (g_heading_exec_phase == HR_WAIT)
    {
        if ((uint32_t)(now - s_heading_exec_wait_ms) >= 5000U)
            App_HeadingRuntimeTrip(4U);
        else if (status.active != 0U || status.result != 0U)
            App_HeadingRuntimeTrip(5U);
    }
    else
    {
        normal_done = ((g_heading_exec_state == HS_STOPPING) &&
            App_HeadingRuntimeOwns(&status) && (status.active == 0U) &&
            (status.result == 1U) && (status.rotation_stop_mask == 0x0FU)) ? 1U : 0U;
        if (!App_HeadingRuntimeOwns(&status)) App_HeadingRuntimeTrip(5U);
        else if ((uint32_t)(now - s_heading_exec_wait_ms) >=
                 (HS_RUN_MS + HS_STOP_MS)) App_HeadingRuntimeTrip(7U);
        else if (!normal_done && ((status.active != 1U) ||
                 (status.result != 0U) || (g_safety_ready != 1U) ||
                 (g_can_guard_ready != 1U))) App_HeadingRuntimeTrip(3U);
    }
    taskEXIT_CRITICAL();
}

/* 只由唯一CAN任务调用，每轮计算后最多执行原执行器的一步交易。 */
void App_HeadingRuntimeStep(const AppHeadingRuntimeInputs *inputs)
{
    AppHeadingSource heading;
    HeadingSessionInput input = {0};
    HeadingSessionOutput output = {0};
    ChassisAutoStatus status;
    uint32_t request, accepted = 0U, now, record_zero = 0U;
    uint32_t critical_cycle,prepare_cycle,prepared_ok,expected_session;
    uint32_t uart_base,overflow_base,checksum_base;

    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    critical_cycle=HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    App_HeadingStopObserve(now);
    request = g_heading_exec_request;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_heading_exec_phase >= HR_DONE)
    {
        /* 终态保持观察结果；新request不排队，上电后只运行一次。 */
        g_vm_test_request = (g_heading_exec_phase == HR_DONE) ? (*inputs->keep_vm) : 0U;
        HWT101_TimingEnd(0U,critical_cycle);
        taskEXIT_CRITICAL();
        return;
    }
    if ((request == 2U) &&
        ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN)))
        App_HeadingRuntimeTrip(1U);
    if ((request == 1U) && (g_heading_exec_phase == HR_IDLE))
    {
        s_heading_exec_relative = g_heading_exec_relative_deg;
        /* 本轮只接受正负5度或30度，不允许Watch任意扩大角度。 */
        if (((s_heading_exec_relative != 5.0f) && (s_heading_exec_relative != -5.0f) &&
             (s_heading_exec_relative != 30.0f) && (s_heading_exec_relative != -30.0f) &&
             !((APP_TEST_MODE == 3U) &&
               (((s_heading_exec_relative >= 5.0f) && (s_heading_exec_relative <= 90.0f)) ||
                ((s_heading_exec_relative <= -5.0f) && (s_heading_exec_relative >= -90.0f))))) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != (*inputs->powered_segment)) || (g_heading_exec_trip != 0U) ||
            (status.active != 0U) || (status.result != 0U) ||
            (status.cut_vm != 0U) ||
            !(heading.rate_dps >= -3.0f && heading.rate_dps <= 3.0f))
            App_HeadingRuntimeTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
#endif
            s_heading_exec_wait_ms = now;
            g_heading_exec_phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    if ((g_heading_exec_phase == HR_WAIT) && (g_heading_exec_trip == 0U))
    {
        if (!App_HeadingRuntimeSource(now)) App_HeadingRuntimeTrip(2U);
        else if (!App_HeadingRuntimeExclusive() || g_vm_test_request != 1U)
            App_HeadingRuntimeTrip(3U);
        else if ((uint32_t)(now - s_heading_exec_wait_ms) >= 5000U)
            App_HeadingRuntimeTrip(4U);
        else if ((App_CanJogReady() == 1U) && (App_HeadingPowerReady(now) == 1U))
        {
            input.now_ms = now;
#if APP_TEST_MODE == 3U
            input.max_rpm = (*inputs->turn_max_rpm);
#endif
            input.sample_ms = heading.sample_ms;
            input.sequence = heading.sequence;
            input.healthy = input.platform_ok = 1U;
            input.yaw_deg = heading.yaw_deg;
            input.rate_dps = heading.rate_dps;
            input.backend_active = status.active;
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            expected_session=status.session;
            uart_base=g_hwt_uart_errors;overflow_base=g_hwt_overflows;
            checksum_base=g_hwt_checksum_errors;
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();
            prepare_cycle=HWT101_TimingBegin();
            HeadingSession_Init(&s_heading_prepared);
            prepared_ok=(*inputs->route_turn_active) ?
                HeadingSession_StartReference(&s_heading_prepared,&input,
                    s_heading_exec_relative,(*inputs->route_target),&output) :
                HeadingSession_Start(&s_heading_prepared,&input,
                    s_heading_exec_relative,&output);
            if(prepared_ok)prepared_ok=ChassisAuto_PrepareHeading(0U,output.command_rpm,
                s_heading_prepared.turn_trial ? s_heading_prepared.initial_sign : 0,
                &s_heading_backend_prepared);
            g_heading_prepare_cycles=HWT101_TimingBegin()-prepare_cycle;
            if(g_heading_prepare_cycles>g_heading_prepare_max_cycles)
                g_heading_prepare_max_cycles=g_heading_prepare_cycles;
            taskENTER_CRITICAL();
            (void)App_HeadingSourceRead(&heading);
            critical_cycle=HWT101_TimingBegin();
            now=(uint32_t)xTaskGetTickCount();
            (void)ChassisAuto_ReadStatus(&status);
            /* Preparation never grants motion: recheck cancellation, source epoch and ownership. */
            if(g_heading_exec_request==2U)App_HeadingRuntimeTrip(1U);
            else if(g_heading_exec_trip!=0U) { /* Keep the first safety failure. */ }
            else if(!App_HeadingRuntimeSource(now) || now-input.sample_ms>=HS_SOURCE_MS ||
                uart_base!=g_hwt_uart_errors || overflow_base!=g_hwt_overflows ||
                checksum_base!=g_hwt_checksum_errors)App_HeadingRuntimeTrip(2U);
            else if(g_heading_exec_phase!=HR_WAIT || !App_HeadingRuntimeExclusive() ||
                g_vm_test_request!=1U || !App_CanJogReady() || !App_HeadingPowerReady(now))
                App_HeadingRuntimeTrip(3U);
            else if(now-s_heading_exec_wait_ms>=5000U)App_HeadingRuntimeTrip(4U);
            else if(status.session!=expected_session || status.active || status.result || status.cut_vm)
                App_HeadingRuntimeTrip(5U);
            else if(!prepared_ok)App_HeadingRuntimeTrip(6U);
            else
            {
                s_heading_exec=s_heading_prepared;
                prepare_cycle=HWT101_TimingBegin();
                accepted=ChassisAuto_BeginPreparedHeading(&s_heading_backend_prepared,expected_session);
                HWT101_TimingEnd(1U,prepare_cycle);
                HeadingSession_ReportBegin(&s_heading_exec, accepted, &output);
                if (accepted != 1U) App_HeadingRuntimeTrip(6U);
                else
                {
                    (void)ChassisAuto_ReadStatus(&status);
                    g_heading_exec_backend_session = status.session;
                    g_heading_exec_phase = HR_RUN;
                    s_heading_exec_wait_ms = now;
                    g_heading_stop_trace.target_deg = output.target_deg;
                    App_HeadingStopCapture(&g_heading_stop_trace.start, now);
                    g_heading_stop_trace.started = 1U;
                }
            }
        }
    }
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_HeadingRuntimeOwns(&status)) App_HeadingRuntimeTrip(5U);
        else
        {
            input.now_ms = now;
            input.sample_ms = heading.sample_ms;
            input.sequence = heading.sequence;
            input.healthy = App_HeadingRuntimeSource(now);
            input.platform_ok = ((App_HeadingRuntimeExclusive() == 1U) &&
                (g_vm_test_request == 1U) &&
                ((App_CanJogReady() == 1U) ||
                 ((s_heading_exec.state == HS_STOPPING) &&
                  (status.active == 0U) && (status.result == 1U) &&
                  (status.rotation_stop_mask == 0x0FU)))) ? 1U : 0U;
            input.yaw_deg = heading.yaw_deg;
            input.rate_dps = heading.rate_dps;
            input.backend_active = status.active;
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            if (output.action == HS_ACTION_UPDATE)
            {
                record_zero = ((output.command_rpm == 0) &&
                    (!s_heading_exec.turn_trial || s_heading_exec.brake.moved) &&
                    (g_heading_stop_trace.zero_request.captured == 0U)) ? 1U : 0U;
                if (record_zero != 0U)
                    App_HeadingStopCapture(&g_heading_stop_trace.zero_request, now);
                accepted = ChassisAuto_UpdateHeading(output.command_rpm);
                if (record_zero != 0U)
                    g_heading_stop_trace.zero_submit_accepted = accepted;
                HeadingSession_ReportUpdate(&s_heading_exec, accepted, &output);
            }
            else if (output.action == HS_ACTION_STOP)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.reached, now);
                accepted = ChassisAuto_RequestStop();
                HeadingSession_ReportStop(&s_heading_exec, accepted, &output);
            }
            if (output.action == HS_ACTION_ABORT) App_HeadingRuntimeTrip(6U);
            else if (output.action == HS_ACTION_COMPLETE)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.completed, now);
                g_heading_exec_phase = HR_DONE;
                g_vm_test_request = (*inputs->keep_vm);
            }
        }
    }
    if (g_heading_exec_trip != 0U)
    {
        if (g_heading_exec_trip == 1U)
        {
            HeadingSession_Cancel(&s_heading_exec, &output);
            g_heading_exec_phase = HR_CANCELLED;
        }
        else
        {
            /* 只让CAN任务终结会话；故障不伪装成正常到位。 */
            input.healthy = input.platform_ok = 0U;
            input.backend_result = 2U;
            input.now_ms = now;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            g_heading_exec_phase = HR_FAULT;
        }
        output.action = HS_ACTION_ABORT;
        output.command_rpm = 0;
    }
    g_heading_exec_state = output.state;
    g_heading_exec_cause = output.cause;
    g_heading_exec_action = output.action;
    g_heading_exec_rpm = output.command_rpm;
    if (g_heading_exec_phase != HR_IDLE)
    {
        g_heading_exec_target_deg = s_heading_exec.controller.target_deg;
        g_heading_exec_error_deg = s_heading_exec.controller.error_deg;
    }
    HWT101_TimingEnd(0U,critical_cycle);
    taskEXIT_CRITICAL();

    /* 真正的CAN交易放在临界区外，仍只有这一个CAN任务。 */
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
        ChassisAuto_CanStep();
}

void AppHeadingRuntime_ResetSession(void)
{
    HeadingSession_Init(&s_heading_exec);
}
#endif
