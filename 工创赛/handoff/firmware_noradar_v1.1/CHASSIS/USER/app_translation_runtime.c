#include "app_translation_runtime.h"
#include "app_motion_internal.h"
#include "app_can_diagnostics.h"
#include "heading_source.h"
#include "freertos_hooks.h"
#include "hwt101.h"
#include "power.h"
#include "safety.h"
#include "vehicle_tuning.h"

extern volatile uint32_t g_vm_test_request;
extern volatile uint32_t g_stage1_stop_requested;
extern volatile uint32_t g_motion_fault_diag_valid;
#if APP_HEADING_SESSION_TEST
volatile ForwardHoldDebug g_forward_hold = {.motion_mode = APP_TRANSLATION_HOLD_MODE};
volatile uint32_t g_translation_hold_selftest = 0U;
static uint32_t s_hold_wait_ms, s_hold_begin_ms, s_hold_run_ms, s_hold_stop_ms;
static uint32_t s_hold_sequence, s_hold_new_ms;
static float s_hold_target;
static HeadingHoldController s_hold_controller = {0};
volatile ForwardHoldTraceSample g_forward_hold_trace[96];
volatile uint32_t g_forward_hold_trace_count = 0U;
static uint32_t s_hold_trace_ms;

void App_ForwardHoldTrip(uint32_t cause)
{
    AppHeadingSource heading;
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    if (g_forward_hold.trip == 0U)
    {
        AppMotion_InternalFaultCapture(cause);
        g_forward_hold.trip = cause;
    }
    g_forward_hold.phase = (g_forward_hold.trip == 1U) ? HR_CANCELLED : HR_FAULT;
    g_forward_hold.correction_rpm = 0;
    g_forward_hold.final_deg = heading.yaw_deg;
    g_forward_hold.final_valid = App_HeadingRuntimeSource((uint32_t)xTaskGetTickCount());
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    taskEXIT_CRITICAL();
}

uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *status, const AppTranslationRuntimeInputs *inputs)
{
    return ((status->session == g_forward_hold.backend_session) &&
        (status->heading_mode == 1U) && (status->forward_hold_mode == 1U) &&
        (status->translation_mode == (*inputs->mode)) &&
        (status->segment == 2U) && (status->dry_run == 0U)) ? 1U : 0U;
}

/* 独立Safety监督，既有100ms源/提交失效和有界等待均不放宽。 */
void App_ForwardHoldSafety(const AppTranslationRuntimeInputs *inputs)
{
    AppHeadingSource heading;
    ChassisAutoStatus status;
    uint32_t now, normal_done;
    float correction;
    float error;
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    if ((g_forward_hold.phase != HR_WAIT) && (g_forward_hold.phase != HR_RUN))
    { taskEXIT_CRITICAL(); return; }
    now = (uint32_t)xTaskGetTickCount();
    (void)ChassisAuto_ReadStatus(&status);
    /* 等待新段时后端仍保留上一段到距结果，只有当前已归属会话才能触发停止。 */
    if ((g_forward_hold.phase == HR_RUN) && App_ForwardHoldOwns(&status,inputs) &&
        status.distance_mode && status.distance_reached && !g_forward_hold.stopping)
    {
        g_forward_hold.stopping = 1U; g_forward_hold.correction_rpm = 0;
        s_hold_stop_ms = now;
    }
    if (g_forward_hold.request == 2U) App_ForwardHoldTrip(1U);
    else if (!App_HeadingRuntimeSource(now) ||
        ((g_forward_hold.phase == HR_RUN) && ((uint32_t)(now - s_hold_new_ms) >= HS_SOURCE_MS)))
        App_ForwardHoldTrip(2U);
    else if ((g_translation_hold_selftest != 1U) ||
        !App_HeadingRuntimeExclusive() || (g_vm_test_request != 1U))
        App_ForwardHoldTrip(3U);
    else if (g_forward_hold.phase == HR_WAIT)
    {
        if ((uint32_t)(now - s_hold_wait_ms) >= 5000U) App_ForwardHoldTrip(4U);
        else if (status.active != 0U || status.result != 0U) App_ForwardHoldTrip(5U);
    }
    else
    {
        normal_done = ((g_forward_hold.stopping == 1U) &&
            (status.active == 0U) && (status.result == 1U) &&
            (status.rotation_stop_mask == 0x0FU)) ? 1U : 0U;
        if (!App_ForwardHoldOwns(&status,inputs)) App_ForwardHoldTrip(5U);
        else if ((uint32_t)(now - s_hold_begin_ms) >=
            (((*inputs->distance_mm)>0.0f) ? (*inputs->run_ms) : APP_HEADING_RUN_MS))
            App_ForwardHoldTrip(7U);
        else if (g_forward_hold.stopping &&
            ((uint32_t)(now - s_hold_stop_ms) >= HS_STOP_MS)) App_ForwardHoldTrip(7U);
        else if (!normal_done && ((status.active != 1U) || (status.result != 0U) ||
            (g_safety_ready != 1U) || (g_can_guard_ready != 1U))) App_ForwardHoldTrip(5U);
        else if (HeadingController_HoldCorrection(s_hold_target, heading.yaw_deg,
            heading.rate_dps, &correction, &error) != 1U) App_ForwardHoldTrip(6U);
    }
    taskEXIT_CRITICAL();
}

/* 仅CAN任务执行；本模式清除旧转角请求，避免误触发原地转动。 */
void App_ForwardHoldStep(const AppTranslationRuntimeInputs *inputs)
{
    AppHeadingSource heading;
    ChassisAutoStatus status;
    uint32_t now, request, delta, accepted, critical_cycle, begin_cycle;
#if APP_DISTANCE_SCURVE_ACTIVE
    /* 唯一CAN任务准备，避免新增轨迹对象占用CAN任务栈；不供其他上下文复用。 */
    static ChassisAutoPreparedTranslation prepared;
#else
    ChassisAutoPreparedTranslation prepared;
#endif
    uint32_t prepared_ok, expected_session, mode, base_rpm, run_ms;
    float route_speed;
    float distance_mm;
    float correction = 0;
    float error = 0.0f, desired = 0.0f;
    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    critical_cycle = HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    request = g_forward_hold.request;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_forward_hold.phase >= HR_DONE)
    { g_vm_test_request = (g_forward_hold.phase == HR_DONE) ? (*inputs->keep_vm) : 0U; HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
    if ((request == 2U) && (g_forward_hold.phase != HR_IDLE)) App_ForwardHoldTrip(1U);
    if ((request == 1U) && (g_forward_hold.phase == HR_IDLE))
    {
#if APP_SERIAL_MOTION_CONTROL && (APP_TEST_MODE != 3U)
        if (s_serial_start_permit != 1U)
        { App_ForwardHoldTrip(3U); HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
#endif
        if ((g_translation_hold_selftest != 1U) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != (*inputs->powered_segment)) || (status.active != 0U) ||
            (status.result != 0U) || (status.cut_vm != 0U) ||
            !(heading.rate_dps >= -3.0f && heading.rate_dps <= 3.0f))
            App_ForwardHoldTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
#endif
            s_hold_wait_ms = now;
            g_forward_hold.phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    App_ForwardHoldSafety(inputs);
    if ((g_forward_hold.phase == HR_WAIT) && (App_CanJogReady() == 1U) &&
        (App_HeadingPowerReady(now) == 1U))
    {
        /* 只携带参数和旧会话号出锁，不携带过期的运动许可。 */
        mode = (*inputs->mode); base_rpm = (*inputs->base_rpm);
        run_ms = (*inputs->run_ms); distance_mm = (*inputs->distance_mm);
#if APP_TEST_MODE == 3U
        route_speed=(*inputs->reference_mm_s);
#else
        route_speed=0.0f;
#endif
        (void)ChassisAuto_ReadStatus(&status); expected_session = status.session;
        HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
        prepared_ok = route_speed!=0.0f ?
            ChassisAuto_PrepareRouteTranslation(mode,base_rpm,run_ms,distance_mm,route_speed,&prepared) :
            ChassisAuto_PrepareTranslation(mode,base_rpm,run_ms,distance_mm,&prepared);
        taskENTER_CRITICAL();
        (void)App_HeadingSourceRead(&heading);
        critical_cycle = HWT101_TimingBegin();
        now = (uint32_t)xTaskGetTickCount();
        App_HeadingSourceObserve();
        App_ForwardHoldSafety(inputs);
        if (g_forward_hold.phase != HR_WAIT || g_forward_hold.trip)
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        (void)ChassisAuto_ReadStatus(&status);
        if (!prepared_ok || status.session != expected_session ||
            mode != (*inputs->mode) || base_rpm != (*inputs->base_rpm) ||
            run_ms != (*inputs->run_ms) || distance_mm != (*inputs->distance_mm)
#if APP_TEST_MODE == 3U
            || route_speed!=(*inputs->reference_mm_s)
#endif
            )
        {
            App_ForwardHoldTrip(5U);
            HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return;
        }
        /* 源/故障/取消已重查；准备条件变化则继续有界等待，不能用旧许可提交。 */
        if (!App_CanJogReady() || !App_HeadingPowerReady(now))
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        /* 提交运动前验证固定参考，异常目标不获得电机启动许可。 */
        if (!inputs->route_heading_reference->valid ||
            !HeadingController_HoldCorrection((*inputs->route_target),heading.yaw_deg,
                heading.rate_dps,&desired,&error))
        {
            App_ForwardHoldTrip(6U);
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();return;
        }
        begin_cycle = HWT101_TimingBegin();
        accepted = ChassisAuto_BeginPreparedTranslation(&prepared,expected_session);
        HWT101_TimingEnd(1U, begin_cycle);
        if (accepted != 1U) App_ForwardHoldTrip(5U);
        else
        {
            (void)ChassisAuto_ReadStatus(&status);
            g_forward_hold.backend_session = status.session;
            s_hold_target = (*inputs->route_target);
            g_forward_hold.start_deg = heading.yaw_deg;
            g_forward_hold.target_deg = s_hold_target;
            s_hold_sequence = heading.sequence;
            s_hold_begin_ms = s_hold_new_ms = now;
            s_hold_controller = (HeadingHoldController){0};
            g_forward_hold.phase = HR_RUN;
        }
    }
    if (g_forward_hold.phase == HR_RUN)
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_ForwardHoldOwns(&status,inputs)) App_ForwardHoldTrip(5U);
        else if (HeadingController_HoldCorrection(s_hold_target, heading.yaw_deg,
            heading.rate_dps, &desired, &error) != 1U) App_ForwardHoldTrip(6U);
        else
        {
            g_forward_hold.error_deg = error;
            /* 有效任务码提前结束本段；沿用唯一CAN任务有序停车及零速核验。 */
            if (VEHICLE_STAGE1_ENABLE && g_stage1_stop_requested &&
                !g_forward_hold.stopping && status.active && status.state>=CHASSIS_AUTO_START &&
                status.state<=CHASSIS_AUTO_SETTLE)
            {
                if (ChassisAuto_RequestStop()!=1U) App_ForwardHoldTrip(8U);
                else {g_forward_hold.stopping=1U;g_forward_hold.correction_rpm=0;s_hold_stop_ms=now;}
            }
            if (status.distance_mode && status.distance_reached && !g_forward_hold.stopping)
            {
                g_forward_hold.stopping = 1U; g_forward_hold.correction_rpm = 0;
                s_hold_stop_ms = now;
            }
            if ((g_forward_hold.run_started == 0U) && (status.state == CHASSIS_AUTO_RUN) &&
                (g_chassis_auto_start_mask[2] == 0x0FU))
            {
                s_hold_run_ms = now;
                g_forward_hold.run_started = 1U;
            }
            if ((g_forward_hold.run_started != 0U) && (g_forward_hold.stopping == 0U))
                g_forward_hold.elapsed_ms = now - s_hold_run_ms;
            if ((g_forward_hold.stopping == 0U) && g_forward_hold.run_started &&
                (g_forward_hold.elapsed_ms >= (*inputs->run_ms)))
            {
                if ((*inputs->distance_mm) > 0.0f) App_ForwardHoldTrip(2U);
                else if (ChassisAuto_RequestStop() != 1U) App_ForwardHoldTrip(8U);
                else
                {
                    g_forward_hold.stopping = 1U;
                    g_forward_hold.correction_rpm = 0;
                    s_hold_stop_ms = now;
                }
            }
            delta = heading.sequence - s_hold_sequence;
            if (delta > 0x7FFFFFFFU) App_ForwardHoldTrip(2U);
            else if (delta != 0U)
            {
                if (now - s_hold_new_ms > g_forward_hold.max_sample_gap_ms)
                    g_forward_hold.max_sample_gap_ms = now - s_hold_new_ms;
                s_hold_sequence = heading.sequence;
                s_hold_new_ms = now;
                g_forward_hold.sample_count++;
                g_forward_hold.rate_dps = heading.rate_dps;
                if (((error < 0.0f) ? -error : error) > g_forward_hold.peak_error_deg)
                    g_forward_hold.peak_error_deg = ((error < 0.0f) ? -error : error);
                if (((heading.rate_dps < 0.0f) ? -heading.rate_dps : heading.rate_dps) > g_forward_hold.peak_rate_dps)
                    g_forward_hold.peak_rate_dps = ((heading.rate_dps < 0.0f) ? -heading.rate_dps : heading.rate_dps);
                if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 0U))
                {
                    if (HeadingController_HoldStep(&s_hold_controller, s_hold_target,
                        heading.yaw_deg, heading.rate_dps, now,
                        &correction, &error) != 1U) App_ForwardHoldTrip(2U);
                    else if (ChassisAuto_UpdateForwardHold(correction) != 1U) App_ForwardHoldTrip(8U);
                    else
                    {
                        g_forward_hold.correction_rpm = correction;
                        if (correction != 0)
                        {
                            float magnitude = (correction < 0.0f) ? -correction : correction;
                            g_forward_hold.correction_nonzero_samples++;
                            if (magnitude > g_forward_hold.correction_peak_abs_rpm)
                                g_forward_hold.correction_peak_abs_rpm = magnitude;
                            g_forward_hold.correction_last_nonzero_rpm = correction;
                        }
                        if ((g_forward_hold.run_started == 1U) &&
                            (g_forward_hold_trace_count < 96U) &&
                            ((g_forward_hold_trace_count == 0U) ||
                             ((uint32_t)(now - s_hold_trace_ms) >= 100U)))
                        {
                            uint32_t index = g_forward_hold_trace_count;
                            g_forward_hold_trace[index].elapsed_ms = g_forward_hold.elapsed_ms;
                            g_forward_hold_trace[index].error_deg = error;
                            g_forward_hold_trace[index].rate_dps = heading.rate_dps;
                            g_forward_hold_trace[index].correction_rpm = correction;
                            s_hold_trace_ms = now;
                            g_forward_hold_trace_count = index + 1U;
                        }
                    }
                }
            }
            if ((g_forward_hold.trip == 0U) && (g_forward_hold.stopping == 1U) &&
                (status.active == 0U) && (status.result == 1U) &&
                (status.rotation_stop_mask == 0x0FU))
            {
                g_forward_hold.final_deg = heading.yaw_deg;
                g_forward_hold.final_valid = App_HeadingRuntimeSource(now);
                g_forward_hold.correction_rpm = 0;
                g_forward_hold.phase = HR_DONE;
                g_vm_test_request = (*inputs->keep_vm);
            }
        }
    }
    HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL();
    if ((g_forward_hold.phase == HR_RUN) && (g_forward_hold.trip == 0U)) ChassisAuto_CanStep();
}
#endif
