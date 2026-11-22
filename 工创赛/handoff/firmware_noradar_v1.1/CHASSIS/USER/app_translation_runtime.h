#ifndef APP_TRANSLATION_RUNTIME_H
#define APP_TRANSLATION_RUNTIME_H
#include "app_heading_runtime.h"
#include "route_heading_reference.h"
#if APP_HEADING_SESSION_TEST
typedef struct
{
    uint32_t request, phase, trip, backend_session;
    uint32_t run_started, stopping, elapsed_ms, final_valid;
    float correction_rpm;
    float start_deg, target_deg, error_deg, final_deg;
    float rate_dps, peak_error_deg, peak_rate_dps;
    uint32_t sample_count, max_sample_gap_ms;
    /* 已接受的非零纠偏提交样本统计，停车后保留，不代表CAN执行次数。 */
    uint32_t motion_mode; /* 编译选择，仅观察，不用Watch切换方向。 */
    uint32_t correction_nonzero_samples;
    float correction_peak_abs_rpm;
    float correction_last_nonzero_rpm;
} ForwardHoldDebug;
typedef struct
{
    uint32_t elapsed_ms;
    float error_deg, rate_dps;
    float correction_rpm;
} ForwardHoldTraceSample;

extern volatile ForwardHoldDebug g_forward_hold;
extern volatile uint32_t g_translation_hold_selftest;
extern volatile ForwardHoldTraceSample g_forward_hold_trace[96];
extern volatile uint32_t g_forward_hold_trace_count;
/* 原motion owner持有状态；平移执行仅在原表达式处读live值。 */
typedef struct {
    const uint32_t *mode, *keep_vm, *powered_segment, *base_rpm, *run_ms;
    const float *distance_mm, *reference_mm_s, *route_target;
    const RouteHeadingReference *route_heading_reference;
} AppTranslationRuntimeInputs;
void App_ForwardHoldTrip(uint32_t cause);
uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *status, const AppTranslationRuntimeInputs *inputs);
void App_ForwardHoldSafety(const AppTranslationRuntimeInputs *inputs);
void App_ForwardHoldStep(const AppTranslationRuntimeInputs *inputs);
#endif
#endif
