#ifndef APP_HEADING_RUNTIME_H
#define APP_HEADING_RUNTIME_H
#include "app_config.h"
#include "chassis_auto_test.h"
#include "heading_session.h"

#if APP_HEADING_SESSION_TEST
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
typedef struct
{
    uint32_t captured, valid;
    uint32_t event_ms, angle_sample_ms, angle_age_ms, rate_age_ms;
    uint32_t source_sequence, pe2_active;
    float yaw_deg, error_deg, rate_dps;
} HeadingStopAngleSample;
typedef struct
{
    uint32_t started, zero_submit_accepted;
    uint32_t off_observed_ms;
    uint32_t post_status; /* 0等待，1已采到关闭后新帧，2两秒内无有效帧。 */
    float target_deg;
    HeadingStopAngleSample start;
    HeadingStopAngleSample zero_request;
    HeadingStopAngleSample reached;
    HeadingStopAngleSample completed;
    HeadingStopAngleSample off_seen;
    HeadingStopAngleSample after_off;
} HeadingStopTrace;

extern volatile uint32_t g_heading_exec_request;
extern volatile float g_heading_exec_relative_deg;
extern volatile uint32_t g_heading_exec_phase;
extern volatile uint32_t g_heading_exec_trip;
extern volatile uint32_t g_heading_exec_state;
extern volatile uint32_t g_heading_exec_cause;
extern volatile uint32_t g_heading_exec_action;
extern volatile float g_heading_exec_rpm;
extern volatile float g_heading_exec_target_deg;
extern volatile float g_heading_exec_error_deg;
extern volatile uint32_t g_heading_exec_backend_session;
extern volatile uint32_t g_heading_power_wait_state;
extern volatile uint32_t g_heading_power_on_age_ms;
extern volatile uint32_t g_heading_power_post_settle_mask;
extern volatile HeadingStopTrace g_heading_stop_trace;
extern volatile uint32_t g_heading_prepare_cycles,g_heading_prepare_max_cycles;
extern volatile uint32_t g_heading_fault_phase;
extern volatile uint32_t g_heading_fault_source_flags;
extern volatile uint32_t g_heading_fault_sample_age_ms;
extern volatile uint32_t g_heading_fault_angle_age_ms;
extern volatile uint32_t g_heading_fault_rate_age_ms;
extern volatile uint32_t g_heading_fault_hwt_update_age_ms;
extern volatile uint32_t g_heading_fault_tick_ms;

/* 仅持有原motion owner状态的只读live地址；每次在原表达式位置读取。 */
typedef struct {
    const uint32_t *keep_vm, *powered_segment, *turn_max_rpm, *route_turn_active;
    const float *route_target;
} AppHeadingRuntimeInputs;
void AppHeadingRuntime_ResetSession(void);
void App_HeadingRuntimeTrip(uint32_t cause);
uint32_t App_HeadingRuntimeExclusive(void);
uint32_t App_HeadingRuntimeSource(uint32_t now);
uint32_t App_HeadingPowerReady(uint32_t now);
uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *status);
void App_HeadingRuntimeSafety(const AppHeadingRuntimeInputs *inputs);
void App_HeadingRuntimeStep(const AppHeadingRuntimeInputs *inputs);
#endif
#endif
