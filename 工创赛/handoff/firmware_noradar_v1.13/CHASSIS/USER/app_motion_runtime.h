#ifndef APP_MOTION_RUNTIME_H
#define APP_MOTION_RUNTIME_H
#include "app_translation_runtime.h"
#include "motion_action_plan.h"
#include "runtime_route.h"
#if APP_HEADING_SESSION_TEST && APP_TEST_MODE == 3U
typedef struct
{
    uint32_t request, state, index, cause, completed;
    uint32_t phase[MOTION_ACTION_CAPACITY], session[MOTION_ACTION_CAPACITY], elapsed_ms[MOTION_ACTION_CAPACITY];
    float final_yaw[MOTION_ACTION_CAPACITY], final_error[MOTION_ACTION_CAPACITY];
} CombinedTestDebug;
extern volatile CombinedTestDebug g_combined_test;
extern volatile uint32_t g_debug_plan_id,g_motion_plan_valid;
/* 原owner状态的只读live地址；调用者仍在原表达式和临界区内读取。 */
typedef struct {
    const uint32_t *action_count,*session_open,*has_action,*turn,*mode,*base_rpm;
    const float *distance_mm,*reference_mm_s;
    const uint32_t *begin_ms;
    const volatile uint32_t *end_ms;
    const uint32_t *stage_ms;
    const MotionAction *actions;
    const float (*perimeter_targets)[3];
} AppMotionObservation;
const AppMotionObservation *AppMotion_Live(void);
uint32_t App_DebugSelectPlan(uint32_t id);
void App_CombinedTestStop(uint32_t cause);
void App_CombinedTestSafety(void);
void App_CombinedTestStep(void);
uint32_t App_RuntimeRouteInstall(const RuntimeRoutePrepared *prepared,uint32_t now);
#endif
void App_MotionFaultTelemetry(void);
void App_MotionTimingTelemetry(void);
uint32_t AppMotion_SetStartPermit(uint32_t permit);
void AppMotion_LoadDefaultPlan(void);
void AppMotion_InitStage1Plan(void);
void AppMotion_InitOdometry(void);
void AppMotion_StepOdometry(void);
void AppMotion_Stage1Service(void);
void AppMotion_Stage1Display(uint32_t ready);
void AppMotion_Stage1Telemetry(void);
void AppMotion_Stage1CodeTelemetry(void);
void AppMotion_MissionService(void);
void AppMotion_MissionTelemetry(void);
extern volatile uint32_t g_stage1_state,g_stage1_cause,g_stage1_key_request,g_vehicle_key;
#endif
