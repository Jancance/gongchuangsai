#ifndef APP_CAN_DIAGNOSTICS_H
#define APP_CAN_DIAGNOSTICS_H
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "chassis_auto_test.h"

#define CAN_POLL_VALID_MS 1000U

/* 保留既有Watch符号；CAN私有可写状态只归诊断模块持有。 */
extern volatile uint32_t g_can_test_result;
extern volatile uint32_t g_can_jog_request;
extern volatile uint32_t g_can_poll_enable;
extern volatile uint32_t g_can_poll_result[4];
extern volatile uint32_t g_can_poll_flags[4];
extern volatile TickType_t g_can_poll_tick[4];
extern volatile uint32_t g_can_poll_count[4];
extern volatile uint32_t g_can_poll_valid_mask;
extern volatile uint32_t g_can_poll_protect_mask;
extern volatile uint32_t g_can_feedback_enable;
extern volatile uint32_t g_motion_diag_hold_report;
extern volatile uint32_t g_motion_diag_hold_stop_service;
extern volatile uint32_t g_motion_diag_fail_stop_once;
extern volatile uint32_t g_motion_diag_report_count;
extern volatile TickType_t g_can_feedback_speed_tick[4];
extern volatile TickType_t g_can_feedback_position_tick[4];
extern volatile uint32_t g_can_feedback_speed_valid_mask;
extern volatile uint32_t g_can_feedback_position_valid_mask;

/* Copy successful source samples, independently of the safety valid mask. */
void AppCanDiagnostics_ReadPositionSamples(ChassisOdometrySamples *samples);
void App_CanPollRefresh(void);
void App_CanFeedbackStep(void);
void App_CanFeedbackRefresh(void);
uint32_t App_CanJogOtherRequest(void);
uint32_t App_CanJogReady(void);
const uint32_t *AppCanDiagnostics_JogPreparing(void);
void AppCanDiagnostics_Dispatch(void);
#endif
