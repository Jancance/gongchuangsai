#include "app_chassis_service.h"
#include "app_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "app_can_diagnostics.h"
#include "app_heading_runtime.h"
#include "app_translation_runtime.h"
#include "app_motion_runtime.h"
#include "app_serial_runtime.h"
#include "app_debug_runtime.h"
#include "app_input_runtime.h"
#include "startup_axes.h"
#include "vehicle_arm.h"
#include "dm_can2.h"
#include "can.h"
#include "power.h"
#include "safety.h"
#include "motion_control.h"
#include "chassis_auto_test.h"
#if APP_ZONE2_NAV_ENABLE
#include "app_zone2.h"
#endif

extern volatile uint32_t g_vm_test_request;


#if !APP_VM_ONLY_ENABLE
static void App_ComboIdlePoll(void)
{
    static uint32_t position_turn;
    uint32_t combined = (g_combined_test.state==1U || g_combined_test.state==2U);
    if(combined && position_turn) {
        position_turn=0U;
        ChassisAuto_OdometryPoll();
    } else {
        position_turn=combined;
        App_CanFeedbackStep();
    }
}

#endif
void AppChassis_CanInit(void)
{
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
    AppMotion_InitOdometry();
#endif
    g_can_test_result = CAN1_LoopbackTest();
}

#if APP_VM_ONLY_ENABLE
volatile uint32_t g_vm_only_cancelled;
static void App_VmOnlySafety(void)
{
    if(g_combined_test.request==2U) {g_combined_test.request=0U;g_vm_only_cancelled=1U;}
    if(g_rtos_fault || g_safety_fault || g_can_ctrl_fault_latched || g_can_test_result>1U)
        g_vm_only_cancelled=1U;
    if(g_vm_only_cancelled) {g_vm_test_request=0U;Power_Disable();}
}
static void App_VmOnlyStep(void)
{
    App_VmOnlySafety();
    taskENTER_CRITICAL();
    if(!g_vm_only_cancelled && g_safety_ready==1U && g_can_test_result==1U)g_vm_test_request=1U;
    taskEXIT_CRITICAL();
    if(g_vm_only_cancelled)return;
    App_CanFeedbackStep();
    App_CanPollRefresh();
    Safety_CanReport(g_can_poll_enable,g_can_poll_valid_mask,g_can_poll_protect_mask,g_can_poll_tick);
}
#endif
void AppChassis_CanStep(void)
{
#if APP_VM_ONLY_ENABLE
        App_VmOnlyStep();
#else
        if (APP_UART_LINK_TEST != 0U)
        {
            g_vm_test_request = 0U;
            Power_Disable();
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
            Safety_MotionFeedbackReport(0U, 0U, 0U, NULL, NULL);
        }
        else
        {
#if APP_SERIAL_MOTION_CONTROL
        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();
#endif
        #if APP_HEADING_SESSION_TEST
                #if APP_TEST_MODE == 3U
#if APP_BASELINE40_ACTIVE
                App_CombinedTestStep();
#else
                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
                if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(VehicleArm_Busy() || g_vehicle_arm_stop_pending)VehicleArm_Service();
                if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();
#endif
#else
                if (APP_FORWARD_HOLD_TEST != 0U) App_ForwardHoldStep(&s_translation_runtime_inputs);
                else App_HeadingRuntimeStep(&s_heading_runtime_inputs);
#endif
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
                AppDebug_HeadingObserve();
        #endif
        if (g_can_test_result == 1U)
        {

        /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
        /* 静态停止测试优先，每轮仍只执行一笔CAN交易 */
        /* 自动停止优先，每轮只执行一笔CAN交易 */
        #if APP_HEADING_SESSION_TEST
                if (g_chassis_auto_active == 0U) {
                    App_ComboIdlePoll();
                    if(!APP_BASELINE40_ACTIVE && StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                }
        #endif
                if (APP_HEADING_SESSION_TEST == 0U)
                {
        AppCanDiagnostics_Dispatch();
            } /* 原调试交易只在非目标航向模式执行 */
        App_CanPollRefresh();
        App_CanFeedbackRefresh();
        /* 临时故障注入，正式监督与正常轮询仍继续运行 */
        Safety_CanReport(g_can_poll_enable, g_can_poll_valid_mask,
        g_can_poll_protect_mask, g_can_poll_tick);
        /* 诊断暂停仅冻结Safety的两类快照；原3A状态保护照常发布 */
        if ((g_can_feedback_enable != 1U) || (g_can_poll_enable != 1U) ||
            (g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
            (g_rtos_fault != 0U) || (g_safety_fault != 0U))
        {
            /* 会话结束或故障不保留诊断暂停，仍发布无效快照 */
            g_motion_diag_hold_report = 0U;
            g_motion_diag_hold_stop_service = 0U;
            g_motion_diag_fail_stop_once = 0U;
        }
        if (g_motion_diag_hold_report != 1U)
        {
            /* 将真实采样时间交给Safety，不能替换成报告时刻 */
            Safety_MotionFeedbackReport(
                ((g_can_feedback_enable == 1U) &&
                 (g_can_poll_enable == 1U) && (g_vm_test_request == 1U) &&
                 (g_safety_ready == 1U) && (g_rtos_fault == 0U) &&
                 (g_safety_fault == 0U)) ? 1U : 0U,
                g_can_feedback_speed_valid_mask,
                g_can_feedback_position_valid_mask,
                g_can_feedback_speed_tick, g_can_feedback_position_tick);
            g_motion_diag_report_count++;
        }
        }
        else
        {
            /* 回环失败时不发电机查询，监督沿用原启动故障判定 */
            Safety_CanReport(0U, 0U, 0U, g_can_poll_tick);
            g_motion_diag_hold_report = 0U;
            g_motion_diag_hold_stop_service = 0U;
            g_motion_diag_fail_stop_once = 0U;
            Safety_MotionFeedbackReport(0U, 0U, 0U, NULL, NULL);
            g_motion_diag_report_count++;
        }
        } /* 串口测试之外才允许原CAN交易路径 */
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
        AppMotion_StepOdometry();
#endif
#endif
}

#if APP_ZONE2_NAV_ENABLE
void AppChassis_Zone2CanStep(uint32_t slot_ms,uint32_t phase)
{
    if (g_can_test_result==1U) {
        if (!(phase&1U) && (g_chassis_auto_state==CHASSIS_AUTO_START ||
            g_chassis_auto_state==CHASSIS_AUTO_RUN) && !AppZone2_CanPermit(slot_ms))
            (void)ChassisAuto_RequestStop();
        ChassisAuto_Zone2CanStep(slot_ms,phase);
    }
    ChassisAuto_Zone2SafetyReport();
    AppMotion_StepOdometry();
    AppZone2_Step();
}
#endif

void AppChassis_SafetyCheck(void)
{
#if APP_VM_ONLY_ENABLE
    App_VmOnlySafety();
#else
if (APP_UART_LINK_TEST != 0U)
        { g_vm_test_request = 0U; Power_Disable(); }
        else
        {
#if APP_SERIAL_MOTION_CONTROL
        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();
#endif
#if APP_HEADING_SESSION_TEST
        #if APP_TEST_MODE == 3U
#if !APP_BASELINE40_ACTIVE
        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);
        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();
#endif
        App_CombinedTestSafety();
#else
        if (APP_FORWARD_HOLD_TEST != 0U) App_ForwardHoldSafety(&s_translation_runtime_inputs);
        else App_HeadingRuntimeSafety(&s_heading_runtime_inputs);
#endif
#elif APP_HWT_STATIC_TEST
        g_vm_test_request = 0U;
#elif APP_HWT_TURN_TEST
        App_HwtTurnSafety();
#endif
        }
#endif
}
