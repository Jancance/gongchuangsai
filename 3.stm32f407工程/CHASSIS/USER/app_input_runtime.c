#include "app_input_runtime.h"
#include "vehicle_tuning.h"
#include "task.h"
#include "freertos_hooks.h"
#include "app_entry.h"
#include "app_serial_internal.h"
#include "app_can_diagnostics.h"
#include "app_heading_runtime.h"
#include "app_translation_runtime.h"
#include "app_motion_runtime.h"
#include "startup_axes.h"
#include "chassis_auto_test.h"
#include "motion_control.h"
#include "radar_zone_switch.h"
#include "radar_lift.h"
#include "radar_servo.h"
#include "Key.h"
#include "hwt101.h"
#include "safety.h"
#include "screen.h"
#include "saoma.h"
#include <stdio.h>

extern volatile uint32_t g_vm_test_request;

/* 启动按键事件数 */
volatile uint32_t g_start_key_count = 0U;
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_scan_display_result = 0U;
volatile uint32_t g_scan_display_count = 0U;
volatile uint32_t g_scan_log_send_ok = 0U;
/* 当前已接模块的启动请求门禁，尚未驱动底盘 */
volatile uint32_t g_start_gate_ready = 0U;
volatile uint32_t g_start_request = 0U;
volatile uint32_t g_start_accept_count = 0U;
volatile uint32_t g_start_deny_count = 0U;
char g_start_task_code[16] = {0};
#if APP_IS_COMPETITION || (!APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST)
static uint32_t s_start_gate_revision = 0U;
#endif
static uint32_t s_start_pending = 0U;
static uint32_t s_start_pending_revision = 0U;

/* 只有任务A调用；先检查当前平台，再确认按键对应的扫码版本 */
void App_StartGateUpdate(void)
{
    uint32_t platform_ok;
    uint32_t code_ok;
    uint32_t accepted = 0U;
    uint32_t i;
    char log[24];

    taskENTER_CRITICAL();
    platform_ok = (g_rtos_fault == 0U && g_safety_fault == 0U &&
                   g_safety_ready == 1U) ? 1U : 0U;
    if (platform_ok == 0U)
    {
        g_start_request = 0U;
    }
    code_ok = (g_saoma_last_result == 1U &&
               g_saoma_accept_count != 0U &&
               g_scan_display_result == SCREEN_OK) ? 1U : 0U;
    g_start_gate_ready = (platform_ok != 0U && code_ok != 0U &&
                          g_start_request == 0U) ? 1U : 0U;
#if APP_IS_COMPETITION || (!APP_CHASSIS_KEY_TEST_ENABLE && !APP_HEADING_SESSION_TEST)
    s_start_gate_revision = g_saoma_accept_count;
#endif
    if (s_start_pending != 0U)
    {
        s_start_pending = 0U;
        if (g_start_gate_ready != 0U &&
            s_start_pending_revision == g_saoma_accept_count)
        {
            for (i = 0U; i < sizeof(g_start_task_code); i++)
            {
                g_start_task_code[i] = SAOMA_Data[i];
            }
            g_start_request = 1U;
            g_start_gate_ready = 0U;
            g_start_accept_count++;
            accepted = 1U;
        }
        else
        {
            g_start_deny_count++;
        }
    }
    taskEXIT_CRITICAL();
    if (accepted != 0U)
    {
#if APP_IS_COMPETITION
        AppCompetitionEntry_Submit(g_start_task_code,platform_ok);
#endif
        sprintf(log, "START:%s\r\n", g_start_task_code);
        (void)App_DebugSendString(log);
    }
}

void AppInput_ScanDisplay(TickType_t screen_start_tick)
{
        /* 确认启动请求后，保留本轮码和屏幕，不替换任务对象 */
        if (g_start_request != 0U)
        {
            SAOMA_DataReady = 0U;
        }
        if (SAOMA_DataReady != 0U &&
            1U &&
            (TickType_t)(xTaskGetTickCount() - screen_start_tick) >=
            pdMS_TO_TICKS(1000U))
        {
            char scan_log[24];
            SAOMA_DataReady = 0U;
            g_scan_display_result = Screen_ShowTaskCode(SAOMA_Data);
            if (g_scan_display_result == SCREEN_OK)
            {
                g_scan_display_count++;
            }
            /* 诊断发送接受不等于PC已经收到；忙时返回0 */
            sprintf(scan_log, "SCAN:%s\r\n", SAOMA_Data);
            g_scan_log_send_ok = App_DebugSendString(scan_log);
        }
}

/* 每1毫秒扫描，只记录启动按键事件 */
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST
/* 0等按键、2等CAN就绪、3测试中、4完成、5取消/失败。
 * 本开发固件每次上电仅一次；再次按键不会重启，复位后才允许重测。 */
volatile uint32_t g_chassis_key_state = 0U;
volatile uint32_t g_chassis_key_cause = 0U; /* 1取消、2门控、3就绪超时、4执行失败、5HWT条件 */
static TickType_t s_chassis_key_tick = 0U;

#if APP_HWT_TURN_TEST
volatile uint32_t g_hwt_turn_request = 0U; /* 1开始一次实转，2取消；默认0 */
volatile float g_hwt_turn_yaw_start = 0.0f, g_hwt_turn_yaw_end = 0.0f;
volatile float g_hwt_turn_yaw_delta = 0.0f;
static float s_hwt_turn_last_yaw;
static uint32_t s_hwt_turn_angle_count;
/* 采样任务与数据均须新鲜，不把旧valid_mask当成新数据。 */
static uint32_t App_HwtTurnHealthy(void)
{
    return ((g_hwt_version_result == 1U) && (g_hwt_valid_mask == 3U) &&
        ((uint32_t)xTaskGetTickCount() - g_hwt_update_tick <= 100U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}
#endif

static void App_ChassisKeyStop(uint32_t cause)
{
    taskENTER_CRITICAL();
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    g_chassis_key_state = 5U;
    g_chassis_key_cause = cause;
    taskEXIT_CRITICAL();
}

/* 只提交请求；不等待、不调用CAN、不直接接通电源。 */
static void App_ChassisKeyStep(uint32_t event)
{
    TickType_t now = xTaskGetTickCount();
    uint32_t state = g_chassis_key_state;
    if (event != 0U)
    {
        if ((state >= 1U) && (state <= 3U))
        { App_ChassisKeyStop(1U); return; }
        if (state != 0U) return;
        if ((g_vm_test_request != 0U) || (g_rtos_fault != 0U) ||
            (g_safety_fault != 0U) ||
            (g_chassis_auto_active != 0U) || (g_chassis_auto_request != 0U) ||
            (g_chassis_auto_cut_vm != 0U) || (g_wheel_test_active != 0U) ||
            (g_wheel_test_cut_vm != 0U) || ((*AppCanDiagnostics_JogPreparing()) != 0U) ||
            (g_can_jog_request != 0U) || (App_CanJogOtherRequest() != 0U) ||
            (g_can_feedback_enable != 0U) || (g_motion_diag_fault_latched != 0U) ||
            (g_motion_auto_stop_state != MOTION_STOP_IDLE) ||
            (g_motion_auto_stop_cut_vm != 0U))
        { App_ChassisKeyStop(2U); return; }
        g_vm_test_request = 1U;
        g_chassis_key_state = 2U;
        s_chassis_key_tick = now;
        return;
    }
    if ((state == 0U) || (state >= 4U)) return;
    if ((g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
        ((state == 3U) && (g_safety_ready != 1U)) || (g_can_feedback_enable != 0U) ||
        (g_can_poll_enable != 1U) || (g_can_jog_request != 0U) ||
        (App_CanJogOtherRequest() != 0U) ||
        (g_motion_diag_hold_report != 0U) || (g_motion_diag_hold_stop_service != 0U) ||
        (g_motion_diag_fail_stop_once != 0U))
    { App_ChassisKeyStop(2U); return; }
    if (state == 2U)
    {
        if (g_vm_test_request != 1U)
        { App_ChassisKeyStop(2U); return; }
        if ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(APP_CHASSIS_KEY_READY_MS))
        { App_ChassisKeyStop(3U); return; }
        if ((g_can_test_result == 1U) && (App_CanJogReady() == 1U))
        {
            taskENTER_CRITICAL();
            g_chassis_auto_result = 0U;
            g_chassis_auto_dry_run = 0U; /* 按键明确选择实转；Watch入口默认仍为演练 */
            g_chassis_auto_request = 1U;
            g_chassis_key_state = 3U;
            s_chassis_key_tick = now;
            taskEXIT_CRITICAL();
        }
    }
    else if (state == 3U)
    {
        if ((g_chassis_auto_request == 0U) && (g_chassis_auto_active == 0U) &&
            (g_chassis_auto_result != 0U))
        {
            if (g_chassis_auto_result == 1U)
            { g_chassis_key_state = 4U; g_vm_test_request = 0U; }
            else App_ChassisKeyStop(4U);
        }
        else if ((g_vm_test_request != 1U) ||
            ((TickType_t)(now - s_chassis_key_tick) >= pdMS_TO_TICKS(50000U)))
        { App_ChassisKeyStop(4U); }
    }
}
#if APP_HWT_TURN_TEST
/* 只由Watch开始；PC13只取消活动测试。重复请求不续时、不排队。 */
static void App_HwtTurnStep(uint32_t request, uint32_t key_event)
{
    uint32_t state = g_chassis_key_state;
    if (((request == 2U) || (key_event != 0U)) && (state >= 2U) && (state <= 3U))
    { App_ChassisKeyStop(1U); return; }
    if (((state == 2U) || (state == 3U)) && !App_HwtTurnHealthy())
    { App_ChassisKeyStop(5U); return; }
    if ((request == 1U) && (state == 0U))
    {
        if (!App_HwtTurnHealthy()) { App_ChassisKeyStop(5U); return; }
        g_hwt_turn_yaw_start = g_hwt_turn_yaw_end = g_hwt_yaw_deg;
        g_hwt_turn_yaw_delta = 0.0f;
        s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
        App_ChassisKeyStep(1U);
        return;
    }
    if (((state == 2U) || (state == 3U)) &&
        (s_hwt_turn_angle_count != g_hwt_angle_count))
    {
        float delta = g_hwt_yaw_deg - s_hwt_turn_last_yaw;
        if (delta > 180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        g_hwt_turn_yaw_delta += delta;
        g_hwt_turn_yaw_end = s_hwt_turn_last_yaw = g_hwt_yaw_deg;
        s_hwt_turn_angle_count = g_hwt_angle_count;
    }
    App_ChassisKeyStep(0U);
}
/* Safety任务也独立检查，Watch直接改VM不能绕过显式测试会话。 */
void App_HwtTurnSafety(void)
{
    if ((g_chassis_key_state == 2U) || (g_chassis_key_state == 3U))
    {
        if (!App_HwtTurnHealthy()) App_ChassisKeyStop(5U);
    }
    else g_vm_test_request = 0U;
}
#endif
#endif

void AppInput_KeyStep(void)
{
#if APP_BASELINE40_ACTIVE
    if (Key_GetEvent()!=0U) {
        g_start_key_count++;
        taskENTER_CRITICAL();g_combined_test.request=2U;taskEXIT_CRITICAL();
    }
#else
        RadarZoneSwitch_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));
        RadarLift_Service((uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS),
            (g_vm_test_request==0U && g_chassis_auto_active==0U &&
             g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
        RadarServo_DebugService((g_vm_test_request==0U && g_chassis_auto_active==0U &&
            g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
/* Armed servo test consumes PC13 without starting the chassis. */
        if(VEHICLE_INTEGRATED_ENABLE)
        {if(Key_GetEvent()!=0U){taskENTER_CRITICAL();g_vehicle_key=1U;taskEXIT_CRITICAL();}}
        else if (VEHICLE_STAGE1_ENABLE)
        {
            if(Key_GetEvent()!=0U) {
                g_start_key_count++;taskENTER_CRITICAL();
                if(g_stage1_state==1U || g_stage1_state==2U || g_stage1_state==6U || g_stage1_state==7U) {
                    g_stage1_key_request=2U;
                    if(g_stage1_state==7U) StartupAxes_Cancel();
                    else if(g_stage1_state==1U || g_stage1_state==6U) g_combined_test.request=2U;
                } else g_stage1_key_request=1U;
                taskEXIT_CRITICAL();
            }
        }
        else if (g_radar_servo_key_test_enable != 0U)
        {
            if (Key_GetEvent() != 0U)
            {
                g_start_key_count++;
                taskENTER_CRITICAL();
                (void)RadarServo_KeyTest(g_radar_zone_ready?g_radar_zone_selected:0U,
                    (g_vm_test_request==0U && g_chassis_auto_active==0U &&
                     g_safety_fault==0U && g_rtos_fault==0U && g_can_ctrl_fault_latched==0U)?1U:0U);
                taskEXIT_CRITICAL();
            }
        }
        else if (APP_UART_LINK_TEST != 0U)
        { if (Key_GetEvent() != 0U) g_start_key_count++; }
        else
        {
#if APP_IS_COMPETITION
        if (Key_GetEvent()!=0U)
        {
            g_start_key_count++;
            taskENTER_CRITICAL();
            if (g_start_gate_ready && !g_start_request &&
                g_competition_entry.state==COMP_ENTRY_IDLE)
            { s_start_pending_revision=s_start_gate_revision; s_start_pending=1U; }
            else g_start_deny_count++;
            taskEXIT_CRITICAL();
        }
#elif APP_HEADING_SESSION_TEST
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* PC13只取消，不用按键启动本次开发试验。 */
            taskENTER_CRITICAL();
#if APP_TEST_MODE == 3U
            g_combined_test.request = 2U;
#else
            if (APP_FORWARD_HOLD_TEST != 0U)
            {
                if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
                    (APP_SERIAL_MOTION_CONTROL && (g_forward_hold.request == 1U)))
                    g_forward_hold.request = 2U;
            }
            else if ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN))
                g_heading_exec_request = 2U;
#endif
            taskEXIT_CRITICAL();
        }
#elif APP_HWT_STATIC_TEST
        if (Key_GetEvent() != 0U) g_start_key_count++;
#elif APP_HWT_TURN_TEST
        {
            uint32_t event = Key_GetEvent();
            uint32_t request = g_hwt_turn_request;
            g_hwt_turn_request = 0U;
            if (event != 0U) g_start_key_count++;
            App_HwtTurnStep(request, event);
        }
#elif APP_CHASSIS_KEY_TEST_ENABLE
        {
            uint32_t event = Key_GetEvent();
            if (event != 0U) g_start_key_count++;
            App_ChassisKeyStep(event);
        }
#else
        if (Key_GetEvent() != 0U)
        {
            g_start_key_count++;
            /* 未就绪的按键当场拒绝，不留到扫码后自动启动 */
            taskENTER_CRITICAL();
            if (g_start_gate_ready != 0U && g_start_request == 0U)
            {
                s_start_pending_revision = s_start_gate_revision;
                s_start_pending = 1U;
            }
            else
            {
                g_start_deny_count++;
            }
            taskEXIT_CRITICAL();
        }

#endif
        }
#endif
}
