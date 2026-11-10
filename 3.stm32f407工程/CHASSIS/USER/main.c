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
#include "app_motion_runtime.h"
#include "app_runtime_route.h"
#include "app_serial_internal.h"
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

/* 临时静态供电测试：0关闭，1请求开启 */
volatile uint32_t g_vm_test_request = 0U;
/* 测试状态，可在Watch中观察 */
volatile uint32_t g_test_count_a = 0U;
volatile uint32_t g_test_count_b = 0U;
volatile TickType_t g_test_tick = 0U;
volatile BaseType_t g_test_create_a = pdFAIL;
volatile BaseType_t g_test_create_b = pdFAIL;
volatile uint32_t g_test_scheduler_returned = 0U;
/* 1表示发送请求被接受，不代表电脑已经收到 */
volatile uint32_t g_serial_send_ok = 0U;
/* 已处理的接收字节数与最后一个字节 */
volatile uint32_t g_serial_rx_count = 0U;
volatile uint32_t g_serial_rx_last = 0U;
/* 接收任务创建结果、栈余量及本轮序列错误数 */
volatile BaseType_t g_rx_task_create = pdFAIL;
volatile UBaseType_t g_rx_stack_words = 0U;
/* 启动按键事件数、任务创建结果及栈余量 */
volatile uint32_t g_start_key_count = 0U;
volatile BaseType_t g_key_task_create = pdFAIL;
volatile UBaseType_t g_key_stack_words = 0U;
/* 看门狗初始化结果，成功后才启动应用任务 */
volatile uint32_t g_watchdog_init_result = 0U;
/* 本步只显示实际扫码结果，不自动显示固定任务码 */
volatile uint32_t g_stage1_stop_requested=0U;
volatile uint32_t g_stage1_motion_skipped=0U;
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

/* CAN独立采集任务的创建结果、循环次数和历史最小余栈 */
volatile BaseType_t g_can_task_create = pdFAIL;
volatile uint32_t g_can_task_count = 0U;
volatile UBaseType_t g_can_task_stack_words = 0U;

/* 监督任务创建结果及历史最小剩余栈 */
volatile BaseType_t g_safety_task_create = pdFAIL;
volatile UBaseType_t g_safety_stack_words = 0U;

/* 栈余量单位为字，每字4字节 */
volatile UBaseType_t g_stack_a_words = 0U;
volatile UBaseType_t g_stack_b_words = 0U;
/* 堆余量单位为字节 */
volatile size_t g_heap_free_bytes = 0U;
volatile size_t g_heap_min_bytes = 0U;

/* 两个任务共用调试发送缓冲区，短临界区内只复制，不等待串口 */
uint8_t App_DebugSendString(const char *text)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = USART1_SendString(text);
    taskEXIT_CRITICAL();
    return result;
}
/* 只有任务A调用；先检查当前平台，再确认按键对应的扫码版本 */
static void App_StartGateUpdate(void)
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
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 以下仅显示任务间数据，不作为运动允许信号。 */
volatile uint32_t g_heading_bus_valid = 0U;
volatile int32_t g_heading_bus_rpm = 0;
volatile uint32_t g_heading_bus_stop = 1U;
volatile uint32_t g_heading_bus_sequence = 0U;
volatile uint32_t g_heading_publish_result = 0U;
#endif

#if APP_HEADING_SESSION_TEST
/* 只允许CAN任务操作会话对象；Safety只锁存trip并撤销执行。 */
#if APP_SERIAL_MOTION_CONTROL
volatile uint32_t g_serial_motion_state = 0U, g_serial_motion_result = 0U;
volatile uint32_t g_serial_motion_rejected = 0U, g_serial_motion_accepted = 0U;
volatile uint32_t g_serial_motion_link_lost = 0U;
static uint32_t s_serial_token_seq, s_serial_token_tick, s_serial_arm_ms;
static uint32_t s_serial_keepalive_ms;
static uint32_t s_serial_tx_seq, s_serial_dropped, s_serial_tx_tick;
static uint32_t s_serial_tx_valid; /* 至少一条当前状态已排入串口 */
static uint32_t s_serial_proto_ok;
static const AppSerialObservation s_serial_observation = {
    &s_serial_token_seq,&s_serial_token_tick,&s_serial_arm_ms,&s_serial_tx_valid,&s_serial_tx_seq
};
const AppSerialObservation *AppSerial_Live(void) { return &s_serial_observation; }
void AppSerial_ResetToken(void)
{ s_serial_token_seq=s_serial_token_tick=s_serial_arm_ms=s_serial_keepalive_ms=0U; }
void AppSerial_SetKeepalive(uint32_t now) { s_serial_keepalive_ms=now; }
static void App_SerialMotionSafety(void);
uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t earliest_ms);
#endif

/* 每100ms记录运行期提交值，非实际轮速；容量覆盖8秒，满后不覆盖。 */


/* trip:1取消、2源失效、3平台、4上电等待、5执行器、6偏角、7截止、8提交。 */


#if APP_SERIAL_MOTION_CONTROL
/* RC1/ TM1：CRC16-CCITT，ARM/START两步，只启动既有单次平移会话。 */
uint16_t App_SerialCrc(const char *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1));
    }
    return crc;
}
static int32_t App_SerialMilli(float value)
{
    if (!(value >= -2000.0f && value <= 2000.0f)) return (-2147483647L - 1L);
    return (int32_t)(value * 1000.0f);
}
/* 距离独立量纲范围，支持2100mm及原50mm越界诊断；不扩大角度范围。 */
int32_t App_SerialDistanceMilli(float value)
{
    if (!(value>=-5.0f && value<=2500.0f)) return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static uint32_t App_SerialUInt(const char **text, uint32_t *out)
{
    uint32_t n = 0U, digit;
    const char *p = *text;
    if (*p < '0' || *p > '9') return 0U;
    while (*p >= '0' && *p <= '9')
    {
        digit = (uint32_t)(*p - '0');
        if (n > (0xFFFFFFFFU - digit) / 10U) return 0U;
        n = n * 10U + digit; p++;
    }
    if (*p != ',') return 0U;
    *text = p + 1; *out = n; return 1U;
}
uint32_t App_SerialIdleReady(uint32_t now)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)now;return 0U;
#else
    AppHeadingSource heading;
    ChassisAutoStatus status;
    PowerCommandStatus power;
    if(!CAN1_EmmSettingsConfirmed())return 0U;
    (void)ChassisAuto_ReadStatus(&status);
    Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if ((g_combined_test.state != 0U) || (g_combined_test.request != 0U) ||
        (g_motion_plan_valid != 1U)) return 0U;
#endif
    (void)App_HeadingSourceRead(&heading);
    return ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 0U) &&
        (g_vm_test_request == 0U) && ((power.active == 0U) || StartupAxes_Holding()) &&
        (status.active == 0U) && (status.result == 0U) && (status.cut_vm == 0U) &&
        (g_translation_hold_selftest == 1U) && (s_serial_proto_ok == 1U) &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        (g_can_ctrl_fault_latched == 0U) &&
        (heading.rate_dps >= -3.0f && heading.rate_dps <= 3.0f)) ? 1U : 0U;
#endif
}
/* 调用者持锁；只容忍成功终态的晚到KEEP，不续期、不重新发车。 */
uint32_t App_SerialCompletedStopped(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    if (g_serial_motion_state != 3U || g_serial_motion_result != 2U) return 0U;
#if APP_TEST_MODE == 3U
    if (g_combined_test.state != 3U || g_combined_test.cause ||
        g_combined_test.completed != (*AppMotion_Live()->action_count)) return 0U;
#else
    if (g_forward_hold.phase != HR_DONE || g_forward_hold.trip) return 0U;
#endif
    Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
    return (!g_vm_test_request && !power.active && !status.active &&
        status.result == 1U && !status.cause && status.rotation_stop_mask == 15U &&
        !g_can_ctrl_fault_latched && !g_rtos_fault && !g_safety_fault) ? 1U : 0U;
}
static void App_SerialReject(uint32_t reason)
{ g_serial_motion_rejected++; g_serial_motion_result = reason; }


static void App_SerialMotionCommand(char *line)
{
    static const volatile uint32_t source_parameter_entry=VEHICLE_INTEGRATED_ENABLE;
    if(source_parameter_entry)return;

    if(FieldMap_LocalMode()) { App_SerialReject(9U);return; }
    if(strncmp(line,"RQ1,",4U)==0) { App_RuntimeRouteCommand(line);return; }
    char *last = strrchr(line, ',');
    const char *p;
    uint32_t seq, tick, crc = 0U, i, digit, now;
    if ((strncmp(line, "RC1,", 4U) != 0) || (last == NULL) || strlen(last + 1) != 4U)
    { App_SerialReject(4U); return; }
    for (i = 1U; i <= 4U; i++)
    {
        char c = last[i];
        if (c >= '0' && c <= '9') digit = (uint32_t)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (uint32_t)(c - 'A') + 10U;
        else { App_SerialReject(4U); return; }
        crc = (crc << 4) | digit;
    }
    if ((uint32_t)App_SerialCrc(line, (uint32_t)(last - line + 1)) != crc)
    { App_SerialReject(4U); return; }
    *last = '\0'; p = line + 4;
    if (!App_SerialUInt(&p, &seq) || !App_SerialUInt(&p, &tick))
    { App_SerialReject(4U); return; }
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if (strcmp(p, "STOP") == 0)
    {
        AppMotion_SetStartPermit(0U);
#if APP_TEST_MODE == 3U
        App_CombinedTestStop(1U);
#else
        if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
            (g_forward_hold.request == 1U)) g_forward_hold.request = 2U;
#endif
        g_serial_motion_state = 3U; g_serial_motion_result = 3U;
        g_serial_motion_accepted++;
    }
#if APP_TEST_MODE == 3U
    else if (strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<=('0'+DEBUG_PLAN_MAX) && p[5]=='\0')
    {
        if (!s_serial_tx_seq || (uint32_t)(s_serial_tx_seq-1U-seq)>5U ||
            (uint32_t)(now-tick)>=500U) App_SerialReject(5U);
        else if (g_serial_motion_state || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->committed.id || !App_SerialIdleReady(now) ||
                 !App_DebugSelectPlan((uint32_t)(p[4]-'0'))) App_SerialReject(6U);
        else g_serial_motion_accepted++;
    }
#endif
    else if (strcmp(p, "ARM") == 0)
    {
        if ((s_serial_tx_seq == 0U) || ((uint32_t)(s_serial_tx_seq - 1U - seq) > 5U) ||
            ((uint32_t)(now - tick) >= 500U)) App_SerialReject(5U);
        else if ((g_serial_motion_state != 0U) || !App_SerialIdleReady(now) ||
                 !AppRuntimeRoute_Live()->ready || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->consumed) App_SerialReject(6U);
        else
        {
            s_serial_token_seq = seq; s_serial_token_tick = tick; s_serial_arm_ms = now;
            g_serial_motion_state = 1U; g_serial_motion_result = 1U;
            g_serial_motion_accepted++;
        }
    }
    else if ((seq != s_serial_token_seq) || (tick != s_serial_token_tick)) App_SerialReject(5U);
    else if (strcmp(p, "START") == 0)
    {
#if APP_TEST_MODE == 3U
        App_SerialReject(6U);
#else
        if (g_serial_motion_state == 2U) { /* 重复帧只确认，不重新启动或续期。 */ }
        else if ((g_serial_motion_state != 1U) || ((uint32_t)(now - s_serial_arm_ms) >= 2000U) ||
            !App_SerialIdleReady(now)) App_SerialReject(6U);
        else
        {
            AppMotion_SetStartPermit(1U); s_serial_keepalive_ms = now;
#if APP_TEST_MODE == 3U
            g_combined_test.request = 1U;
#else
            g_forward_hold.request = 1U;
#endif
            g_serial_motion_state = 2U; g_serial_motion_result = 2U;
            g_serial_motion_accepted++;
        }
#endif
    }
    else if (strcmp(p, "KEEP") == 0)
    {
        if (g_serial_motion_state == 2U)
        { s_serial_keepalive_ms = now; g_serial_motion_accepted++; }
        else if (App_SerialCompletedStopped())
        { g_serial_motion_accepted++; } /* 同令牌已在上方核对；终态只确认，不续期。 */
        else App_SerialReject(6U);
    }
    else App_SerialReject(4U);
    taskEXIT_CRITICAL();
}
static void App_SerialMotionByte(uint8_t data)
{
    static char line[RQ_LINE_MAX];
    static uint32_t used, invalid, last_ms;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - last_ms) >= 200U) { used = 0U; invalid = 0U; }
    last_ms = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if (!invalid && used) { line[used] = '\0'; App_SerialMotionCommand(line); }
        used = invalid = 0U; return;
    }
    if (invalid) return;
    if (data < 32U || data > 126U || used >= sizeof(line) - 1U)
    { invalid = 1U; App_SerialReject(4U); return; }
    line[used++] = (char)data;
}
static void App_SerialMotionSafety(void)
{
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if ((g_serial_motion_state == 1U) && ((uint32_t)(now - s_serial_arm_ms) >= 2000U))
    { g_serial_motion_state = 0U; g_serial_motion_result = 7U; AppMotion_SetStartPermit(0U); }
    if (g_serial_motion_state == 2U)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state >= 3U) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            AppMotion_SetStartPermit(0U); g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_CombinedTestStop(1U);
        }
#else
        if ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 2U))
        {
            AppMotion_SetStartPermit(0U);
            g_serial_motion_state = 3U; g_serial_motion_result = 3U;
            App_ForwardHoldTrip(1U);
        }
        else if (g_forward_hold.phase >= HR_DONE) g_serial_motion_state = 3U;
        else if (!VEHICLE_STAGE1_ENABLE && (uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            AppMotion_SetStartPermit(0U); g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_ForwardHoldTrip(1U);
        }
#endif
    }
    taskEXIT_CRITICAL();
}
/* 当前状态优先：忙时下一轮重试，只有排队成功才推进100ms周期。
 * 历史诊断不得在状态已到期时抢占单缓冲；不改变主机超时或运动保护。 */
#define APP_SERIAL_STATUS_PERIOD_MS 100U
static uint32_t App_SerialStatusDue(uint32_t now)
{
    return !s_serial_tx_valid ||
        (uint32_t)(now - s_serial_tx_tick) >= APP_SERIAL_STATUS_PERIOD_MS;
}
uint32_t App_SerialDiagnosticWindow(uint32_t now, uint32_t earliest_ms)
{
    return s_serial_tx_valid && !App_SerialStatusDue(now) &&
        (uint32_t)(now - s_serial_tx_tick) >= earliest_ms;
}
static void App_SerialStatusSend(const char *frame, uint32_t now)
{
    if (App_DebugSendString(frame) != 1U) { s_serial_dropped++; return; }
    s_serial_tx_tick = now;
    s_serial_tx_valid = 1U;
}
#if APP_POSITION_DIAGNOSTIC
/* PE2是带方向的完整历史起止记录，按索引只排队成功发送一次；不当作实时有效位。 */
static void App_PositionEndpointTelemetry(void)
{
    static uint32_t index;
    static char frame[512];
    ChassisPositionEndpoints endpoint;
    uint32_t i, used, phase;
    int count;
    uint16_t crc;
    if (!App_SerialDiagnosticWindow((uint32_t)xTaskGetTickCount(), 40U)) return;
    if (!ChassisAuto_ReadPositionEndpoints(index, &endpoint)) return;
    if (endpoint.outcome == 2U) { index++; return; }
    if (endpoint.outcome != 1U || endpoint.start_mask != 15U || endpoint.end_mask != 15U) return;
    count = snprintf(frame, sizeof(frame), "PE3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)xTaskGetTickCount(), (unsigned long)endpoint.session,
        (unsigned long)endpoint.outcome, (unsigned long)endpoint.start_mask,
        (unsigned long)endpoint.end_mask, (unsigned long)endpoint.translation_mode,
        (unsigned long)endpoint.distance_mode);
    if (count < 0 || (uint32_t)count >= sizeof(frame)) return;
    used = (uint32_t)count;
    for (phase = 0U; phase < 2U; phase++)
        for (i = 0U; i < 4U; i++)
        {
            count = snprintf(frame + used, sizeof(frame) - used, "%lu,%lu,%lu,",
                (unsigned long)(phase ? endpoint.end_sign[i] : endpoint.start_sign[i]),
                (unsigned long)(phase ? endpoint.end_raw[i] : endpoint.start_raw[i]),
                (unsigned long)(phase ? endpoint.end_ms[i] : endpoint.start_ms[i]));
            if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
            used += (uint32_t)count;
        }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    if (App_DebugSendString(frame) == 1U) index++;
}

#if APP_HEADING_SESSION_TEST
/* OD1是只读诊断，1Hz尽力发送；忙则跳过，不参与启动/KEEP判定。 */
static void App_PerimeterTelemetry(void)
{
    static uint32_t cursor,last;static char frame[256];
    PerimeterRecord record;uint32_t now=(uint32_t)xTaskGetTickCount();int n;uint16_t crc;
    if(g_combined_test.state<3U || cursor>8U || now-last<100U ||
       !App_SerialDiagnosticWindow(now,80U)) return;
    taskENTER_CRITICAL();record=g_perimeter_records[cursor];taskEXIT_CRITICAL();
    n=snprintf(frame,sizeof(frame),"OP1,%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%ld,%ld,%ld,",
       (unsigned long)cursor,(unsigned long)record.captured,(unsigned long)record.tick_ms,
       (long)record.pose.x_mm,(long)record.pose.y_mm,(long)(record.pose.heading_deg*1000.0f),
       (unsigned long)record.pose.valid,(unsigned long)record.pose.reasons,
       (unsigned long)record.pose.continuity_lost,(unsigned long)record.pose.age_ms,
       (long)AppMotion_Live()->perimeter_targets[cursor][0],(long)AppMotion_Live()->perimeter_targets[cursor][1],
       (long)(AppMotion_Live()->perimeter_targets[cursor][2]*1000.0f));
    if(n<0 || (uint32_t)n>=sizeof(frame)-7U) return;
    crc=App_SerialCrc(frame,(uint32_t)n);
    (void)snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U){cursor++;last=now;}
}

static void App_OdometryTelemetry(void)
{
    static char frame[192];
    static uint32_t last;
    ChassisOdometryStatus status;
    uint32_t now=(uint32_t)xTaskGetTickCount();
    int count;
    uint16_t crc;
    if(now-last<1000U || !App_SerialDiagnosticWindow(now,80U)) return;
    taskENTER_CRITICAL(); status=g_chassis_odometry; taskEXIT_CRITICAL();
    count=snprintf(frame,sizeof(frame),"OD1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%lu,",
        (unsigned long)now,(unsigned long)status.sample_ms,(unsigned long)status.updates,
        (unsigned long)status.valid,(unsigned long)status.reasons,
        (unsigned long)status.continuity_lost,(unsigned long)status.local_origin,
        (long)status.x_mm,(long)status.y_mm,(long)(status.heading_deg*1000.0f),
        (unsigned long)status.age_ms);
    if(count<0 || (uint32_t)count>=sizeof(frame)-7U) return;
    crc=App_SerialCrc(frame,(uint32_t)count);
    (void)snprintf(frame+count,sizeof(frame)-(uint32_t)count,"%04X\r\n",crc);
    if(App_DebugSendString(frame)==1U) last=now;
}
#endif
/* PM1与TM1错开至少40ms，静态缓冲不叠加TaskA栈；忙时丢诊断帧。 */
/* WF1：补齐系统辨识需要的应答目标/真实轮速/样本时刻；低优先级观察帧。 */
static void App_WheelLogTelemetry(void)
{
#if VEHICLE_WHEEL_LOG_ENABLE
    static char frame[512];static ChassisWheelLog sample;
    static uint32_t last,last_active,seen_active,sequence,dropped;
    uint32_t now=(uint32_t)xTaskGetTickCount(),i,used;int count;uint16_t crc;
    if (g_chassis_auto_active) {last_active=now;seen_active=1U;}
    if (!seen_active || (!g_chassis_auto_active && (uint32_t)(now-last_active)>1000U) ||
        (uint32_t)(now-last)<VEHICLE_WHEEL_LOG_PERIOD_MS ||
        !App_SerialDiagnosticWindow(now,40U)) return;
    last=now;(void)ChassisAuto_ReadWheelLog(&sample);
    count=snprintf(frame,sizeof(frame),"WF1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)sample.tick_ms,(unsigned long)sample.session,
        (unsigned long)sample.active,(unsigned long)sample.state,(unsigned long)sample.dry_run,
        (unsigned long)sample.speed_valid_mask,(unsigned long)sample.ack_valid_mask,(unsigned long)dropped);
    if(count<0 || (uint32_t)count>=sizeof(frame)) {dropped++;return;}
    used=(uint32_t)count;
    for(i=0U;i<4U;i++) {
        count=snprintf(frame+used,sizeof(frame)-used,"%ld,%ld,%lu,%lu,%lu,",
            (long)sample.acknowledged_decirpm[i],(long)sample.measured_decirpm[i],
            (unsigned long)sample.ack_ms[i],(unsigned long)sample.sample_ms[i],(unsigned long)sample.age_ms[i]);
        if(count<0 || (uint32_t)count>=sizeof(frame)-used) {dropped++;return;}
        used+=(uint32_t)count;
    }
    crc=App_SerialCrc(frame,used);count=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(count<0 || (uint32_t)count>=sizeof(frame)-used || !App_DebugSendString(frame)) dropped++;
#endif
}

static void App_PositionTelemetry(void)
{
    static char frame[512];
    static uint32_t last;
    ChassisPositionSnapshot position;
    uint32_t now = (uint32_t)xTaskGetTickCount(), used, i;
    int count;
    uint16_t crc;
    if ((uint32_t)(now - last) < 200U ||
        !App_SerialDiagnosticWindow(now, 40U)) return;
    last = now;
    (void)ChassisAuto_ReadPositions(&position);
    count = snprintf(frame, sizeof(frame), "PM1,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)now, (unsigned long)position.session,
        (unsigned long)position.sequence, (unsigned long)position.valid_mask,
        (unsigned long)position.vm_active);
    if (count < 0 || (uint32_t)count >= sizeof(frame)) return;
    used = (uint32_t)count;
    for (i = 0U; i < 4U; i++)
    {
        count = snprintf(frame + used, sizeof(frame) - used, "%lu,%lu,%lu,%lu,%lu,",
            (unsigned long)position.result[i], (unsigned long)position.sign[i],
            (unsigned long)position.raw[i], (unsigned long)position.sample_ms[i],
            (unsigned long)position.age_ms[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    (void)App_DebugSendString(frame);
}
#endif


/* BT1仅在VM请求/输出关闭且后端不活动时逐条导出，不在运动中格式化。 */
static void App_BatchDiagnosticTelemetry(void)
{
#if APP_IS_DEBUG
    static char frame[448];
    static ChassisBatchDiagnostic record;
    static ChassisBatchStatistics statistics;
    static uint32_t cursor, last, values[35];
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    if (g_vm_test_request || g_chassis_auto_active ||
        (uint32_t)(now - last) < 100U || !App_SerialDiagnosticWindow(now, 70U)) return;
    Power_ReadCommandStatus(&power);
    if (power.active) return;
    if (!ChassisAuto_ReadBatchDiagnostic(cursor, &record, &statistics))
    {
        if ((uint32_t)(now-last) >= 1000U) cursor = 0U;
        return;
    }
    values[0] = (uint32_t)record.session;
    values[1] = (uint32_t)record.batch_id;
    values[2] = (uint32_t)record.segment;
    values[3] = (uint32_t)record.dry_run;
    values[4] = (uint32_t)record.generation_source;
    values[5] = (uint32_t)record.generated_ms;
    values[6] = (uint32_t)record.start_ms;
    values[7] = (uint32_t)record.end_ms;
    values[8] = (uint32_t)record.outcome;
    values[9] = (uint32_t)record.sent_mask;
    values[10] = (uint32_t)record.ack_mask;
    values[11] = (uint32_t)record.failed_address;
    values[12] = (uint32_t)record.io_result;
    values[13] = (uint32_t)FineRpm_Deci(record.correction_rpm);
    values[14] = (uint32_t)record.base_rpm;
    values[15] = (uint32_t)record.pending_changes;
    values[16] = (uint32_t)FineRpm_Deci(record.pending_rpm);
    values[17] = (uint32_t)record.send_ms[0];
    values[18] = (uint32_t)record.send_ms[1];
    values[19] = (uint32_t)record.send_ms[2];
    values[20] = (uint32_t)record.send_ms[3];
    values[21] = (uint32_t)record.reply_ms[0];
    values[22] = (uint32_t)record.reply_ms[1];
    values[23] = (uint32_t)record.reply_ms[2];
    values[24] = (uint32_t)record.reply_ms[3];
    values[25] = (uint32_t)record.result[0];
    values[26] = (uint32_t)record.result[1];
    values[27] = (uint32_t)record.result[2];
    values[28] = (uint32_t)record.result[3];
    values[29] = statistics.overwritten;
    values[30] = statistics.max_queue_ms;
    values[31] = statistics.max_batch_ms;
    values[32] = statistics.max_call_ms;
    values[33] = statistics.total_success;
    values[34] = statistics.total_interrupted;
    memcpy(frame, "BTQ,", 4U);
    for (i = 0U; i < 35U; i++)
    {
        if (i == 13U || i == 16U)
            count = snprintf(frame+used, sizeof(frame)-used, "%ld,", (long)(int32_t)values[i]);
        else count = snprintf(frame+used, sizeof(frame)-used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame+used, sizeof(frame)-used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame)-used) return;
    if (App_DebugSendString(frame) == 1U) { cursor++; last = now; }
#endif
}


static void App_ChassisLoadTelemetry(void)
{
    static uint32_t cursor=0U,session=0U,last=0U;
    static char frame[512];
    ChassisLoadRecord record;ChassisLoadStatistics stats;PowerCommandStatus power;
    uint32_t now=xTaskGetTickCount(),n,i,j,used;int count;uint16_t crc;
    const uint8_t *bytes;static const char hex[]="0123456789ABCDEF";
    Power_ReadCommandStatus(&power);
    if(g_chassis_auto_active || power.active || g_vm_test_request || (uint32_t)(now-last)<70U ||
       !App_SerialDiagnosticWindow(now,40U))return;
    if(!ChassisLoad_Read(0U,&record,&stats))return;
    if(session!=stats.session){session=stats.session;cursor=0U;}
    if(cursor>=stats.count)return;
    n=stats.count-cursor;if(n>8U)n=8U;
    count=snprintf(frame,sizeof(frame),"XD1,%lu,%lu,%lu,%lu,%lu,",(unsigned long)session,
        (unsigned long)cursor,(unsigned long)n,(unsigned long)stats.count,(unsigned long)stats.dropped);
    if(count<0 || (uint32_t)count>=sizeof(frame))return;used=(uint32_t)count;
    for(i=0U;i<n;i++){
        if(!ChassisLoad_Read(cursor+i,&record,&stats))return;
        bytes=(const uint8_t*)&record;
        for(j=0U;j<sizeof(record);j++){if(used+2U>=sizeof(frame)-8U)return;frame[used++]=hex[bytes[j]>>4];frame[used++]=hex[bytes[j]&15U];}
    }
    frame[used++]=',';crc=App_SerialCrc(frame,used);
    count=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(count<0 || (uint32_t)count>=sizeof(frame)-used)return;
    if(App_DebugSendString(frame)==1U){cursor+=n;last=now;}
}

static void App_SerialMotionTelemetry(void)
{
    AppHeadingSource heading;
    static int32_t values[(APP_TEST_MODE == 3U) ? 66U : 49U];
    static char frame[768];
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    PowerCommandStatus power;
    ChassisAutoStatus status;
    if (!App_SerialStatusDue(now)) return;
    taskENTER_CRITICAL();
    (void)App_HeadingSourceRead(&heading);
    Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
    values[0] = (int32_t)(s_serial_tx_seq++); /* seq */
    values[1] = (int32_t)(now); /* tick_ms */
    values[2] = (int32_t)(g_forward_hold.phase); /* phase */
    values[3] = (int32_t)(g_forward_hold.trip); /* trip */
    values[4] = (int32_t)(g_forward_hold.elapsed_ms); /* elapsed_ms */
    values[5] = (int32_t)(App_HeadingRuntimeSource(now)); /* source_valid */
    values[6] = (int32_t)(App_SerialMilli(heading.yaw_deg)); /* yaw_mdeg */
    values[7] = (int32_t)(App_SerialMilli(g_forward_hold.target_deg)); /* target_mdeg */
    values[8] = (int32_t)(App_SerialMilli(g_forward_hold.error_deg)); /* error_mdeg */
    values[9] = (int32_t)(App_SerialMilli(heading.rate_dps)); /* rate_mdps */
    values[10] = (int32_t)(App_HeadingFrameAge(&heading,now,1U)); /* angle_age_ms */
    values[11] = (int32_t)(App_HeadingFrameAge(&heading,now,2U)); /* rate_age_ms */
    values[12] = FineRpm_Deci(g_forward_hold.correction_rpm); /* correction_rpm */
    values[13] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[0]); /* rpm1 */
    values[14] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[1]); /* rpm2 */
    values[15] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[2]); /* rpm3 */
    values[16] = ZdtEmm_RpmDeci(g_chassis_auto_driver_rpm[3]); /* rpm4 */
    values[17] = (int32_t)(g_vm_test_request); /* vm_request */
    values[18] = (int32_t)(power.active); /* pe2_active */
    values[19] = (int32_t)(status.state); /* backend_state */
    values[20] = (int32_t)(status.active); /* backend_active */
    values[21] = (int32_t)(status.result); /* backend_result */
    values[22] = (int32_t)(status.cause); /* backend_cause */
    values[23] = (int32_t)(status.rotation_stop_mask); /* stop_mask */
    values[24] = (int32_t)(g_can_ctrl_fault_latched); /* can_fault */
    values[25] = (int32_t)(g_rtos_fault); /* rtos_fault */
    values[26] = (int32_t)(g_safety_fault); /* safety_fault */
    values[27] = (int32_t)(g_forward_hold.motion_mode); /* motion_mode */
    values[28] = (int32_t)(g_forward_hold.correction_nonzero_samples); /* nonzero_samples */
    values[29] = FineRpm_Deci(g_forward_hold.correction_peak_abs_rpm); /* peak_abs_rpm */
    values[30] = FineRpm_Deci(g_forward_hold.correction_last_nonzero_rpm); /* last_nonzero_rpm */
    values[31] = (int32_t)(g_serial_motion_state); /* remote_state */
    values[32] = (int32_t)(g_serial_motion_result); /* remote_result */
    values[33] = (int32_t)(s_serial_token_seq); /* token_seq */
    values[34] = (int32_t)(s_serial_token_tick); /* token_tick */
    values[35] = (int32_t)(s_serial_dropped); /* telemetry_dropped */
    values[36] = (int32_t)(g_serial_motion_rejected); /* remote_rejected */
    values[37] = (int32_t)(g_translation_hold_selftest); /* selftest */
    values[38] = (int32_t)((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_BASE_RPM : APP_LATERAL_HOLD_BASE_RPM); /* base_rpm */
    values[39] = (int32_t)((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM : APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM); /* feedback_limit_rpm */
    values[40] = (int32_t)(App_SerialMilli(g_forward_hold.peak_error_deg)); /* peak_error_mdeg */
    values[41] = (int32_t)(g_forward_hold.max_sample_gap_ms); /* max_sample_gap_ms */
    values[42] = (int32_t)(g_uart1_rx_errors); /* uart_rx_errors */
    values[43] = (int32_t)(g_rx_stack_words); /* rx_stack_words */
    values[44] = (int32_t)(g_heap_min_bytes); /* heap_min_bytes */
    values[45] = (int32_t)(g_serial_motion_link_lost); /* link_lost */
    values[46] = (int32_t)(s_serial_proto_ok); /* protocol_selftest */
    values[47] = (int32_t)(g_serial_motion_accepted); /* remote_accepted */
#if APP_TEST_MODE == 3U
    /* 整组阶段独立于单段DONE，避免电脑在第一段完成时误判整组完成。 */
    values[2] = (g_combined_test.state == 0U) ? 0 :
        ((g_combined_test.state >= 3U) ? (int32_t)g_combined_test.state : 2);
    values[3] = (int32_t)g_combined_test.cause;
    values[4] = (g_combined_test.state == 0U) ? 0 : (int32_t)(((g_combined_test.state >= 3U) ? (*AppMotion_Live()->end_ms) : now) - (*AppMotion_Live()->begin_ms));
    values[7] = App_SerialMilli((*AppMotion_Live()->turn) ? g_heading_exec_target_deg : g_forward_hold.target_deg);
    values[8] = App_SerialMilli((*AppMotion_Live()->turn) ? g_heading_exec_error_deg : g_forward_hold.error_deg);
    values[12] = FineRpm_Deci((*AppMotion_Live()->turn) ? g_heading_exec_rpm : g_forward_hold.correction_rpm);
    values[27] = 3; /* 编译为组合测试，主机必须显式--mode 3 */
    values[38] = (int32_t)((g_combined_test.state == 0U) ?
        ((AppMotion_Live()->actions[0].kind == MOTION_TURN) ? 15U : AppMotion_Live()->actions[0].base_rpm) :
        ((*AppMotion_Live()->turn) ? 15U : (*AppMotion_Live()->base_rpm)));
    if (APP_SPEED20_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_FORWARD210_20)
        values[39]=(int32_t)APP_SPEED20_FEEDBACK_RPM;
    if(APP_ROUTE_SPEED_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT)
        values[39]=(int32_t)ROUTE_TRIAL_FEEDBACK_RPM;
    if(APP_TURN_BRAKE_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
       ((g_combined_test.state==0U && AppMotion_Live()->actions[0].kind==MOTION_TURN) || (*AppMotion_Live()->turn))) {
        values[38]=(int32_t)APP_TURN_BRAKE_MAX_RPM;
        values[39]=(int32_t)APP_TURN_BRAKE_FEEDBACK_RPM;
    }
    values[48] = (int32_t)g_combined_test.state;
    values[49] = (int32_t)g_combined_test.index;
    values[50] = (int32_t)g_combined_test.completed;
    values[51] = (int32_t)g_can_guard_cause;
    values[52] = (int32_t)g_can_guard_bad_nodes;
    values[53] = (int32_t)g_heading_power_wait_state;
    values[54] = (int32_t)g_heading_power_post_settle_mask;
    values[55] = (int32_t)(*AppMotion_Live()->action_count);
    values[58] = (int32_t)status.distance_mode;
    values[59] = App_SerialDistanceMilli(status.distance_target_mm);
    values[60] = App_SerialDistanceMilli(status.distance_progress_mm);
    values[61] = (int32_t)status.distance_base_rpm;
    values[62] = (int32_t)status.distance_reached;
    values[63] = (int32_t)status.distance_fault_reason;

    values[56] = (int32_t)((*AppMotion_Live()->turn) ? MOTION_TURN :
        (((*AppMotion_Live()->distance_mm) > 0.0f) ? (MOTION_FORWARD_DISTANCE + (*AppMotion_Live()->mode)) : (*AppMotion_Live()->mode)));
    if (status.distance_mode && (status.distance_base_rpm >= 3U || status.distance_profile_mode))
        values[38] = (int32_t)status.distance_base_rpm;
    values[57] = (*AppMotion_Live()->turn) ? (-2147483647L - 1L) :
        App_SerialMilli(MotionRpmToCmPerSec((float)values[38]));
    /* 新候选字段显示参考速度上限，不能把含2RPM余量的基速当成参考速度。 */
    if(APP_ROUTE_SPEED_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT && !(*AppMotion_Live()->turn))
        values[57]=App_SerialMilli(((g_combined_test.state==0U) ?
            AppMotion_Live()->actions[0].reference_mm_s : (*AppMotion_Live()->reference_mm_s))/10.0f);
    if (g_motion_plan_valid != 1U) values[37] = 2;
    if (g_combined_test.state == 0U && (*AppMotion_Live()->has_action))
    {
        uint32_t state = MotionGetState();
        values[2] = (state == MOTION_BUSY) ? 2 :
            ((state == MOTION_DONE) ? 3 : ((state == MOTION_CANCELLED) ? 5 : 4));
        values[3] = (*AppMotion_Live()->turn) ? (int32_t)g_heading_exec_trip : (int32_t)g_forward_hold.trip;
        values[4] = (int32_t)(((*AppMotion_Live()->session_open) ? now : (*AppMotion_Live()->end_ms)) - (*AppMotion_Live()->begin_ms));
        values[48] = values[2]; values[49] = 0;
        values[50] = (state == MOTION_DONE) ? 1 : 0;
        values[55] = 1;
        if (g_translation_hold_selftest == 1U) values[37] = 1;
    }
#endif
    values[(APP_TEST_MODE == 3U) ? 64U : 48U] = (int32_t)g_can_task_stack_words; /* CAN生命周期最低剩余words */
#if APP_TEST_MODE == 3U
    values[65]=(int32_t)g_debug_plan_id;
#endif
    taskEXIT_CRITICAL();
    frame[0] = 'T'; frame[1] = 'M'; frame[2] = 'Q'; frame[3] = ',';
    for (i = 0U; i < ((APP_TEST_MODE == 3U) ? 66U : 49U); i++)
    {
        count = snprintf(&frame[used], sizeof(frame) - used, "%ld,", (long)values[i]);
        if ((count < 0) || ((uint32_t)count >= sizeof(frame) - used)) { s_serial_dropped++; return; }
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(&frame[used], sizeof(frame) - used, "%04X\r\n", (unsigned int)crc);
    if ((count < 0) || ((uint32_t)count >= sizeof(frame) - used)) { s_serial_dropped++; return; }
    App_SerialStatusSend(frame, now);
}
#endif

#endif



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

static void App_CanTask(void *argument)
{
    (void)argument;
#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
    AppMotion_InitOdometry();
#endif
    g_can_test_result = CAN1_LoopbackTest();
    while (1)
    {
        g_can_task_count++;
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
                StartupAxes_Service(g_safety_ready && g_can_test_result==1U,g_can_guard_ready);
                if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(VehicleArm_Busy() || g_vehicle_arm_stop_pending)VehicleArm_Service();
                if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
                if(!VehicleArm_Busy() && (!VEHICLE_INTEGRATED_ENABLE || StartupAxes_Holding()) && g_startup_axes_state!=1U && g_startup_axes_state!=2U && g_startup_axes_state!=3U) App_CombinedTestStep();
#else
                if (APP_FORWARD_HOLD_TEST != 0U) App_ForwardHoldStep(&s_translation_runtime_inputs);
                else App_HeadingRuntimeStep(&s_heading_runtime_inputs);
#endif
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
                App_HeadingSourceObserve();
                {
                    ChassisHeadingRequest heading_request = {0, 1U, 0U};
                    uint32_t valid = ChassisAuto_HeadingRead(&heading_request);
                    /* 只观察：不会把这里的轮速发给电机。 */
                    g_heading_bus_rpm = heading_request.command_rpm;
                    g_heading_bus_stop = heading_request.stop_required;
                    g_heading_bus_sequence = heading_request.sequence;
                    g_heading_bus_valid = valid;
                }
        #endif
        if (g_can_test_result == 1U)
        {
            
        /* 同一轮只执行一个查询，位置请求优先于转速/版本/状态 */
        /* 静态停止测试优先，每轮仍只执行一笔CAN交易 */
        /* 自动停止优先，每轮只执行一笔CAN交易 */
        #if APP_HEADING_SESSION_TEST
                if (g_chassis_auto_active == 0U) {
                    App_ComboIdlePoll();
                    if(StartupAxes_Holding())DM_CAN2_ModuleService(1U,0U);
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
        g_can_task_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(APP_UART_LINK_TEST ? 50U : (APP_HEADING_SESSION_TEST ? 10U : 50U)));
    }
}

/* 仅虚拟数据的软件自检：0未执行，1通过，2失败。case非0定位失败组。 */
volatile uint32_t g_heading_dry_result = 0U;
volatile uint32_t g_heading_dry_case = 0U;
volatile int32_t g_heading_dry_ccw[5] = {0};
volatile int32_t g_heading_dry_cw[5] = {0};
volatile float g_heading_dry_cross_error = 0.0f;

static void App_HeadingDryRun(void)
{
    HeadingController controller = {0};
    HeadingControllerOutput output = {0};
    const HeadingControllerConfig *config = &g_heading_controller_default_config;
    static const float yaw[2][5] = {
        {0.0f, 25.0f, 39.0f, 44.0f, 44.0f},
        {0.0f, -25.0f, -39.0f, -44.0f, -44.0f}
    };
    static const uint32_t tick[5] = {0U, 100U, 200U, 300U, 1000U};
    static const int32_t expected[5] = {5, 3, 1, 0, 0};
    uint32_t direction, i, expected_state;
    int32_t expected_command;
    float error;

    g_heading_dry_result = 2U;
    /* 两组各5次计算，虚拟tick不会让任务真实等待这些时间。 */
    for (direction = 0U; direction < 2U; direction++)
    {
        g_heading_dry_case = direction + 1U;
        if (HeadingController_Start(&controller, config,
            (direction == 0U) ? 45.0f : -45.0f, 0.0f, 0U) != 1U) return;
        for (i = 0U; i < 5U; i++)
        {
            if (HeadingController_Step(&controller, tick[i], yaw[direction][i],
                0.0f, 1U, &output) != 1U) return;
            expected_command = (direction == 0U) ? expected[i] : -expected[i];
            if (direction == 0U) g_heading_dry_ccw[i] = output.command_rpm;
            else g_heading_dry_cw[i] = output.command_rpm;
            expected_state = (i < 3U) ? HEADING_CTRL_TURNING :
                ((i == 3U) ? HEADING_CTRL_SETTLING : HEADING_CTRL_DONE);
            if ((output.command_rpm != expected_command) ||
                (output.state != expected_state) ||
                (output.cause != HEADING_CAUSE_NONE) ||
                (output.stop_required != ((i < 3U) ? 0U : 1U))) return;
        }
    }

    /* +179到-179最短误差为+2度，在当前2度候选容差内。 */
    g_heading_dry_case = 3U;
    if (HeadingController_Start(&controller, config, -179.0f, 179.0f, 0U) != 1U)
        return;
    if (HeadingController_Step(&controller, 0U, 179.0f, 0.0f, 1U, &output) != 1U)
        return;
    g_heading_dry_cross_error = output.error_deg;
    if ((output.error_deg < 1.999f) || (output.error_deg > 2.001f) ||
        (output.state != HEADING_CTRL_SETTLING) || (output.command_rpm != 0)) return;
    if (HeadingController_Step(&controller, 700U, -179.0f, 0.0f, 1U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_DONE) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 4U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, 1U, 0.0f, 0.0f, 0U, &output) != 1U)
        return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_SENSOR_INVALID) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 5U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    if (HeadingController_Step(&controller, config->timeout_ms,
        0.0f, 0.0f, 1U, &output) != 1U) return;
    if ((output.state != HEADING_CTRL_FAULT) ||
        (output.cause != HEADING_CAUSE_TIMEOUT) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    g_heading_dry_case = 6U;
    if (HeadingController_Start(&controller, config, 45.0f, 0.0f, 0U) != 1U) return;
    HeadingController_Cancel(&controller, &output);
    if ((output.state != HEADING_CTRL_CANCELLED) ||
        (output.cause != HEADING_CAUSE_CANCELLED) ||
        (output.command_rpm != 0) || (output.stop_required != 1U)) return;

    /* 验证刚修正的两角分别归一化，不允许误返回0。 */
    g_heading_dry_case = 7U;
    error = HeadingController_ErrorDeg(179.0f, -3500.0f);
    if ((error < 78.999f) || (error > 79.001f)) return;
    error = HeadingController_ErrorDeg(-179.0f, 3500.0f);
    if ((error < -79.001f) || (error > -78.999f)) return;

    g_heading_dry_case = 0U;
    g_heading_dry_result = 1U;
}

#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 1开始一次预览，2取消；默认0，拒绝请求不会留待以后自动执行。 */
volatile uint32_t g_heading_live_request = 0U;
/* 相对本次开始时的航向，第一轮保持0度；范围仅用于静态预览。 */
volatile float g_heading_live_relative_deg = 0.0f;
volatile float g_heading_live_target_deg = 0.0f;
volatile float g_heading_live_error_deg = 0.0f;
volatile int32_t g_heading_live_command_rpm = 0;
volatile uint32_t g_heading_live_state = HEADING_CTRL_IDLE;
volatile uint32_t g_heading_live_cause = 0U;
volatile uint32_t g_heading_live_stop_required = 0U;
/* 拒绝原因：1本次上电已用，2静态条件不满足，3HWT未就绪，4角度无效。 */
volatile uint32_t g_heading_live_reject = 0U;
/* 仅供Watch观察：转换结果与四轮目标，不发送CAN。 */
volatile uint32_t g_heading_live_encode_result = 0U;
volatile ChassisMotorCommand g_heading_live_motor[4] = {0};
static HeadingController s_heading_live_controller = {0};
static uint32_t s_heading_live_used = 0U;

static uint32_t App_HeadingLiveHealthy(uint32_t now)
{
    /* 沿现有解析器500ms新鲜度，仅用于本次不驱动电机的输入检查。 */
    return ((g_hwt_version_result == 1U) && (value.valid_mask == 3U) &&
        (g_hwt_angle_age_ms <= 500U) && (g_hwt_rate_age_ms <= 500U) &&
        ((uint32_t)(now - g_hwt_update_tick) <= 100U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}

/* 只由TaskA在HWT101_Update之后调用，不读写电机或VM输出。 */
static void App_HeadingLiveStep(void)
{
    HeadingControllerOutput output = {0};
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT] = {0};
    uint32_t i;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t request = g_heading_live_request;
    uint32_t healthy = App_HeadingLiveHealthy(now);
    uint32_t static_ok = ((g_vm_test_request == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U)) ? 1U : 0U;
    float relative;

    g_heading_live_request = 0U;
    if ((request == 2U) &&
        ((s_heading_live_controller.state == HEADING_CTRL_TURNING) ||
         (s_heading_live_controller.state == HEADING_CTRL_SETTLING)))
        HeadingController_Cancel(&s_heading_live_controller, &output);
    if (request == 1U)
    {
        g_heading_live_reject = 0U;
        relative = g_heading_live_relative_deg;
        if (s_heading_live_used != 0U) g_heading_live_reject = 1U;
        else if (static_ok == 0U) g_heading_live_reject = 2U;
        else if (healthy == 0U) g_heading_live_reject = 3U;
        else if (!(relative >= -90.0f && relative <= 90.0f))
            g_heading_live_reject = 4U; /* 此写法也拒绝NaN/无穷值。 */
        else if (HeadingController_Start(&s_heading_live_controller,
            &g_heading_controller_default_config,
            g_hwt_yaw_deg + relative, g_hwt_yaw_deg, now) != 1U)
            g_heading_live_reject = 4U;
        else s_heading_live_used = 1U;
    }
    /* 失效时传0，让控制器锁定故障并把建议轮速撤为0。 */
    (void)HeadingController_Step(&s_heading_live_controller, now,
        g_hwt_yaw_deg, g_hwt_wz_dps,
        ((healthy != 0U) && (static_ok != 0U)) ? 1U : 0U, &output);
    g_heading_live_state = output.state;
    g_heading_live_cause = output.cause;
    g_heading_live_target_deg = output.target_deg;
    g_heading_live_error_deg = output.error_deg;
    g_heading_live_command_rpm = output.command_rpm;
    g_heading_live_stop_required = output.stop_required;
        /* 将本轮航向输出转换为四轮指令，仅发布观察结果。 */
    g_heading_live_encode_result =
        Chassis_HeadingCommands(output.command_rpm, commands);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        g_heading_live_motor[i] = commands[i];
    }
    /* TaskA发布完整的一轮计算结果；CAN任务只读取预览。 */
    g_heading_publish_result = ChassisAuto_HeadingPublish(
        output.command_rpm, output.stop_required);
}
#endif

/* 任务A：每100毫秒计数一次 */
/* 独立串口测试任务：替代业务任务A，不执行扫码显示或运动门控。 */
static void App_UartLinkTestTask(void *argument)
{
    (void)argument;
    if (APP_IS_DEBUG) USART1_EnableReceive();
    HWT101_EnableReceive();
    Buzzer_Start(APP_INIT_BEEP_COUNT);
    while (1)
    {
        HWT101_Update();
        Buzzer_Update();
        g_test_count_a++;
        g_test_tick = xTaskGetTickCount();
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 完成真实接收解析后报告进展，原安全监督和看门狗不变。 */
        Safety_Report(SAFETY_TASK_APP);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void App_TestTaskA(void *argument)
{
    TickType_t screen_start_tick = xTaskGetTickCount();
    (void)argument;
    /* FreeRTOS运行后才开启接收中断 */
    if (APP_IS_DEBUG) USART1_EnableReceive();
    /* 启动时只请求发送一次基础平台就绪信息 */
    SAOMA_EnableReceive();
    HWT101_EnableReceive();
    g_serial_send_ok = App_DebugSendString("F407 FreeRTOS READY\r\n");
    /* 已验收的基础平台就绪后，只请求一次提示音 */
    Buzzer_Start(APP_INIT_BEEP_COUNT);
        /* 一次性虚拟角度自检，不驱动小车。 */
    App_HeadingDryRun();
#if APP_SERIAL_MOTION_CONTROL
    s_serial_proto_ok = (App_SerialCrc("123456789", 9U) == 0x29B1U) ? 1U : 0U;
    AppRuntimeRoute_Init();
#endif
#if APP_HEADING_SESSION_TEST && APP_FORWARD_HOLD_TEST
    /* 只运行数学自检，失败禁止平移启动，默认不发动作。 */
    g_translation_hold_selftest = Chassis_TranslationHoldSelfTest() ? 1U : 2U;
#if APP_TEST_MODE == 3U
    AppMotion_LoadDefaultPlan();
#endif
#endif
    AppMotion_InitStage1Plan();
    while (1)
    {
        g_test_count_a++;
        
        /* 每100毫秒更新响停状态 */
        Buzzer_Update();
        HWT101_Update();
        #if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
        App_HeadingSourcePublish();
        #endif
        #if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
        App_HeadingLiveStep();
        #endif
        /* 一轮最多处理64字节；只有任务A交接并更新屏幕 */
        SAOMA_Update();
        if(VEHICLE_INTEGRATED_ENABLE){AppMotion_MissionService();AppMotion_MissionTelemetry();}
        else if(VEHICLE_STAGE1_ENABLE)
        {
            AppMotion_Stage1Service();
            AppMotion_Stage1Display((TickType_t)(xTaskGetTickCount()-screen_start_tick)>=pdMS_TO_TICKS(1000U));
        }
        else
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

        App_StartGateUpdate();
#if APP_SERIAL_MOTION_CONTROL
        /* 先尝试当前状态；诊断只用成功状态之后的空档。 */
        App_SerialMotionTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)AppMotion_Stage1Telemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)AppMotion_Stage1CodeTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_RuntimeRouteTelemetry();
#if APP_POSITION_DIAGNOSTIC
        App_PositionEndpointTelemetry();
#endif
        App_MotionFaultTelemetry();
        App_MotionTimingTelemetry();
        App_BatchDiagnosticTelemetry();
        App_ChassisLoadTelemetry();
#if APP_POSITION_DIAGNOSTIC
        App_WheelLogTelemetry();
        App_PositionTelemetry();
#if APP_HEADING_SESSION_TEST
        App_OdometryTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_PerimeterTelemetry();
#endif
#endif
#endif
        g_test_tick = xTaskGetTickCount();
        /* 查询当前任务的最小剩余栈及堆余量 */
        g_stack_a_words = uxTaskGetStackHighWaterMark(NULL);
        g_heap_free_bytes = xPortGetFreeHeapSize();
        g_heap_min_bytes = xPortGetMinimumEverFreeHeapSize();
        /* 一轮查询和平台工作结束后，报告真实进展 */
        Safety_Report(SAFETY_TASK_APP);
        
        
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/* 任务B：每500毫秒计数一次 */
static void App_TestTaskB(void *argument)
{
    (void)argument;

    while (1)
    {
        g_test_count_b++;
        /* 每500毫秒翻转一次，完整亮灭周期约1秒 */
        LED_Turn();
        /* 查询任务B自己的最小剩余栈 */
        g_stack_b_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(500U));
    }
}

#if APP_UART_LINK_TEST
/* 仅测试数据：无运动命令、无参数写入、无VM允许。 */
volatile uint32_t g_uart_link_test_value = 0U;
volatile uint32_t g_uart_link_rx_lines = 0U, g_uart_link_tx_replies = 0U;
volatile uint32_t g_uart_link_errors = 0U, g_uart_link_dropped = 0U;
volatile uint32_t g_uart_link_hb_count = 0U;
static char s_link_line[48], s_link_reply[64];
static uint32_t s_link_used, s_link_overflow, s_link_pending;
static uint32_t s_link_last_byte, s_link_hb_tick, s_link_yaw_pending;

static void App_UartLinkCommand(void)
{
    const char *digit;
    uint32_t value = 0U, valid = 1U;
    g_uart_link_rx_lines++;
    if (s_link_pending != 0U) { g_uart_link_dropped++; return; }
    if ((strncmp(s_link_line, "PING,", 5U) == 0) &&
        (strlen(s_link_line + 5U) > 0U) && (strlen(s_link_line + 5U) <= 32U))
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "PONG,%s\r\n", s_link_line + 5U);
    else if (strncmp(s_link_line, "ECHO,", 5U) == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "ECHO,%s\r\n", s_link_line + 5U);
    else if (strcmp(s_link_line, "INFO") == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "INFO,UART1,115200,8N1,VM0\r\n");
    else if (strcmp(s_link_line, "GET") == 0)
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "VALUE,%lu\r\n", (unsigned long)g_uart_link_test_value);
    else if (strncmp(s_link_line, "SET,", 4U) == 0)
    {
        digit = s_link_line + 4U;
        if (*digit == '\0') valid = 0U;
        while (*digit != '\0')
        {
            if ((*digit < '0') || (*digit > '9')) { valid = 0U; break; }
            value = value * 10U + (uint32_t)(*digit - '0');
            if (value > 1000U) { valid = 0U; break; }
            digit++;
        }
        if (valid != 0U)
        {
            g_uart_link_test_value = value;
            (void)snprintf(s_link_reply, sizeof(s_link_reply), "VALUE,%lu\r\n", (unsigned long)value);
        }
        else
        {
            g_uart_link_errors++;
            (void)snprintf(s_link_reply, sizeof(s_link_reply), "ERR,RANGE\r\n");
        }
    }
    else
    {
        g_uart_link_errors++;
        (void)snprintf(s_link_reply, sizeof(s_link_reply), "ERR,COMMAND\r\n");
    }
    s_link_pending = 1U;
}

static void App_UartLinkByte(uint8_t data)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - s_link_last_byte) >= 500U)
    { s_link_used = 0U; s_link_overflow = 0U; }
    s_link_last_byte = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if ((s_link_overflow == 0U) && (s_link_used > 0U))
        {
            s_link_line[s_link_used] = '\0';
            App_UartLinkCommand();
        }
        s_link_used = 0U; s_link_overflow = 0U;
        return;
    }
    if (s_link_overflow != 0U) return;
    if ((data < 32U) || (data > 126U) || (s_link_used >= sizeof(s_link_line) - 1U))
    { g_uart_link_errors++; s_link_overflow = 1U; return; }
    s_link_line[s_link_used++] = (char)data;
}

static void App_UartLinkPoll(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    char heartbeat[64];
    if (s_link_pending != 0U)
    {
        if (App_DebugSendString(s_link_reply) == 1U)
        { s_link_pending = 0U; g_uart_link_tx_replies++; }
        return;
    }
    /* 与命令回复共用同一个发送入口，避免两个任务竞争串口。 */
    if (s_link_yaw_pending != 0U)
    {
        float yaw;
        uint32_t valid, age;
        taskENTER_CRITICAL();
        yaw = g_hwt_yaw_deg;
        valid = g_hwt_valid_mask;
        age = now - g_hwt_update_tick;
        taskEXIT_CRITICAL();
        /* 定点毫度，避免依赖浮点printf；无效值用哨兵表示。 */
        (void)snprintf(heartbeat, sizeof(heartbeat), "LINK,YAW,%ld,%lu,%lu\r\n",
            (((yaw >= -180.0f) && (yaw <= 180.0f)) ?
                (long)(yaw * 1000.0f) : (-2147483647L - 1L)),
            (unsigned long)valid, (unsigned long)age);
        if (App_DebugSendString(heartbeat) == 1U) s_link_yaw_pending = 0U;
        return;
    }
    if ((uint32_t)(now - s_link_hb_tick) >= 1000U)
    {
        (void)snprintf(heartbeat, sizeof(heartbeat), "LINK,HB,%lu,%lu,VM0\r\n",
            (unsigned long)g_uart_link_hb_count, (unsigned long)now);
        if (App_DebugSendString(heartbeat) == 1U)
        { g_uart_link_hb_count++; s_link_hb_tick = now; s_link_yaw_pending = 1U; }
    }
}
#endif

/* 独立接收任务：有数据立即处理，无数据进入等待 */
static void App_UartRxTask(void *argument)
{
    uint8_t data;

    (void)argument;

    while (1)
    {
        if (USART1_WaitByte(&data, 100U) != 0U)
        {
            g_serial_rx_count++;
            g_serial_rx_last = data;
#if APP_UART_LINK_TEST
            App_UartLinkByte(data);
#elif APP_SERIAL_MOTION_CONTROL
            App_SerialMotionByte(data);
#endif
        }
        else
        {
            /* 空闲时记录历史最小剩余栈，单位为字 */
            g_rx_stack_words = uxTaskGetStackHighWaterMark(NULL);
        }
#if APP_UART_LINK_TEST
        App_UartLinkPoll();
#endif
        /* 有数据或正常空闲等待返回，均表示任务仍在运行 */
        Safety_Report(SAFETY_TASK_RX);
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
static void App_HwtTurnSafety(void)
{
    if ((g_chassis_key_state == 2U) || (g_chassis_key_state == 3U))
    {
        if (!App_HwtTurnHealthy()) App_ChassisKeyStop(5U);
    }
    else g_vm_test_request = 0U;
}
#endif
#endif

static void App_KeyTask(void *argument)
{
    uint32_t diagnostic_count = 0U;

    (void)argument;

    while (1)
    {
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
        diagnostic_count++;
        if (diagnostic_count >= 100U)
        {
            g_key_stack_words = uxTaskGetStackHighWaterMark(NULL);
            diagnostic_count = 0U;
        }
        Safety_Report(SAFETY_TASK_KEY);
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

/* 监督不等待CAN或串口，检查后立即让出CPU */
static void App_SafetyTask(void *argument)
{
    (void)argument;

    while (1)
    {
if (APP_UART_LINK_TEST != 0U)
        { g_vm_test_request = 0U; Power_Disable(); }
        else
        {
#if APP_SERIAL_MOTION_CONTROL
        if(!VEHICLE_INTEGRATED_ENABLE)App_SerialMotionSafety();
#endif
#if APP_HEADING_SESSION_TEST
        #if APP_TEST_MODE == 3U
        StartupAxes_Safety(!g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched);
        if(g_startup_axes_state==5U && (g_combined_test.state==1U || g_combined_test.state==2U)) MotionStop();
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
        Safety_Update(g_vm_test_request, g_can_test_result);
        g_safety_stack_words = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

int main(void)
{
    /* 更新时钟，将4位优先级全部用于抢占优先级 */
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* 调度启动前初始化状态灯 */
    LED_Init();
    /* 调度前初始化蜂鸣器，保持关闭 */
    Buzzer_Init();
    /* 初始化调试串口，本轮仅发送 */
    USART1_Config();
    /* 调度前初始化启动按键 */
    /* UART4仅初始化，实际发送放到任务A */
    Screen_Init();
    Key_Init();
    /* 初始化PE2，默认关闭VM允许请求 */
    Power_Init();
    StartupPose_Init();
    RadarZoneSwitch_Init();
    (void)RadarServo_Init();
    Safety_Init();
    SAOMA_Init();
    HWT101_Init();
    g_watchdog_init_result = Watchdog_Init();
    configASSERT(g_watchdog_init_result == 1U);
    
    /* 每个任务栈为256个字，即1024字节 */
    /* 编译常量选择任务，只创建一方；关闭测试即可恢复业务。 */
    if (APP_UART_LINK_TEST != 0U)
        g_test_create_a = xTaskCreate(App_UartLinkTestTask, "UartTest",
                                     256U, NULL, 2U, NULL);
    else
        g_test_create_a = xTaskCreate(App_TestTaskA, "TestA",
                                     (APP_SERIAL_MOTION_CONTROL ? 512U : 256U), NULL, 2U, NULL);
    configASSERT(g_test_create_a == pdPASS);
    g_test_create_b = xTaskCreate(App_TestTaskB, "TestB",
                                 256U, NULL, 1U, NULL);
    configASSERT(g_test_create_b == pdPASS);
    /* 本轮接收任务栈256字，优先级3 */
    g_rx_task_create = xTaskCreate(App_UartRxTask, "UartRx",
                                   (APP_SERIAL_MOTION_CONTROL ? 384U : 256U), NULL, 3U, NULL);
    configASSERT(g_rx_task_create == pdPASS);
    /* 本轮按键扫描任务，栈256字、优先级2 */
    g_key_task_create = xTaskCreate(App_KeyTask, "Key",
                                    256U, NULL, 2U, NULL);
    configASSERT(g_key_task_create == pdPASS);
    /* 优先级4，高于当前接收3、任务A/按键2；无阻塞外设操作 */
    g_safety_task_create = xTaskCreate(App_SafetyTask, "Safety",
                                      256U, NULL, 4U, NULL);
    configASSERT(g_safety_task_create == pdPASS);
    
    
    
    /* CAN栈384字，优先级3；监督仍为4 */
    g_can_task_create = xTaskCreate(App_CanTask, "CanPoll",
                                      384U, NULL, 3U, NULL);
    configASSERT(g_can_task_create == pdPASS);
    /* 启动后由任务运行 */
    vTaskStartScheduler();

    /* 调度器异常返回时记录并停止 */
    g_test_scheduler_returned = 1U;
    configASSERT(0);

    while (1)
    {
    }
}
