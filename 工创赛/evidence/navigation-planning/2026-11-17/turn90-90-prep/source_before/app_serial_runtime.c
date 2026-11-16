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
#include "app_serial_runtime.h"
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
#include "icm45686_raw.h"
#include "app_icm_observer.h"

extern volatile uint32_t g_vm_test_request;
extern volatile UBaseType_t g_rx_stack_words,g_can_task_stack_words;
extern volatile size_t g_heap_min_bytes;
/* 两个任务共用调试发送缓冲区，短临界区内只复制，不等待串口 */
uint8_t App_DebugSendString(const char *text)
{
    uint8_t result;
    taskENTER_CRITICAL();
    result = USART1_SendString(text);
    taskEXIT_CRITICAL();
    return result;
}

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
void App_SerialMotionSafety(void);
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
#if !APP_VM_ONLY_ENABLE
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
#endif
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
#if APP_BASELINE40_ACTIVE
static uint32_t App_SerialPoweredDone(const PowerCommandStatus *power,const ChassisAutoStatus *status)
{
    return (g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 ||
        g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD150_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40) && g_combined_test.state==3U &&
        !g_combined_test.cause && g_combined_test.completed==*AppMotion_Live()->action_count &&
        g_vm_test_request==1U && power->active==1U && *AppMotion_Live()->session_open &&
        g_combined_test.index<*AppMotion_Live()->action_count &&
        status->session==g_combined_test.session[g_combined_test.index] &&
        !status->active && status->result==1U && !status->cause && !status->cut_vm &&
        status->state==CHASSIS_AUTO_DONE && status->rotation_stop_mask==15U &&
        !g_can_ctrl_fault_latched && !g_rtos_fault && !g_safety_fault;
}
#endif
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
#if APP_BASELINE40_ACTIVE
    if(App_SerialPoweredDone(&power,&status))return 1U;
#endif
    return (!g_vm_test_request && !power.active && !status.active &&
        status.result == 1U && !status.cause && status.rotation_stop_mask == 15U &&
        !g_can_ctrl_fault_latched && !g_rtos_fault && !g_safety_fault) ? 1U : 0U;
}
static void App_SerialReject(uint32_t reason)
{ g_serial_motion_rejected++; g_serial_motion_result = reason; }


static void App_SerialMotionCommand(char *line)
{
#if APP_VM_ONLY_ENABLE
    App_SerialReject(6U);
#else
    static const volatile uint32_t source_parameter_entry=VEHICLE_INTEGRATED_ENABLE;
    if(source_parameter_entry)return;

    if(FieldMap_LocalMode()) { App_SerialReject(9U);return; }
    if(strncmp(line,"RQ1,",4U)==0) {
#if APP_BASELINE40_ACTIVE
        App_SerialReject(6U);return;
#else
        App_RuntimeRouteCommand(line);return;
#endif
    }
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
    else if ((strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<='9' && p[5]=='\0') ||
             strcmp(p,"PLAN10")==0 || strcmp(p,"PLAN11")==0 || strcmp(p,"PLAN12")==0 || strcmp(p,"PLAN13")==0 ||
             strcmp(p,"PLAN14")==0 || strcmp(p,"PLAN15")==0 || strcmp(p,"PLAN16")==0 || strcmp(p,"PLAN17")==0 || strcmp(p,"PLAN18")==0 || strcmp(p,"PLAN19")==0)
    {
        if (!s_serial_tx_seq || (uint32_t)(s_serial_tx_seq-1U-seq)>5U ||
            (uint32_t)(now-tick)>=500U) App_SerialReject(5U);
        else if (g_serial_motion_state || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->committed.id || !App_SerialIdleReady(now) ||
                 !App_DebugSelectPlan(p[5] ? (10U+(uint32_t)(p[5]-'0')) : (uint32_t)(p[4]-'0'))) App_SerialReject(6U);
        else g_serial_motion_accepted++;
    }
#endif
    else if (strcmp(p, "ARM") == 0)
    {
        if ((s_serial_tx_seq == 0U) || ((uint32_t)(s_serial_tx_seq - 1U - seq) > 5U) ||
            ((uint32_t)(now - tick) >= 500U)) App_SerialReject(5U);
        else if ((g_serial_motion_state != 0U) || !App_SerialIdleReady(now) ||
                 (!APP_BASELINE40_ACTIVE && (!AppRuntimeRoute_Live()->ready || AppRuntimeRoute_Live()->staging || AppRuntimeRoute_Live()->consumed)) ||
                 (APP_BASELINE40_ACTIVE && ((g_debug_plan_id!=DEBUG_PLAN_FORWARD210_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT210_40 &&
                  g_debug_plan_id!=DEBUG_PLAN_RIGHT210_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD210_40 && g_debug_plan_id!=DEBUG_PLAN_RIGHT150_40 && g_debug_plan_id!=DEBUG_PLAN_RIGHT30_40 &&
                   g_debug_plan_id!=DEBUG_PLAN_FORWARD150_40 && g_debug_plan_id!=DEBUG_PLAN_FORWARD30_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD150_40 && g_debug_plan_id!=DEBUG_PLAN_BACKWARD30_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT150_40 && g_debug_plan_id!=DEBUG_PLAN_LEFT30_40) ||
                  *AppMotion_Live()->action_count!=1U))) App_SerialReject(6U);
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
#if APP_TEST_MODE == 3U && !APP_BASELINE40_ACTIVE
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
#endif
}
void App_SerialMotionByte(uint8_t data)
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
void App_SerialMotionSafety(void)
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
#if APP_BASELINE40_ACTIVE
/* IO1 is diagnostic only; no retries or commands. Millimetres scale by 1000. */
static int32_t App_ObserverMilli(float value)
{
    if(!(value>=-2000000.0f && value<=2000000.0f))return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static void App_IcmObserverTelemetry(void)
{
    static char frame[384];static uint32_t last,sent,sequence,dropped;
    AppIcmObserverStatus o; int32_t values[27];
    uint32_t now=(uint32_t)xTaskGetTickCount(),phase=now-s_serial_tx_tick,i,used=4U;
    int n;uint16_t crc;
    if((sent && (uint32_t)(now-last)<500U) || !App_SerialDiagnosticWindow(now,40U) || phase>55U)return;
    last=now;sent=1U; /* Attempt budget, including a busy transmitter. */
    AppIcmObserver_Read(&o);
    values[0]=(int32_t)sequence++;values[1]=(int32_t)now;values[2]=1;
    values[3]=(int32_t)o.sequence;values[4]=(int32_t)o.session;
    values[5]=(int32_t)o.valid;values[6]=(int32_t)o.reasons;values[7]=(int32_t)o.flags;
    values[8]=(int32_t)o.estimate_ms;values[9]=(int32_t)o.sample_sequence;
    values[10]=(int32_t)o.reference_ms;values[11]=(int32_t)o.t0_ms;values[12]=(int32_t)o.t1_ms;
    values[13]=(int32_t)o.wheel_age_ms;values[14]=(int32_t)o.hwt_age_ms;
    values[15]=(int32_t)o.cal_samples;values[16]=(int32_t)o.history_samples;
    values[17]=(int32_t)o.gaps;
    for(i=18U;i<26U;i++)values[i]=(-2147483647L-1L);
    if(o.flags&ICM_OBS_WHEEL){values[18]=App_ObserverMilli(o.wheel_f_mm);values[19]=App_ObserverMilli(o.wheel_r_mm);}
    if(o.valid){values[20]=App_ObserverMilli(o.position_f_mm);values[21]=App_ObserverMilli(o.position_r_mm);
        values[22]=App_ObserverMilli(o.velocity_f_mm_s);values[23]=App_ObserverMilli(o.velocity_r_mm_s);}
    if(o.flags&ICM_OBS_ACCEL){values[24]=App_ObserverMilli(o.accel_f_mm_s2);values[25]=App_ObserverMilli(o.accel_r_mm_s2);}
    values[26]=(int32_t)dropped;memcpy(frame,"IO1,",4U);
    for(i=0U;i<27U;i++) {
        if(i<18U || i==26U)n=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)(uint32_t)values[i]);
        else n=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)values[i]);
        if(n<0 || (uint32_t)n>=sizeof(frame)-used){dropped++;return;}used+=(uint32_t)n;
    }
    crc=App_SerialCrc(frame,used);n=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(n<0 || (uint32_t)n>=sizeof(frame)-used || !App_DebugSendString(frame)){dropped++;return;}
}
/* IR1: software association times; DWT cycles are not sensor ADC timestamps. */
static void App_IcmRawTelemetry(void)
{
    static char frame[512];static IcmRawStatus sample;
    static uint32_t sequence,last,last_sample,dropped,seen;
    uint32_t now=(uint32_t)xTaskGetTickCount(),publication,used,i;
    uint32_t words[24];int n;uint16_t crc;
    if ((uint32_t)(now-last)<200U || !App_SerialDiagnosticWindow(now,70U)) return;
    if (!AppIcm_ReadPublished(&sample,&publication) || !sample.sample_sequence ||
        (seen && sample.sample_sequence==last_sample)) return;
    now=(uint32_t)xTaskGetTickCount();
    sample.age_ms=now-sample.sample_tick_ms;
    sample.fresh=sample.fresh && sample.age_ms<=20U;
    words[0]=sequence++;words[1]=now;words[2]=publication;words[3]=sample.sample_sequence;
    words[4]=sample.state;words[5]=sample.last_error;words[6]=sample.failed_reg;words[7]=sample.whoami;
    words[8]=sample.accel_config;words[9]=sample.gyro_config;words[10]=sample.big_endian;
    words[11]=sample.edge_sequence;words[12]=sample.edge_cycle;
    words[13]=sample.read_start_cycle;words[14]=sample.read_end_cycle;
    words[15]=sample.sample_tick_ms;words[16]=sample.age_ms;words[17]=sample.fresh;
    words[18]=sample.transfer_errors;words[19]=sample.skipped_edges;words[20]=sample.read_races;
    words[21]=dropped;words[22]=seen ? sample.sample_sequence-last_sample-1U : 0U;
    words[23]=SystemCoreClock;
    memcpy(frame,"IR1,",4U);used=4U;
    for (i=0U;i<31U;i++) {
        if (i<24U) n=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)words[i]);
        else n=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)(i<27U ? sample.raw_accel[i-24U] :
            (i<30U ? sample.raw_gyro[i-27U] : sample.raw_temperature)));
        if (n<0 || (uint32_t)n>=sizeof(frame)-used) {dropped++;return;}used+=(uint32_t)n;
    }
    crc=App_SerialCrc(frame,used);n=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if (n<0 || (uint32_t)n>=sizeof(frame)-used || !App_DebugSendString(frame)) {dropped++;return;}
    last=now;last_sample=sample.sample_sequence;seen=1U;
}
static void App_Baseline40Telemetry(void)
{
    static char frame[160];static uint32_t last,sent,sequence;
    uint32_t now=(uint32_t)xTaskGetTickCount();int n;uint16_t crc;
    if ((sent && now-last<1000U) || !App_SerialDiagnosticWindow(now,40U)) return;
    n=snprintf(frame,sizeof(frame),"BF1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
        (unsigned long)sequence++,(unsigned long)now,(unsigned long)g_debug_plan_id,
        (unsigned long)((g_debug_plan_id==DEBUG_PLAN_LEFT210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40) ? APP_BASELINE40_LEFT_RPM : APP_BASELINE40_RPM),
        (unsigned long)((g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40) ? 1500U :
            ((g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40) ? 300U : APP_BASELINE40_MM)),(unsigned long)APP_BASELINE40_MM_S,
        (unsigned long)APP_BASELINE40_ACCEL_MM_S2,(unsigned long)APP_BASELINE40_JERK_MM_S3,
        (unsigned long)APP_BASELINE40_FEEDBACK_RPM,(unsigned long)APP_BASELINE40_RUN_MS);
    if (n<0 || (uint32_t)n>=sizeof(frame)) return;
    crc=App_SerialCrc(frame,(uint32_t)n);
    if (snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\r\n",crc)<0) return;
    if (App_DebugSendString(frame)) {last=now;sent=1U;}
}
#endif

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
#if !APP_BASELINE40_ACTIVE
    last=now;
#endif
    (void)ChassisAuto_ReadWheelLog(&sample);
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
#if APP_BASELINE40_ACTIVE
    else last=now;
#endif
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
#if !APP_BASELINE40_ACTIVE
    last = now;
#endif
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
#if APP_BASELINE40_ACTIVE
    if (App_DebugSendString(frame)) last=now;
#else
    (void)App_DebugSendString(frame);
#endif
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
#if APP_BASELINE40_ACTIVE
    if(g_chassis_auto_active || (uint32_t)(now-last)<70U || !App_SerialDiagnosticWindow(now,40U))return;
    if(power.active || g_vm_test_request) {
        ChassisAutoStatus status;
        (void)ChassisAuto_ReadStatus(&status);
        if(!App_SerialPoweredDone(&power,&status))return;
    }
#else
    if(g_chassis_auto_active || power.active || g_vm_test_request || (uint32_t)(now-last)<70U ||
       !App_SerialDiagnosticWindow(now,40U))return;
#endif
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
    if (APP_BASELINE40_ACTIVE && (g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 ||
        g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD150_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40))
        values[39]=(int32_t)APP_BASELINE40_FEEDBACK_RPM;
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
    if (APP_BASELINE40_ACTIVE && (g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 ||
        g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD150_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40))
    {
        values[57]=40000; /* 40cm/s reference ceiling, not wheel/ground measurement. */
        if (g_combined_test.state==0U && !(*AppMotion_Live()->has_action) &&
            (*AppMotion_Live()->action_count)==1U) values[56]=(int32_t)AppMotion_Live()->actions[0].kind;
    }
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

#if APP_SERIAL_MOTION_CONTROL
void AppSerial_Init(void)
{
    s_serial_proto_ok = (App_SerialCrc("123456789", 9U) == 0x29B1U) ? 1U : 0U;
    AppRuntimeRoute_Init();
}
void AppSerial_Telemetry(void)
{
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
#if APP_BASELINE40_ACTIVE
        App_IcmRawTelemetry();
        App_Baseline40Telemetry();
#endif
        App_PositionTelemetry();
#if APP_HEADING_SESSION_TEST
        App_OdometryTelemetry();
        if(!VEHICLE_INTEGRATED_ENABLE)App_PerimeterTelemetry();
#endif
#endif
#if APP_BASELINE40_ACTIVE
        App_IcmObserverTelemetry();
#endif
}
#endif
