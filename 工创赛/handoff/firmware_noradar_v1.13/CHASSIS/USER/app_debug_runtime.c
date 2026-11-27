#include "app_debug_runtime.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "app_serial_internal.h"
#include "heading_source.h"
#include "hwt101.h"
#include "heading_controller.h"
#include "chassis_kinematics.h"
#include "chassis_auto_test.h"
#include "safety.h"
#include <stdio.h>
#include <string.h>

extern volatile uint32_t g_vm_test_request;

#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
/* 以下仅显示任务间数据，不作为运动允许信号。 */
volatile uint32_t g_heading_bus_valid = 0U;
volatile int32_t g_heading_bus_rpm = 0;
volatile uint32_t g_heading_bus_stop = 1U;
volatile uint32_t g_heading_bus_sequence = 0U;
volatile uint32_t g_heading_publish_result = 0U;
#endif

/* 仅虚拟数据的软件自检：0未执行，1通过，2失败。case非0定位失败组。 */
volatile uint32_t g_heading_dry_result = 0U;
volatile uint32_t g_heading_dry_case = 0U;
volatile int32_t g_heading_dry_ccw[5] = {0};
volatile int32_t g_heading_dry_cw[5] = {0};
volatile float g_heading_dry_cross_error = 0.0f;

void App_HeadingDryRun(void)
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
void App_HeadingLiveStep(void)
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

#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
void AppDebug_HeadingObserve(void)
{
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
}
#endif

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

void App_UartLinkByte(uint8_t data)
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

void App_UartLinkPoll(void)
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
