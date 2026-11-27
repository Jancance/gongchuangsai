#include "hwt101.h"
#include "hwt101_parser.h"
#include "app_config.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
/* USART2 PD6接收、PD5发送；读取2E版本，另提供用户明确触发的A6静态试验；不写出厂参数。 */
typedef struct { uint32_t tick; uint8_t data; } HwtByte;
static HwtByte s_rx[256];
static volatile uint16_t s_head = 0U, s_tail = 0U;
static volatile uint32_t s_error_epoch = 0U;
static uint32_t s_seen_error = 0U;
static Hwt101Parser s_parser;
volatile uint32_t g_hwt_baud = APP_HWT_BAUD;
volatile uint32_t g_hwt_rx_bytes = 0U, g_hwt_uart_errors = 0U, g_hwt_overflows = 0U;
volatile uint32_t g_hwt_angle_count = 0U, g_hwt_rate_count = 0U, g_hwt_checksum_errors = 0U;
volatile uint32_t g_hwt_valid_mask = 0U;
volatile uint32_t g_hwt_update_tick = 0U;
volatile uint32_t g_hwt_angle_age_ms = 0xFFFFFFFFU, g_hwt_rate_age_ms = 0xFFFFFFFFU;
volatile float g_hwt_yaw_deg = 0.0f, g_hwt_wz_dps = 0.0f;
volatile uint8_t g_hwt_last_frame[11] = {0U};
/* 版本/A7共用单笔只读事务；5F不回显地址，不并发查询。 */
volatile uint32_t g_hwt_version_request = 1U, g_hwt_version_result = 0U;
volatile uint32_t g_hwt_version_raw = 0U, g_hwt_version_low16 = 0U;
volatile uint32_t g_hwt_version_state = 0U, g_hwt_version_reply_count = 0U;
volatile uint32_t g_hwt_tx_bytes = 0U;
volatile uint8_t g_hwt_version_reply[11] = {0U};
/* A7只读查询，默认不发送，不修改自动零偏配置。 */
volatile uint32_t g_hwt_a7_request = 0U, g_hwt_a7_result = 0U;
volatile uint32_t g_hwt_a7_raw = 0U, g_hwt_a7_reply_count = 0U;
volatile uint8_t g_hwt_a7_reply[11] = {0U};
/* 回传速率只读：request1手动读取，result1只表示收到本次5F回复。 */
volatile uint32_t g_hwt_rrate_request = 0U, g_hwt_rrate_result = 0U;
volatile uint32_t g_hwt_rrate_raw = 0U, g_hwt_rrate_reply_count = 0U;
volatile uint8_t g_hwt_rrate_reply[11] = {0U};
static uint32_t s_read_rrate = 0U;
static const uint8_t s_rrate_command[5] = {0xFFU, 0xAAU, 0x27U, 0x03U, 0x00U};
static uint32_t s_read_a7 = 0U;
static const uint8_t s_a7_command[5] = {0xFFU, 0xAAU, 0x27U, 0xA7U, 0x00U};
static const uint8_t s_version_command[5] = {0xFFU, 0xAAU, 0x27U, 0x2EU, 0x00U};
static volatile uint32_t s_version_index, s_version_last_tx_tick;
static const uint8_t *s_tx_command = s_version_command;
static volatile uint32_t s_tx_owner = 0U; /* 1版本，2静态A6试验 */
static uint8_t s_cal_command[5] = {0xFFU, 0xAAU, 0xA6U, 0U, 0U};
extern volatile uint32_t g_vm_test_request, g_rtos_fault, g_safety_fault;
volatile uint32_t g_hwt_cal_request = 0U, g_hwt_cal_active = 0U;
volatile uint32_t g_hwt_cal_state = 0U, g_hwt_cal_result = 0U, g_hwt_cal_cause = 0U;
volatile uint32_t g_hwt_cal_start_tx_result = 0U, g_hwt_cal_end_tx_result = 0U;
volatile uint32_t g_hwt_cal_elapsed_ms = 0U;
/* mode仅显示：0未开始、1旧A6、2本轮0x48；内部地址启动时锁存。 */
volatile uint32_t g_hwt_cal_mode = 0U;
/* cause4只代表输出越限：记录当时值，退出后保持到下次获准试验。 */
volatile uint32_t g_hwt_cal_trip_kind = 0U, g_hwt_cal_trip_state = 0U;
volatile uint32_t g_hwt_cal_trip_ms = 0U;
volatile float g_hwt_cal_trip_wz_dps = 0.0f, g_hwt_cal_trip_delta_deg = 0.0f;
volatile float g_hwt_cal_trip_yaw_deg = 0.0f, g_hwt_cal_trip_prev_deg = 0.0f;
/* 仅记录本轮0x48采集中一次符合静止条件的输出归零。 */
volatile uint32_t g_hwt_cal_rezero_count = 0U, g_hwt_cal_rezero_ms = 0U;
volatile float g_hwt_cal_rezero_prev_deg = 0.0f;
volatile uint32_t g_hwt_cal_resume_count = 0U;
static uint32_t s_cal_rezero_seen = 0U, s_cal_resume_seen = 0U;
static float s_cal_rezero_origin = 0.0f;
static uint8_t s_cal_register = 0xA6U;
volatile float g_hwt_cal_before_deg = 0.0f, g_hwt_cal_after_deg = 0.0f;
static uint32_t s_cal_tick, s_cal_total_tick, s_cal_epoch, s_cal_bad_sum;
static uint32_t s_cal_angle_count, s_cal_end_attempts;
static float s_cal_last_yaw, s_cal_still_yaw;
static uint32_t s_cal_failed, s_cal_started;

static uint32_t s_version_tick, s_version_rx_base, s_version_error;
static uint32_t s_version_start_tick;
/* 静态一次性50Hz设置：0未请求，1读回8，2发送超时，3回复超时，
 * 4错误/互斥，5读回非8，9入口拒绝。result1仍须实测频率及断电保持。 */
volatile uint32_t g_hwt_rate50_request = 0U, g_hwt_rate50_active = 0U;
volatile uint32_t g_hwt_rate50_result = 0U, g_hwt_rate50_state = 0U;
volatile uint32_t g_hwt_rate50_command = 0U, g_hwt_rate50_raw = 0U;
volatile uint8_t g_hwt_rate50_reply[11] = {0U};
static uint32_t s_rate50_used = 0U, s_rate50_tick, s_rate50_start;
static uint32_t s_rate50_epoch, s_rate50_rx_base, s_rate50_read_tick;
static const uint8_t s_rate50_commands[4][5] = {
    {0xFFU,0xAAU,0x69U,0x88U,0xB5U}, /* 解锁；绝不能用0x76归零替代 */
    {0xFFU,0xAAU,0x03U,0x08U,0x00U}, /* 仅修改RRATE到50Hz */
    {0xFFU,0xAAU,0x00U,0x00U,0x00U}, /* 保存，不重启/不恢复出厂 */
    {0xFFU,0xAAU,0x27U,0x03U,0x00U}  /* 只读回RRATE */
};

static uint32_t Hwt_Rate50Safe(void)
{
    return ((APP_HWT_STATIC_TEST == 1U) && (APP_HWT_TURN_TEST == 0U) &&
        (APP_HEADING_SESSION_TEST == 0U) && (g_vm_test_request == 0U) &&
        (g_rtos_fault == 0U) && (g_safety_fault == 0U) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U) && (g_hwt_rrate_request == 0U)) ? 1U : 0U;
}

static void Hwt_Rate50Finish(uint32_t result)
{
    taskENTER_CRITICAL();
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    s_tx_owner = 0U;
    g_hwt_rate50_result = result;
    g_hwt_rate50_active = 0U;
    g_hwt_rate50_state = 0U;
    taskEXIT_CRITICAL();
}

static void Hwt_Rate50Send(uint32_t now)
{
    taskENTER_CRITICAL();
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    s_tx_command = s_rate50_commands[g_hwt_rate50_command];
    s_version_index = 0U; s_tx_owner = 3U;
    s_rate50_tick = now;
    if (g_hwt_rate50_command == 3U)
    {
        s_rate50_rx_base = s_parser.register_count;
        s_rate50_read_tick = now;
    }
    g_hwt_rate50_state = 1U;
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
}

/* 仅由TaskA的HWT101_Update调用，全部等待按状态推进，不阻塞任务。 */
static void Hwt_Rate50Update(uint32_t now)
{
    uint32_t i;
    if (g_hwt_rate50_active == 0U)
    {
        if (g_hwt_rate50_request == 0U) return;
        i = g_hwt_rate50_request;
        g_hwt_rate50_request = 0U;
        if ((i != 1U) || s_rate50_used || !Hwt_Rate50Safe() ||
            (g_hwt_version_result != 1U) || (g_hwt_valid_mask != 3U) ||
            (g_hwt_rrate_result != 1U) || (g_hwt_rrate_raw != 6U) ||
            ((uint32_t)(now - s_parser.register_tick) < 1500U))
        { g_hwt_rate50_result = 9U; return; }
        s_rate50_used = 1U; g_hwt_rate50_active = 1U;
        g_hwt_rate50_result = 0U; g_hwt_rate50_raw = 0U;
        g_hwt_rate50_command = 0U;
        s_rate50_epoch = s_error_epoch; s_rate50_start = now;
        Hwt_Rate50Send(now);
        return;
    }
    g_hwt_rate50_request = 0U; /* 忙时不排队，也不重新发送 */
    if (!Hwt_Rate50Safe() || (s_error_epoch != s_rate50_epoch) ||
        ((uint32_t)(now - s_rate50_start) >= 8000U))
    {
        /* 冲突请求丢弃，防止退出本事务后紧接着发送其他命令。 */
        g_hwt_version_request = g_hwt_a7_request = 0U;
        g_hwt_rrate_request = g_hwt_cal_request = 0U;
        Hwt_Rate50Finish(4U); return;
    }
    if (g_hwt_rate50_state == 1U)
    {
        if ((uint32_t)(now - s_rate50_tick) >= 200U)
        { Hwt_Rate50Finish(2U); return; }
        if (s_version_index < 5U) return;
        if (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) return;
        s_tx_owner = 0U;
        if (g_hwt_rate50_command == 3U)
            g_hwt_rate50_state = 3U;
        else
        { g_hwt_rate50_state = 2U; s_rate50_tick = now; }
    }
    if (g_hwt_rate50_state == 2U)
    {
        /* 命令间至少100ms；保存后留1500ms静默隔离再读回。 */
        i = (g_hwt_rate50_command == 2U) ? 1500U : 100U;
        if ((uint32_t)(now - s_rate50_tick) < i) return;
        g_hwt_rate50_command++;
        Hwt_Rate50Send(now);
        return;
    }
    if (g_hwt_rate50_state == 3U)
    {
        if ((uint32_t)(now - s_rate50_read_tick) >= 1000U)
        { Hwt_Rate50Finish(3U); return; }
        if ((s_parser.register_count == s_rate50_rx_base) ||
            ((uint32_t)(s_parser.register_tick - s_rate50_read_tick) >= 1000U) ||
            ((uint32_t)(now - s_parser.register_tick) >= 1000U)) return;
        g_hwt_rate50_raw = (uint32_t)s_parser.register_data[0] |
            ((uint32_t)s_parser.register_data[1] << 8);
        g_hwt_rate50_reply[0] = 0x55U; g_hwt_rate50_reply[1] = 0x5FU;
        for (i = 0U; i < 8U; i++)
            g_hwt_rate50_reply[i + 2U] = s_parser.register_data[i];
        g_hwt_rate50_reply[10] = 0U;
        for (i = 0U; i < 10U; i++) g_hwt_rate50_reply[10] =
            (uint8_t)(g_hwt_rate50_reply[10] + g_hwt_rate50_reply[i]);
        Hwt_Rate50Finish((g_hwt_rate50_raw == 8U) ? 1U : 5U);
    }
}

static void Hwt_VersionUpdate(uint32_t now)
{
    uint32_t i, errors;
    volatile uint32_t *result;
    volatile uint8_t *reply;
    if (g_hwt_rate50_active != 0U) return;
    /* 读RRATE过程中失去静态条件立即结束，不等待发送/回复截止。 */
    if ((s_read_rrate != 0U) && (g_hwt_version_state != 0U) &&
        ((APP_HWT_STATIC_TEST != 1U) || (APP_HWT_TURN_TEST != 0U) ||
         (APP_HEADING_SESSION_TEST != 0U) || (g_vm_test_request != 0U) ||
         (g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
         (g_hwt_cal_request != 0U) || (g_hwt_cal_active != 0U)))
    {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        g_hwt_rrate_request = 0U; g_hwt_rrate_result = 4U;
        g_hwt_version_state = 0U; return;
    }
    /* 只读RRATE仅在静态VM0模式受理，不改变原版本/A7行为。 */
    if ((g_hwt_rrate_request != 0U) &&
        ((APP_HWT_STATIC_TEST != 1U) || (APP_HWT_TURN_TEST != 0U) ||
         (APP_HEADING_SESSION_TEST != 0U) || (g_vm_test_request != 0U) ||
         (g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
         (g_hwt_cal_request != 0U) || (g_hwt_cal_active != 0U)))
    { g_hwt_rrate_request = 0U; g_hwt_rrate_result = 9U; }
    if (g_hwt_cal_active != 0U)
    {
        if (g_hwt_a7_request != 0U) g_hwt_a7_result = 9U;
        g_hwt_a7_request = g_hwt_version_request = 0U;
        return;
    }
    errors = s_error_epoch;
    if (g_hwt_version_state == 0U)
    {
        if ((g_hwt_version_request != 1U) && (g_hwt_a7_request != 1U) &&
            (g_hwt_rrate_request != 1U)) return;
        /* 给模块上电初始化留2秒，不依赖用户计时，不重复发设置。 */
        if ((uint32_t)(now - s_version_start_tick) < 2000U) return;
        s_read_rrate = ((g_hwt_rrate_request == 1U) &&
            (g_hwt_version_request != 1U) && (g_hwt_a7_request != 1U)) ? 1U : 0U;
        /* 5F不回显地址；RRATE前要求已成功读版本，并留出静默隔离窗口。 */
        if ((s_read_rrate != 0U) && ((g_hwt_version_result != 1U) ||
            ((uint32_t)(now - s_parser.register_tick) < 1500U)))
        { g_hwt_rrate_request = 0U; g_hwt_rrate_result = 9U; return; }
        if ((g_hwt_rrate_request != 0U) && (s_read_rrate == 0U))
            g_hwt_rrate_result = 9U;
        g_hwt_rrate_request = 0U;
        s_read_a7 = ((s_read_rrate == 0U) && (g_hwt_version_request != 1U));
        if ((s_read_a7 == 0U) && (g_hwt_a7_request != 0U)) g_hwt_a7_result = 9U;
        g_hwt_version_request = g_hwt_a7_request = 0U;
        if (s_read_rrate != 0U) g_hwt_rrate_result = 0U;
        else if (s_read_a7 != 0U) g_hwt_a7_result = 0U;
        else g_hwt_version_result = 0U;
        s_version_index = 0U; s_version_tick = now;
        s_version_rx_base = s_parser.register_count;
        s_version_error = errors;
        taskENTER_CRITICAL();
        s_tx_command = s_read_rrate ? s_rrate_command :
            (s_read_a7 ? s_a7_command : s_version_command); s_tx_owner = 1U;
        g_hwt_version_state = 1U;
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        taskEXIT_CRITICAL();
    }
    else
    {
        /* 忙时消耗请求，不交错发送版本、A7或A6。 */
        if ((g_hwt_a7_request != 0U) && (s_read_a7 == 0U)) g_hwt_a7_result = 9U;
        if ((g_hwt_rrate_request != 0U) && (s_read_rrate == 0U))
            g_hwt_rrate_result = 9U;
        g_hwt_rrate_request = 0U;
        g_hwt_a7_request = g_hwt_version_request = 0U;
    }
    result = s_read_rrate ? &g_hwt_rrate_result :
        (s_read_a7 ? &g_hwt_a7_result : &g_hwt_version_result);
    reply = s_read_rrate ? g_hwt_rrate_reply :
        (s_read_a7 ? g_hwt_a7_reply : g_hwt_version_reply);
    if (errors != s_version_error)
    { USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
      *result = 4U; g_hwt_version_state = 0U; return; }
    if (g_hwt_version_state == 1U)
    {
        if ((uint32_t)(now - s_version_tick) >= 200U)
        { USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
          *result = 2U; g_hwt_version_state = 0U; return; }
        /* 字节由TXE中断连续发送，不受任务10ms轮询间隔影响。 */
        if (s_version_index < 5U) return;
        s_version_tick = s_version_last_tx_tick;
        g_hwt_version_state = 2U;
    }
    if (g_hwt_version_state == 2U)
    {
        if ((uint32_t)(now - s_version_tick) >= 200U)
        { USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
          *result = 2U; g_hwt_version_state = 0U; return; }
        if (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) return;
        g_hwt_version_state = 3U;
    }
    if (g_hwt_version_state == 3U)
    {
        if ((s_parser.register_count != s_version_rx_base) &&
            ((uint32_t)(s_parser.register_tick - s_version_tick) <= 1000U) &&
            ((uint32_t)(now - s_parser.register_tick) <= 1000U))
        {
            /* 用无符号类型组合，不把版本字节误作带符号角度。 */
            if (s_read_rrate != 0U)
            {
                g_hwt_rrate_raw = (uint32_t)s_parser.register_data[0] |
                    ((uint32_t)s_parser.register_data[1] << 8);
                g_hwt_rrate_reply_count++;
            }
            else if (s_read_a7 != 0U)
            {
                g_hwt_a7_raw = (uint32_t)s_parser.register_data[0] |
                    ((uint32_t)s_parser.register_data[1] << 8);
                g_hwt_a7_reply_count++;
            }
            else
            {
                g_hwt_version_raw = (uint32_t)s_parser.register_data[0] |
                    ((uint32_t)s_parser.register_data[1] << 8) |
                    ((uint32_t)s_parser.register_data[2] << 16) |
                    ((uint32_t)s_parser.register_data[3] << 24);
                g_hwt_version_low16 = g_hwt_version_raw & 0xFFFFU;
                g_hwt_version_reply_count++;
            }
            reply[0] = 0x55U; reply[1] = 0x5FU;
            for (i = 0U; i < 8U; i++) reply[i + 2U] = s_parser.register_data[i];
            reply[10] = 0U;
            for (i = 0U; i < 10U; i++) reply[10] =
                (uint8_t)(reply[10] + reply[i]);
            *result = 1U; g_hwt_version_state = 0U;
        }
        else if ((uint32_t)(now - s_version_tick) >= 1000U)
        { *result = 3U; g_hwt_version_state = 0U; }
    }
}

/* 显式静态试验：request1保留A6，request3为FAQ的0x48流程。
 * state:1静止确认1s，2基线60s，3发开始，4采集A6十秒/48二十秒，5发结束，
 * 6退出后静置1s，7观察60s，8完成，9失败。整个试验保持VM0。 */
static float Hwt_AngleDelta(float now, float old)
{
    float d = now - old;
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}
static uint32_t Hwt_CalTx(uint8_t value, uint32_t now)
{
    taskENTER_CRITICAL();
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    s_cal_command[2] = s_cal_register;
    s_cal_command[3] = value;
    s_tx_command = s_cal_command; s_tx_owner = 2U;
    s_version_index = 0U; s_cal_tick = now;
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    taskEXIT_CRITICAL();
    return 1U;
}
static void Hwt_CalFinish(uint32_t failed)
{
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    g_hwt_cal_active = 0U; s_tx_owner = 0U;
    g_hwt_cal_result = failed ? 2U : 1U;
    g_hwt_cal_state = failed ? 9U : 8U;
}
static void Hwt_CalEnd(uint32_t cause, uint32_t now)
{
    if (cause != 0U) { s_cal_failed = 1U; g_hwt_cal_cause = cause; }
    if (!s_cal_started) { Hwt_CalFinish(1U); return; }
    g_hwt_cal_state = 5U; s_cal_end_attempts = 1U;
    (void)Hwt_CalTx((s_cal_register == 0x48U) ? 0U : 4U, now); /* 发生异常仍尝试退出，不开始运动 */
}
static void Hwt_CalUpdate(uint32_t now)
{
    uint32_t event = g_hwt_cal_request;
    float delta = 0.0f, previous_yaw;
    uint32_t new_angle, trip_kind;
    if (g_hwt_rate50_active != 0U) return;
    g_hwt_cal_request = 0U;
    if (g_hwt_cal_active == 0U)
    {
        if ((event != 1U) && (event != 3U)) return;
        /* 请求1或3表示操作者确认整车没有搬动/滑动；软件不能单独证明静止。 */
        if (!APP_HWT_STATIC_TEST || (g_vm_test_request != 0U) ||
            (g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_hwt_version_state != 0U) || (g_hwt_version_request != 0U) ||
            (g_hwt_version_result != 1U) || (g_hwt_valid_mask != 3U) ||
            (g_hwt_cal_end_tx_result == 2U))
        { g_hwt_cal_result = 9U; g_hwt_cal_cause = 1U; return; }
        s_cal_register = (event == 3U) ? 0x48U : 0xA6U;
        g_hwt_cal_mode = (event == 3U) ? 2U : 1U;
        g_hwt_cal_result = g_hwt_cal_cause = 0U;
        g_hwt_cal_trip_kind = g_hwt_cal_trip_state = g_hwt_cal_trip_ms = 0U;
        g_hwt_cal_trip_wz_dps = g_hwt_cal_trip_delta_deg = 0.0f;
        g_hwt_cal_trip_yaw_deg = g_hwt_cal_trip_prev_deg = 0.0f;
        s_cal_rezero_seen = s_cal_resume_seen = 0U;
        s_cal_rezero_origin = 0.0f;
        g_hwt_cal_resume_count = 0U;
        g_hwt_cal_rezero_count = g_hwt_cal_rezero_ms = 0U;
        g_hwt_cal_rezero_prev_deg = 0.0f;
        g_hwt_cal_start_tx_result = g_hwt_cal_end_tx_result = 0U;
        g_hwt_cal_before_deg = g_hwt_cal_after_deg = 0.0f;
        s_cal_tick = s_cal_total_tick = now;
        s_cal_epoch = s_error_epoch; s_cal_bad_sum = s_parser.checksum_errors;
        s_cal_angle_count = s_parser.angle_count;
        s_cal_last_yaw = s_cal_still_yaw = g_hwt_yaw_deg;
        s_cal_failed = s_cal_started = 0U;
        g_hwt_cal_state = 1U; g_hwt_cal_active = 1U;
    }
    g_hwt_cal_elapsed_ms = now - s_cal_total_tick;
    /* 退出发送优先；即使RX坏了也不能省略退出尝试。最多两次，均有截止。 */
    if (g_hwt_cal_state == 5U)
    {
        if ((s_version_index == 5U) && (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET))
        {
            g_hwt_cal_end_tx_result = 1U; s_tx_owner = 0U;
            if (s_cal_failed) Hwt_CalFinish(1U);
            else { g_hwt_cal_state = 6U; s_cal_tick = now; }
        }
        else if ((uint32_t)(now - s_cal_tick) >= 200U)
        {
            if (s_cal_end_attempts < 2U)
            { s_cal_end_attempts++; (void)Hwt_CalTx((s_cal_register == 0x48U) ? 0U : 4U, now); }
            else
            { g_hwt_cal_end_tx_result = 2U; g_hwt_cal_cause = 6U; Hwt_CalFinish(1U); }
        }
        return;
    }
    if (event == 2U) { Hwt_CalEnd(7U, now); return; }
    if ((g_vm_test_request != 0U) || (g_rtos_fault != 0U) || (g_safety_fault != 0U))
    { Hwt_CalEnd(1U, now); return; }
    if ((g_hwt_valid_mask != 3U) || (s_cal_epoch != s_error_epoch) ||
        (s_cal_bad_sum != s_parser.checksum_errors))
    { Hwt_CalEnd(3U, now); return; }
    if ((uint32_t)(now - s_cal_total_tick) >= ((s_cal_register == 0x48U) ? 150000U : 140000U))
    { Hwt_CalEnd(8U, now); return; }
    previous_yaw = s_cal_last_yaw;
    new_angle = (s_cal_angle_count != s_parser.angle_count);
    if (new_angle)
    {
        delta = Hwt_AngleDelta(g_hwt_yaw_deg, s_cal_last_yaw);
        s_cal_last_yaw = g_hwt_yaw_deg; s_cal_angle_count = s_parser.angle_count;
    }
    /* 本次实物在0x48采集中出现角度直接归零。只接受一次精确0度、
     * 新角度样本且低角速度的基准切换，不重启20s计时或修改模块参数。
     * A6、非零跳变、再次归零及速率/通讯异常仍保护；退出恢复另见下方。 */
    if ((s_cal_register == 0x48U) && (g_hwt_cal_state == 4U) &&
        (s_cal_rezero_seen == 0U) && new_angle && (s_parser.yaw_raw == 0) &&
        (g_hwt_wz_dps <= 0.5f) && (g_hwt_wz_dps >= -0.5f) &&
        ((delta > 2.0f) || (delta < -2.0f)))
    {
        s_cal_rezero_seen = 1U;
        g_hwt_cal_rezero_count = 1U;
        g_hwt_cal_rezero_ms = now - s_cal_total_tick;
        g_hwt_cal_rezero_prev_deg = previous_yaw;
        s_cal_rezero_origin = previous_yaw;
        delta = 0.0f; /* 上面已把last_yaw更新为0；不把基准切换计作转动。 */
    }
    /* 与已记录的采集归零配对：48正常模式TC完成后的1s静置窗口，
     * 只接受一次0度恢复到归零前角度附近（跨界差<=2度），并要求低角速度。
     * 此窗口尚未开始后60s测量；不重启静置计时，不吞掉后采样跳变。 */
    if ((s_cal_register == 0x48U) && (g_hwt_cal_state == 6U) &&
        (s_cal_rezero_seen != 0U) && (s_cal_resume_seen == 0U) &&
        new_angle && (previous_yaw == 0.0f) &&
        (g_hwt_wz_dps <= 0.5f) && (g_hwt_wz_dps >= -0.5f) &&
        ((delta > 2.0f) || (delta < -2.0f)))
    {
        float origin_delta = Hwt_AngleDelta(g_hwt_yaw_deg, s_cal_rezero_origin);
        if ((origin_delta <= 2.0f) && (origin_delta >= -2.0f))
        {
            s_cal_resume_seen = 1U;
            g_hwt_cal_resume_count = 1U;
            delta = 0.0f; /* last_yaw已指向真实恢复值，仅重建本地比较基准。 */
        }
    }
    /* 输出越限不等于人工移动；校准过程可能改变输出，先锁存证据。 */
    trip_kind = 0U;
    if ((g_hwt_wz_dps > 2.0f) || (g_hwt_wz_dps < -2.0f)) trip_kind |= 1U;
    if ((delta > 2.0f) || (delta < -2.0f)) trip_kind |= 2U;
    if (trip_kind != 0U)
    {
        g_hwt_cal_trip_kind = trip_kind;
        g_hwt_cal_trip_state = g_hwt_cal_state;
        g_hwt_cal_trip_ms = now - s_cal_total_tick;
        g_hwt_cal_trip_wz_dps = g_hwt_wz_dps;
        g_hwt_cal_trip_delta_deg = delta;
        g_hwt_cal_trip_yaw_deg = g_hwt_yaw_deg;
        g_hwt_cal_trip_prev_deg = previous_yaw;
        Hwt_CalEnd(4U, now); return;
    }
    if (g_hwt_cal_state == 1U)
    {
        float d = Hwt_AngleDelta(g_hwt_yaw_deg, s_cal_still_yaw);
        if ((g_hwt_wz_dps > 0.5f) || (g_hwt_wz_dps < -0.5f) || (d > 0.5f) || (d < -0.5f))
        { Hwt_CalEnd(2U, now); return; }
        if ((uint32_t)(now - s_cal_tick) >= 1000U)
        { g_hwt_cal_state = 2U; s_cal_tick = now; }
    }
    else if (g_hwt_cal_state == 2U)
    {
        g_hwt_cal_before_deg += delta;
        if ((uint32_t)(now - s_cal_tick) >= 60000U)
        {
            g_hwt_cal_state = 3U; s_cal_started = 1U;
            (void)Hwt_CalTx(1U, now);
        }
    }
    else if (g_hwt_cal_state == 3U)
    {
        if ((s_version_index == 5U) && (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET))
        { g_hwt_cal_start_tx_result = 1U; s_tx_owner = 0U; g_hwt_cal_state = 4U; s_cal_tick = now; }
        else if ((uint32_t)(now - s_cal_tick) >= 200U)
        { g_hwt_cal_start_tx_result = 2U; Hwt_CalEnd(5U, now); }
    }
    else if (g_hwt_cal_state == 4U)
    {
        if ((uint32_t)(now - s_cal_tick) >= ((s_cal_register == 0x48U) ? 20000U : 10000U)) Hwt_CalEnd(0U, now);
    }
    else if (g_hwt_cal_state == 6U)
    {
        if ((uint32_t)(now - s_cal_tick) >= 1000U)
        { g_hwt_cal_state = 7U; s_cal_tick = now; }
    }
    else if (g_hwt_cal_state == 7U)
    {
        g_hwt_cal_after_deg += delta;
        if ((uint32_t)(now - s_cal_tick) >= 60000U) Hwt_CalFinish(0U);
    }
}
void HWT101_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_UP; gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_Init(GPIOD, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = APP_HWT_BAUD; uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 6U;
    nvic.NVIC_IRQChannelSubPriority = 0U; nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    Hwt101Parser_Init(&s_parser);
    USART_Cmd(USART2, ENABLE);
}
void HWT101_EnableReceive(void)
{
    (void)USART2->SR; (void)USART2->DR;
    s_version_start_tick = (uint32_t)xTaskGetTickCount();
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;
    uint8_t byte;
    uint16_t head;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        byte = (uint8_t)USART2->DR; /* SR后读DR清RXNE及错误 */
        if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
        {
            g_hwt_uart_errors++; s_error_epoch++;
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
        else
        {
            g_hwt_rx_bytes++; head = s_head;
            if ((uint16_t)(head - s_tail) >= 256U)
            { g_hwt_overflows++; s_error_epoch++; USART_ITConfig(USART2, USART_IT_TXE, DISABLE); }
            else
            {
                s_rx[head & 255U].data = byte;
                s_rx[head & 255U].tick = (uint32_t)xTaskGetTickCountFromISR();
                __DMB(); s_head = (uint16_t)(head + 1U);
            }
        }
    }
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        if ((((s_tx_owner == 1U) && (g_hwt_version_state == 1U)) ||
             ((s_tx_owner == 2U) && (g_hwt_cal_active != 0U)) ||
             ((s_tx_owner == 3U) && (g_hwt_rate50_active != 0U) &&
              (g_hwt_rate50_state == 1U))) && (s_version_index < 5U))
        {
            USART_SendData(USART2, s_tx_command[s_version_index]);
            g_hwt_tx_bytes++;
            s_version_last_tx_tick = (uint32_t)xTaskGetTickCountFromISR();
            __DMB(); s_version_index++;
            if (s_version_index == 5U) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
        else USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    }
}
/* 任务A每10ms调用；每轮最多128字节，ISR时间戳不以解析时间充新。 */
void HWT101_Update(void)
{
    uint32_t n, i, now;
    HwtByte item;
    for (n = 0U; n < 128U; n++)
    {
        taskENTER_CRITICAL();
        if (s_seen_error != s_error_epoch)
        {
            s_seen_error = s_error_epoch; s_tail = s_head;
            Hwt101Parser_Invalidate(&s_parser);
        }
        if (s_tail == s_head) { taskEXIT_CRITICAL(); break; }
        item = s_rx[s_tail & 255U]; s_tail = (uint16_t)(s_tail + 1U);
        taskEXIT_CRITICAL();
        Hwt101Parser_Feed(&s_parser, item.data, item.tick);
    }
    now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if (s_seen_error != s_error_epoch)
    { s_seen_error = s_error_epoch; s_tail = s_head; Hwt101Parser_Invalidate(&s_parser); }
    g_hwt_update_tick = now;
    g_hwt_valid_mask = Hwt101Parser_Valid(&s_parser, now);
    g_hwt_angle_count = s_parser.angle_count; g_hwt_rate_count = s_parser.rate_count;
    g_hwt_checksum_errors = s_parser.checksum_errors;
    g_hwt_angle_age_ms = (s_parser.seen & 1U) ? now - s_parser.angle_tick : 0xFFFFFFFFU;
    g_hwt_rate_age_ms = (s_parser.seen & 2U) ? now - s_parser.rate_tick : 0xFFFFFFFFU;
    g_hwt_yaw_deg = (float)s_parser.yaw_raw * (180.0f / 32768.0f);
    g_hwt_wz_dps = (float)s_parser.wz_raw * (2000.0f / 32768.0f);
    for (i = 0U; i < 11U; i++) g_hwt_last_frame[i] = s_parser.last_frame[i];
    taskEXIT_CRITICAL();
    Hwt_Rate50Update(now);
    Hwt_VersionUpdate(now);
    Hwt_CalUpdate(now);
}
