#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hwt101_parser.h"
#define APP_HWT_BAUD 115200U
#define USART_SR_RXNE 32U
#define USART_FLAG_TXE 128U
#define USART_FLAG_TC 64U
#define RESET 0U
#define USART_IT_TXE 128U
#define ENABLE 1U
#define DISABLE 0U
#define USART_SR_ORE 8U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_PE 1U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define __DMB() ((void)0)
typedef struct {uint32_t SR,DR;} MockUART;
static MockUART uart2;
#define USART2 (&uart2)
static uint32_t tick;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t xTaskGetTickCountFromISR(void){return tick;}
static unsigned tx_budget=1000,tc_ready=1,tx_length;static uint8_t tx_log[256];
static uint32_t USART_GetFlagStatus(MockUART *p,uint32_t flag){return (flag==USART_FLAG_TXE)?(tx_budget!=0):tc_ready;}
static void USART_SendData(MockUART *p,uint16_t v){if(tx_length<sizeof(tx_log))tx_log[tx_length++]=(uint8_t)v;if(tx_budget)tx_budget--;}
static unsigned tx_irq_enabled,auto_irq=1;
void USART2_IRQHandler(void);
static uint32_t USART_GetITStatus(MockUART *u,uint32_t it){return tx_irq_enabled&&(tx_budget!=0);}
static void USART_ITConfig(MockUART *u,uint32_t it,unsigned enable){unsigned i;tx_irq_enabled=enable;if(enable&&auto_irq)for(i=0;i<5&&tx_irq_enabled&&tx_budget;i++)USART2_IRQHandler();}

#define APP_HWT_STATIC_TEST 1U
volatile uint32_t g_vm_test_request=0,g_rtos_fault=0,g_safety_fault=0;
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
volatile float g_hwt_cal_before_deg = 0.0f, g_hwt_cal_after_deg = 0.0f;
static uint32_t s_cal_tick, s_cal_total_tick, s_cal_epoch, s_cal_bad_sum;
static uint32_t s_cal_angle_count, s_cal_end_attempts;
static float s_cal_last_yaw, s_cal_still_yaw;
static uint32_t s_cal_failed, s_cal_started;

static uint32_t s_version_tick, s_version_rx_base, s_version_error;
static uint32_t s_version_start_tick;

static void Hwt_VersionUpdate(uint32_t now)
{
    uint32_t i, errors;
    volatile uint32_t *result;
    volatile uint8_t *reply;
    if (g_hwt_cal_active != 0U)
    {
        if (g_hwt_a7_request != 0U) g_hwt_a7_result = 9U;
        g_hwt_a7_request = g_hwt_version_request = 0U;
        return;
    }
    errors = s_error_epoch;
    if (g_hwt_version_state == 0U)
    {
        if ((g_hwt_version_request != 1U) && (g_hwt_a7_request != 1U)) return;
        /* 给模块上电初始化留2秒，不依赖用户计时，不重复发设置。 */
        if ((uint32_t)(now - s_version_start_tick) < 2000U) return;
        s_read_a7 = (g_hwt_version_request != 1U);
        if ((s_read_a7 == 0U) && (g_hwt_a7_request != 0U)) g_hwt_a7_result = 9U;
        g_hwt_version_request = g_hwt_a7_request = 0U;
        if (s_read_a7 != 0U) g_hwt_a7_result = 0U;
        else g_hwt_version_result = 0U;
        s_version_index = 0U; s_version_tick = now;
        s_version_rx_base = s_parser.register_count;
        s_version_error = errors;
        taskENTER_CRITICAL();
        s_tx_command = s_read_a7 ? s_a7_command : s_version_command; s_tx_owner = 1U;
        g_hwt_version_state = 1U;
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        taskEXIT_CRITICAL();
    }
    else
    {
        /* 忙时消耗请求，不交错发送版本、A7或A6。 */
        if ((g_hwt_a7_request != 0U) && (s_read_a7 == 0U)) g_hwt_a7_result = 9U;
        g_hwt_a7_request = g_hwt_version_request = 0U;
    }
    result = s_read_a7 ? &g_hwt_a7_result : &g_hwt_version_result;
    reply = s_read_a7 ? g_hwt_a7_reply : g_hwt_version_reply;
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
            if (s_read_a7 != 0U)
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
/* A6试验基于用户明确同意的兼容性假设。发送完成不是设备确认。
 * state:1静止确认1s，2基线60s，3发开始，4采集10s，5发结束，
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
    (void)Hwt_CalTx(4U, now); /* 发生异常仍尝试退出，不开始运动 */
}
static void Hwt_CalUpdate(uint32_t now)
{
    uint32_t event = g_hwt_cal_request;
    float delta = 0.0f;
    uint32_t new_angle;
    g_hwt_cal_request = 0U;
    if (g_hwt_cal_active == 0U)
    {
        if (event != 1U) return;
        /* 按下request1表示操作者确认整车没有搬动/滑动；软件不能单独证明静止。 */
        if (!APP_HWT_STATIC_TEST || (g_vm_test_request != 0U) ||
            (g_rtos_fault != 0U) || (g_safety_fault != 0U) ||
            (g_hwt_version_state != 0U) || (g_hwt_version_request != 0U) ||
            (g_hwt_version_result != 1U) || (g_hwt_valid_mask != 3U) ||
            (g_hwt_cal_end_tx_result == 2U))
        { g_hwt_cal_result = 9U; g_hwt_cal_cause = 1U; return; }
        g_hwt_cal_result = g_hwt_cal_cause = 0U;
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
            { s_cal_end_attempts++; (void)Hwt_CalTx(4U, now); }
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
    if ((uint32_t)(now - s_cal_total_tick) >= 140000U)
    { Hwt_CalEnd(8U, now); return; }
    new_angle = (s_cal_angle_count != s_parser.angle_count);
    if (new_angle)
    {
        delta = Hwt_AngleDelta(g_hwt_yaw_deg, s_cal_last_yaw);
        s_cal_last_yaw = g_hwt_yaw_deg; s_cal_angle_count = s_parser.angle_count;
    }
    /* 开发候选异常运动检测，不代替用户对真实静止的确认。 */
    if ((g_hwt_wz_dps > 2.0f) || (g_hwt_wz_dps < -2.0f) ||
        (delta > 2.0f) || (delta < -2.0f))
    { Hwt_CalEnd(4U, now); return; }
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
        if ((uint32_t)(now - s_cal_tick) >= 10000U) Hwt_CalEnd(0U, now);
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
             ((s_tx_owner == 2U) && (g_hwt_cal_active != 0U))) && (s_version_index < 5U))
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
    g_hwt_valid_mask = Hwt101Parser_Valid(&s_parser, now);
    g_hwt_angle_count = s_parser.angle_count; g_hwt_rate_count = s_parser.rate_count;
    g_hwt_checksum_errors = s_parser.checksum_errors;
    g_hwt_angle_age_ms = (s_parser.seen & 1U) ? now - s_parser.angle_tick : 0xFFFFFFFFU;
    g_hwt_rate_age_ms = (s_parser.seen & 2U) ? now - s_parser.rate_tick : 0xFFFFFFFFU;
    g_hwt_yaw_deg = (float)s_parser.yaw_raw * (180.0f / 32768.0f);
    g_hwt_wz_dps = (float)s_parser.wz_raw * (2000.0f / 32768.0f);
    for (i = 0U; i < 11U; i++) g_hwt_last_frame[i] = s_parser.last_frame[i];
    taskEXIT_CRITICAL();
    Hwt_VersionUpdate(now);
    Hwt_CalUpdate(now);
}
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL line %d checks %u calState%u cause%u result%u tx%u start%u end%u attempts%u tick%u\n",__LINE__,checks,g_hwt_cal_state,g_hwt_cal_cause,g_hwt_cal_result,tx_length,g_hwt_cal_start_tx_result,g_hwt_cal_end_tx_result,s_cal_end_attempts,tick);return 1;}}while(0)
static void frame(uint8_t *f,uint8_t type,int32_t raw){unsigned i;uint16_t u=(uint16_t)raw;memset(f,0,11);f[0]=0x55;f[1]=type;f[6]=(uint8_t)u;f[7]=(uint8_t)(u>>8);for(i=0;i<10;i++)f[10]=(uint8_t)(f[10]+f[i]);}
static void feed(Hwt101Parser *p,uint8_t *f,uint32_t time){unsigned i;for(i=0;i<11;i++)Hwt101Parser_Feed(p,f[i],time);}
static void push(uint8_t v){uart2.SR=USART_SR_RXNE;uart2.DR=v;USART2_IRQHandler();uart2.SR=0;}
static void push_frame(uint8_t *f){unsigned i;for(i=0;i<11;i++)push(f[i]);}
static void reset_rx(void){g_hwt_a7_request=g_hwt_a7_result=g_hwt_a7_raw=g_hwt_a7_reply_count=s_read_a7=0;g_hwt_cal_request=g_hwt_cal_active=g_hwt_cal_state=g_hwt_cal_result=g_hwt_cal_cause=0;g_hwt_cal_start_tx_result=g_hwt_cal_end_tx_result=0;g_vm_test_request=g_rtos_fault=g_safety_fault=0;tx_irq_enabled=0;auto_irq=1;g_hwt_version_request=g_hwt_version_state=g_hwt_version_result=0;g_hwt_version_raw=g_hwt_version_low16=g_hwt_version_reply_count=g_hwt_tx_bytes=0;s_version_start_tick=0;tx_budget=1000;tc_ready=1;tx_length=0;s_head=s_tail=0;s_error_epoch=s_seen_error=0;g_hwt_rx_bytes=g_hwt_uart_errors=g_hwt_overflows=0;Hwt101Parser_Init(&s_parser);tick=100;HWT101_Update();}
int rx_checks(void){Hwt101Parser p;uint8_t f[11],g[11];unsigned i,j;int32_t v;uint32_t seed=7;
Hwt101Parser_Init(&p);CHECK(Hwt101Parser_Valid(&p,0)==0);Hwt101Parser_Feed(NULL,0,0);CHECK(Hwt101Parser_Valid(NULL,0)==0);
/* Signed bounds and randomized payloads including embedded 0x55. */
for(i=0;i<1000;i++){seed=seed*1664525U+1013904223U;v=(int32_t)(seed&65535U)-32768;Hwt101Parser_Init(&p);frame(f,0x53,v);feed(&p,f,100);CHECK(p.yaw_raw==v&&p.angle_count==1&&Hwt101Parser_Valid(&p,100)==1);frame(f,0x52,-v-(v==-32768));feed(&p,f,110);CHECK(p.wz_raw==-v-(v==-32768)&&Hwt101Parser_Valid(&p,110)==3);}
Hwt101Parser_Init(&p);frame(f,0x53,-32768);feed(&p,f,0);CHECK(p.yaw_raw==-32768);frame(f,0x52,32767);feed(&p,f,0);CHECK(p.wz_raw==32767);
CHECK(Hwt101Parser_Valid(&p,500)==3);CHECK(Hwt101Parser_Valid(&p,501)==0);frame(f,0x52,1);feed(&p,f,510);CHECK(Hwt101Parser_Valid(&p,510)==2);
Hwt101Parser_Init(&p);frame(f,0x53,16384);feed(&p,f,0xfffffff0U);CHECK(Hwt101Parser_Valid(&p,100)==1);CHECK(Hwt101Parser_Valid(&p,600)==0);
/* Bad checksum never updates value/tick; byte sliding recovers following frame. */
Hwt101Parser_Init(&p);frame(f,0x53,123);feed(&p,f,100);frame(g,0x53,456);g[10]^=1;feed(&p,g,110);CHECK(p.angle_count==1&&p.yaw_raw==123&&p.angle_tick==100&&Hwt101Parser_Valid(&p,110)==0);feed(&p,f,120);CHECK(p.angle_count==2&&Hwt101Parser_Valid(&p,120)==1);
Hwt101Parser_Init(&p);for(i=0;i<11;i++)if(i!=4)Hwt101Parser_Feed(&p,f[i],100);feed(&p,f,101);CHECK(p.angle_count==1);
Hwt101Parser_Init(&p);Hwt101Parser_Feed(&p,0x55,0);for(i=1;i<11;i++)Hwt101Parser_Feed(&p,f[i],100);CHECK(!p.angle_count);feed(&p,f,101);CHECK(p.angle_count==1);
Hwt101Parser_Init(&p);frame(g,0x51,1);feed(&p,g,100);CHECK(p.other_count==1&&!p.angle_count&&!Hwt101Parser_Valid(&p,100));
Hwt101Parser_Invalidate(&p);CHECK(p.length==0&&p.seen==0);
/* Actual RX ring/IRQ/Update: old arrival time cannot become fresh on drain. */
reset_rx();frame(f,0x53,16384);push_frame(f);frame(g,0x52,-16384);push_frame(g);HWT101_Update();CHECK(g_hwt_rx_bytes==22&&g_hwt_valid_mask==3&&g_hwt_yaw_deg==90.0f&&g_hwt_wz_dps==-1000.0f);
tick=601;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_angle_age_ms==501);
reset_rx();push_frame(f);tick=700;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_angle_age_ms==600);
/* Per-call budget, queued multiple frames, 16-bit ring counter rollover. */
reset_rx();for(j=0;j<22;j++)push_frame(f);HWT101_Update();CHECK(s_tail==128&&s_parser.angle_count==11);HWT101_Update();CHECK(s_tail==242&&s_parser.angle_count==22);
reset_rx();s_head=s_tail=65530U;push_frame(f);HWT101_Update();CHECK(s_parser.angle_count==1&&s_head==5&&s_tail==5&&g_hwt_valid_mask==1);
/* Every UART hardware error and overflow invalidates old/pending data. */
for(i=0;i<4;i++){static const uint32_t errors[4]={USART_SR_ORE,USART_SR_FE,USART_SR_NE,USART_SR_PE};reset_rx();push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);uart2.SR=USART_SR_RXNE|errors[i];uart2.DR=0x55;USART2_IRQHandler();uart2.SR=0;HWT101_Update();CHECK(g_hwt_valid_mask==0&&g_hwt_uart_errors==1&&s_tail==s_head);push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);}
reset_rx();push_frame(f);HWT101_Update();for(i=0;i<257;i++)push((uint8_t)i);HWT101_Update();CHECK(g_hwt_overflows==1&&g_hwt_valid_mask==0&&s_tail==s_head);push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1);
printf("PASS %u checks (actual parser and actual IRQ/ring/update, mocked UART/tick)\n",checks);return 0;}

static void start_version(void){reset_rx();s_version_start_tick=tick;g_hwt_version_request=1;tick+=2000;HWT101_Update();}
static void register_frame(uint8_t *f,uint32_t raw){unsigned i;memset(f,0,11);f[0]=0x55;f[1]=0x5f;for(i=0;i<4;i++)f[i+2]=(uint8_t)(raw>>(i*8));for(i=0;i<10;i++)f[10]=(uint8_t)(f[10]+f[i]);}
int version_checks(void){uint8_t f[11];unsigned previous,i;CHECK(rx_checks()==0);previous=checks;
/* Startup read sends exactly documented five bytes; never sends A6/A7/unlock/save. */
reset_rx();g_hwt_version_request=1;s_version_start_tick=tick;tick+=1999;HWT101_Update();CHECK(tx_length==0&&g_hwt_version_request==1);tick++;HWT101_Update();CHECK(tx_length==5&&g_hwt_tx_bytes==5&&g_hwt_version_state==3&&!g_hwt_version_request);CHECK(memcmp(tx_log,"\xff\xaa\x27\x2e\x00",5)==0);
register_frame(f,0x89fe4101U);tick+=20;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x89fe4101U&&g_hwt_version_low16==0x4101U&&g_hwt_version_reply_count==1&&g_hwt_version_state==0);CHECK(memcmp(f,(const void*)g_hwt_version_reply,11)==0);tick+=10000;HWT101_Update();CHECK(tx_length==5);
/* A preexisting register packet is not a reply to a new request. */
reset_rx();register_frame(f,7);push_frame(f);HWT101_Update();CHECK(s_parser.register_count==1);g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==3&&g_hwt_version_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3&&tx_length==5);
/* No unbounded TX/RX waits, partial TX and stalled TC diagnosed separately. */
reset_rx();tx_budget=0;g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==1&&tx_length==0);tick+=200;HWT101_Update();CHECK(g_hwt_version_result==2&&g_hwt_version_state==0&&tx_length==0);
reset_rx();g_hwt_version_request=1;tick=2100;for(i=0;i<5;i++){tx_budget=1;if(tx_irq_enabled)USART2_IRQHandler();HWT101_Update();tick+=10;}CHECK(tx_length==5&&g_hwt_version_state==3);
reset_rx();tc_ready=0;g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(g_hwt_version_state==2);tick+=200;HWT101_Update();CHECK(g_hwt_version_result==2&&g_hwt_version_state==0);
start_version();register_frame(f,123);f[10]^=1;tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==0&&g_hwt_checksum_errors==1);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3);
/* Valid angle/gyro output is not mistaken for a version, and continues while waiting. */
start_version();frame(f,0x53,16384);tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_valid_mask==1&&g_hwt_yaw_deg==90.0f&&g_hwt_version_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_version_result==3);
/* Duplicate request while busy is consumed; no automatic endless retries. */
start_version();g_hwt_version_request=1;tick+=20;HWT101_Update();CHECK(!g_hwt_version_request&&tx_length==5);tick+=1000;HWT101_Update();tick+=10000;HWT101_Update();CHECK(tx_length==5&&g_hwt_version_result==3);
/* Real RX-error epoch cancels even when a reply was buffered; explicit retry only. */
start_version();register_frame(f,42);push_frame(f);uart2.SR=USART_SR_ORE;USART2_IRQHandler();uart2.SR=0;HWT101_Update();CHECK(g_hwt_version_result==4&&g_hwt_version_raw==0&&g_hwt_version_state==0);g_hwt_version_request=1;HWT101_Update();CHECK(tx_length==10);register_frame(f,0xffffffffU);tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0xffffffffU);
/* Timestamp wrap and late reply cannot masquerade as an in-window success. */
reset_rx();s_version_start_tick=0xfffff000U;tick=s_version_start_tick+2000U;g_hwt_version_request=1;HWT101_Update();tick+=100;register_frame(f,0x12345678U);push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x12345678U);
start_version();tick+=1100;register_frame(f,99);push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==3&&g_hwt_version_raw==0);
/* IRQ must transmit with no RXNE and no 10ms task calls between bytes. */
reset_rx();auto_irq=0;g_hwt_version_request=1;tick=2100;HWT101_Update();CHECK(tx_irq_enabled&&tx_length==0&&g_hwt_version_state==1);for(i=0;i<5;i++){uart2.SR=0;USART2_IRQHandler();}CHECK(tx_length==5&&!tx_irq_enabled&&g_hwt_tx_bytes==5);CHECK(memcmp(tx_log,"\xff\xaa\x27\x2e\x00",5)==0);HWT101_Update();CHECK(g_hwt_version_state==3);
/* An early reply is retained even before next task observes TC. */
register_frame(f,0x12345678U);tick++;push_frame(f);HWT101_Update();CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x12345678U);
/* Mixed RX+TX in one IRQ does not skip either direction. */
reset_rx();auto_irq=0;g_hwt_version_request=1;tick=2100;HWT101_Update();uart2.SR=USART_SR_RXNE;uart2.DR=0x55;USART2_IRQHandler();uart2.SR=0;CHECK(g_hwt_rx_bytes==1&&tx_length==1);
/* Stalled interrupt shuts off TXE at deadline and cannot leak late bytes. */
reset_rx();auto_irq=0;g_hwt_version_request=1;tick=2100;HWT101_Update();tick+=200;HWT101_Update();CHECK(g_hwt_version_result==2&&!tx_irq_enabled);USART2_IRQHandler();CHECK(tx_length==0);
/* Hardware RX fault while sending cancels remaining bytes immediately. */
reset_rx();auto_irq=0;g_hwt_version_request=1;tick=2100;HWT101_Update();USART2_IRQHandler();uart2.SR=USART_SR_ORE;USART2_IRQHandler();uart2.SR=0;CHECK(tx_length==1&&!tx_irq_enabled);HWT101_Update();CHECK(g_hwt_version_result==4);
printf("PASS %u new version checks; %u total (mocked UART/tick, actual parser/transport/version logic)\n",checks-previous,checks);return 0;}

static float yaw_sim;
static void cal_frames(void){uint8_t f[11];int32_t raw;while(yaw_sim>=180)yaw_sim-=360;while(yaw_sim<-180)yaw_sim+=360;raw=(int32_t)(yaw_sim*32768.0f/180.0f);frame(f,0x53,raw);push_frame(f);frame(f,0x52,0);push_frame(f);}
static void cal_base(void){reset_rx();g_hwt_version_result=1;g_hwt_version_request=0;yaw_sim=0;cal_frames();HWT101_Update();}
static void cal_trigger(void){g_hwt_cal_request=1;HWT101_Update();}
static void cal_pump(unsigned ms,int improved){unsigned t;for(t=10;t<=ms;t+=10){tick+=10;if(t%100==0){yaw_sim+=(g_hwt_cal_state>=7&&improved)?0.0001f:0.001f;cal_frames();}HWT101_Update();}}
int cal_checks(void){unsigned before,i;uint8_t f[11];CHECK(version_checks()==0);before=checks;
cal_base();cal_pump(150000,0);CHECK(!g_hwt_cal_active&&g_hwt_cal_state==0&&tx_length==0);
cal_base();cal_trigger();CHECK(g_hwt_cal_active&&g_hwt_cal_state==1&&!tx_length);cal_pump(60990,1);CHECK(g_hwt_cal_state==2&&!tx_length);cal_pump(10,1);CHECK(g_hwt_cal_state==3&&tx_length==5);CHECK(memcmp(tx_log,"\xff\xaa\xa6\x01\x00",5)==0);cal_pump(73000,1);CHECK(g_hwt_cal_result==1&&g_hwt_cal_state==8&&!g_hwt_cal_active);CHECK(tx_length==10&&memcmp(tx_log+5,"\xff\xaa\xa6\x04\x00",5)==0);CHECK(g_hwt_cal_start_tx_result==1&&g_hwt_cal_end_tx_result==1);CHECK(g_hwt_cal_before_deg>0.58f&&g_hwt_cal_before_deg<0.62f&&g_hwt_cal_after_deg>0.04f&&g_hwt_cal_after_deg<0.08f);
/* Unsupported/ineffective command cannot be inferred from transmit success: equal drift. */
cal_base();cal_trigger();cal_pump(134000,0);CHECK(g_hwt_cal_result==1&&tx_length==10);CHECK(g_hwt_cal_before_deg>0.58f&&g_hwt_cal_after_deg>0.58f);
/* Duplicate request cannot extend time or queue version IO into the capture. */
cal_base();cal_trigger();cal_pump(10000,0);i=s_cal_total_tick;g_hwt_cal_request=1;g_hwt_version_request=1;HWT101_Update();CHECK(s_cal_total_tick==i&&!g_hwt_version_request&&!g_hwt_cal_request&&tx_length==0);cal_pump(124000,0);CHECK(g_hwt_cal_result==1&&tx_length==10);
cal_base();cal_trigger();g_hwt_cal_request=2;HWT101_Update();CHECK(g_hwt_cal_result==2&&g_hwt_cal_cause==7&&!tx_length);
cal_base();cal_trigger();cal_pump(61500,0);CHECK(g_hwt_cal_state==4);g_hwt_cal_request=2;HWT101_Update();HWT101_Update();CHECK(g_hwt_cal_result==2&&g_hwt_cal_cause==7&&g_hwt_cal_end_tx_result==1&&tx_length==10);
/* Freshness / VM / movement / UART / checksum errors during capture still exit. */
for(i=0;i<5;i++){cal_base();cal_trigger();cal_pump(61500,0);if(i==0)tick+=600;if(i==1)g_vm_test_request=1;if(i==2){frame(f,0x52,100);push_frame(f);}if(i==3){uart2.SR=USART_SR_ORE;USART2_IRQHandler();uart2.SR=0;}if(i==4){frame(f,0x52,0);f[10]^=1;push_frame(f);}HWT101_Update();HWT101_Update();CHECK(!g_hwt_cal_active&&g_hwt_cal_result==2&&g_hwt_cal_end_tx_result==1&&tx_length==10);CHECK(g_hwt_cal_cause==((i==1)?1:(i==2)?4:3));}
/* Gates before A6, partial TX bounded, two exit attempts maximum. */
cal_base();s_parser.seen=1;cal_trigger();CHECK(g_hwt_cal_result==9&&!tx_length);
cal_base();g_hwt_version_result=3;cal_trigger();CHECK(g_hwt_cal_result==9&&!tx_length);
cal_base();cal_trigger();frame(f,0x52,16);push_frame(f);HWT101_Update();CHECK(g_hwt_cal_result==2&&g_hwt_cal_cause==2&&!tx_length);
cal_base();cal_trigger();auto_irq=0;tx_budget=0;cal_pump(62000,0);CHECK(g_hwt_cal_result==2&&g_hwt_cal_start_tx_result==2&&g_hwt_cal_end_tx_result==2&&s_cal_end_attempts==2&&!tx_irq_enabled);i=tx_length;cal_trigger();CHECK(g_hwt_cal_result==9&&tx_length==i);
/* A stalled start may recover only to send exit, never restart collection. */
cal_base();cal_trigger();cal_pump(60000,0);auto_irq=0;tx_budget=0;cal_pump(1200,0);CHECK(g_hwt_cal_state==5);tx_budget=1000;for(i=0;i<5;i++)USART2_IRQHandler();HWT101_Update();CHECK(g_hwt_cal_result==2&&g_hwt_cal_cause==5&&g_hwt_cal_end_tx_result==1&&tx_length==5);CHECK(memcmp(tx_log,"\xff\xaa\xa6\x04\x00",5)==0);
/* Independent overall lease and unsigned time wrap. */
cal_base();cal_trigger();cal_pump(61500,0);tick=s_cal_total_tick+140000U;cal_frames();HWT101_Update();HWT101_Update();CHECK(g_hwt_cal_result==2&&g_hwt_cal_cause==8&&g_hwt_cal_end_tx_result==1);
cal_base();tick=0xffff0000U;cal_frames();HWT101_Update();cal_trigger();cal_pump(134000,1);CHECK(g_hwt_cal_result==1&&g_hwt_cal_state==8&&tx_length==10);
printf("PASS %u A6-trial checks; %u total (mocked UART/tick and synthetic drift, actual state machines)\n",checks-before,checks);return 0;}

static void start_a7(void){reset_rx();tick=2100;g_hwt_version_result=1;g_hwt_version_raw=0x000a27e2U;g_hwt_a7_request=1;HWT101_Update();}
int main(void){uint8_t f[11];unsigned n,i;CHECK(cal_checks()==0);n=checks;
reset_rx();tick=4000;HWT101_Update();CHECK(tx_length==0&&g_hwt_a7_request==0);
for(i=0;i<3;i++){
 start_a7();CHECK(tx_length==5&&memcmp(tx_log,"\xff\xaa\x27\xa7\x00",5)==0&&!g_hwt_a7_request);
 register_frame(f,i==2?0x1234U:i);tick+=10;push_frame(f);HWT101_Update();
 CHECK(g_hwt_a7_result==1&&g_hwt_a7_raw==(i==2?0x1234U:i)&&g_hwt_a7_reply_count==1);
 CHECK(g_hwt_version_result==1&&g_hwt_version_raw==0x000a27e2U&&g_hwt_version_reply_count==0);
 CHECK(memcmp((const void*)g_hwt_a7_reply,f,11)==0);tick+=5000;HWT101_Update();CHECK(tx_length==5);
}
start_a7();tick+=1000;HWT101_Update();CHECK(g_hwt_a7_result==3&&g_hwt_version_result==1);
start_a7();register_frame(f,1);f[10]^=1;tick+=10;push_frame(f);HWT101_Update();CHECK(g_hwt_a7_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_a7_result==3);
start_a7();tick+=1100;register_frame(f,1);push_frame(f);HWT101_Update();CHECK(g_hwt_a7_result==3&&g_hwt_a7_reply_count==0);
start_a7();uart2.SR=USART_SR_ORE;USART2_IRQHandler();uart2.SR=0;HWT101_Update();CHECK(g_hwt_a7_result==4&&g_hwt_version_result==1);
reset_rx();tick=2100;tx_budget=0;g_hwt_a7_request=1;HWT101_Update();tick+=200;HWT101_Update();CHECK(g_hwt_a7_result==2&&tx_length==0&&!tx_irq_enabled);
reset_rx();tick=2100;tc_ready=0;g_hwt_a7_request=1;HWT101_Update();tick+=200;HWT101_Update();CHECK(g_hwt_a7_result==2&&tx_length==5);
start_a7();g_hwt_a7_request=g_hwt_version_request=1;HWT101_Update();CHECK(!g_hwt_a7_request&&!g_hwt_version_request&&tx_length==5);register_frame(f,1);tick++;push_frame(f);HWT101_Update();CHECK(g_hwt_a7_result==1&&g_hwt_version_raw==0x000a27e2U);
start_version();g_hwt_a7_request=1;HWT101_Update();CHECK(g_hwt_a7_result==9&&!g_hwt_a7_request&&tx_length==5);
reset_rx();tick=2100;g_hwt_version_request=g_hwt_a7_request=1;HWT101_Update();CHECK(g_hwt_a7_result==9&&memcmp(tx_log,"\xff\xaa\x27\x2e\x00",5)==0);
cal_base();cal_trigger();g_hwt_a7_request=1;HWT101_Update();CHECK(g_hwt_a7_result==9&&!g_hwt_a7_request&&g_hwt_cal_active&&tx_length==0);
start_a7();cal_frames();g_hwt_cal_request=1;HWT101_Update();CHECK(g_hwt_cal_result==9&&!g_hwt_cal_active&&tx_length==5);
reset_rx();register_frame(f,1);push_frame(f);HWT101_Update();tick=2100;g_hwt_a7_request=1;HWT101_Update();CHECK(g_hwt_a7_result==0);tick+=1000;HWT101_Update();CHECK(g_hwt_a7_result==3);
start_a7();frame(f,0x53,16384);tick++;push_frame(f);HWT101_Update();CHECK(g_hwt_yaw_deg==90.0f&&g_hwt_a7_result==0);
printf("PASS %u A7 checks; %u total (actual driver/parser; simulated UART/tick)\n",checks-n,checks);return 0;}
