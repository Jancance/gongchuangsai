#include "carrier.h"
#include "carrier_pwm.h"
#include "carrier_config.h"

volatile uint32_t g_carrier_test_enable = 0U;
volatile uint32_t g_carrier_request = 0U;
volatile float g_carrier_debug_angle = CARRIER_TEST_ANGLE_DEG;
volatile float g_carrier_latched_angle = 0.0f;
volatile uint32_t g_carrier_init_result = 0U;
volatile uint32_t g_carrier_state = 0U, g_carrier_result = 0U;
volatile uint32_t g_carrier_pulse_us = 0U, g_carrier_signal_active = 0U;
volatile uint32_t g_carrier_elapsed_ms = 0U, g_carrier_rejected_count = 0U;
static uint32_t s_session = 0U, s_started = 0U, s_last_service = 0U;
static uint32_t s_boot = 0U, s_boot_timed = 0U;

uint32_t Carrier_IsBusy(void) { return s_session; }

void Carrier_Init(void)
{
    g_carrier_init_result = CarrierPwm_Init();
    /* 用户要求：初始化后立即发0度，不依赖VM。启动阶段占用机构互锁。 */
    if (g_carrier_init_result != 1U) { g_carrier_state = 9U; g_carrier_result = 4U; return; }
#if CARRIER_BOOT_ZERO_ENABLE
    if (CarrierPwm_Write(CARRIER_PULSE_MIN_US) != 1U)
    { CarrierPwm_Off(); g_carrier_state = 9U; g_carrier_result = 6U; return; }
    g_carrier_debug_angle = CARRIER_SLOT_1_DEG;
    g_carrier_latched_angle = CARRIER_SLOT_1_DEG;
    g_carrier_pulse_us = CARRIER_PULSE_MIN_US;
    g_carrier_signal_active = 1U;
    g_carrier_state = 1U;
    s_session = 1U;
    s_boot = 1U;
#endif
}

static void Carrier_Off(void)
{
    CarrierPwm_Off();
    g_carrier_signal_active = 0U;
    s_boot = 0U;
}

static void Carrier_Fail(uint32_t result)
{
    Carrier_Off();
    g_carrier_result = result;
    g_carrier_state = 9U;
    g_carrier_test_enable = 0U;
    /* 已启动后失败仍占用，人工确认停稳/request2后才能放开其他轴。 */
}

void Carrier_Service(uint32_t ready, uint32_t other_busy, uint32_t now_ms)
{
    uint32_t request = g_carrier_request;
    float angle;
    uint32_t pulse;
    g_carrier_request = 0U;
    if (request == 2U)
    {
        Carrier_Off();
        g_carrier_test_enable = 0U;
        s_session = 0U;
        g_carrier_state = 3U;
        g_carrier_result = 2U;
        return;
    }
    if (request > 2U) { Carrier_Fail(9U); return; }
    /* 启动0度独立于手动VM准入；窗口后继续PWM保持，不冒充到位反馈。 */
    if (s_boot)
    {
        if (other_busy != 0U) { Carrier_Fail(5U); return; }
        if (!s_boot_timed) { s_started = now_ms; s_last_service = now_ms; s_boot_timed = 1U; }
        if (now_ms - s_last_service > CARRIER_SERVICE_GAP_MS)
        { Carrier_Fail(8U); return; }
        s_last_service = now_ms;
        g_carrier_elapsed_ms = now_ms - s_started;
        if (request == 1U) g_carrier_rejected_count++;
        if (g_carrier_elapsed_ms >= CARRIER_TEST_WINDOW_MS)
        {
            s_boot = 0U;
            s_session = 0U;
            g_carrier_state = 4U;
            g_carrier_result = 1U;
        }
        return;
    }
    if (s_session)
    {
        if (g_carrier_state == 9U) return;
        g_carrier_elapsed_ms = now_ms - s_started;
        if (ready != 1U || g_carrier_test_enable != 1U)
        { Carrier_Fail(4U); return; }
        if (other_busy != 0U) { Carrier_Fail(5U); return; }
        if (g_carrier_state == 1U && now_ms - s_last_service > CARRIER_SERVICE_GAP_MS)
        { Carrier_Fail(8U); return; }
        s_last_service = now_ms;
        if (request == 1U) g_carrier_rejected_count++;
        if (g_carrier_state == 1U &&
            g_carrier_elapsed_ms >= CARRIER_TEST_WINDOW_MS)
        {
            Carrier_Off();
            g_carrier_state = 2U;
            g_carrier_result = 1U;
        }
        return;
    }
    if (request == 0U || g_carrier_state == 9U) return;
    if (ready != 1U || g_carrier_test_enable != 1U || g_carrier_init_result != 1U)
    { Carrier_Fail(4U); return; }
    if (other_busy != 0U) { Carrier_Fail(5U); return; }
    angle = g_carrier_debug_angle;
    /* 同时拒绝NaN/无穷值，不把错误角度夹到端点。 */
    if (!(angle >= CARRIER_TEST_MIN_DEG && angle <= CARRIER_TEST_MAX_DEG))
    { Carrier_Fail(3U); return; }
    pulse = CARRIER_PULSE_MIN_US + (uint32_t)(
        angle * (float)(CARRIER_PULSE_MAX_US - CARRIER_PULSE_MIN_US)
        / CARRIER_ANGLE_MAX_DEG + 0.5f);
    if (CarrierPwm_Write(pulse) != 1U) { Carrier_Fail(6U); return; }
    g_carrier_latched_angle = angle;
    g_carrier_pulse_us = pulse;
    g_carrier_signal_active = 1U;
    g_carrier_elapsed_ms = 0U;
    g_carrier_result = 0U;
    g_carrier_state = 1U;
    s_session = 1U;
    s_started = now_ms;
    s_last_service = now_ms;
}
