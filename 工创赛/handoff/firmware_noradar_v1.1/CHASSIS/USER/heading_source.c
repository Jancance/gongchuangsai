#include "heading_source.h"
#include "app_config.h"
#include "chassis_odometry.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hwt101.h"

#if APP_HEADING_SESSION_TEST || (APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST)
#if configTICK_RATE_HZ != 1000U
#error Heading_source_requires_1000Hz_tick
#endif
#define HEADING_SOURCE_FRESH_MS ODOM_YAW_MAX_AGE_MS

static AppHeadingSource s_heading_source = {0};
static uint32_t s_heading_source_ready = 0U;
volatile uint32_t g_heading_source_valid = 0U;
volatile uint32_t g_heading_source_sequence = 0U;
volatile uint32_t g_heading_source_age_ms = 0U;
volatile uint32_t g_heading_source_angle_age_ms = 0U;
volatile uint32_t g_heading_source_rate_age_ms = 0U;
volatile float g_heading_source_yaw_deg = 0.0f;
volatile float g_heading_source_rate_dps = 0.0f;

/* Safe for task readers, including callers already in a critical section. */
uint32_t App_HeadingSourceRead(AppHeadingSource *snapshot)
{
    uint32_t ready;
    taskENTER_CRITICAL();
    *snapshot = s_heading_source;
    ready = s_heading_source_ready;
    taskEXIT_CRITICAL();
    return ready;
}

/* 唯一生产者TaskA：紧接HWT101_Update调用，不代表运动许可。 */
void App_HeadingSourcePublish(void)
{
    AppHeadingSource value = {0};
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    value.sample_ms = now;
    /* 用真实接收帧龄期还原时间，不能用本次发布时刻冒充新帧。 */
    value.valid_mask=g_hwt_valid_mask & 3U;
    if(g_hwt_angle_age_ms==0xFFFFFFFFU)value.valid_mask &= ~1U;
    if(g_hwt_rate_age_ms==0xFFFFFFFFU)value.valid_mask &= ~2U;
    value.angle_ms=(value.valid_mask & 1U) ? g_hwt_update_tick-g_hwt_angle_age_ms : 0U;
    value.rate_ms=(value.valid_mask & 2U) ? g_hwt_update_tick-g_hwt_rate_age_ms : 0U;
    value.yaw_deg = g_hwt_yaw_deg;
    value.rate_dps = g_hwt_wz_dps;
    value.healthy = ((g_hwt_version_result == 1U) &&
        (value.valid_mask == 3U) &&
        (g_hwt_angle_age_ms < HEADING_SOURCE_FRESH_MS) &&
        (g_hwt_rate_age_ms < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - g_hwt_update_tick) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,1U)) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,2U)) < HEADING_SOURCE_FRESH_MS) &&
        (value.yaw_deg >= -180.0f && value.yaw_deg <= 180.0f) &&
        (value.rate_dps >= -2000.0f && value.rate_dps <= 2000.0f) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
    /* 序号只在完整发布时递增；读者不修改它，允许uint32回绕。 */
    value.sequence = s_heading_source.sequence + 1U;
    s_heading_source = value;
    s_heading_source_ready = 1U;
    taskEXIT_CRITICAL();
}

/* 唯一观察者CAN任务：只取快照，禁止从这些Watch变量驱动电机。 */
void App_HeadingSourceObserve(void)
{
    AppHeadingSource value;
    uint32_t now, ready, valid;
    taskENTER_CRITICAL();
    value = s_heading_source;
    ready = s_heading_source_ready;
    now = (uint32_t)xTaskGetTickCount();
    taskEXIT_CRITICAL();
    valid = ((ready == 1U) && (value.healthy == 1U) &&
        ((uint32_t)(now - value.sample_ms) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,1U)) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,2U)) < HEADING_SOURCE_FRESH_MS)) ? 1U : 0U;
    g_heading_source_sequence = value.sequence;
    g_heading_source_age_ms = now - value.sample_ms;
    g_heading_source_angle_age_ms = App_HeadingFrameAge(&value,now,1U);
    g_heading_source_rate_age_ms = App_HeadingFrameAge(&value,now,2U);
    g_heading_source_yaw_deg = value.yaw_deg;
    g_heading_source_rate_dps = value.rate_dps;
    /* 失效仍保留原始观测值供排查，必须同时检查valid。 */
    g_heading_source_valid = valid;
}
#endif
