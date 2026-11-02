#include <stdint.h>
#include "hwt101_parser.h"
#define HEADING_SOURCE_FRESH_MS 100U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready,clock_ms;
static uint32_t g_hwt_update_tick,g_hwt_angle_age_ms,g_hwt_rate_age_ms,g_hwt_version_result,g_hwt_valid_mask;
static uint32_t g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
static float g_hwt_yaw_deg,g_hwt_wz_dps;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
volatile uint32_t results[12];
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)
static void App_HeadingSourcePublish(void)
{
    AppHeadingSource value = {0};
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    value.sample_ms = now;
    /* 用真实接收帧龄期还原时间，不能用本次发布时刻冒充新帧。 */
    value.angle_ms = g_hwt_update_tick - g_hwt_angle_age_ms;
    value.rate_ms = g_hwt_update_tick - g_hwt_rate_age_ms;
    value.yaw_deg = g_hwt_yaw_deg;
    value.rate_dps = g_hwt_wz_dps;
    value.healthy = ((g_hwt_version_result == 1U) &&
        (g_hwt_valid_mask == 3U) &&
        (g_hwt_angle_age_ms < HEADING_SOURCE_FRESH_MS) &&
        (g_hwt_rate_age_ms < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - g_hwt_update_tick) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.angle_ms) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - value.rate_ms) < HEADING_SOURCE_FRESH_MS) &&
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
uint32_t test_entry(void){
 Hwt101Parser parser;uint32_t i;uint8_t angle[11]={0x55,0x53},rate[11]={0x55,0x52};
 g_hwt_version_result=1;g_hwt_yaw_deg=-93.043f;g_hwt_valid_mask=3;
 clock_ms=g_hwt_update_tick=22930;g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy==1);
 results[0]=s_heading_source.angle_ms;CHECK(results[0]==22923);
 Hwt101Parser_Init(&parser);
 for(i=0;i<10;i++){angle[10]+=angle[i];rate[10]+=rate[i];}
 for(i=0;i<11;i++)Hwt101Parser_Feed(&parser,angle[i],22923);
 for(i=0;i<11;i++)Hwt101Parser_Feed(&parser,rate[i],22923);
 CHECK(Hwt101Parser_Valid(&parser,22930)==3);
 Hwt101Parser_Invalidate(&parser);CHECK(Hwt101Parser_Valid(&parser,22930)==0);
 g_hwt_valid_mask=0;g_hwt_angle_age_ms=g_hwt_rate_age_ms=0xffffffffU;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy==0);
 results[1]=s_heading_source.sample_ms;results[2]=s_heading_source.angle_ms;
 results[3]=22938U-s_heading_source.angle_ms;results[4]=s_heading_source.healthy;
 CHECK(results[1]==22930 && results[2]==22931 && results[3]==7 && results[4]==0);
 g_hwt_valid_mask=2;g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(!s_heading_source.healthy);results[5]=22938U-s_heading_source.angle_ms;
 results[6]=22938U-s_heading_source.rate_ms;CHECK(results[5]==7 && results[6]==15);
 /* Genuine UINT32 clock wrap must remain supported, separately from invalid age. */
 clock_ms=g_hwt_update_tick=3;g_hwt_valid_mask=3;
 g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(s_heading_source.healthy==1);CHECK((uint32_t)(3U-s_heading_source.angle_ms)==7);
 results[7]=1;return 0;
}
