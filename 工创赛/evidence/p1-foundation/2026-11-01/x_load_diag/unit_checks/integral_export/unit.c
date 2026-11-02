#include <stdint.h>
#include <string.h>
#include <stdio.h>
#define HEADING_INTEGRAL_TRIAL_ENABLE 1U
#define HEADING_INTEGRAL_TRIAL_KI 0.03f
#define HEADING_INTEGRAL_LIMIT_RPM 0.3f
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)
static uint32_t tick,window,attempts;
uint32_t g_chassis_auto_active,g_vm_test_request;
static uint32_t s_xload_i_session,s_xload_i_samples,s_xload_i_nonzero;
static float s_xload_i_peak,s_xload_i_last;
typedef struct{uint32_t active;}PowerCommandStatus;static PowerCommandStatus power;
char exported_frames[3][512];uint32_t exported_lengths[3];
static uint32_t xTaskGetTickCount(void){return tick;}
static void Power_ReadCommandStatus(PowerCommandStatus *p){*p=power;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t earliest){(void)now;(void)earliest;return window;}
static uint32_t App_DebugSendString(const char *s){strcpy(exported_frames[0],s);exported_lengths[0]=strlen(s);attempts++;return 1;}
static uint16_t App_SerialCrc(const char *data, uint32_t length)
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
static void App_XIntegralTelemetry(void)
{
 static char frame[192];static uint32_t last=0U;PowerCommandStatus power;
 uint32_t now=xTaskGetTickCount(),used;int count;uint16_t crc;
 Power_ReadCommandStatus(&power);
 if(g_chassis_auto_active || power.active || g_vm_test_request || !s_xload_i_samples ||
    (uint32_t)(now-last)<200U || !App_SerialDiagnosticWindow(now,40U))return;
 count=snprintf(frame,sizeof(frame),"IG1,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%lu,%lu,",
  (unsigned long)s_xload_i_session,(unsigned long)now,(unsigned long)HEADING_INTEGRAL_TRIAL_ENABLE,
  (long)(HEADING_INTEGRAL_TRIAL_KI*1000.0f),(long)(HEADING_INTEGRAL_LIMIT_RPM*1000.0f),
  (long)(s_xload_i_peak*1000.0f),(long)(s_xload_i_last*1000.0f),
  (unsigned long)s_xload_i_samples,(unsigned long)s_xload_i_nonzero);
 if(count<0 || (uint32_t)count>=sizeof(frame)-7U)return;used=(uint32_t)count;
 crc=App_SerialCrc(frame,used);(void)snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
 if(App_DebugSendString(frame)==1U)last=now;
}
uint32_t test_entry(void){tick=1000;window=1;s_xload_i_session=7;s_xload_i_samples=2000;s_xload_i_nonzero=1500;s_xload_i_peak=.123f;s_xload_i_last= -.055f;
 g_chassis_auto_active=1;App_XIntegralTelemetry();CHECK(!attempts);g_chassis_auto_active=0;power.active=1;App_XIntegralTelemetry();CHECK(!attempts);
 power.active=0;g_vm_test_request=1;App_XIntegralTelemetry();CHECK(!attempts);g_vm_test_request=0;App_XIntegralTelemetry();CHECK(attempts==1);return 100U;}
