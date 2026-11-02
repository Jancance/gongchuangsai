#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MOTION_FAULT_DIAG_FIELDS 43U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t g_motion_fault_diag_valid,g_motion_fault_diag[43];
volatile char exported_frame[512];
static uint32_t xTaskGetTickCount(void){return 1000U;}
static uint32_t App_SerialDiagnosticWindow(uint32_t n,uint32_t w){(void)n;(void)w;return 1U;}
static uint32_t App_DebugSendString(const char *s){strcpy((char*)exported_frame,s);return 1U;}
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

static void App_MotionFaultTelemetry(void)
{
    static char frame[512];
    static uint32_t values[MOTION_FAULT_DIAG_FIELDS], last;
    uint32_t now = (uint32_t)xTaskGetTickCount(), i, used = 4U;
    int count;
    uint16_t crc;
    if (!g_motion_fault_diag_valid || (uint32_t)(now - last) < 1000U ||
        !App_SerialDiagnosticWindow(now, 60U)) return;
    taskENTER_CRITICAL();
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++) values[i] = g_motion_fault_diag[i];
    taskEXIT_CRITICAL();
    memcpy(frame, "HF3,", 4U);
    for (i = 0U; i < MOTION_FAULT_DIAG_FIELDS; i++)
    {
        count = snprintf(frame + used, sizeof(frame) - used, "%lu,", (unsigned long)values[i]);
        if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
        used += (uint32_t)count;
    }
    crc = App_SerialCrc(frame, used);
    count = snprintf(frame + used, sizeof(frame) - used, "%04X\r\n", crc);
    if (count < 0 || (uint32_t)count >= sizeof(frame) - used) return;
    if (App_DebugSendString(frame) == 1U) last = now;
}
uint32_t test_entry(void){
 g_motion_fault_diag_valid=1;g_motion_fault_diag[6]=1;g_motion_fault_diag[29]=11;g_motion_fault_diag[30]=6;
 g_motion_fault_diag[32]=1;g_motion_fault_diag[34]=g_motion_fault_diag[35]=15;
 g_motion_fault_diag[38]=(uint32_t)-10;g_motion_fault_diag[39]=3;
 g_motion_fault_diag[40]=900;g_motion_fault_diag[41]=950;g_motion_fault_diag[42]=2;
 App_MotionFaultTelemetry();return exported_frame[0]=='H'?1U:0x80000001U;
}
