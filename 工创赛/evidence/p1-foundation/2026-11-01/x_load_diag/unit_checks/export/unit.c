#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "src/x_load_diag.h"
uint32_t g_chassis_auto_active,g_chassis_auto_session,g_chassis_auto_state;
static uint32_t depth;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (--depth)
#define CHECK(x) do{if(!(x))return 0x80000000U|__LINE__;}while(0)

#include "src/x_load_diag.c"
static uint32_t tick=100,window=1,send_ok=0,attempts;
volatile uint32_t g_vm_test_request;
typedef struct{uint32_t active;}PowerCommandStatus;static PowerCommandStatus power;
char exported_frames[3][512];uint32_t exported_lengths[3];
static uint32_t xTaskGetTickCount(void){return tick;}
static void Power_ReadCommandStatus(PowerCommandStatus *p){*p=power;}
static uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t earliest){(void)now;(void)earliest;return window;}
static uint32_t App_DebugSendString(const char *s){if(attempts<3){strcpy(exported_frames[attempts],s);exported_lengths[attempts]=strlen(s);}attempts++;return send_ok;}

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
static void App_XLoadTelemetry(void)

{

    static uint32_t cursor=0U,session=0U,last=0U;

    static char frame[512];

    XLoadRecord record;XLoadStatistics stats;PowerCommandStatus power;

    uint32_t now=xTaskGetTickCount(),n,i,j,used;int count;uint16_t crc;

    const uint8_t *bytes;static const char hex[]="0123456789ABCDEF";

    Power_ReadCommandStatus(&power);

    if(g_chassis_auto_active || power.active || g_vm_test_request || (uint32_t)(now-last)<70U ||

       !App_SerialDiagnosticWindow(now,40U))return;

    if(!XLoad_Read(0U,&record,&stats))return;

    if(session!=stats.session){session=stats.session;cursor=0U;}

    if(cursor>=stats.count)return;

    n=stats.count-cursor;if(n>8U)n=8U;

    count=snprintf(frame,sizeof(frame),"XD1,%lu,%lu,%lu,%lu,%lu,",(unsigned long)session,

        (unsigned long)cursor,(unsigned long)n,(unsigned long)stats.count,(unsigned long)stats.dropped);

    if(count<0 || (uint32_t)count>=sizeof(frame))return;used=(uint32_t)count;

    for(i=0U;i<n;i++){

        if(!XLoad_Read(cursor+i,&record,&stats))return;

        bytes=(const uint8_t*)&record;

        for(j=0U;j<sizeof(record);j++){if(used+2U>=sizeof(frame)-8U)return;frame[used++]=hex[bytes[j]>>4];frame[used++]=hex[bytes[j]&15U];}

    }

    frame[used++]=',';crc=App_SerialCrc(frame,used);

    count=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);

    if(count<0 || (uint32_t)count>=sizeof(frame)-used)return;

    if(App_DebugSendString(frame)==1U){cursor+=n;last=now;}

}

uint32_t test_entry(void){uint32_t i;tick=100;g_chassis_auto_active=1;g_chassis_auto_session=7;g_chassis_auto_state=5;
 for(i=0;i<9;i++)XLoad_RecordCommand(3,10+i,XLOAD_ACK,1,i);
 App_XLoadTelemetry();
CHECK(!attempts);g_chassis_auto_active=0;power.active=1;App_XLoadTelemetry();
CHECK(!attempts);
 power.active=0;g_vm_test_request=1;App_XLoadTelemetry();
CHECK(!attempts);g_vm_test_request=0;window=0;App_XLoadTelemetry();
CHECK(!attempts);
 window=1;App_XLoadTelemetry();
CHECK(attempts==1);
send_ok=1;
tick=200;App_XLoadTelemetry();
CHECK(attempts==2);
 tick=300;App_XLoadTelemetry();
CHECK(attempts==3);
tick=400;App_XLoadTelemetry();
CHECK(attempts==3);
CHECK(!depth);return 100U;
}
