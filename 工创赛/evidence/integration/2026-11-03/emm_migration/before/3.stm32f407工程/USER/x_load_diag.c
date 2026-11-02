#include "x_load_diag.h"
#include "chassis_auto_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 有界且不覆盖的RAM日志；本模块不访问CAN、串口、电源或看门狗。 */
static XLoadRecord s_records[XLOAD_CAPACITY];
static uint32_t s_session, s_count, s_dropped;
static int16_t s_target[4];
static uint32_t s_ack_ms[4], s_target_valid;

static uint32_t XLoad_OpenLocked(void)
{
    if (!g_chassis_auto_active) return 0U;
    if (s_session != g_chassis_auto_session) {
        s_session=g_chassis_auto_session; s_count=0U; s_dropped=0U;
        s_target_valid=0U;
        memset(s_target,0,sizeof(s_target)); memset(s_ack_ms,0,sizeof(s_ack_ms));
    }
    return 1U;
}
static int16_t XLoad_SpeedField(int32_t value, uint8_t *flags)
{
    if (value>32767) { *flags|=XLOAD_SPEED_CLIPPED; return 32767; }
    if (value< -32767) { *flags|=XLOAD_SPEED_CLIPPED; return -32767; }
    return (int16_t)value;
}
static void XLoad_AppendLocked(const XLoadRecord *record)
{
    if (s_count<XLOAD_CAPACITY) s_records[s_count++]=*record;
    else if (s_dropped!=0xFFFFFFFFU) s_dropped++;
}
void XLoad_RecordCommand(uint8_t address,int32_t target,uint8_t kind,
    uint32_t result,uint32_t tick_ms)
{
    XLoadRecord r={0}; uint32_t i;
    if (address<1U || address>4U) return;
    i=address-1U;
    taskENTER_CRITICAL();
    if (XLoad_OpenLocked()) {
        r.tick_ms=tick_ms; r.address_kind=(uint8_t)((kind<<4)|address); r.result=(uint8_t)result;
        r.flags=(uint8_t)((g_chassis_auto_state&15U)<<3);
        r.target_decirpm=XLoad_SpeedField(target,&r.flags);
        if ((kind==XLOAD_ACK || kind==XLOAD_STOP_ACK) && result==1U &&
            !(r.flags&XLOAD_SPEED_CLIPPED)) {
            s_target[i]=r.target_decirpm; s_ack_ms[i]=tick_ms;
            s_target_valid|=1UL<<i;
        }
        r.target_ack_ms=s_ack_ms[i];
        if((kind==XLOAD_ACK || kind==XLOAD_STOP_ACK) && result==1U &&
           (s_target_valid&(1UL<<i))) r.flags|=XLOAD_TARGET_VALID;
        XLoad_AppendLocked(&r);
    }
    taskEXIT_CRITICAL();
}
void XLoad_RecordSnapshot(uint8_t address,uint32_t result,
    const XSystemSnapshot *sample,uint32_t tick_ms)
{
    XLoadRecord r={0}; uint32_t i;
    if(address<1U || address>4U)return;
    i=address-1U;
    taskENTER_CRITICAL();
    if(XLoad_OpenLocked()) {
        r.tick_ms=tick_ms;r.address_kind=(uint8_t)((XLOAD_SAMPLE<<4)|address);r.result=(uint8_t)result;
        r.target_decirpm=s_target[i];r.target_ack_ms=s_ack_ms[i];
        r.flags=(uint8_t)((g_chassis_auto_state&15U)<<3);
        if(s_target_valid&(1UL<<i))r.flags|=XLOAD_TARGET_VALID;
        if(result==1U && sample) {
            r.actual_decirpm=XLoad_SpeedField(sample->speed_decirpm,&r.flags);
            r.position_deci_deg=sample->position_deci_deg;
            r.phase_ma=sample->phase_ma;r.bus_mv=sample->bus_mv;
            r.faults=(uint8_t)((sample->home_flags&3U)|((sample->home_flags&0x30U)>>2)|
                ((sample->motor_flags&0x0CU)<<2));
            if(sample->position_sign)r.flags|=XLOAD_POSITION_NEGATIVE;
        }
        XLoad_AppendLocked(&r);
    }
    taskEXIT_CRITICAL();
}
uint32_t XLoad_Read(uint32_t index,XLoadRecord *record,XLoadStatistics *statistics)
{
    uint32_t valid=0U;
    if(!record || !statistics)return 0U;
    taskENTER_CRITICAL();
    statistics->session=s_session;statistics->count=s_count;statistics->dropped=s_dropped;
    /* 主循环还会核对VM实际控制状态，关闭后才允许导出。 */
    if(!g_chassis_auto_active && index<s_count) { *record=s_records[index];valid=1U; }
    taskEXIT_CRITICAL();
    return valid;
}
