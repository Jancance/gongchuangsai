/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
/* reset/IREG/端序依据 InvenSense SDK 2.1.0；接入计划记录固定来源。 */
#include "icm45686_raw.h"
#include "icm45686_port.h"
#include <string.h>
static IcmRawStatus status;
static uint32_t consumed_edge, last_service_ms;
static int fail(uint32_t error, uint32_t reg)
{
    status.last_error=error; status.failed_reg=reg; status.fresh=0U;
    if (error==ICM_RAW_TRANSFER) ++status.transfer_errors;
    return -1;
}
static int read_reg(uint8_t reg, uint8_t *data, uint32_t n)
{
    if (IcmPort_Read(reg,data,n)) return fail(ICM_RAW_TRANSFER,reg);
    return 0;
}
static int write_reg(uint8_t reg, const uint8_t *data, uint32_t n)
{
    if (IcmPort_Write(reg,data,n)) return fail(ICM_RAW_TRANSFER,reg);
    return 0;
}
static int set_reg(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t data, check;
    if (read_reg(reg,&data,1U)) return -1;
    data=(data & (uint8_t)~mask) | value;
    if (write_reg(reg,&data,1U) || read_reg(reg,&check,1U)) return -1;
    if ((check & mask)!=value) return fail(ICM_RAW_CONFIG,reg);
    return 0;
}
static int ireg(uint16_t reg, uint8_t *data, uint32_t write)
{
    uint8_t buf[3];
    buf[0]=(uint8_t)(reg>>8); buf[1]=(uint8_t)reg; buf[2]=*data;
    if (IcmPort_DelayUs(4U)) return fail(ICM_RAW_TIMEBASE,reg);
    if (write_reg(0x7cU,buf,write ? 3U : 2U)) return -1;
    if (IcmPort_DelayUs(4U)) return fail(ICM_RAW_TIMEBASE,reg);
    return write ? 0 : read_reg(0x7eU,data,1U);
}
static int filter(uint16_t reg)
{
    uint8_t data=0U;
    if (ireg(reg,&data,0U)) return -1;
    data=(data & 0xf8U)|1U;
    if (ireg(reg,&data,1U) || ireg(reg,&data,0U)) return -1;
    return (data&7U)==1U ? 0 : fail(ICM_RAW_CONFIG,reg);
}
int IcmRaw_Init(void)
{
    uint8_t data, drive, intf;
    memset(&status,0,sizeof(status)); consumed_edge=0U;
    status.state=ICM_RAW_STARTING;
    if (IcmPort_Init()) { fail(ICM_RAW_TIMEBASE,0U); goto failed; }
    IcmPort_DelayMs(3U);
    if (read_reg(0x72U,&data,1U)) goto failed;
    status.whoami=data;
    if (data!=0xe9U) { fail(ICM_RAW_ID,0x72U); goto failed; }
    /* SPI slew 7ns，保留I2C/无关位；reset须保存恢复接口配置。 */
    if (set_reg(0x32U,0x0eU,6U) || read_reg(0x32U,&drive,1U) ||
        read_reg(0x2dU,&intf,1U)) goto failed;
    data=2U;
    if (write_reg(0x7fU,&data,1U)) goto failed;
    IcmPort_DelayMs(1U);
    if (write_reg(0x32U,&drive,1U) || write_reg(0x2dU,&intf,1U) ||
        read_reg(0x19U,&data,1U)) goto failed;
    if (!(data&0x80U)) { fail(ICM_RAW_RESET,0x19U); goto failed; }
    if (read_reg(0x72U,&data,1U)) goto failed;
    status.whoami=data;
    if (data!=0xe9U) { fail(ICM_RAW_ID,0x72U); goto failed; }
    /* +/-4g、+/-1000dps，200Hz；配置只用于独立RAW观察。 */
    if (set_reg(0x1bU,0x7fU,0x38U) || set_reg(0x1cU,0xffU,0x28U)) goto failed;
    if (read_reg(0x1bU,&data,1U)) goto failed;
    status.accel_config=data;
    if (read_reg(0x1cU,&data,1U)) goto failed;
    status.gyro_config=data;
    data=0U;
    if (ireg(0xa267U,&data,0U)) goto failed;
    status.big_endian=(data>>1)&1U;
    if (filter(0xa4acU) || filter(0xa583U) || set_reg(0x10U,15U,15U)) goto failed;
    IcmPort_DelayMs(70U);
    /* 仅DRDY；高电平、脉冲、推挽，其他中断源关闭。 */
    if (set_reg(0x16U,0xffU,0U) || set_reg(0x17U,0x7fU,0U) ||
        set_reg(0x18U,7U,1U) || read_reg(0x19U,&data,1U) ||
        set_reg(0x16U,0xffU,4U)) goto failed;
    IcmPort_EnableEdge();
    last_service_ms=IcmPort_Millis(); status.state=ICM_RAW_READY;
    return 0;
failed:
    status.state=ICM_RAW_FAILED; return -1;
}
static int16_t decode(const uint8_t *p)
{
    uint16_t value=status.big_endian ? ((uint16_t)p[0]<<8)|p[1] : ((uint16_t)p[1]<<8)|p[0];
    return (int16_t)value;
}
void IcmRaw_Service(void)
{
    uint32_t seq,cycle,after,after_cycle,lock,start,end,now,gap,i;
    uint8_t data[14];
    if (status.state!=ICM_RAW_READY) return;
    now=IcmPort_Millis(); gap=(uint32_t)(now-last_service_ms); last_service_ms=now;
    lock=IcmPort_Lock(); IcmPort_Edge(&seq,&cycle); IcmPort_Unlock(lock);
    if (seq==consumed_edge) { if (gap>20U) status.fresh=0U; return; }
    status.skipped_edges+=(uint32_t)(seq-consumed_edge)-1U; consumed_edge=seq;
    start=IcmPort_Cycles();
    /* RTOS长间隙也拒绝一次，避免DWT整周回绕伪造近期边沿。 */
    if (gap>20U || (uint32_t)(start-cycle)>=5000U*IcmPort_CyclesPerUs()) {
        ++status.skipped_edges; ++status.read_races; fail(ICM_RAW_ASSOCIATION,0U); return;
    }
    if (read_reg(0U,data,14U)) { ++status.skipped_edges; return; }
    end=IcmPort_Cycles();
    lock=IcmPort_Lock(); IcmPort_Edge(&after,&after_cycle);
    i=IcmPort_Pending(); IcmPort_Unlock(lock);
    if (after!=seq || i || (uint32_t)(end-cycle)>=5000U*IcmPort_CyclesPerUs()) {
        ++status.skipped_edges; ++status.read_races; fail(ICM_RAW_ASSOCIATION,0U); return;
    }
    for (i=0U;i<3U;++i) { status.raw_accel[i]=decode(data+2U*i); status.raw_gyro[i]=decode(data+6U+2U*i); }
    status.raw_temperature=decode(data+12U);
    status.edge_sequence=seq; status.edge_cycle=cycle;
    status.read_start_cycle=start; status.read_end_cycle=end; status.sample_tick_ms=now;
    ++status.sample_sequence; status.last_error=ICM_RAW_OK; status.fresh=1U;
}
void IcmRaw_GetStatus(IcmRawStatus *out)
{
    uint32_t lock=IcmPort_Lock();
    *out=status;
    out->age_ms=out->sample_sequence ? (uint32_t)(IcmPort_Millis()-out->sample_tick_ms) : 0xffffffffU;
    out->fresh=out->fresh && out->sample_sequence && out->age_ms<=20U;
    IcmPort_Unlock(lock);
}
