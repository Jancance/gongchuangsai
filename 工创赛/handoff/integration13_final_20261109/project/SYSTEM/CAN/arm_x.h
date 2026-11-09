#ifndef ARM_X_H
#define ARM_X_H
#include <stdint.h>
#include <string.h>
typedef struct { uint32_t pulses_per_turn, units_per_turn, valid; } ArmXEncoding;
/* 保留旧Emm脉冲输入的机械行程；细分取切换前实读值，不假定16。 */
static __inline uint32_t ArmX_Configure(uint8_t options, uint32_t legacy_mstep, ArmXEncoding *out)
{
    if (!out) return 0U;
    memset(out, 0, sizeof(*out));
    if ((options & 2U) || legacy_mstep == 0U || legacy_mstep > 256U) return 0U;
    out->pulses_per_turn = legacy_mstep * ((options & 1U) ? 400U : 200U);
    out->units_per_turn = (options & 0x80U) ? 36000U : 3600U;
    out->valid = 1U;
    return 1U;
}
static __inline uint32_t ArmX_AngleUnits(const ArmXEncoding *e, uint32_t pulses, uint32_t *units)
{
    uint64_t v;
    if (!e || !units || !e->valid || e->pulses_per_turn == 0U || pulses == 0U) return 0U;
    v = ((uint64_t)pulses * e->units_per_turn + e->pulses_per_turn / 2U) / e->pulses_per_turn;
    if (v == 0U || v > 0xFFFFFFFFULL) return 0U;
    *units = (uint32_t)v; return 1U;
}
static __inline uint32_t ArmX_BuildMove(const ArmXEncoding *e, uint8_t direction,
    uint32_t pulses, uint32_t max_pulses, uint16_t speed_deci_rpm,
    uint16_t acceleration, uint16_t deceleration, uint8_t out[15])
{
    uint32_t angle;
    if (!out || direction > 1U || pulses > max_pulses ||
        !speed_deci_rpm || speed_deci_rpm > 30000U || !acceleration || !deceleration ||
        !ArmX_AngleUnits(e, pulses, &angle)) return 0U;
    out[0]=0xFDU;out[1]=direction;
    out[2]=(uint8_t)(acceleration>>8);out[3]=(uint8_t)acceleration;
    out[4]=(uint8_t)(deceleration>>8);out[5]=(uint8_t)deceleration;
    out[6]=(uint8_t)(speed_deci_rpm>>8);out[7]=(uint8_t)speed_deci_rpm;
    out[8]=(uint8_t)(angle>>24);out[9]=(uint8_t)(angle>>16);out[10]=(uint8_t)(angle>>8);out[11]=(uint8_t)angle;
    out[12]=2U;out[13]=0U;out[14]=0x6BU;
    return 1U;
}
#endif
