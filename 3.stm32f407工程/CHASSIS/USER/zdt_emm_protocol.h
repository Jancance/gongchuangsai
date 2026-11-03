#ifndef ZDT_EMM_PROTOCOL_H
#define ZDT_EMM_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include "zdt_emm_config.h"
/* CAN负载不含地址：Emm速度帧7字节，输入须缩小10倍。
 * 加速度为档位而非RPM/s；超过255直接拒绝，避免静默截断。 */
static __inline uint32_t ZdtEmm_EncodeSpeed(uint8_t dir,uint16_t speed,uint16_t accel,
    uint8_t sync,uint8_t out[7])
{
 if(!out || dir>1U || speed>ZDT_EMM_APP_MAX_SPEED_RAW || accel>255U || sync>1U)return 0U;
 out[0]=0xF6U;out[1]=dir;out[2]=(uint8_t)(speed>>8);out[3]=(uint8_t)speed;
 out[4]=(uint8_t)accel;out[5]=sync;out[6]=0x6BU;return 1U;
}
static __inline uint32_t ZdtEmm_EncodeSyncTrigger(uint8_t out[3])
{ if(!out)return 0U;out[0]=0xFFU;out[1]=0x66U;out[2]=0x6BU;return 1U; }
/* 速度缩放仅影响输入；Emm实时反馈为整数RPM，不能再除10。 */
static __inline uint32_t ZdtEmm_DecodeSpeed(const uint8_t *p,uint32_t n,float *rpm)
{
 uint32_t raw;
 if(!rpm)return 0U;*rpm=0.0f;
 if(!p || n!=5U || p[0]!=0x35U || p[1]>1U || p[4]!=0x6BU)return 0U;
 raw=((uint32_t)p[2]<<8)|p[3];*rpm=(float)raw;if(p[1])*rpm= -*rpm;return 1U;
}
/* Emm位置原生65536计数/圈，直接传递，不做0.1度位置换算。 */
static __inline uint32_t ZdtEmm_DecodePosition(const uint8_t *p,uint32_t n,
    uint8_t *sign,uint32_t *counts)
{
 if(!sign || !counts)return 0U;*sign=0U;*counts=0U;
 if(!p || n!=7U || p[0]!=0x36U || p[1]>1U || p[6]!=0x6BU)return 0U;
 *sign=p[1];*counts=((uint32_t)p[2]<<24)|((uint32_t)p[3]<<16)|((uint32_t)p[4]<<8)|p[5];return 1U;
}
static __inline uint32_t ZdtEmm_DecodeAck(uint8_t function,const uint8_t *p,uint32_t n)
{ return p && n==3U && p[0]==function && p[1]==0x02U && p[2]==0x6BU; }
/* 日志统一0.1RPM编码，不表示反馈也有0.1RPM分辨率。 */
static __inline int32_t ZdtEmm_RpmDeci(float rpm)
{
 if(!(rpm>=-65535.0f && rpm<=65535.0f))return (-2147483647L-1L);
 return (int32_t)(rpm<0?rpm*10.0f-0.5f:rpm*10.0f+0.5f);
}
typedef struct {
 int32_t speed_decirpm;
 uint32_t position_counts,position_deci_deg;
 uint16_t phase_ma,bus_mv;
 uint8_t position_sign,home_flags,motor_flags;
} ZdtEmmSystemData;
/* Emm系统包共31字节，去地址后30字节；拒绝其他固件格式。
 * 仅日志位置换成0.1度，里程读取始终用原生计数。 */
static __inline uint32_t ZdtEmm_DecodeSystem(const uint8_t *p,uint32_t n,ZdtEmmSystemData *s)
{
 uint32_t raw;
 if(!s)return 0U;*s=(ZdtEmmSystemData){0};
 if(!p || n!=30U || p[0]!=0x43U || p[1]!=31U || p[2]!=9U || p[29]!=0x6BU ||
    p[9]>1U || p[14]>1U || p[17]>1U || p[22]>1U)return 0U;
 raw=((uint32_t)p[15]<<8)|p[16];s->speed_decirpm=(int32_t)(raw*10U);
 if(p[14])s->speed_decirpm= -s->speed_decirpm;
 s->position_counts=((uint32_t)p[18]<<24)|((uint32_t)p[19]<<16)|((uint32_t)p[20]<<8)|p[21];
 s->position_deci_deg=(uint32_t)(((uint64_t)s->position_counts*3600U+32768U)/65536U);
 s->position_sign=p[17];s->phase_ma=(uint16_t)(((uint16_t)p[5]<<8)|p[6]);
 s->bus_mv=(uint16_t)(((uint16_t)p[3]<<8)|p[4]);s->home_flags=p[27];s->motor_flags=p[28];return 1U;
}
#endif
