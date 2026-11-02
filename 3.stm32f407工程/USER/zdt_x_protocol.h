#ifndef ZDT_X_PROTOCOL_H
#define ZDT_X_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include "zdt_x_config.h"
/* CAN数据不含地址，8字节X速度指令恰好单帧；单位为0.1RPM和RPM/s。 */
static __inline uint32_t ZdtX_EncodeSpeed(uint8_t dir,uint16_t speed,uint16_t accel,
    uint8_t sync,uint8_t out[8])
{
 if(!out || dir>1U || speed>30000U || sync>1U)return 0U;
 out[0]=0xF6U;out[1]=dir;out[2]=(uint8_t)(accel>>8);out[3]=(uint8_t)accel;
 out[4]=(uint8_t)(speed>>8);out[5]=(uint8_t)speed;out[6]=sync;out[7]=0x6BU;
 return 1U;
}
/* 同步广播编码仅供协议检查；逐轮候选不暴露运行时广播入口。 */
static __inline uint32_t ZdtX_EncodeSyncTrigger(uint8_t out[3])
{ if(!out)return 0U;out[0]=0xFFU;out[1]=0x66U;out[2]=0x6BU;return 1U; }
static __inline uint32_t ZdtX_DecodeSpeed(const uint8_t *p,uint32_t n,float *rpm)
{
 uint32_t raw;
 if(!rpm)return 0U;*rpm=0.0f;
 if(!p || n!=5U || p[0]!=0x35U || p[1]>1U || p[4]!=0x6BU)return 0U;
 raw=((uint32_t)p[2]<<8)|p[3];*rpm=(float)raw*0.1f;if(p[1])*rpm= -*rpm;return 1U;
}
/* X实时位置为0.1度；归一为原里程计65536计数/圈，不能直接当Emm计数。 */
static __inline uint32_t ZdtX_DecodePosition(const uint8_t *p,uint32_t n,
    uint8_t *sign,uint32_t *counts)
{
 uint32_t raw;uint64_t scaled;
 if(!sign || !counts)return 0U;*sign=0U;*counts=0U;
 if(!p || n!=7U || p[0]!=0x36U || p[1]>1U || p[6]!=0x6BU)return 0U;
 raw=((uint32_t)p[2]<<24)|((uint32_t)p[3]<<16)|((uint32_t)p[4]<<8)|p[5];
 scaled=((uint64_t)raw*65536U+1800U)/ZDT_X_POSITION_UNITS_PER_REV;
 if(scaled>0xFFFFFFFFULL)return 0U;
 *sign=p[1];*counts=(uint32_t)scaled;return 1U;
}
static __inline uint32_t ZdtX_DecodeAck(uint8_t function,const uint8_t *p,uint32_t n)
{ return p && n==3U && p[0]==function && p[1]==0x02U && p[2]==0x6BU; }
static __inline int32_t ZdtX_RpmDeci(float rpm)
{
 if(!(rpm>=-6553.5f && rpm<=6553.5f))return (-2147483647L-1L);
 return (int32_t)(rpm<0?rpm*10.0f-0.5f:rpm*10.0f+0.5f);
}
#endif
