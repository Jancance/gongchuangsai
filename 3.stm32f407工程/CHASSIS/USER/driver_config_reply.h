#ifndef DRIVER_CONFIG_REPLY_H
#define DRIVER_CONFIG_REPLY_H
#include <stdint.h>
#include <string.h>
/* X固件0x42配置只读回复：37字节含地址，CAN拼接体36字节。 */
typedef struct {uint8_t data[36];uint32_t length,packet;} DriverConfigReply;
static uint32_t DriverConfigReply_Push(DriverConfigReply *state,uint32_t packet,
 const uint8_t *data,uint32_t count)
{
 uint32_t offset;
 if(!state || !data || count<2U || count>8U || data[0]!=0x42U || packet!=state->packet)return 2U;
 offset=packet?1U:0U;
 if(state->length+count-offset>36U)return 2U;
 memcpy(state->data+state->length,data+offset,count-offset);
 state->length+=count-offset;state->packet++;
 if(state->length<36U)return 0U;
 return state->data[0]==0x42U && state->data[1]==0x25U &&
  state->data[2]==0x18U && state->data[35]==0x6bU?1U:2U;
}
#endif
