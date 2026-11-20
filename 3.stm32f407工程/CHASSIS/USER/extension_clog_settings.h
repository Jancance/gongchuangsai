#ifndef EXTENSION_CLOG_SETTINGS_20261115_H
#define EXTENSION_CLOG_SETTINGS_20261115_H
#include <stdint.h>
#include <string.h>
/* X configuration, without address. Preserve every setting except Clog_Ms. */
static uint32_t ExtensionClog3000_Frame(const uint8_t current[36],uint8_t frame[36])
{
 if(!current || !frame || current[0]!=0x42U || current[1]!=0x25U ||
    current[2]!=0x18U || current[35]!=0x6bU || current[26]!=1U)return 0U;
 memcpy(frame,current,36U);frame[0]=0x48U;frame[1]=0xd1U;frame[2]=1U;
 frame[31]=0x0bU;frame[32]=0xb8U;return 1U;
}
static uint32_t ExtensionClog3000_Readback(const uint8_t before[36],const uint8_t after[36])
{
 uint8_t expected[36];
 if(!after || !ExtensionClog3000_Frame(before,expected))return 0U;
 expected[0]=0x42U;expected[1]=0x25U;expected[2]=0x18U;
 return memcmp(expected,after,36U)==0;
}
#endif
