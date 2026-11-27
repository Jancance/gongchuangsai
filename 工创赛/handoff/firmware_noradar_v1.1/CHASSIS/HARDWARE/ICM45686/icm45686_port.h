#ifndef ICM45686_PORT_H
#define ICM45686_PORT_H
#include <stdint.h>
int IcmPort_Init(void);
int IcmPort_Read(uint8_t reg, uint8_t *data, uint32_t length);
int IcmPort_Write(uint8_t reg, const uint8_t *data, uint32_t length);
int IcmPort_DelayUs(uint32_t us);
void IcmPort_DelayMs(uint32_t ms);
uint32_t IcmPort_Cycles(void);
uint32_t IcmPort_Millis(void);
uint32_t IcmPort_CyclesPerUs(void);
uint32_t IcmPort_Lock(void);
void IcmPort_Unlock(uint32_t saved);
void IcmPort_Edge(uint32_t *sequence, uint32_t *cycle);
uint32_t IcmPort_Pending(void);
void IcmPort_EnableEdge(void);
#endif
