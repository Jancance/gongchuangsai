#ifndef POWER_H
#define POWER_H
#include <stdint.h>
typedef struct{uint32_t active,enable_seen,enable_tick,enable_count;}PowerCommandStatus;
void Power_ReadCommandStatus(PowerCommandStatus*);
#endif
