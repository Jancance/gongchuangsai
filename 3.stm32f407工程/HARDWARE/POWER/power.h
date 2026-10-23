#ifndef POWER_H
#define POWER_H

#include <stdint.h>

void Power_Init(void);
void Power_Enable(void);
void Power_Disable(void);

/* 仅记录PE2软件控制历史，不是VM电压采样。 */
typedef struct
{
    uint32_t active;
    uint32_t enable_seen;
    uint32_t enable_tick;
    uint32_t enable_count;
} PowerCommandStatus;

void Power_ReadCommandStatus(PowerCommandStatus *status);

#endif
