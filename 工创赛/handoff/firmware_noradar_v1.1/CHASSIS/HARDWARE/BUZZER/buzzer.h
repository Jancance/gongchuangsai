#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Start(uint32_t count);
void Buzzer_StartPulse(uint32_t duration_ms);
void Buzzer_StartTimed(uint32_t count, uint32_t duration_ms, uint32_t gap_ms);
uint32_t Buzzer_IsBusy(void);
void Buzzer_Update(void);

#endif
