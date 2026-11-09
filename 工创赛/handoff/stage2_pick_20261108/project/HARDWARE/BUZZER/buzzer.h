#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Start(uint32_t count);
void Buzzer_Update(void);

#endif
