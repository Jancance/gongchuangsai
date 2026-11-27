#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

extern volatile uint32_t g_watchdog_started;
extern volatile uint32_t g_watchdog_feed_count;
extern volatile uint32_t g_reset_iwdg;
extern volatile uint32_t g_reset_flags;

uint32_t Watchdog_Init(void);
void Watchdog_Feed(void);

#endif
