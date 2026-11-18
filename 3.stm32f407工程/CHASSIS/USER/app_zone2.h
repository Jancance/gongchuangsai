#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

/* Key task latches one run request. The CAN task owns the navigation runtime. */
void AppZone2_RequestStart(void);
uint32_t AppZone2_CanPermit(uint32_t slot_ms);
void AppZone2_Step(void);

#endif
