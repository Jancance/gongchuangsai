#ifndef APP_CHASSIS_SERVICE_H
#define APP_CHASSIS_SERVICE_H

#include "app_config.h"
#include <stdint.h>
#if APP_ZONE2_NAV_ENABLE
void AppChassis_Zone2CanStep(uint32_t slot_ms,uint32_t phase);
#endif

void AppChassis_CanInit(void);
void AppChassis_CanStep(void);
void AppChassis_SafetyCheck(void);
#endif
