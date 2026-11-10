#ifndef APP_INPUT_RUNTIME_H
#define APP_INPUT_RUNTIME_H
#include "app_config.h"
#include "FreeRTOS.h"

void App_StartGateUpdate(void);
void AppInput_ScanDisplay(TickType_t screen_start_tick);
void AppInput_KeyStep(void);
#if APP_CHASSIS_KEY_TEST_ENABLE && !APP_HWT_STATIC_TEST && APP_HWT_TURN_TEST
void App_HwtTurnSafety(void);
#endif
#endif
