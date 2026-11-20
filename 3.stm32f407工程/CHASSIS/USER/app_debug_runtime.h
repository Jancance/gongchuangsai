#ifndef APP_DEBUG_RUNTIME_H
#define APP_DEBUG_RUNTIME_H
#include "app_config.h"
#include <stdint.h>

void App_HeadingDryRun(void);
#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST
void App_HeadingLiveStep(void);
void AppDebug_HeadingObserve(void);
#endif
#if APP_UART_LINK_TEST
void App_UartLinkByte(uint8_t data);
void App_UartLinkPoll(void);
#endif
#endif
