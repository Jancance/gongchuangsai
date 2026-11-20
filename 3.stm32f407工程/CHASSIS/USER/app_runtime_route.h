#ifndef APP_RUNTIME_ROUTE_H
#define APP_RUNTIME_ROUTE_H
#include "app_config.h"
#include "runtime_route.h"
#if APP_SERIAL_MOTION_CONTROL
const RuntimeRoute *AppRuntimeRoute_Live(void);
void AppRuntimeRoute_Init(void);
void AppRuntimeRoute_Install(const RuntimeRoutePrepared *prepared);
uint32_t App_RuntimeRouteEditable(uint32_t now);
void App_RuntimeRouteCommand(const char *line);
void App_RuntimeRouteTelemetry(void);
#endif
#endif
