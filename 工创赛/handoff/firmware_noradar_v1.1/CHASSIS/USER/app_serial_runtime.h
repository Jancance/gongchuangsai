#ifndef APP_SERIAL_RUNTIME_H
#define APP_SERIAL_RUNTIME_H
#include "app_serial_internal.h"
#if APP_SERIAL_MOTION_CONTROL
void AppSerial_Init(void);
void AppSerial_Telemetry(void);
void App_SerialMotionByte(uint8_t data);
void App_SerialMotionSafety(void);
#endif
#endif
