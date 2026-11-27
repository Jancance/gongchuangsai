#ifndef APP_SERIAL_INTERNAL_H
#define APP_SERIAL_INTERNAL_H
#include "app_config.h"
#include <stdint.h>
uint8_t App_DebugSendString(const char *text);
#if APP_SERIAL_MOTION_CONTROL
extern volatile uint32_t g_serial_motion_state,g_serial_motion_result;
extern volatile uint32_t g_serial_motion_rejected,g_serial_motion_accepted,g_serial_motion_link_lost;
typedef struct {
    const uint32_t *token_seq,*token_tick,*arm_ms,*tx_valid,*tx_seq;
} AppSerialObservation;
const AppSerialObservation *AppSerial_Live(void);
void AppSerial_ResetToken(void);
void AppSerial_SetKeepalive(uint32_t now);
uint16_t App_SerialCrc(const char *data,uint32_t length);
int32_t App_SerialDistanceMilli(float value);
uint32_t App_SerialIdleReady(uint32_t now);
uint32_t App_SerialCompletedStopped(void);
uint32_t App_SerialDiagnosticWindow(uint32_t now,uint32_t earliest_ms);
#endif
#endif
