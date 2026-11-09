#ifndef LIFT_RULER_ADAPTER_H
#define LIFT_RULER_ADAPTER_H
#include <stdint.h>
#include "lift_ruler_calibration.h"
extern LiftRulerController g_lift_ruler_controller;
extern volatile uint32_t g_lift_ruler_request,g_lift_ruler_rejected;
extern volatile uint32_t g_lift_ruler_state,g_lift_ruler_phase,g_lift_ruler_cause,g_lift_ruler_samples;
extern volatile uint32_t g_lift_ruler_options,g_lift_ruler_encoder_before,g_lift_ruler_encoder_after;
extern volatile uint32_t g_lift_ruler_stop_pending,g_lift_ruler_stop_unconfirmed,g_lift_ruler_vm_request;
extern volatile int32_t g_lift_ruler_before_raw,g_lift_ruler_after_raw,g_lift_ruler_target_raw;
void LiftRuler_DebugKey(void);
void LiftRuler_DebugService(void);
void LiftRuler_DebugSafety(void);
uint32_t LiftRuler_DebugCanAllowed(uint8_t,const uint8_t*,uint32_t);
#endif
