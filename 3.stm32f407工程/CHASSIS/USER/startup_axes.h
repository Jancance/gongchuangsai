#ifndef STARTUP_AXES_H
#define STARTUP_AXES_H
#include <stdint.h>
extern volatile uint32_t g_startup_axes_state,g_startup_axes_cause;
extern volatile uint32_t g_startup_axes_vm_request;
extern volatile uint32_t g_startup_extension_feedback_valid;
extern volatile float g_startup_extension_current_deg,g_startup_extension_target_deg;
extern volatile float g_extension_home_reference_deg;
extern volatile uint32_t g_extension_home_reference_valid;
extern volatile uint32_t g_startup_group_fault[12];
uint32_t StartupAxes_Begin(void);
void StartupAxes_Cancel(void);
void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready);
void StartupAxes_Safety(uint32_t healthy);
uint32_t StartupAxes_Holding(void);
uint32_t StartupAxes_NormalStopHold(void);
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_length,void *reply);
/* Read-only first extension-home/hold fault, retained until reset. */
extern volatile uint32_t g_eh_first_reason,g_eh_first_command,g_eh_first_exchange,g_eh_first_flags;
extern volatile uint32_t g_eh_first_tick,g_eh_first_service_gap,g_eh_first_phase;
#define EXTENSION_HOLD_HANDSHAKE_V1 1U
/* Completion proof is retained after faults; it is not a motion permit. */
extern volatile uint32_t g_extension_hold_enable_count,g_extension_hold_samples;
extern volatile uint32_t g_extension_hold_confirmed,g_extension_hold_ready_tick,g_extension_hold_delta_mdeg;
extern volatile uint32_t g_eh_first_step,g_eh_speed_sign,g_eh_speed_rpm;
#endif
