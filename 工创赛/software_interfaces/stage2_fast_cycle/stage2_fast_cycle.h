#ifndef STAGE2_FAST_CYCLE_H
#define STAGE2_FAST_CYCLE_H
#include "source_parallel_flow.h"
/* Commissioning scheduler for the taught down/left/right mechanical cycle.
 * This does not select task colors or fabricate an autonomous vision result.
 * Depths are virtual lift pulses BELOW the current measured mechanical top.
 * A profile must describe ONE verified installation and sweep, not a timer.
 */
typedef struct {
 uint32_t run_id,pose_version,paths_verified,timing_verified,driving_stow_verified;
 float pick_deg[3],extension_mm[3],slot_deg;
 uint32_t pick_pulses,safe_pulses,retract_at_pulses[3],turn_at_pulses[3];
 uint32_t close_lead_pulses,close_ms,release_ms,carrier_ms;
} S2Fast_Profile;
typedef struct {
 void *context;
 /* 0 pending, 1 actual fresh placed/released observation, 2 failure.
  * Must identify this run and slot. Servo PWM/elapsed time is insufficient. */
 uint32_t (*placed)(void *,uint32_t run,uint32_t slot,uint32_t now);
 /* Same route owner contract as the normal stage2 backend. No default ACK. */
 uint32_t (*route_begin)(void *,const SPF_Command *,uint32_t now);
 uint32_t (*route_poll)(void *,const SPF_Command *,uint32_t now);
 uint32_t (*route_stop)(void *,uint32_t now);
} S2Fast_Hooks;
enum { S2F_IDLE,S2F_AIM,S2F_APPROACH,S2F_CARRY,S2F_RELEASE,S2F_DEPART,S2F_DONE,S2F_FAULT };
uint32_t Stage2Fast_Start(const S2Fast_Profile *,const S2Fast_Hooks *,uint32_t now);
uint32_t Stage2Fast_IsActive(void);
void Stage2Fast_Cancel(uint32_t reason);
void Stage2Fast_Service(uint32_t now);
extern volatile uint32_t g_stage2_fast_state,g_stage2_fast_fault,g_stage2_fast_item;
extern volatile uint32_t g_stage2_fast_loaded_mask,g_stage2_fast_release_ms,g_stage2_fast_departure_ms;
#endif
