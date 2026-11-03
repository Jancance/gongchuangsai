#ifndef RADAR_LIFT_H
#define RADAR_LIFT_H
#include <stdint.h>
/* First rig only: PD10/PD11/PD12/PB8 -> ULN2003 IN1..4. No homing input.
 * All positions/counts are relative half-step commands, not measured height. */
#define RADAR_LIFT_MAX_STEPS 64U
#define RADAR_LIFT_INTERVAL_MS 4U
extern volatile uint32_t g_radar_lift_enable,g_radar_lift_request;
extern volatile uint32_t g_radar_lift_steps,g_radar_lift_direction;
extern volatile uint32_t g_radar_lift_state,g_radar_lift_result,g_radar_lift_sequence;
extern volatile uint32_t g_radar_lift_done,g_radar_lift_phase_bits;
/* enable defaults 0; request1=start,2=cancel and retain holding phase.
 * direction0/1 are opposite sequences; physical up/down is uncalibrated.
 * state0=untouched,1=moving,2=holding complete,3=cancelled/fault holding.
 * result1=start accepted,2=unsafe/disabled,3=invalid,4=busy,5=cancelled.
 * Completed/cancelled moves keep the last phase energized to avoid dropping.
 * No automatic release, reverse, return-to-origin or chassis command. */
void RadarLift_Service(uint32_t now_ms,uint32_t safe_idle);
#endif
