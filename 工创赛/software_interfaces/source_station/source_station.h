#ifndef SOURCE_STATION_H
#define SOURCE_STATION_H
#include <stdint.h>
/* TaskA owns arrival/alignment/physical scanner. CAN task owns vision and pzq.
 * No new hardware task and no artificial arrival, QR or aligned flag. */
enum { SS_IDLE,SS_WAIT_PROFILE,SS_ALIGN,SS_WAIT_QR,SS_READY,SS_PICKING,SS_DONE,SS_FAULT=9 };
extern volatile uint32_t g_source_station_state,g_source_station_reason,g_source_station_run;
extern volatile uint32_t g_source_station_scan_count,g_source_station_pose;
extern volatile uint32_t g_it_auto_vision_mode,g_it_auto_phase;
uint32_t SourceStation_Service(uint32_t now);
uint32_t SourceStation_PickAllowed(uint32_t run);
uint32_t SourceStation_HoldsBody(void);
uint32_t SourceStation_ReadTask(char code[16],uint32_t *run,uint32_t *batch);
void SourceStation_Cancel(uint32_t reason);
uint32_t SourceStation_ManualWait(void);
/* Read-only pose/reference readiness; no homing or assumed origin. */
uint32_t SourceStation_MechanicsReady(void);
/* Actual capture/tracking provider calls this in the CAN owner. No fabricated
 * identity/time from CC02 receive sequence or color. captured_ms is reconciled
 * MCU time; usable_until_ms includes the verified grab/exit window. */
typedef struct {
 uint32_t run,pose_version,material_cycle,instance_id,captured_ms,usable_until_ms;
 uint32_t capture_verified,stationary,sweep_verified,exit_budget_ms;
} SourceStationTarget;
uint32_t SourceStation_OfferTarget(const SourceStationTarget *,uint32_t now);
#endif
