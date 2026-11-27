#ifndef FUSION_ADAPTER_H
#define FUSION_ADAPTER_H
#include "app_odometry.h"
#include "fusion_reference.h"
/* Pure copied values only. No getter, caller, actuator or deployment permission.
 * Plan must be prepared and immutable; state has one owner. Non-NULL arguments
 * must be valid and must not alias. Raw finite-window feedback is ESTIMATED. */
#define FUSION_ADAPTER_WAIT 0U
#define FUSION_ADAPTER_READY 1U
#define FUSION_ADAPTER_UNAVAILABLE 2U
#define FUSION_ADAPTER_BAD_PLAN 1U
#define FUSION_ADAPTER_IDENTITY 2U
#define FUSION_ADAPTER_MAP_INVALID 3U
#define FUSION_ADAPTER_OLD_SAMPLE 4U
#define FUSION_ADAPTER_CORE_CONFLICT 5U
#define FUSION_ADAPTER_PRIME 6U
#define FUSION_ADAPTER_SHORT_WINDOW 7U
#define FUSION_ADAPTER_LONG_WINDOW 8U
#define FUSION_ADAPTER_NUMERIC 9U
#define FUSION_ADAPTER_STOP_INVALID 10U
typedef struct {
    AppMapPose map;
    FusionIdentity identity; /* layout/path/direction are explicit caller binding. */
    uint32_t now_ms,stop_confirmed,stop_sample_ms,stop_run_id;
} FusionAdapterInput;
typedef struct {
    uint32_t sample_ms,begin_ms;
    float raw_x_mm,raw_y_mm,x_mm,y_mm,heading_deg;
} FusionAdapterCore;
typedef struct {
    uint32_t bound,signature,anchor_ms,updates,ready,window_ms;
    FusionIdentity identity;
    float anchor_raw_x_mm,anchor_raw_y_mm,anchor_heading_deg;
    FusionAdapterCore latest;
    FusionObservation cached;
    uint32_t stop_sample_ms,stop_run_id;
} FusionAdapterState;
typedef struct {
    uint32_t status,reason,window_ms,feedback_source,estimated_stationary,stop_pending;
    /* Current map audit, not the metadata of the cached Observation. */
    uint32_t source,calibration_scope,nav_applied,action_epoch;
    uint32_t calibration_flags,calibration_direction,wheel_ms[4],angle_ms;
    uint32_t map_now_ms,pose_age_ms;
    float raw_x_mm,raw_y_mm,correction_x_mm,correction_y_mm;
} FusionAdapterReport;
void FusionAdapter_Reset(FusionAdapterState *state);
/* Safe output is always cleared on failure/WAIT. Report may be NULL.
 * First/short/long windows cannot assert zero velocity or physical stopping.
 * A duplicate preserves the complete cached Observation except caller now. */
uint32_t FusionAdapter_Observe(const FusionPlan *plan,FusionAdapterState *state,
    const FusionAdapterInput *input,FusionObservation *observation,FusionAdapterReport *report);
#endif
