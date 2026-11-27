#ifndef FUSION_REFERENCE_H
#define FUSION_REFERENCE_H
#include "chassis_kinematics.h"
/* Pure desired reference only: no motion permission, actuator, A/J or braking guarantee.
 * All positions/body dimensions reference the chassis geometric center in world mm.
 * model_only must be 1. Caller owns immutable plan and single-owner state. */
#define FUSION_POINTS 32U
#define FUSION_BOXES 32U
#define FUSION_INVALID 0U
#define FUSION_FOLLOW 1U
#define FUSION_TURN_START 2U
#define FUSION_TURN_END 3U
#define FUSION_DONE 4U
#define FUSION_RECOVERY 5U
#define FUSION_BAD_PLAN 1U
#define FUSION_BAD_OBSERVATION 2U
#define FUSION_IDENTITY 3U
#define FUSION_GEOMETRY 4U
#define FUSION_OUTSIDE_BUDGET 5U
#define FUSION_SPEED_STATE 6U
#define FUSION_ESTIMATED_FEEDBACK 1U
#define FUSION_MEASURED_FEEDBACK 2U

typedef struct { float x_mm,y_mm; } FusionPoint;
typedef struct { float min_x,min_y,max_x,max_y; } FusionBox;
typedef struct { uint32_t run_id,start_id,map_version,layout_id,path_id,direction; } FusionIdentity;
typedef struct {
    float body_length_mm,body_width_mm,position_budget_mm,heading_budget_deg;
    float position_tolerance_mm,max_error_mm,stop_speed_mm_s,stop_omega_deg_s;
    float start_heading_deg,goal_heading_deg;
    float acceleration_hint_mm_s2,cross_gain_s,lateral_cap_mm_s,preview_s;
    float wheel_limit_rpm,heading_reserve_rpm,speed_state_margin_mm_s;
    uint32_t max_age_ms;
    ChassisGeometry geometry;
} FusionConfig;
typedef struct {
    FusionIdentity identity;
    uint32_t model_only,count,box_count;
    FusionConfig config;
    FusionPoint points[FUSION_POINTS];
    float speed_mm_s[FUSION_POINTS-1U]; /* Along-path cap; total desired translation <=600. */
    FusionBox boxes[FUSION_BOXES]; /* Additional obstacles; four yellow blocks are built in. */
} FusionInput;
typedef struct {
    uint32_t valid,signature;
    FusionInput input;
    float lengths[FUSION_POINTS-1U],prefix[FUSION_POINTS];
    float node_cap[FUSION_POINTS],hold_heading_deg;
    int32_t turn_direction; /* -1=CW, +1=CCW, 0=no split 180. */
} FusionPlan;
typedef struct { uint32_t reason,edge; } FusionCheck;
typedef struct {
    FusionIdentity identity;
    FusionPoint position;
    float heading_deg,vx_mm_s,vy_mm_s,omega_deg_s;
    uint32_t sample_ms,now_ms,valid,continuity_lost,reasons,feedback_source,stopped;
} FusionObservation;
typedef struct {
    uint32_t status,reason,edge,model_only,deployment_allowed,mixed,calibration_verified;
    float signed_s_mm,cross_mm,remaining_mm,endpoint_residual_mm;
    float heading_target_deg,heading_error_deg,speed_cap_mm_s,wheel_scale;
    float world_vx_mm_s,world_vy_mm_s;
    ChassisBodyVelocity body;
    ChassisWheelSpeeds wheels;
} FusionOutput;
typedef struct {
    uint32_t signature,edge,phase,seen,last_sample_ms,observation_signature;
    FusionOutput cached;
} FusionState;
uint32_t FusionReference_Prepare(FusionPlan *plan,const FusionInput *input,FusionCheck *check);
void FusionReference_Reset(FusionState *state);
/* Invalid/recovery returns zero desired output; never reuse a prior output as fresh.
 * Duplicate sample may be observed but cannot advance phase/edge. Bad input does not
 * mutate state. DONE also requires actual caller-supplied velocity and stopped flag. */
uint32_t FusionReference_Step(const FusionPlan *plan,FusionState *state,
    const FusionObservation *observation,FusionOutput *output);
#endif
