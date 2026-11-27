#ifndef FUSION_MOTION_H
#define FUSION_MOTION_H
#include "chassis_kinematics.h"

#define FUSION_MOTION_HEADING_TOLERANCE_DEG 0.2f
/* Allow single-precision error when converting headings to radians. */
#define FUSION_MOTION_HEADING_ROUNDOFF_DEG 0.00005f

/* Pure math. x/y use mm and yaw uses rad with the corresponding time powers.
 * Caller supplies current applied-command boundary, timing/error envelopes,
 * and safe space in the initial body frame. No CAN, STOP acknowledgement or
 * physical tracking claim. Inputs and outputs must not alias. */
typedef struct { float x,y,yaw; } FusionMotionVector;
typedef struct {
    float component_mm_s,translation_a,translation_j,yaw_v,yaw_a,yaw_j;
    float wheel_rpm,wheel_a,wheel_j;
} FusionMotionLimits;
extern const FusionMotionLimits g_fusion_motion_limits;
typedef struct {
    FusionMotionVector control[4];
    FusionMotionVector initial_a;
    float distance_mm,rotation_rad;
    uint32_t duration_ms;
} FusionMotionCurve;
typedef struct { FusionMotionVector q,a,j; } FusionMotionSample;
typedef struct {
    uint32_t now_ms,source_ms[5]; /* LR/RR/LF/RF speed, HWT rate. */
    uint32_t max_age_ms,compute_ms,queue_ms,batch_ms,dt_ms;
    float tangent_x,tangent_y,remaining_mm,back_mm,lateral_mm,angle_rad;
    float feedback_xy_mm_s,feedback_yaw_rad_s,mixed_mm,mixed_rad;
    int32_t turn_sign; /* -1/+1 locked rotation, 0 heading hold. */
    uint32_t mixed_lost;
} FusionMotionRequest;
typedef enum {
    FUSION_MOTION_OK = 0, FUSION_MOTION_INPUT, FUSION_MOTION_STALE,
    FUSION_MOTION_PARTIAL, FUSION_MOTION_NO_BRAKE, FUSION_MOTION_REVERSE,
    FUSION_MOTION_DISTANCE, FUSION_MOTION_BACK, FUSION_MOTION_LATERAL,
    FUSION_MOTION_ANGLE, FUSION_MOTION_NO_CANDIDATE, FUSION_MOTION_PREFIX_SPACE
} FusionMotionReason;
typedef enum {
    FUSION_MOTION_FAULT = 0, FUSION_MOTION_BRAKE, FUSION_MOTION_ADVANCE
} FusionMotionDecision;
typedef struct {
    FusionMotionCurve curve;
    float distance_mm,back_mm,lateral_mm,rotation_rad;
    uint32_t held_ms;
    FusionMotionReason reason;
} FusionMotionReservation;
typedef struct {
    FusionMotionReservation before,after;
    FusionMotionCurve candidate;
    FusionMotionSample prefix;
    float prefix_mm;
    FusionMotionReason reason;
} FusionMotionProposal;

/* Prepare searches a bounded 20..8000 ms duration grid. Sample only a curve
 * returned by Prepare/Reserve/Propose, within its closed time interval. */
uint32_t FusionMotion_Prepare(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    uint32_t dt_ms,const FusionMotionLimits *limits,FusionMotionCurve *curve);
uint32_t FusionMotion_Sample(const FusionMotionCurve *curve,uint32_t elapsed_ms,FusionMotionSample *sample);
FusionMotionReason FusionMotion_Reserve(FusionMotionVector q,FusionMotionVector a,
    const FusionMotionRequest *request,const FusionMotionLimits *limits,FusionMotionReservation *out);
/* BRAKE keeps the old valid certificate (before). ADVANCE also provides a
 * proposed command prefix and its suffix reservation; caller must confirm
 * complete, timely delivery before advancing command state. */
FusionMotionDecision FusionMotion_Propose(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    const FusionMotionRequest *request,const FusionMotionLimits *limits,FusionMotionProposal *out);

#define FUSION_MOTION_FEEDBACK_PERIOD_MS 80U

typedef struct { float x,y; } FusionMotionPoint;
typedef struct {
    FusionMotionVector reference,measured; /* Same capture time, field mm and rad. */
    FusionMotionVector committed; /* Last ACK-committed correction, field mm/s and rad/s. */
    /* Each LR/RR/LF/RF slot uses its own reference displacement over 80 ms. */
    FusionMotionPoint slot_from[CHASSIS_WHEEL_COUNT],slot_to[CHASSIS_WHEEL_COUNT];
    float slot_yaw_delta[CHASSIS_WHEEL_COUNT]; /* Signed reference radians over the slot. */
    uint32_t reference_complete; /* Nominal duration reached at preparation time. */
    uint32_t capture_complete; /* Nominal duration reached at pose capture time. */
} FusionMotionFeedbackInput;
typedef struct {
    FusionMotionVector correction; /* Candidate, commit only after a timely complete ACK. */
    ChassisWheelSpeeds wheels; /* Signed logical RPM, rounded to 0.1 RPM. */
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT];
    uint32_t finish_requested; /* Zero batch request. Caller verifies physical stop. */
} FusionMotionFeedbackBatch;

/* Computes an 80 ms batch from an already validated reference and fresh pose.
 * Returns 0 on invalid inputs or unrepresentable commands, with out untouched. */
uint32_t FusionMotion_FeedbackBatch(const FusionMotionFeedbackInput *input,FusionMotionFeedbackBatch *out);
#endif
