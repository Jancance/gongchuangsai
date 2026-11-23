#ifndef ZONE2_MOTION_REFERENCE_H
#define ZONE2_MOTION_REFERENCE_H

#include "fusion_motion.h"
#include "fusion_reference.h"

#define ZONE2_MOTION_SPAN_COUNT (2U*FUSION_POINTS-1U)
#define ZONE2_MOTION_SECTION_COUNT (FUSION_POINTS+1U)

typedef struct {
    uint32_t duration_ms;
    uint32_t first_span,span_count,terminal;
} Zone2MotionSection;

typedef struct {
    uint32_t duration_ms,kind,ramp_ms;
    union {
        FusionPoint control[6];
        struct {
            FusionPoint from,to;
            float entry,cruise,exit;
            float rise_s,fall_s;
        } line;
        struct { FusionPoint center; float start_rad,delta_rad; } turn;
    } data;
} Zone2MotionSpan;

typedef struct {
    uint32_t section_count,duration_ms,span_count;
    float hold_rad;
    Zone2MotionSection sections[ZONE2_MOTION_SECTION_COUNT];
    Zone2MotionSpan spans[ZONE2_MOTION_SPAN_COUNT];
} Zone2MotionReference;

/* Use distinct objects. The caller supplies a valid immutable FusionPlan and
 * static reference storage. On failure section_count is zero. The modeled
 * time includes 160 ms at interior translation stops; real stops are separate. */
uint32_t Zone2MotionReference_Prepare(const FusionPlan *plan,Zone2MotionReference *reference);
/* Explicit configured raw-parking return only (node 1, route path 3 or 6).
 * Uses the ordinary real section-end stop/quiet handshake, with no fixed dwell. */
uint32_t Zone2MotionReference_PrepareWithStop(const FusionPlan *plan,
    Zone2MotionReference *reference,uint32_t stop_node);
/* Time is relative to the selected section, clamped at both ends. The
 * caller waits for real stop evidence before selecting the next section. */
uint32_t Zone2MotionReference_Sample(const Zone2MotionReference *reference,
    uint32_t section,int32_t elapsed_ms,FusionMotionVector *pose);

#endif
