#ifndef TEST_MOTION_H
#define TEST_MOTION_H
#include "motion_action_plan.h"
typedef struct {
    const uint32_t *action_count,*session_open,*has_action,*turn,*mode,*base_rpm;
    const float *distance_mm,*reference_mm_s;
    const uint32_t *begin_ms;
    const volatile uint32_t *end_ms;
    const uint32_t *stage_ms;
    const MotionAction *actions;
    const float (*perimeter_targets)[3];
} AppMotionObservation;
extern volatile uint32_t g_debug_plan_id;
const AppMotionObservation *AppMotion_Live(void);
#endif
