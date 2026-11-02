#ifndef SAFETY_H
#define SAFETY_H
#include <stdint.h>
#define MOTION_STOP_IDLE 0U
extern volatile uint32_t g_safety_fault,g_safety_ready,g_can_guard_ready;
extern volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
extern volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm;
#endif
