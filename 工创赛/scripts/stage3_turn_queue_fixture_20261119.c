#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_TEST_MODE 3U
#define APP_FAULT_CUT_VM_ENABLE 1U
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HS_STOPPING 4U
#define HS_RUN_MS 4000U
#define HS_STOP_MS 1000U
#define App_StageDoneVm() (s_combined_keep_vm)
#define App_StageStartVm() (s_combined_powered_segment)
typedef struct { uint32_t active, result, cut_vm, rotation_stop_mask; } ChassisAutoStatus;
static ChassisAutoStatus backend;
static uint32_t g_heading_exec_phase, g_heading_exec_request, g_heading_exec_trip;
static uint32_t g_heading_exec_state, g_heading_start_reject_mask, g_can_ctrl_fault_latched;
static uint32_t g_vm_test_request, g_safety_ready, g_can_guard_ready, s_heading_exec_wait_ms;
static uint32_t s_motion_session_open, s_motion_has_action, s_combined_turn;
static uint32_t s_combined_powered_segment, s_combined_keep_vm, tick, depth, checks;
static uint32_t healthy, exclusive, backend_owned;
static struct { uint32_t state, cause; } g_combined_test;
static struct { uint32_t request, trip; } g_forward_hold;
static struct { float rate_dps; } s_heading_source;
static uint32_t s_combined_end_ms;
static float s_heading_exec_relative, g_heading_exec_relative_deg;
#define taskENTER_CRITICAL() (++depth)
#define taskEXIT_CRITICAL() (assert(depth), --depth)
static uint32_t xTaskGetTickCount(void) { return tick; }
static uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s) { *s=backend; return 1; }
static uint32_t App_HeadingRuntimeSource(uint32_t now) { (void)now; return healthy; }
static uint32_t App_HeadingRuntimeExclusive(void) { return exclusive; }
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *s) { (void)s; return backend_owned; }
static void App_HeadingRuntimeTrip(uint32_t cause) {
    if (!g_heading_exec_trip) g_heading_exec_trip=cause;
    g_vm_test_request=0;
}
static uint32_t Stage3_FineTurnAllowed(void) { return 0; }
static void ChassisAuto_Abort(void) {}
static void Power_Disable(void) {}
static void App_ForwardHoldTrip(uint32_t cause) { g_forward_hold.trip=cause; }
static void check(uint32_t ok) { assert(ok); assert(depth==0); ++checks; }

// INSERT_EXTRACTED_FIRMWARE

static void accept_turn(void) {
    memset(&backend, 0, sizeof backend);
    memset(&g_combined_test, 0, sizeof g_combined_test);
    g_heading_exec_request=g_vm_test_request=s_motion_session_open=s_motion_has_action=1;
    s_combined_powered_segment=s_combined_turn=s_combined_keep_vm=1;
    healthy=exclusive=backend_owned=1;
    g_heading_exec_phase=HR_IDLE;
    g_heading_exec_trip=g_heading_start_reject_mask=g_can_ctrl_fault_latched=0;
    g_heading_exec_relative_deg=30;
    s_heading_source.rate_dps=0;
}

int main(void) {
    accept_turn();
    old_safety();
    check(g_vm_test_request==0);
    can_admit();
    check(g_heading_start_reject_mask==8 && g_heading_exec_trip==3 && g_heading_exec_phase==HR_IDLE);
    puts("Baseline reproduced mask=8, trip=3 after Safety before CAN.");
    accept_turn();
    new_safety();
    new_safety();
    check(g_vm_test_request==1 && g_heading_exec_request==1 && g_heading_exec_phase==HR_IDLE);
    can_admit();
    check(!g_heading_start_reject_mask && !g_heading_exec_trip && g_heading_exec_phase==HR_WAIT);
    new_safety();
    check(g_vm_test_request==1 && !g_heading_exec_trip);
    for (uint32_t i=0; i<14; ++i) {
        accept_turn();
        switch(i) {
            case 0: g_heading_exec_request=0; break;
            case 1: g_heading_exec_request=2; break;
            case 2: s_motion_session_open=0; break;
            case 3: s_motion_has_action=0; break;
            case 4: s_combined_turn=0; break;
            case 5: s_combined_powered_segment=0; break;
            case 6: g_vm_test_request=0; break;
            case 7: g_heading_exec_trip=2; break;
            case 8: healthy=0; break;
            case 9: exclusive=0; break;
            case 10: g_can_ctrl_fault_latched=1; break;
            case 11: backend.active=1; break;
            case 12: backend.result=1; break;
            case 13: backend.cut_vm=1; break;
        }
        new_safety();
        check(!g_vm_test_request && g_heading_exec_phase==HR_IDLE);
        if (i==7) check(g_heading_exec_trip==2);
        if (i>=6) {
            can_admit();
            check(g_heading_exec_phase==HR_IDLE && !g_vm_test_request);
        }
    }
    for (uint32_t i=0; i<2; ++i) {
        accept_turn();
        if (i==0) MotionStop();
        new_safety();
        if (i==1) MotionStop();
        can_admit();
        check(!g_vm_test_request && !g_heading_exec_request && !s_motion_session_open &&
              g_heading_exec_phase==HR_IDLE && g_combined_test.state==5);
    }
    accept_turn();
    g_heading_exec_phase=HR_DONE;
    new_safety();
    check(g_vm_test_request==1);
    s_combined_keep_vm=0;
    new_safety();
    check(g_vm_test_request==0);
    printf("PASS extracted Safety, CAN admission and STOP: %u checks; no hardware\n", checks);
    return 0;
}
