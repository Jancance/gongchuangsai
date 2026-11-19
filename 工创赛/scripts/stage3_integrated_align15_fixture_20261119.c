#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

static uint32_t g_stage3_run=1, g_stage3_replay_run=1, g_source_body_step_reason;
static uint32_t g_source_align_rejected, s3_replay_rejected, g_stage3_replay_state=1, g_stage3_state=2;
static uint32_t g_stage3_replay_step, g_source_body_step_request, g_source_align_command_seq;
static uint32_t g_source_body_step_state=3, g_source_body_step_session, s3_replay_session;
static uint32_t s3_replay_expected, s3_replay_previous, g_chassis_auto_active, g_chassis_auto_request;
static uint32_t g_source_align_chain_active=1, g_it_request, g_it_state=2, g_stage3_body_steps;
static uint32_t g_source_align_direction, g_low_wheel_hold_stage=13, g_low_wheel_hold_cycle=5;
static uint32_t g_low_wheel_hold_ready=1, s3_replay_lock_cycle, g_low_wheel_hold_request;
static uint32_t g_stage3_align_done, g_stage3_task_run=1, g_stage3_config_ready=1;
static uint32_t g_stage3_error_norm_um=2000, g_stage3_error_yaw_mdeg=1000;
static uint32_t fresh, failed, beeping, beep_count, config_count, poses;
static float g_source_align_distance_mm;
static uint32_t s3_first,s3_count,s3_round_trip,s3_colors[3],s3_positions[3],s3_slots[3];
static uint32_t g_stage3_test_first_item,g_stage3_test_count=3,g_stage3_placed_count,g_stage3_recovered_count;
static uint32_t g_stage3_colors[3],g_stage3_positions[3],g_stage3_slot_colors[3],g_stage3_item,g_stage3_phase;
static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3],s3_recovery_depth[3];
static float g_stage3_target_yaw[3],g_stage3_target_x[3],g_stage3_target_depth[3],g_stage3_recovery_depth[3];
static uint32_t S3_Fresh(uint32_t now) { (void)now; return fresh; }
static void S3_Pose(void) { ++poses; }
static void S3_Fail(uint32_t cause) { failed=cause; g_stage3_state=g_stage3_replay_state=9; }
static uint32_t S3_ReplayNext(uint32_t previous) { return previous+1; }
static uint32_t SourceBodyStep_Busy(void) { return 0; }
static uint32_t SourceBodyStep_WheelWindow(void) { return g_source_align_chain_active; }
static uint32_t Buzzer_IsBusy(void) { return beeping; }
static void Buzzer_StartTimed(uint32_t count, uint32_t on, uint32_t off) {
    assert(count==2 && on==80 && off==80);
    ++beep_count;
    beeping=1;
}

// INSERT_EXTRACTED_FIRMWARE

static void start_visual(void) {
    g_stage3_replay_state=1;
    g_stage3_state=2;
    g_stage3_align_done=0;
    g_source_body_step_request=0;
    g_low_wheel_hold_request=0;
}

int main(void) {
    visual_accept();
    assert(g_stage3_state==2 && g_stage3_replay_state==1);
    fresh=1;
    g_stage3_error_norm_um=2001;
    visual_accept();
    assert(g_stage3_state==2 && g_stage3_replay_state==1);
    g_stage3_error_norm_um=2000;
    visual_accept();
    assert(g_stage3_state==4 && g_stage3_replay_state==3 && g_stage3_integrated_compensation==1);
    assert(g_source_body_step_request==7 && !g_stage3_replay_step && !g_stage3_body_steps);
    final_service(0);
    assert(!g_low_wheel_hold_request && !beep_count);
    /* SBS's existing finish request releases its wheel window. */
    g_source_align_chain_active=g_source_body_step_request=0;
    final_service(0);
    assert(g_low_wheel_hold_request==1 && !beep_count);
    g_low_wheel_hold_stage=13;
    g_low_wheel_hold_ready=1;
    final_service(0);
    assert(g_low_wheel_hold_request==1 && !beep_count);
    ++g_low_wheel_hold_cycle;
    g_low_wheel_hold_request=0;
    final_service(0);
    assert(g_stage3_replay_state==4 && beep_count==1 && !config_count);
    final_service(0);
    assert(beep_count==1 && !config_count);
    beeping=0;
    final_service(0);
    final_service(0);
    assert(g_stage3_state==5 && g_stage3_replay_state==5 && g_stage3_align_done);
    assert(s3_carrier==36.5f && s3_count==3 && s3_round_trip==1);
    assert(beep_count==1 && config_count==1 && !g_stage3_replay_step && !g_stage3_body_steps);
    start_visual();
    visual_accept();
    assert(!g_source_body_step_request && g_stage3_replay_state==3);
    final_service(9);
    assert(failed==2 && g_stage3_state==9 && beep_count==1 && config_count==1);
    start_visual();
    visual_accept();
    g_it_request=9;
    final_service(0);
    assert(failed==2 && g_stage3_state==9 && beep_count==1 && config_count==1);
    start_visual();
    g_it_request=0;
    g_stage3_replay_state=0;
    visual_accept();
    assert(g_stage3_state==4 && g_stage3_align_done && !g_stage3_replay_state);
    puts("PASS extracted Stage3 visual acceptance, fresh hold, double beep, one config lock, and two STOP paths");
    return 0;
}
