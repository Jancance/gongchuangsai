"""Compile the real module dispatcher, Source completion fence and Stack tail."""
from pathlib import Path
import subprocess
import tempfile

WORK = Path(__file__).resolve().parent
USER = WORK / 'stage3_modules_20261120/project/CHASSIS/USER'

STUBS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"fixture:%d: %s\n",__LINE__,#x);exit(1);}} while(0)
#include "region_dispatch.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define SPF_STAGE3_MAX_LIFT_PULSES 82000U
#define S3_FAST_HANDOFF_OFFSET_PULSES 16000.0f
#define SOURCE_LIFT_PULSES_PER_MM 400.0f
#define SPF_DM 1U
#define SPF_EXT 2U
#define S3_RECOVER_FIRST_PHASE 17U
static uint32_t g_source_cycle_fault,g_stage3_fault,g_it_cause,g_low_fault,g_low_state;
static uint32_t g_startup_axes_cause,g_lift_ruler_cause,g_dm_jog_first_fault,g_source_body_step_reason;
static uint32_t g_source_cycle_state,g_stage3_state,g_it_request,g_low_request,g_source_cycle_request,g_stage3_request;
static uint32_t g_stage3_teach_request,s3_teach_pending,g_stage3_manual_state,g_stage3_replay_enable,g_source_body_step_request;
static uint32_t g_chassis_auto_active,g_lift_ruler_request,g_lift_ruler_stop_pending,g_vehicle_arm_stop_pending;
static uint32_t g_dm_jog_request,g_it_auto_state,g_region_state,g_pzq_near_state,g_it_state,g_dm_posvel_holding,g_dm_jog_stop_result;
static uint32_t g_source_cycle_colors[3],sc_bound_slots[3],sc_bound_batch,g_source_cycle_run,g_source_cycle_prep,g_source_cycle_item;
static uint32_t sc_pending,sc_retract_pending,sc_beep_started,g_source_cycle_beeps,g_source_cycle_beep_started,g_source_cycle_beep_done;
static uint32_t g_source_cycle_pick_beeps,g_source_cycle_pick_beep_pending,g_source_cycle_scheduled,g_source_cycle_done,g_source_cycle_body_steps;
static uint32_t g_low_wheel_hold_ready,g_low_wheel_hold_stage,g_low_wheel_hold_request,g_source_cycle_started;
static uint32_t s3_module_run,g_stage3_colors[3],g_stage3_positions[3],g_stage3_slot_colors[3],g_stage3_cycle_mode;
static uint32_t g_stage3_test_first_item,g_stage3_test_count,g_stage3_task_run,g_stage3_task_batch,g_stage3_run;
static uint32_t g_stage3_placed_count,g_stage3_recovered_count,g_stage3_locked_run,g_stage3_phase,g_stage3_item;
static uint32_t g_stage3_first_mapping_valid,g_stage3_first_colors[3],g_stage3_first_positions[3];
static uint32_t g_source_cycle_due_tick,g_source_cycle_rejected,g_source_cycle_start_lag_ms,g_low_done_sequence;
static uint32_t g_source_cycle_piece_ms[3],g_source_cycle_grip_command_ms[3],g_low_elapsed_ms,g_low_grip_command_ms;
static uint32_t s3_first,s3_count,s3_cycle_mode,s3_low_handoff,s3_round_trip,s3_tray_valid,s3_slots[3],s3_colors[3],s3_positions[3];
static uint32_t s3_locked_top_scale,g_extension_home_reference_deg,s3_locked_extension_home;
static uint32_t g_it_action,g_it_rejected,s3_pending,last_axis;
static uint32_t xTaskGetTickCount(void){return 0;}
static uint32_t s3_locked_top_raw,top;
static struct {uint32_t top_scale,top_position_raw;} g_lift_ruler_controller;
static float g_stage3_pick_depth=8003.47f,g_stage3_carrier_yaw=36.5f;
static float g_stage3_target_yaw[3]={224.487f,180.55f,139.55f},g_stage3_target_x[3]={43,0,70.5f};
static float g_stage3_target_depth[3]={61802.2227f,60795.0238f,63002.2227f};
static float g_stage3_recovery_depth[3]={61802.2227f,61595.0238f,63002.2227f};
static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3],s3_recovery_depth[3];
static uint32_t S3_Platform(void){return 1U;}
static uint32_t Low_Busy(void){return g_low_state==1U;}
static uint32_t SourceBodyStep_WheelWindow(void){return 0U;}
static uint32_t SourceBodyStep_Busy(void){return 0U;}
static uint32_t VehicleArm_Busy(void){return 0U;}
static uint32_t VehicleSourceParallel_Owns(void){return 0U;}
static uint32_t Stage2Fast_IsActive(void){return 0U;}
static uint32_t SourceStage2_IsActive(void){return 0U;}
static uint32_t DM_CAN2_JogIsBusy(void){return 0U;}
uint32_t Stage2Body_Owns(void){return 0U;}
static void SC_Pose(void){}
static void S3_Pose(void){}
static void S3_Command(uint32_t action){s3_pending=g_it_request=action;}
static void S3_Axis(uint32_t axis,float v){assert(v==0.0f);last_axis=axis;S3_Command(3);}
static uint32_t it_at_top(void){return top;}
static void SC_Fail(uint32_t cause){g_source_cycle_fault=cause;g_source_cycle_state=9;}
static void S3_Fail(uint32_t cause){g_stage3_fault=cause;g_stage3_state=9;}
static void Stage3_Arrive(void){g_stage3_request=1;}
static void Stage3_KeyEvent(void){}
static void SourceCycle_KeyEvent(void){}
static void SourceCycle_ScanService(void){}
static uint32_t Stage3_SharedByte(uint8_t b,uint32_t n,uint32_t a){return (uint32_t)b+n+a;}
static uint32_t SourceCycle_SharedByte(uint8_t b,uint32_t n,uint32_t a){return (uint32_t)b+n+a;}
static void Stage3_Telemetry(uint32_t n){(void)n;}
static void SourceCycle_Telemetry(uint32_t n){(void)n;}
static uint32_t Low_TryStart(uint32_t n){(void)n;return 0U;}
static void SC_PickBeep(uint32_t n){(void)n;}
'''

TEST = r'''
int main(void){
 RegionModuleRequest r={1,1,REGION_MODULE_SOURCE,{5,2,1},{3,1,2},{2,1,5}};
 assert(RegionDispatch_Begin(&r) && RegionDispatch_Poll(1)==REGION_MODULE_BUSY);
 assert(!RegionDispatch_Stop(2) && RegionDispatch_Stop(1));
 RegionDispatch_Service(0);assert(RegionDispatch_Poll(1)==REGION_MODULE_STOPPED && !g_source_cycle_fault);
 assert(!RegionDispatch_Begin(&r));
 r.run_id=2;assert(RegionDispatch_Begin(&r));
 r.colors[0]=4;assert(!RegionDispatch_Begin(&r));RegionDispatch_Service(0);
 assert(g_source_cycle_colors[0]==5 && sc_bound_batch==1 && sc_bound_slots[2]==5 && g_source_cycle_run==2);
 g_source_cycle_state=7;g_source_cycle_item=2;sc_pending=11;g_low_state=2;g_low_done_sequence=10;
 RegionDispatch_Service(0);assert(RegionDispatch_Poll(2)==REGION_MODULE_BUSY);
 g_low_done_sequence=11;RegionDispatch_Service(0);assert(RegionDispatch_Poll(2)==REGION_MODULE_DONE);
 r.colors[0]=5;r.run_id=3;r.batch_id=2;r.module=REGION_MODULE_ROUGH;
 assert(RegionDispatch_Begin(&r));RegionDispatch_Service(0);assert(g_stage3_run==3 && s3_cycle_mode==1);
 g_stage3_phase=31;g_stage3_placed_count=g_stage3_recovered_count=3;
 RegionDispatch_Service(0);assert(RegionDispatch_Poll(3)==REGION_MODULE_BUSY);
 g_stage3_phase=32;g_stage3_state=6;RegionDispatch_Service(0);assert(RegionDispatch_Poll(3)==REGION_MODULE_DONE);
 r.run_id=4;r.module=REGION_MODULE_STACK;
 assert(RegionDispatch_Begin(&r));RegionDispatch_Service(0);
 assert(s3_cycle_mode==2 && s3_positions[0]==3 && s3_positions[1]==1 && s3_positions[2]==2);
 assert(s3_depth[0]==g_stage3_target_depth[0]-24000.0f && !g_stage3_recovered_count);
 g_stage3_phase=30;g_stage3_item=g_stage3_placed_count=2;
 RegionDispatch_Service(0);assert(g_stage3_phase==33 && g_stage3_placed_count==3 && !g_it_request);
 RegionDispatch_Service(0);assert(g_stage3_phase==33 && g_it_request==4);
 g_it_request=0;g_it_state=2;g_it_action=3;RegionDispatch_Service(0);assert(g_stage3_phase==33 && last_axis==0);
 top=1;g_it_action=4;RegionDispatch_Service(0);assert(g_stage3_phase==34 && last_axis==SPF_EXT);
 g_it_request=0;g_it_state=1;RegionDispatch_Service(0);assert(g_stage3_phase==34 && RegionDispatch_Poll(4)==REGION_MODULE_BUSY);
 g_it_state=2;g_it_action=3;RegionDispatch_Service(0);assert(g_stage3_phase==35 && last_axis==SPF_DM);
 g_it_request=0;g_it_state=1;RegionDispatch_Service(0);assert(RegionDispatch_Poll(4)==REGION_MODULE_BUSY);
 g_it_state=2;RegionDispatch_Service(0);assert(g_stage3_phase==36 && RegionDispatch_Poll(4)==REGION_MODULE_DONE);
 r.run_id=5;assert(RegionDispatch_Begin(&r));RegionDispatch_Service(0);
 assert(s3_depth[0]==g_stage3_target_depth[0]-24000.0f);
 g_dm_posvel_holding=1;assert(RegionDispatch_Stop(5));RegionDispatch_Service(0);
 assert(RegionDispatch_Poll(5)==REGION_MODULE_STOPPING && g_stage3_fault==2);
 g_it_request=0;g_dm_posvel_holding=0;RegionDispatch_Service(0);assert(RegionDispatch_Poll(5)==REGION_MODULE_STOPPING);
 g_dm_jog_stop_result=1;RegionDispatch_Service(0);assert(RegionDispatch_Poll(5)==REGION_MODULE_STOPPED);
 r.run_id=6;assert(!RegionDispatch_Begin(&r));
 puts("PASS module lifecycle, immutable task, matching Source DONE, Rough terminal fence, Stack safe top/X/DM tail and STOP ACK");
 return 0;
}
'''


def main():
    stage = (USER/'stage3_cycle.inc').read_text(encoding='latin1')
    source = (USER/'source_mcu_cycle.inc').read_text(encoding='latin1')
    dispatch = (USER/'region_dispatch.inc').read_text(encoding='latin1')
    target = stage[stage.index('static uint32_t S3_TargetPosition('):stage.index('static uint32_t S3_Config(')]
    lock = stage[stage.index('static void S3_LockConfig('):stage.index('static void S3_Command(')]
    stack = stage[stage.index('  case 30U:'):stage.index('  case 15U:',stage.index('  case 30U:'))]
    tail = source[source.index(' if(g_source_cycle_state==7U){'):].rstrip()[:-1]
    compiled = '\n'.join((STUBS,target,lock,
        'static void SourceCycle_Service(uint32_t now){'+tail+'}',
        '''static void Stage3_Service(uint32_t now){(void)now;
        if(g_stage3_request==9U){g_stage3_request=0;return;}
        if(g_stage3_request==1U){g_stage3_request=0;g_stage3_run++;S3_LockConfig();return;}
        if(s3_pending){if(g_it_request || g_it_state!=2 || s3_pending!=g_it_action)return;s3_pending=0;g_stage3_phase++;}
        switch(g_stage3_phase){'''+stack+'default:return;}}',dispatch,TEST))
    with tempfile.TemporaryDirectory(dir=WORK) as tmp:
        path = Path(tmp)/'module_fixture.c'
        path.write_text(compiled,encoding='latin1')
        exe = path.with_suffix('.exe')
        subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',
                        '-I',str(USER),str(path),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    main()
