"""Extract/test actual VSP decode/prepare/STOP code, no device connection."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
OUT = ROOT / 'tmp/stage2_sub20_20261109'
E = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
text = (P / 'vehicle_source_parallel.inc').read_bytes().decode('utf-8', errors='replace')
types = text[text.index('typedef struct {uint32_t phase'):text.index('static uint32_t VSP_Query')]
functions = text[text.index('static uint32_t VSP_Query'):text.index('static uint32_t VSP_Start')]
stop = text[text.index('static uint32_t VSP_Stop'):text.index('static uint32_t VSP_Clock')]
it = (P / 'integration10_complete.inc').read_bytes().decode('utf-8', errors='replace')
empty = it[it.index('static uint32_t it_empty_overlap_begin'):it.index('/* Left single-piece loaded commissioning')]
loaded = it[it.index('static uint32_t it_loaded_overlap_begin'):it.index('#include "integration13_immediate.inc"')]
unit = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
#undef assert
#define assert(x) do{if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define STARTUP_EXTENSION_EXTEND_SIGN (-1.0f)
#define VEHICLE_LIFT_DOWN_SIGN 1.0f
#define VEHICLE_EXTENSION_PULSES_PER_MM 315.5f
static uint32_t tick=50,options,raw=36000,sign;
static uint32_t mock_speed,mock_speed_miss,mock_speed_sign;
static struct {uint8_t Data[8];} s_rx;
static LiftRulerController g_lift_ruler_controller;
static uint32_t g_vehicle_lift_feedback_valid,g_startup_extension_feedback_valid;
static float g_vehicle_lift_current_deg,g_startup_extension_current_deg,g_extension_home_reference_deg;
static uint32_t g_dm_can2_feedback_valid,g_dm_can2_feedback_tick,g_dm_parallel_group_active,g_dm_parallel_group_go;
static uint32_t g_dm_source_smooth,g_dm_point_enable,g_dm_jog_request,g_dm_can2_read_cycle;
static uint32_t g_dm_jog_first_fault,g_dm_jog_state,g_dm_jog_result,g_dm_posvel_holding,g_dm_can2_motor_state,g_dm_jog_stop_result;
static float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg;
static uint32_t xTaskGetTickCount(void){return tick;}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t VehicleArm_Exchange(uint8_t addr,const uint8_t *q,uint32_t n,uint32_t count,void *out){
 (void)count;(void)out;(void)n;assert(addr==5 || addr==6);memset(&s_rx,0,sizeof s_rx);
 if(q[0]==0x35){if(mock_speed_miss)return 0;s_rx.Data[1]=(uint8_t)mock_speed_sign;
  s_rx.Data[2]=(uint8_t)(mock_speed>>8);s_rx.Data[3]=(uint8_t)mock_speed;}
 if(q[0]==0x1a)s_rx.Data[2]=(uint8_t)options;
 if(q[0]==0x36){s_rx.Data[1]=(uint8_t)sign;s_rx.Data[2]=(uint8_t)(raw>>24);
  s_rx.Data[3]=(uint8_t)(raw>>16);s_rx.Data[4]=(uint8_t)(raw>>8);s_rx.Data[5]=(uint8_t)raw;}
 if(q[0]==0xf3 || q[0]==0xfe)s_rx.Data[1]=2;
 return 1;
}
''' + types + functions + stop + r'''
#define APP_MOTOR_OFFLINE_ONLY 0
static uint32_t mock_gate=1,mock_busy,g_it_sequence,g_lift_ruler_request,g_lift_ruler_stop_pending;
static uint32_t g_lift_compare_target_pulses,g_it_state,g_it_action;
static uint32_t g_it_lead5_grip_done,g_source_parallel_started_mask,grip_writes;
static float g_it_extension_mm,g_it_lift_pulses,g_it_dm_deg,g_it_carrier_auto_deg=-1.0f,it_carrier_latched_deg;
static uint32_t g_it_carrier_auto_done;
static uint32_t jiazhua(float deg){assert(deg==160.0f || deg==140.0f);if(deg==140)grip_writes++;return 1;}
static void it_fail(uint32_t why){(void)why;assert(0);}
static SPF_Command it_command;
static uint32_t SingleTeach_ReferencesReady(void){return mock_gate;}
static uint32_t it_at_top(void){return mock_gate;}
static uint32_t it_dm_fresh(void){return mock_gate;}
static uint32_t VehicleArm_Busy(void){return mock_busy;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *c,uint32_t sweep,uint32_t stow){
 (void)stow;assert(sweep==1 && (c->axes_mask>=4 && c->axes_mask<=7));vsp_group.command=*c;return 1;
}
static uint32_t VehicleSourceParallel_Owns(void){return 1;}
static void VehicleSourceParallel_Cancel(void){}
''' + empty + loaded + r'''
static uint32_t u16(uint8_t *p){return (uint32_t)p[0]*256+p[1];}
static uint32_t u32(uint8_t *p){return (uint32_t)p[0]*16777216+(uint32_t)p[1]*65536+(uint32_t)p[2]*256+p[3];}
int main(void){
 SPF_Command c={0};c.lift_rpm=1846.4f;c.extension_rpm=106;c.stepper_accel_rpm_s=600;
 c.goal.lift_pulses=6400;c.goal.extension_mm=100;
 for(uint32_t scale=10;scale<=100;scale*=10){
  memset(vsp_axis,0,sizeof vsp_axis);options=scale==100?128:0;g_lift_ruler_controller.top_scale=scale;
  raw=360*scale;sign=0;vsp_fast_lift=1;
  assert(!VSP_Prepare(0,SPF_LIFT,&c));assert(!VSP_Prepare(0,SPF_LIFT,&c));
  assert(fabsf(vsp_axis[1].current-360)<.001f && vsp_axis[1].position_tick==tick);
  assert(u32(vsp_axis[1].move+8)==360*scale);
  assert(u16(vsp_axis[1].move+2)==45055 && u16(vsp_axis[1].move+6)==18464);
 }
 memset(vsp_axis,0,sizeof vsp_axis);options=128;g_lift_ruler_controller.top_scale=10;
 assert(VSP_Prepare(0,SPF_LIFT,&c)==2); /* configuration drift */
 memset(vsp_axis,0,sizeof vsp_axis);options=128;raw=0;sign=0;
 assert(!VSP_Prepare(0,SPF_EXT,&c));assert(!VSP_Prepare(0,SPF_EXT,&c));
 assert(u16(vsp_axis[0].move+2)==600 && u16(vsp_axis[0].move+6)==1060);
 assert(vsp_axis[0].move[1]==1 && u32(vsp_axis[0].move+8)==35494);
 raw=36000;sign=1;assert(VSP_Position(6,&vsp_axis[0]));assert(vsp_axis[0].current==-3600);
 /* A queued retarget is NOT a stopped/holding axis: replace with STOP. */
 g_dm_posvel_holding=1;g_dm_jog_state=7;g_dm_jog_result=1;g_dm_jog_request=4;
 assert(!VSP_Stop(0,SPF_DM) && g_dm_jog_request==3);
 g_dm_posvel_holding=0;g_dm_jog_state=8;g_dm_jog_stop_result=1;g_dm_jog_request=0;
 assert(VSP_Stop(0,SPF_DM));
 g_startup_extension_current_deg=0;g_dm_can2_logical_position_deg=195;g_it_dm_deg=195;
 g_it_extension_mm=156;g_it_lift_pulses=118800;
 assert(it_empty_overlap_begin() && it_command.axes_mask==6 && vsp_fast_lift==1);
 g_lift_ruler_controller.top_scale=100;vsp_axis[1].current=13365.02f;
 it_empty_overlap_publish();assert(g_lift_ruler_controller.operation==2 &&
  !g_lift_ruler_controller.backoff_valid && g_lift_ruler_controller.position_after_raw==1336502);
 g_dm_can2_logical_position_deg=180;assert(!it_empty_overlap_begin());
 g_dm_can2_logical_position_deg=195;g_it_extension_mm=170;assert(!it_empty_overlap_begin());
 g_it_extension_mm=156;g_it_lift_pulses=118801;assert(!it_empty_overlap_begin());
 g_it_lift_pulses=118800;mock_gate=0;assert(!it_empty_overlap_begin());
 mock_gate=1;mock_busy=1;assert(!it_empty_overlap_begin());
 mock_busy=0;g_startup_extension_current_deg=-5537.025f;
 g_lift_ruler_controller.operation=2;g_lift_ruler_controller.samples=1;
 g_lift_compare_target_pulses=118800;g_it_lift_pulses=7200;
 assert(it_loaded_overlap_begin() && vsp_group.delayed_mask==3 && vsp_group.release_mask==4 && it_command.goal.dm_deg==37.0f);
 g_it_state=1;g_it_action=15;vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=tick;
 vsp_axis[1].current=115000*360.0f/3200.0f;
 it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==4);
 vsp_axis[1].current=110000*360.0f/3200.0f;
 it_loaded_overlap_gates(tick+151);assert(vsp_group.release_mask==4);
 it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==6);
 vsp_group.done=4;it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==6);
 vsp_group.done=6;vsp_axis[0].samples=3;vsp_axis[0].current=2.1f;
 it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==6);
 vsp_axis[0].current=.9f;it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==6);
 vsp_axis[1].current=15000*360.0f/3200;it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==7);
 vsp_axis[1].current=810;g_lift_ruler_controller.top_scale=100;
 it_loaded_overlap_publish();assert(g_lift_ruler_controller.safe_position_raw==81000 && g_lift_ruler_controller.operation==3);
 for(uint32_t i=0;i<3;i++){
  float yaw[3]={180,195,165};float ext[3]={0,156,160};
  g_it_dm_deg=g_dm_can2_logical_position_deg=yaw[i];g_it_extension_mm=ext[i];
  g_startup_extension_current_deg=0;g_it_lift_pulses=118800;g_it_carrier_auto_deg=(float)i*120;
  assert(it_empty_overlap_begin() && it_command.axes_mask==(i?6U:4U));
  assert(!g_it_carrier_auto_done && it_carrier_latched_deg==(float)i*120);
  g_lift_ruler_controller.operation=2;g_lift_ruler_controller.samples=1;
  g_it_lift_pulses=7200;g_startup_extension_current_deg=-ext[i]*315.5f*360/3200;
  assert(it_loaded_overlap_begin() && it_command.axes_mask==(i?7U:5U));
 }
 g_it_state=1;g_it_action=15;vsp_group.release_mask=6;vsp_group.done=0;
 vsp_axis[0].samples=0;vsp_axis[0].current=-1000; /* still retracting */
 vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=tick;
 vsp_axis[1].current=15000*360.0f/3200;
 it_loaded_overlap_gates(tick);assert(vsp_group.release_mask==7);
 /* Retry a single missed/nonzero speed, then refresh the movement origin. */
 memset(vsp_axis,0,sizeof vsp_axis);vsp_axis[0].phase=2;mock_speed=10;
 assert(!VSP_Prepare(0,SPF_EXT,&c) && vsp_axis[0].cycle==1 && vsp_axis[0].phase==2);
 mock_speed=0;assert(!VSP_Prepare(0,SPF_EXT,&c) && vsp_axis[0].phase==1);
 raw=0;sign=0;assert(!VSP_Prepare(0,SPF_EXT,&c) && vsp_axis[0].phase==2);
 assert(!VSP_Prepare(0,SPF_EXT,&c) && vsp_axis[0].phase==3);
 vsp_axis[0].phase=2;mock_speed_miss=1;assert(!VSP_Prepare(0,SPF_EXT,&c) && vsp_axis[0].cycle==1);
 mock_speed_miss=0;mock_speed_sign=2;assert(VSP_Prepare(0,SPF_EXT,&c)==2);mock_speed_sign=0;
 /* Close only in last5mm, with fresh lift and settled extension. */
 g_it_state=1;g_it_action=14;g_it_lead5_grip_done=0;g_source_parallel_started_mask=4;
 vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=tick;it_command.axes_mask=6;
 vsp_axis[1].current=115000*360.0f/3200;vsp_group.done=0;
 it_lead5_grip_service(tick);assert(!g_it_lead5_grip_done);
 vsp_group.done=2;it_lead5_grip_service(tick+151);assert(!g_it_lead5_grip_done);
 it_lead5_grip_service(tick);assert(g_it_lead5_grip_done && grip_writes==1);
 it_lead5_grip_service(tick);assert(grip_writes==1);
 /* Only zero-return transient65.8deg may settle, never hard90+ or150ms. */
 memset(vsp_axis,0,sizeof vsp_axis);vsp_axis[0].before=-5678.5f;vsp_axis[0].goal=0;
 vsp_axis[0].current=65.8f;tick=100;assert(VSP_PositionWindow(&vsp_axis[0],1));
 tick=249;assert(VSP_PositionWindow(&vsp_axis[0],1));
 tick=250;assert(!VSP_PositionWindow(&vsp_axis[0],1));
 vsp_axis[0].current=2;assert(VSP_PositionWindow(&vsp_axis[0],1) && !vsp_axis[0].overshoot_active);
 vsp_axis[0].current=90.1f;assert(!VSP_PositionWindow(&vsp_axis[0],1));
 vsp_axis[0].current=65.8f;assert(!VSP_PositionWindow(&vsp_axis[0],0));
 vsp_axis[0].current=670;assert(!VSP_PositionWindow(&vsp_axis[0],1));
 puts("PASS: actual VSP node5 scale10/100 decode+frame, drift rejection, node6 units/speed/accel unchanged, queued retarget cancellation. SIMULATED I/O ONLY.");
 return 0;
}
'''
src = OUT / 'stage2-fast-port-test.c'
exe = OUT / 'stage2-fast-port-test.exe'
src.write_text(unit, encoding='utf-8')
r = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-Wno-misleading-indentation', '-Wno-unused-variable', '-I', str(P),
                    str(src), '-lm', '-o', str(exe)], capture_output=True, text=True)
log = r.stdout+r.stderr
if r.returncode == 0:
    r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=15)
    log += r.stdout+r.stderr
(E / 'stage2-fast-port-check-20261109.log').write_text(log, encoding='utf-8')
print(log)
assert r.returncode == 0
