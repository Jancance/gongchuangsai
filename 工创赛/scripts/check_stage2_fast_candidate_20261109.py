"""Run actual candidate C with explicitly simulated CAN axes/evidence/route."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
P = ROOT / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
OUT = ROOT / 'tmp/stage2_sub20_20261109'
E = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
unit = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"FAIL line %d: %s (tick=%u state=%u fault=%u group=%u done=%u)\n",__LINE__,#x,tick,g_stage2_fast_state,g_stage2_fast_fault,vsp_group.state,vsp_group.done);exit(1);}} while(0)
#include "source_parallel_flow.h"
#include "stage2_fast_cycle.h"
#define APP_MOTOR_OFFLINE_ONLY 0
typedef struct {uint32_t position_cycle,position_tick,samples;float current,goal;} VSP_Axis;
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;
static uint32_t tick,owns,busy,physical_fault,vm=1,servo_writes,close_writes,route_calls,stop_calls;
static uint32_t ready=1,evidence=1,route_failure,route_started_ms,route_before_stow;
static uint32_t g_safety_ready=1,g_can_test_result=1,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static uint32_t g_startup_axes_cause,g_dm_jog_first_fault,g_lift_ruler_cause,g_vehicle_arm_cause;
static uint32_t g_chassis_auto_active,g_it_request,g_it_state,g_lift_ruler_request,g_vehicle_arm_stop_pending;
static uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_stop_mask,g_it_sequence;
static float g_startup_extension_current_deg,g_extension_home_reference_deg,g_vehicle_lift_current_deg;
static struct {uint32_t top_scale,stop_pending,backoff_valid,operation;int64_t position_after_raw,safe_position_raw;} g_lift_ruler_controller;
static uint32_t vsp_fast_lift;
static uint32_t started_ms[3],start_count[3],prepare_count[3],done_counts[3];
static float physical[3],from[3],goal[3];
static uint32_t index_of(uint32_t bit){return bit==SPF_DM?0U:bit==SPF_EXT?1U:2U;}
static uint32_t VehicleSourceParallel_Owns(void){return owns;}
static uint32_t VehicleArm_Busy(void){return busy;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t SingleTeach_ReferencesReady(void){return ready;}
static uint32_t it_at_top(void){return ready;}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t jiazhua(float deg){servo_writes++;if(deg==140)close_writes++;return 1;}
static uint32_t CarrierPwm_Write(uint32_t pwm){assert(pwm==500||pwm==1389||pwm==2278);servo_writes++;return 1;}
static void VehicleSourceParallel_Cancel(void){SPF_GroupCancel(&vsp_group,2);}
static uint32_t prep(void *unused,uint32_t bit,const SPF_Command *c){
 uint32_t i=index_of(bit);(void)unused;prepare_count[i]++;
 if(bit==SPF_DM && c->kind==SPF_CMD_TRANSFER && c->axes_mask==7U)
  assert((vsp_group.done&SPF_EXT) && fabsf(physical[1])<=1.0f);
 return physical_fault?2:1;
}
static uint32_t start(void *unused,uint32_t bit,const SPF_Command *c){
 uint32_t i=index_of(bit);(void)unused;started_ms[i]=tick;start_count[i]++;from[i]=physical[i];
 goal[i]=bit==SPF_DM?c->goal.dm_deg:bit==SPF_EXT?-c->goal.extension_mm*35.5f:c->goal.lift_pulses*360.0f/3200.0f;
 return 1;
}
static uint32_t poll(void *unused,uint32_t bit){
 uint32_t i=index_of(bit),elapsed=tick-started_ms[i];float fraction=(float)elapsed/1000.0f;
 VSP_Axis *a=bit==SPF_EXT?&vsp_axis[0]:&vsp_axis[1];(void)unused;
 if(physical_fault)return 2;
 if(fraction>1.0f)fraction=1.0f;
 physical[i]=from[i]+fraction*(goal[i]-from[i]);
 if(bit!=SPF_DM){a->current=physical[i];a->position_tick=tick;a->position_cycle++;}
 if(bit==SPF_LIFT)g_vehicle_lift_current_deg=physical[i];
 if(bit==SPF_EXT)g_startup_extension_current_deg=physical[i];
 if(elapsed<1000)return 0;
 if(bit==SPF_EXT && ++a->samples<3)return 0;
 done_counts[i]++;return 1;
}
static uint32_t stop(void *unused,uint32_t bit){(void)unused;(void)bit;stop_calls++;return 1;}
static uint32_t clk(void *unused){(void)unused;return tick;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *c,uint32_t verified,uint32_t driving){
 SPF_GroupIO io={0,prep,start,poll,stop,clk};(void)driving;
 if(owns || busy || !ready)return 0;
 assert(SPF_GroupBegin(&vsp_group,c,&io,verified,90,12000));
 memset(vsp_axis,0,sizeof vsp_axis);owns=busy=1;g_source_parallel_state=1;return 1;
}
#include "stage2_fast_cycle.inc"
static uint32_t placed(void *unused,uint32_t run,uint32_t slot,uint32_t now){
 (void)unused;(void)now;assert(run==42 && slot==g_stage2_fast_item+1);return evidence;
}
static uint32_t route_begin(void *unused,const SPF_Command *c,uint32_t now){
 (void)unused;assert(c->goal.dm_deg==0 && c->route_stage==3);
 assert(g_stage2_fast_loaded_mask==7 && g_stage2_fast_state==S2F_DEPART);
 assert(owns && vsp_group.started==0);route_before_stow=1;route_calls++;route_started_ms=now;
 g_chassis_auto_active=1;return route_failure?2:1;
}
static uint32_t route_poll(void *unused,const SPF_Command *c,uint32_t now){
 (void)unused;(void)c;return now-route_started_ms>=3000?1:0;
}
static uint32_t route_stop(void *unused,uint32_t now){(void)unused;(void)now;stop_calls++;g_chassis_auto_active=0;return 1;}
static S2Fast_Hooks hooks={0,placed,route_begin,route_poll,route_stop};
static S2Fast_Profile profile(void){
 S2Fast_Profile p;memset(&p,0,sizeof p);p.run_id=42;p.pose_version=1;
 p.paths_verified=p.timing_verified=p.driving_stow_verified=1;
 p.pick_deg[0]=180;p.pick_deg[1]=195;p.pick_deg[2]=165;
 p.extension_mm[1]=p.extension_mm[2]=156;p.slot_deg=38.380798f;
 p.pick_pulses=118800;p.safe_pulses=7200;p.close_lead_pulses=800;
 p.close_ms=390;p.release_ms=430;p.carrier_ms=700;
 for(uint32_t i=0;i<3;i++){p.retract_at_pulses[i]=50000;p.turn_at_pulses[i]=20000;}
 return p;
}
static void reset_fixture(void){
 memset(&sf,0,sizeof sf);memset(&vsp_group,0,sizeof vsp_group);memset(vsp_axis,0,sizeof vsp_axis);
 memset(start_count,0,sizeof start_count);memset(prepare_count,0,sizeof prepare_count);
 memset(started_ms,0,sizeof started_ms);memset(done_counts,0,sizeof done_counts);
 memset(physical,0,sizeof physical);g_lift_ruler_controller.top_scale=100;
 physical[2]=g_vehicle_lift_current_deg=7200*360.0f/3200.0f;
 physical[0]=180;g_startup_extension_current_deg=0;
 owns=busy=tick=physical_fault=g_source_parallel_cause=g_source_parallel_state=0;
 g_stage2_fast_state=g_stage2_fast_fault=g_stage2_fast_loaded_mask=0;
 g_chassis_auto_active=g_it_request=g_it_state=0;g_lift_ruler_request=0;
 servo_writes=close_writes=route_calls=stop_calls=route_failure=route_before_stow=0;
 ready=evidence=vm=1;g_safety_ready=1;
}
static void service(void){
 tick+=20;
 if(owns){
  SPF_GroupService(&vsp_group);g_source_parallel_state=vsp_group.state;
  g_source_parallel_cause=vsp_group.fault;g_source_parallel_stop_mask=vsp_group.stop_mask;
  if(vsp_group.state==3 || (vsp_group.state==4 && !vsp_group.stop_mask))owns=busy=0;
 }
 Stage2Fast_Service(tick);
}
static void group_gate_test(void){
 SPF_Command c={0};SPF_GroupIO io={0,prep,start,poll,stop,clk};
 c.axes_mask=7;c.goal.dm_deg=30;c.goal.extension_mm=0;c.goal.lift_pulses=7200;
 c.kind=SPF_CMD_TRANSFER;c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=SPF_EXT_RPM;
 c.lift_rpm=SPF_LIFT_RPM;c.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c.keep_vm_on=1;
 assert(SPF_GroupBegin(&vsp_group,&c,&io,1,90,12000));
 vsp_group.delayed_mask=3;vsp_group.release_mask=4;
 for(uint32_t i=0;i<70;i++){tick+=20;SPF_GroupService(&vsp_group);}
 assert(vsp_group.done==4 && prepare_count[0]==0 && prepare_count[1]==0);
 vsp_group.release_mask|=2;
 for(uint32_t i=0;i<70;i++){tick+=20;SPF_GroupService(&vsp_group);}
 assert(vsp_group.done==6 && prepare_count[0]==0);
 vsp_group.release_mask|=1;
 for(uint32_t i=0;i<70;i++){tick+=20;SPF_GroupService(&vsp_group);}
 assert(vsp_group.state==3 && vsp_group.done==7);
}
int main(void){
 S2Fast_Profile p;float depth;
 reset_fixture();group_gate_test();
 reset_fixture();p=profile();p.paths_verified=0;
 assert(!Stage2Fast_Start(&p,&hooks,tick) && !servo_writes && !owns);
 p=profile();p.close_ms=0;assert(!Stage2Fast_Start(&p,&hooks,tick));
 p=profile();p.turn_at_pulses[1]=0;assert(!Stage2Fast_Start(&p,&hooks,tick));
 p=profile();assert(Stage2Fast_Start(&p,&hooks,tick));
 assert(!Stage2Fast_Start(&p,&hooks,tick));
 vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=1;tick=500;
 assert(!SF_LiftDepth(tick,&depth));
 reset_fixture();p=profile();assert(Stage2Fast_Start(&p,&hooks,tick));
 for(uint32_t i=0;i<2000 && g_stage2_fast_state!=S2F_DONE;i++)service();
 assert(g_stage2_fast_state==S2F_DONE && !g_stage2_fast_fault);
 assert(g_stage2_fast_loaded_mask==7 && close_writes==3 && route_calls==1 && route_before_stow);
 assert(g_stage2_fast_release_ms==g_stage2_fast_departure_ms && vm==1);
 assert(physical[0]==0 && start_count[0]==7); /* 3 aim,3 slot,zero */
 assert(!Stage2Fast_Start(&p,&hooks,tick)); /* same run cannot repeat */
 reset_fixture();p=profile();evidence=0;assert(Stage2Fast_Start(&p,&hooks,tick));
 for(uint32_t i=0;i<700 && g_stage2_fast_state!=S2F_RELEASE;i++)service();
 assert(g_stage2_fast_state==S2F_RELEASE && !g_stage2_fast_loaded_mask && !route_calls);
 Stage2Fast_Cancel(22);for(uint32_t i=0;i<20;i++)service();assert(!Stage2Fast_IsActive() && vm==1);
 reset_fixture();p=profile();route_failure=1;assert(Stage2Fast_Start(&p,&hooks,tick));
 for(uint32_t i=0;i<2000 && g_stage2_fast_state!=S2F_FAULT;i++)service();
 for(uint32_t i=0;i<20;i++)service();
 assert(g_stage2_fast_state==S2F_FAULT && !owns && stop_calls && vm==1 && !g_chassis_auto_active);
 reset_fixture();p=profile();assert(Stage2Fast_Start(&p,&hooks,tick));service();physical_fault=1;
 for(uint32_t i=0;i<20;i++)service();
 assert(g_stage2_fast_state==S2F_FAULT && !owns && vm==1);
 reset_fixture();p=profile();tick=0xfffff000U;assert(Stage2Fast_Start(&p,&hooks,tick));
 for(uint32_t i=0;i<2000 && g_stage2_fast_state!=S2F_DONE;i++)service();
 assert(g_stage2_fast_state==S2F_DONE && route_calls==1 && g_stage2_fast_departure_ms<20000U);
 puts("PASS: actual scheduler/group C: gated release, stale feedback, three items, no fake placement, same-service route+zero, cancellation and partial-route failure. SIMULATED I/O ONLY.");
 return 0;
}
'''
src = OUT / 'stage2-fast-host-test.c'
exe = OUT / 'stage2-fast-host-test.exe'
src.write_text(unit, encoding='utf-8')
result = subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-misleading-indentation', '-I', str(P),
                         str(src), str(P / 'source_parallel_flow.c'), '-lm', '-o', str(exe)],
                        capture_output=True, text=True)
log = result.stdout + result.stderr
if result.returncode == 0:
    result = subprocess.run([str(exe)], capture_output=True, text=True)
    log += result.stdout + result.stderr
(E / 'stage2-fast-host-check-20261109.log').write_text(log, encoding='utf-8')
print(log)
assert result.returncode == 0
