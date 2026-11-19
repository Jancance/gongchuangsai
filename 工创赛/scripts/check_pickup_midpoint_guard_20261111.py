"""Actual midpoint adapter/coordinator and native VSP tests with mocked devices."""
from pathlib import Path
import json
import shutil
import subprocess

R = Path(__file__).resolve().parents[1]
U = R / 'tmp/pickup_midpoint_guard_20261111/project/CHASSIS/USER'
E = R / 'evidence/integration/2026-11-11/midpoint-guard'


def run(name, text, extra=()):
    src, exe = E / (name+'.c'), E / (name+'.exe')
    src.write_text(text, encoding='utf-8')
    result = subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-unused-function','-Wno-unused-variable','-I',str(U),str(src),*map(str,extra),'-lm','-o',str(exe)], capture_output=True, text=True)
    log = result.stdout + result.stderr
    if result.returncode == 0:
        result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=15)
        log += result.stdout + result.stderr
    (E / (name+'.log')).write_text(log, encoding='utf-8')
    print(log, flush=True)
    assert result.returncode == 0, name
    return log.strip()


vsp = (U/'vehicle_source_parallel.inc').read_text(encoding='utf-8')
axis = vsp[vsp.index('typedef struct {uint32_t phase'):vsp.index('static VSP_Axis vsp_axis')]
it = (U/'integration10_complete.inc').read_text(encoding='utf-8')
half = it[it.index('static void it_half_extension_gate'):it.index('static void it_empty_overlap_publish')]
grip = it[it.index('volatile uint32_t g_it_lead5_grip_tick'):it.index('#include "integration13_immediate.inc"')]
unit = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#define APP_MOTOR_OFFLINE_ONLY 0
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u: %s\n",__LINE__,#x);exit(1);}}while(0)
volatile uint32_t g_pick_trial_profile=2,g_pick_trial_requested,g_pick_trial_scurve;
static uint32_t checks,tick,owned=1,ext_delay=15,ext_left,fail_ext,fail_prepare,fail_final_start;
static uint32_t starts[8],stops,grips,final_after_ext,stop_during_wait;
__AXIS__
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;static SPF_Command it_command;
static uint32_t g_it_state=1,g_it_action=14,g_it_lead5_grip_done;
static uint32_t g_source_parallel_started_mask,g_it_half_extension_released,g_it_half_extension_release_tick;
static float g_it_half_lift_before,g_it_half_lift_goal,g_it_half_lift_at_release;
static uint32_t xTaskGetTickCount(void){return tick;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static void it_fail(uint32_t why){g_it_state=9;SPF_GroupCancel(&vsp_group,why);}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t jiazhua(float angle){CK(angle==140);grips++;return 1;}
#include "pickup_midpoint_guard.inc"
__HALF__
__GRIP__
static uint32_t VSP_Prepare(void *ctx,uint32_t bit,const SPF_Command *command){
 (void)ctx;VSP_Axis *a=&vsp_axis[bit==SPF_EXT?0:1];
 if(bit==SPF_LIFT && fail_prepare && g_pick_midpoint_stage==PMG_PREP_FINAL)return 2;
 a->before=a->current;a->position_cycle++;a->position_tick=tick;
 a->goal=bit==SPF_LIFT?PMG_LiftGoal(a,command->goal.lift_pulses*360.0f/3200.0f):-command->goal.extension_mm*35.49375f;
 return 1;
}
static uint32_t VSP_Start(void *ctx,uint32_t bit,const SPF_Command *command){
 (void)ctx;CK(command->extension_rpm==200 && command->lift_rpm==2500 && command->stepper_accel_rpm_s==600);
 starts[bit]++;
 if(bit==SPF_EXT){CK(g_pick_midpoint_stage==PMG_WAIT_EXT);CK(fabsf(vsp_axis[1].current-g_pick_midpoint_hold_deg)<=45);ext_left=ext_delay;}
 if(bit==SPF_LIFT && g_pick_midpoint_stage==PMG_PREP_FINAL){CK(vsp_group.done&SPF_EXT);final_after_ext++;if(fail_final_start)return 0;}
 return 1;
}
static uint32_t poll(void *ctx,uint32_t bit){
 (void)ctx;VSP_Axis *a=&vsp_axis[bit==SPF_EXT?0:1];
 if(bit==SPF_EXT){
  if(fail_ext)return 2;
  if(ext_left){ext_left--;return 0;}
  a->current=a->goal;a->flags=3;a->samples=3;a->position_tick=tick;a->position_cycle++;return 1;
 }
 if(vsp_pick_gate_enabled && g_pick_midpoint_stage==PMG_PREP_FINAL)return 0;
 float distance=a->goal-a->current;
 if(distance>400)a->current+=400;else if(distance< -400)a->current-=400;else a->current=a->goal;
 a->position_tick=tick;a->position_cycle++;a->flags=a->current==a->goal?3:1;
 if(a->current==a->goal)a->samples++;else a->samples=0;
 return PMG_LiftCompletion(a->samples>=3);
}
static uint32_t stop(void *ctx,uint32_t bit){(void)ctx;stops|=bit;return 1;}
static uint32_t clock_ms(void *ctx){(void)ctx;return tick;}
static void setup(float mm){
 memset(vsp_axis,0,sizeof vsp_axis);memset(&vsp_group,0,sizeof vsp_group);memset(starts,0,sizeof starts);memset(&it_command,0,sizeof it_command);
 PMG_Reset();tick=100;stops=grips=final_after_ext=0;fail_ext=fail_prepare=fail_final_start=0;ext_delay=15;
 g_it_state=1;g_it_action=14;g_it_lead5_grip_done=g_it_half_extension_released=0;
 vsp_axis[1].current=810;it_command.sequence=7;it_command.kind=SPF_CMD_PICK;
 it_command.axes_mask=SPF_LIFT|(mm>0?SPF_EXT:0);it_command.goal.lift_pulses=118800;it_command.goal.extension_mm=mm;
 it_command.dm_deg_s=300;it_command.extension_rpm=200;it_command.lift_rpm=2500;it_command.stepper_accel_rpm_s=600;it_command.keep_vm_on=1;
 SPF_GroupIO io={0,VSP_Prepare,VSP_Start,poll,stop,clock_ms};CK(SPF_GroupBegin(&vsp_group,&it_command,&io,1,90,12000));
 vsp_pick_gate_enabled=mm>0;if(mm>0){vsp_group.delayed_mask=SPF_EXT;vsp_group.release_mask=SPF_LIFT;}
}
static void step(void){
 tick+=10;if(!PMG_Service())SPF_GroupCancel(&vsp_group,7);
 SPF_GroupService(&vsp_group);g_source_parallel_started_mask=vsp_group.started;
 it_half_extension_gate(tick);it_lead5_grip_service(tick);
 if(vsp_pick_gate_enabled && g_pick_midpoint_stage!=PMG_TO_FINAL)CK(!(vsp_group.done&SPF_LIFT));
}
int main(void){
 for(uint32_t side=0;side<2;side++){
  setup(side?160:156);for(uint32_t i=0;i<500 && vsp_group.state!=3;i++)step();
  CK(vsp_group.state==3 && vsp_group.done==6 && starts[2]==1 && starts[4]==2 && final_after_ext==1);
  CK(grips==1 && g_it_lead5_grip_done && fabsf(vsp_axis[1].current-13365)<.01);
  CK(fabsf(g_it_half_lift_before-810)<.01 && fabsf(g_it_half_lift_goal-13365)<.01 && fabsf(g_it_half_lift_at_release-7087.5f)<.01);
 }
 setup(0);for(uint32_t i=0;i<100 && vsp_group.state!=3;i++)step();CK(vsp_group.state==3 && starts[2]==0 && starts[4]==1 && final_after_ext==0);
 setup(156);ext_delay=100000;for(uint32_t i=0;i<1300 && vsp_group.state!=4;i++)step();
 CK(vsp_group.state==4 && vsp_group.fault==3 && starts[4]==1 && final_after_ext==0 && grips==0);step();CK(stops==6 && !vsp_group.stop_mask);
 setup(156);fail_ext=1;for(uint32_t i=0;i<200 && vsp_group.state!=4;i++)step();CK(vsp_group.state==4 && starts[4]==1 && !grips);step();CK(stops==6);
 setup(156);fail_prepare=1;for(uint32_t i=0;i<200 && vsp_group.state!=4;i++)step();CK(vsp_group.state==4 && final_after_ext==0 && !grips);step();CK(stops==6);
 setup(156);fail_final_start=1;for(uint32_t i=0;i<200 && vsp_group.state!=4;i++)step();CK(vsp_group.state==4 && starts[4]==2 && !grips);step();CK(stops==6);
 setup(156);for(uint32_t i=0;i<200 && g_pick_midpoint_stage!=PMG_WAIT_EXT;i++)step();SPF_GroupCancel(&vsp_group,2);uint32_t before=starts[4];step();CK(starts[4]==before && stops==6 && !grips);
 setup(156);vsp_axis[1].before=INFINITY;CK(isnan(PMG_LiftGoal(&vsp_axis[1],13365)) && g_pick_midpoint_stage==PMG_OFF);
 printf("PASS %u actual midpoint/SPF/half-release/early-grip checks. Mocked axis I/O; not a physical timing model.\n",checks);return 0;
}
'''.replace('__AXIS__',axis).replace('__HALF__',half).replace('__GRIP__',grip)
policy_log = run('midpoint-coordinator',unit,[U/'source_parallel_flow.c'])

# Run actual native VSP/ACK/window checks and add both lift-leg frame checks.
source = (R/'scripts/check_pickup_second_diag_20261110.py').read_text(encoding='utf-8')
source = source.replace('pickup_second_diag_20261110','pickup_midpoint_guard_20261111').replace('2026-11-10/second-diag','2026-11-11/midpoint-guard')
source = source.replace('e[4]={106,200,300,400}','e[4]={106,200,200,400}').replace('dec[4]={600,400,1200,2000}','dec[4]={600,400,600,2000}')
marker=" unit=unit.replace('static struct {uint8_t Data[8];} s_rx;'"
source=source.replace(marker," unit=unit.replace('static uint32_t g_it_carrier_auto_done;', 'static uint32_t g_it_half_extension_released,g_it_half_extension_release_tick;\\nstatic float g_it_half_lift_before,g_it_half_lift_goal,g_it_half_lift_at_release;\\nstatic uint32_t g_it_carrier_auto_done;',1)\n"+marker,1)
marker=" logs['port']=run('port-frames',unit)"
native = r'''
 g_pick_trial_profile=2;g_pick_trial_scurve=0;PMG_Reset();vsp_pick_gate_enabled=1;vsp_fast_lift=1;
 memset(vsp_axis,0,sizeof vsp_axis);options=132;raw=81000;sign=0;mock_speed=mock_speed_miss=mock_speed_sign=0;mock_flags=3;
 g_lift_ruler_controller.top_scale=100;c.goal.lift_pulses=118800;c.goal.extension_mm=156;c.lift_rpm=2500;c.extension_rpm=200;c.stepper_accel_rpm_s=600;
 assert(!VSP_Prepare(0,SPF_LIFT,&c));assert(!VSP_Prepare(0,SPF_LIFT,&c));
 assert(g_pick_midpoint_stage==PMG_TO_MID && fabsf(vsp_axis[1].goal-7087.5f)<.01f);
 assert(u32(vsp_axis[1].move+8)==627750 && u16(vsp_axis[1].move+6)==25000);
 assert(u16(vsp_axis[1].move+2)==45055 && u16(vsp_axis[1].move+4)==45055);
 vsp_axis[1].current=7087.5f;vsp_axis[1].samples=3;vsp_axis[1].flags=3;vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=tick;
 assert(!PMG_LiftCompletion(1) && g_pick_midpoint_stage==PMG_WAIT_EXT);
 vsp_group.state=2;vsp_group.command=c;vsp_group.command.kind=SPF_CMD_PICK;vsp_group.done=0;
 assert(PMG_Service() && g_pick_midpoint_stage==PMG_WAIT_EXT);
 vsp_group.done=SPF_EXT;vsp_axis[1].position_tick=tick-151;assert(PMG_Service() && g_pick_midpoint_stage==PMG_WAIT_EXT);
 vsp_axis[1].position_tick=tick;vsp_axis[1].flags=9;assert(!PMG_Service());vsp_axis[1].flags=3;
 raw=708750;mock_move_result=1;mock_ack=2;
 assert(PMG_Service() && g_pick_midpoint_stage==PMG_PREP_FINAL);
 uint32_t held_phase=vsp_axis[1].phase;assert(!VSP_Poll(0,SPF_LIFT) && vsp_axis[1].phase==held_phase);
 for(uint32_t i=0;i<8 && g_pick_midpoint_stage!=PMG_TO_FINAL;i++)assert(PMG_Service());
 assert(g_pick_midpoint_stage==PMG_TO_FINAL && fabsf(vsp_axis[1].before-7087.5f)<.01f && fabsf(vsp_axis[1].goal-13365)<.01f);
 assert(u32(vsp_axis[1].move+8)==627750 && u16(vsp_axis[1].move+6)==25000 && u16(vsp_axis[1].move+2)==45055);
 raw=1336500;vsp_axis[1].samples=0;mock_flags=3;
 for(uint32_t i=0;i<3;i++){assert(!VSP_Poll(0,SPF_LIFT));assert(!VSP_Poll(0,SPF_LIFT));assert(VSP_Poll(0,SPF_LIFT)==(i==2));}
 PMG_Reset();vsp_pick_gate_enabled=1;memset(vsp_axis,0,sizeof vsp_axis);raw=81000;
 assert(!VSP_Prepare(0,SPF_LIFT,&c));assert(!VSP_Prepare(0,SPF_LIFT,&c));
 raw=714850;vsp_axis[1].phase=1;assert(VSP_Poll(0,SPF_LIFT)==2); /* midpoint drift beyond original60deg still faults */
 PMG_Reset();
'''
injection=" unit=unit.replace('#include \\\"extension_readonly_diag.inc\\\"', 'static uint32_t CAN1_ExtensionReadOnlyBlock(uint8_t cmd,uint8_t data[64],uint32_t *length){(void)cmd;(void)data;if(length)*length=0;return 9U;}\\n#include \\\"extension_readonly_diag.inc\\\"',1)\n"
injection+=" unit=unit.replace(' puts(\"PASS: actual VSP',"+repr(native+"\n puts(\"PASS: actual VSP")+")\n"
injection+=" logs['port']=run('port-frames',unit,[U/'source_parallel_flow.c'])"
assert source.count(marker)==1;source=source.replace(marker,injection,1)
if not (E/'timing-model.json').exists():shutil.copy2(R/'evidence/integration/2026-11-10/half-descent/timing-model.json',E/'timing-model.json')
exec(compile(source,'actual-midpoint-native-VSP','exec'),{'__file__':str(__file__),'__name__':'__main__'})
(E/'midpoint-guard-checks.json').write_text(json.dumps(dict(passed=True,actual_midpoint_coordinator=True,actual_native_vsp_frames=True,
    policy_result=policy_log,extension_rpm=200,extension_accel_decel=600,lift_rpm=2500,lift_accel_decel=45055,
    one_extension_stroke=True,midpoint_not_final_done=True,final_descent_requires_extension_done=True,
    stop_timeout_guards_retained=True,hardware_access=False,physical_stall_resolved=False),indent=2)+'\n',encoding='utf-8')
