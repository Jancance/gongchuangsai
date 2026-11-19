"""Actual half gate/entry/owner + SPF coordinator checks; mocked axis I/O only."""
from pathlib import Path
import json
import shutil
import subprocess

R=Path(__file__).resolve().parents[1]
P=R/'tmp/pickup_half_descent_20261110/project'
U=P/'CHASSIS/USER'
E=R/'evidence/integration/2026-11-10/half-descent'
E.mkdir(parents=True,exist_ok=True)


def function(source,name):
    pos=source.index(name+'(');start=source.rfind('\n',0,pos)+1
    brace=source.index('{',pos);depth=1;end=brace+1
    while depth:
        if source[end]=='{':depth+=1
        if source[end]=='}':depth-=1
        end+=1
    return source[start:end]+'\n'


def run(name,unit,extras=()):
    src=E/(name+'.c');exe=E/(name+'.exe');src.write_text(unit,encoding='utf-8')
    args=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-unused-function','-Wno-unused-variable',
          '-I',str(U),str(src),*map(str,extras),'-lm','-o',str(exe)]
    p=subprocess.run(args,capture_output=True,text=True);log=p.stdout+p.stderr
    if p.returncode==0:
        p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=p.stdout+p.stderr
    (E/(name+'.log')).write_text(log,encoding='utf-8');print(log,flush=True)
    assert p.returncode==0,name
    return log


text=(U/'integration10_complete.inc').read_bytes().decode('utf-8',errors='replace').replace('\r\n','\n')
vsp=(U/'vehicle_source_parallel.inc').read_bytes().decode('utf-8',errors='replace')
axis=vsp[vsp.index('typedef struct {uint32_t phase'):vsp.index('static VSP_Axis')]
case=text[text.index(' case 14U:\n'):text.index(' case 5U:case 6U:',text.index(' case 14U:\n'))]
selected='\n'.join(function(text,n) for n in ('it_empty_overlap_begin','it_half_extension_gate','it_grip_extension_ready','it_lead5_grip_service'))
unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#define APP_MOTOR_OFFLINE_ONLY 0
#define VEHICLE_EXTENSION_PULSES_PER_MM 315.5f
volatile uint32_t g_pick_trial_profile=2,g_pick_trial_requested,g_pick_trial_scurve;
static uint32_t checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u: %s\n",__LINE__,#x);exit(1);}}while(0)
__AXIS__
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;static SPF_Command it_command;
static uint32_t tick,vsp_owned,vsp_fast_lift;
static uint32_t g_it_state,g_it_action,g_it_request,g_it_rejected,g_it_cause,g_it_sequence,g_it_axes;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_it_carrier_auto_done,it_carrier_pwm_tick;
static uint32_t g_it_lead5_grip_done,g_it_half_extension_released,g_it_half_extension_release_tick;
static uint32_t g_it_lead5_grip_tick,g_source_parallel_started_mask,g_source_parallel_done_mask,g_source_parallel_state;
static float g_it_half_lift_before,g_it_half_lift_goal,g_it_half_lift_at_release;
static float g_it_lead5_grip_depth,g_it_lead5_grip_x_error;
static float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses,g_it_carrier_auto_deg,it_carrier_latched_deg;
static float g_startup_extension_current_deg,g_extension_home_reference_deg,g_dm_can2_logical_position_deg;
static float f_before,f_goal,f_position,f_ext_position;
static uint32_t f_age,f_lift_done,f_ext_done,f_ext_ready,f_fault_mask,prepares[8],starts[8],stops[8],start_tick[8];
static uint32_t full_grips,publishes;
static uint32_t xTaskGetTickCount(void){return tick;}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t it_platform(void){return 1;}
static uint32_t it_at_top(void){return 1;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t it_dm_fresh(void){return 1;}
static uint32_t VehicleArm_Busy(void){return vsp_owned;}
static uint32_t VehicleSourceParallel_Owns(void){return vsp_owned;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t Stage2Fast_IsActive(void){return 0;}
static void VehicleSourceParallel_Cancel(void){SPF_GroupCancel(&vsp_group,2);}
static void it_fail(uint32_t why){g_it_cause=why;g_it_state=9;SPF_GroupCancel(&vsp_group,why);}
static uint32_t jiazhua(float deg){if(deg==140)full_grips++;return 1;}
static uint32_t prep(void *ctx,uint32_t bit,const SPF_Command *c){
 (void)ctx;prepares[bit]++;VSP_Axis *a=&vsp_axis[bit==SPF_EXT?0:1];
 if(bit==SPF_LIFT){a->before=f_before;a->goal=f_goal;a->current=f_before;}
 else{a->before=f_ext_position;a->goal=-c->goal.extension_mm*315.5f*360/3200;a->current=f_ext_position;}
 a->position_cycle=1;a->position_tick=tick;return 1;
}
static uint32_t start(void *ctx,uint32_t bit,const SPF_Command *c){(void)ctx;(void)c;starts[bit]++;start_tick[bit]=tick;return 1;}
static uint32_t poll(void *ctx,uint32_t bit){
 (void)ctx;if(f_fault_mask&bit)return 2;
 if(bit==SPF_LIFT){vsp_axis[1].current=f_position;vsp_axis[1].position_cycle++;vsp_axis[1].position_tick=tick-f_age;return f_lift_done;}
 vsp_axis[0].current=f_ext_position;vsp_axis[0].position_cycle++;vsp_axis[0].position_tick=tick;
 vsp_axis[0].samples=f_ext_ready;vsp_axis[0].flags=f_ext_ready?3:1;return f_ext_done;
}
static uint32_t stop(void *ctx,uint32_t bit){(void)ctx;stops[bit]++;return 1;}
static uint32_t clock_ms(void *ctx){(void)ctx;return tick;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *c,uint32_t sweep,uint32_t stow){
 SPF_GroupIO io={0,prep,start,poll,stop,clock_ms};(void)stow;
 if(!SPF_GroupBegin(&vsp_group,c,&io,sweep,90,12000))return 0;
 memset(vsp_axis,0,sizeof vsp_axis);vsp_owned=1;return 1;
}
__FUNCTIONS__
static void it_empty_overlap_publish(void){publishes++;}
static void it_done(void){g_it_state=2;}
static void complete_approach(void){uint32_t now=tick;switch(g_it_action){__CASE__ default:break;}}
#include "integration13_immediate.inc"
static void step(void){
 tick+=10;SPF_GroupService(&vsp_group);
 g_source_parallel_state=vsp_group.state;g_source_parallel_started_mask=vsp_group.started;g_source_parallel_done_mask=vsp_group.done;
 if(vsp_group.state==3 || (vsp_group.state==4 && !vsp_group.stop_mask))vsp_owned=0;
 it_half_extension_gate(tick);it_lead5_grip_service(tick);complete_approach();it_auto_service(tick);
}
static void reset(float before,float extension){
 memset(&vsp_group,0,sizeof vsp_group);memset(vsp_axis,0,sizeof vsp_axis);memset(prepares,0,sizeof prepares);memset(starts,0,sizeof starts);memset(stops,0,sizeof stops);
 tick=100;vsp_owned=0;f_before=before;f_goal=118800.0f*360/3200;f_position=before;f_ext_position=0;
 f_age=f_lift_done=f_ext_done=f_ext_ready=f_fault_mask=full_grips=publishes=0;
 g_it_state=g_it_action=g_it_request=g_it_rejected=g_it_cause=g_it_sequence=0;
 g_lift_ruler_request=g_lift_ruler_stop_pending=0;
 g_startup_extension_current_deg=g_extension_home_reference_deg=0;
 g_it_dm_deg=g_dm_can2_logical_position_deg=extension==0?180:(extension==156?195:165);
 g_it_extension_mm=extension;g_it_lift_pulses=118800;g_it_carrier_auto_deg=-1;
 CK(it_empty_overlap_begin());g_it_state=1;g_it_action=14;
 g_it_auto_state=1;g_it_auto_phase=2;g_it_auto_item=1;g_it_auto_fault=0;it_auto_sequence=0;it_auto_begin=it_auto_phase_begin=tick;
}
int main(void){
 CK(PickupTrial_LiftRpm()==2500 && PickupTrial_ExtRpm()==200 && PickupTrial_ExtAccel()==600 && PickupTrial_ExtDecel()==600);
 for(uint32_t k=0;k<2;k++){
  reset(k?810.0f:270.0f,k?160.0f:156.0f);float half=f_before+(f_goal-f_before)*.5f;
  CK(vsp_group.delayed_mask==SPF_EXT && vsp_group.release_mask==SPF_LIFT);step();step();
  CK(starts[SPF_LIFT]==1 && !prepares[SPF_EXT] && !starts[SPF_EXT]);
  f_position=half-.01f;step();CK(!g_it_half_extension_released && !prepares[SPF_EXT]);
  f_position=half;f_age=151;step();CK(!g_it_half_extension_released);
  f_age=0;f_ext_position=1.25f;tick+=500;step();
  CK(g_it_half_extension_released && !prepares[SPF_EXT] && g_it_half_lift_before==f_before && g_it_half_lift_goal==f_goal && g_it_half_lift_at_release==half);
  uint32_t released=g_it_half_extension_release_tick;step();CK(prepares[SPF_EXT]==1 && vsp_axis[0].before==1.25f);
  step();CK(starts[SPF_EXT]==1 && start_tick[SPF_EXT]>=released && start_tick[SPF_EXT]-start_tick[SPF_LIFT]>90 && vsp_group.state==2);
  f_position=f_goal;f_lift_done=1;for(uint32_t i=0;i<4;i++)step();
  CK(vsp_group.done==SPF_LIFT && g_it_state==1 && !full_grips && !publishes && g_it_request!=15);
  f_ext_position=vsp_axis[0].goal;f_ext_ready=1;step();CK(full_grips==1 && g_it_state==1 && !publishes && g_it_request!=15);
  f_ext_ready=3;f_ext_done=1;step();CK(vsp_group.state==3 && vsp_group.done==6 && publishes==1 && g_it_request==15 && g_it_auto_phase==3);
 }
 reset(810,0);CK(!vsp_group.delayed_mask);step();step();f_position=f_goal;f_lift_done=1;step();
 CK(!prepares[SPF_EXT] && !starts[SPF_EXT] && vsp_group.done==SPF_LIFT && g_it_request==15);
 reset(810,156);step();step();f_position=f_goal;f_lift_done=1;f_age=500;step();
 CK(g_it_half_extension_released); /* current-group completed lift cannot deadlock delayed extension */
 reset(810,156);step();step();tick=12100;step();CK(vsp_group.state==4 && !prepares[SPF_EXT] && !starts[SPF_EXT]);step();CK(stops[2] && stops[4]);
 reset(810,156);step();step();f_fault_mask=SPF_LIFT;step();CK(vsp_group.state==4 && !g_it_half_extension_released && !starts[2]);step();CK(stops[2] && stops[4]);
 for(uint32_t bad=0;bad<5;bad++){
  reset(810,156);step();step();vsp_axis[1].position_tick=tick;
  if(bad==0)vsp_axis[1].before=NAN;
  if(bad==1)vsp_axis[1].goal=INFINITY;
  if(bad==2)vsp_axis[1].current=NAN;
  if(bad==3)vsp_axis[1].goal=vsp_axis[1].before;
  if(bad==4)vsp_axis[1].goal=vsp_axis[1].before-100;
  it_half_extension_gate(tick);CK(g_it_state==9 && vsp_group.state==4 && !g_it_half_extension_released && !starts[2]);
 }
 reset(810,156);step();step();vsp_axis[1].current=f_goal;vsp_group.command.sequence++;
 it_half_extension_gate(tick);CK(!g_it_half_extension_released);vsp_group.command.sequence--;
 vsp_axis[1].position_cycle=0;it_half_extension_gate(tick);CK(!g_it_half_extension_released);
 printf("PASS %u actual half-entry/gate/SPF/owner checks; mocked axis I/O only.\n",checks);return 0;
}
'''.replace('__AXIS__',axis).replace('__FUNCTIONS__',selected).replace('__CASE__',case)
gate_log=run('half-gate-coordinator',unit,[U/'source_parallel_flow.c'])

# Reuse the existing actual VSP/frame checks, rebinding only paths and the
# newly requested second profile. The MATLAB fixture here exercises the
# unchanged lift S-reference implementation, not a fresh cycle time estimate.
source=(R/'scripts/check_pickup_second_diag_20261110.py').read_text(encoding='utf-8')
source=source.replace('pickup_second_diag_20261110','pickup_half_descent_20261110').replace('2026-11-10/second-diag','2026-11-10/half-descent')
source=source.replace('e[4]={106,200,300,400}','e[4]={106,200,200,400}').replace('dec[4]={600,400,1200,2000}','dec[4]={600,400,600,2000}')
marker=" unit=unit.replace('static struct {uint8_t Data[8];} s_rx;'"
insertion=" unit=unit.replace('static uint32_t g_it_carrier_auto_done;', 'static uint32_t g_it_half_extension_released,g_it_half_extension_release_tick;\\nstatic float g_it_half_lift_before,g_it_half_lift_goal,g_it_half_lift_at_release;\\nstatic uint32_t g_it_carrier_auto_done;',1)\n"
assert source.count(marker)==1;source=source.replace(marker,insertion+marker,1)
marker=" logs['port']=run('port-frames',unit)"
insertion=" unit=unit.replace('#include \\\"extension_readonly_diag.inc\\\"', 'static uint32_t CAN1_ExtensionReadOnlyBlock(uint8_t cmd,uint8_t data[64],uint32_t *length){(void)cmd;(void)data;if(length)*length=0;return 9U;}\\n#include \\\"extension_readonly_diag.inc\\\"',1)\n"
assert source.count(marker)==1;source=source.replace(marker,insertion+marker,1)
if not (E/'timing-model.json').exists():shutil.copy2(R/'evidence/integration/2026-11-10/second-diag/timing-model.json',E/'timing-model.json')
exec(compile(source,'actual-half-VSP-check','exec'),{'__file__':str(R/'scripts/check_pickup_half_descent_20261110.py'),'__name__':'__main__'})
(E/'half-descent-checks.json').write_text(json.dumps({
    'passed':True,'actual_gate_entry_coordinator_and_owner':True,
    'actual_native_vsp_frame_checks':True,'gate_result':gate_log.strip(),
    'profile2_lift_rpm':2500,'profile2_lift_accel_decel':45055,
    'profile2_extension_rpm':200,'profile2_extension_accel_decel':600,
    'half_current_stroke_not_fixed_depth':True,'extension_prepare_only_after_release':True,
    'lift_done_extension_pending_no_return':True,'delayed_skew_not_synchronous_fault':True,
    'original_group_timeout_and_stop_retained':True,'hardware_access':False,
    'physical_overshoot_resolved':False,
},indent=2)+'\n',encoding='utf-8')
