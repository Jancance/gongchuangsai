"""Actual piece owner + cycle + queued VSP, with simulated drivers only."""
import json,shutil,subprocess
from pathlib import Path
from prepare_three_frame_warm_20261118 import R,P,E
from check_stage2_extension_mirror_20261117 import function


def run(name,source,extra=()):
    path=E/(name+'.c');exe=path.with_suffix('.exe');path.write_text(source,encoding='utf-8')
    args=['gcc','-std=c99','-O1','-Wall','-Wextra','-Wno-unused-function','-Wno-unused-variable','-Wno-misleading-indentation','-I'+str(E),'-I'+str(P/'CHASSIS/USER'),'-I'+str(P/'USER'),str(path),*[str(x) for x in extra],'-lm','-o',str(exe)]
    b=subprocess.run(args,capture_output=True,text=True);(E/(name+'-build.log')).write_text(b.stdout+b.stderr);assert b.returncode==0,b.stderr
    r=subprocess.run([str(exe)],cwd=E,capture_output=True,text=True);(E/(name+'.log')).write_text(r.stdout+r.stderr);assert r.returncode==0,r.stdout+r.stderr
    print(r.stdout.strip(),flush=True);return r.stdout.strip()


def main():
    old=R/'evidence/integration/2026-11-17/stage2-sync-opt'
    shutil.copy2(old/'can.h',E/'can.h')
    owner=(old/'all-regions-owner.c').read_text(encoding='utf-8')
    owner=owner.replace('-extension*VEHICLE_EXTENSION_PULSES_PER_MM','STARTUP_EXTENSION_EXTEND_SIGN*extension*VEHICLE_EXTENSION_PULSES_PER_MM')
    owner=owner.replace('vsp_group.command.goal.lift_pulses==29700','vsp_group.command.goal.lift_pulses==Low_CommandPickDepth(region)')
    owner=owner.replace('g_low_actual_rise_mm>65','g_low_actual_rise_mm>65-((float)SOURCE_PICK_PULSES-low_pick_depth)/SOURCE_LIFT_PULSES_PER_MM')
    owner=owner.replace('static void reset(void){','static void reset(void){low_pick_depth=SOURCE_PICK_PULSES;')
    needle='if(g_low_phase==4){CHECK('
    assert needle in owner;owner=owner.replace(needle,'if(g_low_phase==4){CHECK(grips==0);CHECK(',1)
    ownmain=owner.rfind('int main(void){');assert ownmain>=0
    owner=owner[:ownmain]+owner[ownmain:].replace('int main(void){','static int previous_owner_main(void){',1)
    actual=(P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(b'#include "low_readonly_diag.inc"',b'/* Read-only diagnostics are outside this owner fixture. */')
    (E/'tested-commit-owner.inc').write_bytes(actual)
    (E/'base-commit-fixture.c').write_text(owner,encoding='utf-8')
    owner_test=owner+r'''
int main(void){unsigned region,slot;for(region=0;region<3;region++)for(slot=1;slot<=3;slot++){
 piece(region,slot);CHECK(records[0].command.goal.lift_pulses==(region<2?25700:29700));
 CHECK(grips==1 && records[1].phase==0); /* Record stores actual command; phase is a host annotation only. */
 CHECK(records[1].tick-low_begin==g_low_grip_command_ms);
 CHECK(g_low_actual_rise_mm>(region<2?55:65));
}
start_region(0,1);Low_Service(tick);CHECK(!grips && owned);g_low_request=9;Low_Service(tick);CHECK(g_low_state==9 && g_it_state==9);
printf("PASS actual owner %u checks:9 region/slot paths, side25700/bottom29700, no early close, close/return same call, unchanged release100 and STOP\n",checks);return 0;}
'''
    results={'owner':run('actual-owner-new-heights',owner_test)}
    for n in ('Buzzer.h','saoma.h','screen.h'):(E/n).write_text('/* External hardware supplied by this fixture. */\n')
    (E/'tested-source-cycle.inc').write_bytes((P/'CHASSIS/USER/source_mcu_cycle.inc').read_bytes())
    cycle=r'''
#include "base-commit-fixture.c"
#define SOURCE_CYCLE_TEST_215 1
static uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_safety_ready=1,g_can_test_result=1;
static uint32_t g_source_body_step_request,g_source_body_step_state,g_source_body_step_reason,g_it_axes;
static float g_startup_extension_current_deg;
volatile uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_chain_active;
volatile float g_source_align_distance_mm;
static unsigned beep_until,beep_calls,last_beep_count;
static uint8_t USART6_VisionSend(const uint8_t*p,uint16_t n){(void)p;(void)n;return 1;}
static void Buzzer_StartTimed(unsigned count,unsigned on,unsigned gap){CHECK((count==1 || count==2) && on==80 && gap==80);beep_calls++;last_beep_count=count;beep_until=tick+on*count+gap*(count-1);}
static unsigned Buzzer_IsBusy(void){return tick<beep_until;}
#include "tested-source-cycle.inc"
static void sample(unsigned seq,unsigned region,unsigned color,unsigned item){char wire[192];unsigned colors[3]={0},i;int n;
 if(item==0){colors[0]=2;colors[1]=1;colors[2]=5;}else if(item==1){colors[1]=1;colors[2]=5;}else colors[2]=5;
 n=snprintf(wire,sizeof(wire),"OC2,%u,%u,%u,1,0,0,0,1,%u,%u,%u,0,%u,%u,%u,%u,",g_source_cycle_run,g_source_cycle_pose,seq,colors[0],colors[1],colors[2],color,region==0?0:1,region==1?0:1,region==2?0:1);
 snprintf(wire+n,sizeof(wire)-n,"%04X\r\n",SC_Crc(wire,n));for(i=0;i<strlen(wire);i++)CHECK(SourceCycle_SharedByte(wire[i],tick,0));}
static void finish_piece(void){unsigned loops=0;
 while(Low_Busy() && ++loops<300){
  if(g_low_phase==6){if(tick<pulse_apply_at)tick=pulse_apply_at;Low_Service(tick);tick+=100;Low_Service(tick);}
  else if(owned)finish_group();else{tick+=700;Low_Service(tick);}SourceCycle_Service(tick);
 }CHECK(loops<300 && !g_low_fault);
}
int main(void){unsigned i,item;static const unsigned prep[5]={1,2,7,6,3};reset();g_it_state=0;g_source_cycle_request=1;
 SourceCycle_Service(tick);CHECK(g_source_cycle_state==1);
 for(i=0;i<5;i++){CHECK(g_it_request==prep[i]);if(i==4)CHECK(g_it_axes==3 && g_it_dm_deg==180 && g_it_extension_mm==65);g_it_state=2;g_it_request=0;g_it_action=prep[i];tick+=100;SourceCycle_Service(tick);}
 CHECK(g_source_cycle_state==2 && g_source_cycle_prep==5);g_source_cycle_state=5;lock_wheels();SourceCycle_Service(tick);
 CHECK(g_source_cycle_state==6 && Buzzer_IsBusy() && beep_calls==1 && last_beep_count==2);
 for(item=0;item<3;item++){
  sample(1,item,(unsigned[]){2,1,5}[item],item);SourceCycle_Service(tick);CHECK(g_source_cycle_state==7 && g_source_cycle_scheduled);
  SourceCycle_Service(tick);CHECK(Low_Busy() && !g_source_cycle_scheduled);
  if(item==0)CHECK(g_source_cycle_pick_beep_pending==1 && !g_source_cycle_pick_beeps);
  finish_piece();CHECK(g_source_cycle_item==item+1);
 }
 CHECK(g_source_cycle_state==8 && !g_source_cycle_fault && g_low_done_sequence==3);
 CHECK(g_source_cycle_pick_beeps==3 && !g_source_cycle_pick_beep_pending && beep_calls==4);
 CHECK(g_source_cycle_beep_done && g_source_cycle_beep_done>g_source_cycle_beep_started);
 printf("PASS actual cycle %u checks:5-step initial DM|X, first-beep release while busy,3pieces/3single cues, no repeated frame cues\n",checks);return 0;}
'''
    results['cycle']=run('actual-cycle-first-beep',cycle)
    queued=(old/'actual-queued-vsp-group.c').read_text()
    source=(P/'CHASSIS/USER/vehicle_source_parallel.inc').read_text(encoding='utf-8')
    for signature in ('static uint32_t VSP_StartQueuedPick(','static uint32_t VSP_Start(','uint32_t VehicleArm_SourceParallelCanAllowed('):
        a=function(queued,signature);b=function(source,signature);queued=queued.replace(a,b,1)
    queued=queued.replace('CHECK(queued==6 && (vsp_group.prepared&6)==6 && (vsp_group.started&6)==6);','{unsigned m=vsp_group.command.axes_mask&6;CHECK(queued==m && (vsp_group.prepared&m)==m && (vsp_group.started&m)==m);}')
    marker=' printf("PASS actual queued VSP/group:'
    tests=r'''
 for(fail=0;fail<=5;fail++){
  setup(3,0);g_dm_stage2_sync=1;
  for(i=0;i<20 && vsp_group.state<3;i++)SPF_GroupService(&vsp_group);
  if(fail && fail!=2){SPF_GroupService(&vsp_group);CHECK(vsp_group.state==4 && stops==3 && !moving && !g_dm_parallel_group_go);}
  else CHECK(vsp_group.state==3 && moving==2 && trigger_count==1 && g_dm_parallel_group_go);
 }
 fail=0;setup(3,0);g_dm_stage2_sync=1;SPF_GroupService(&vsp_group);SPF_GroupCancel(&vsp_group,2);SPF_GroupService(&vsp_group);CHECK(stops==3 && !moving);
 fail=0;setup(3,0);vsp_group.started=3;vsp_pick_queued=2;CHECK(VehicleArm_SourceParallelCanAllowed(0,q,3));vsp_group.started=1;CHECK(!VehicleArm_SourceParallelCanAllowed(0,q,3));
'''
    assert marker in queued;queued=queued.replace(marker,tests+marker,1)
    results['queue']=run('actual-queued-initial-DM-X',queued,[P/'CHASSIS/USER/source_parallel_flow.c'])
    (E/'c-checks.json').write_text(json.dumps(dict(passed=True,results=results,hardware_access=False),indent=2)+'\n')


if __name__=='__main__':main()
