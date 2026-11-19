"""Focused check of actual alignment owner transitions with simulated IO."""
from pathlib import Path
import ast,json,subprocess
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-12/source-align';P=R/'tmp/source_alignment_20261112/project'
assert json.loads((E/'matlab-result.json').read_text())['passed']
tree=ast.parse((R/'scripts/check_source_body_step_20261111.py').read_text(encoding='utf-8'))
values={}
for node in tree.body:
    if isinstance(node,ast.Assign) and isinstance(node.value,ast.Constant) and isinstance(node.value.value,str):
        for name in node.targets:
            if isinstance(name,ast.Name):values[name.id]=node.value.value
prefix=values['prefix'].replace('CK(direction==1 && distance==10 && speed==15);','CK(direction<=3 && distance>=1 && distance<=20 && speed==15);')
prefix=prefix.replace('CK(d==1 && rpm==6 && run==4843 && distance==10 && speed==15 && s_motion_list_submit==1);','CK(d<=3 && rpm==6 && run==4843 && distance>=1 && distance<=20 && speed==15 && s_motion_list_submit==1);')
prefix=prefix.replace('CK(sticky==1);','CK(sticky==1);').replace('CK(offset==0);','(void)offset;')
extra=r'''
#define TURN_CW 0U
#define TURN_CCW 1U
static unsigned s_combined_turn,s_route_heading_turn_active,turn_calls;
static float g_heading_exec_relative_deg;
static void RouteHeading_Reset(void*p){(void)p;}
static unsigned TurnStart(unsigned direction,float angle,unsigned rpm){
 CK(direction<=1 && angle>=5 && angle<=10 && rpm==3 && s_motion_list_submit);
 turn_calls++;s_combined_turn=1;g_heading_exec_relative_deg=direction?angle:-angle;
 s_motion_session_open=backend_submit;s_motion_has_action=backend_submit;return backend_submit;
}
'''
old=values['owner_tests'];start=old.index('static void reset(void)');pos=old.index('{',start);level=1;end=pos+1
while level:
    if old[end]=='{':level+=1
    elif old[end]=='}':level-=1
    end+=1
reset=old[start:end].replace('static void reset(void)','static void base_reset(void)')
tests=r'''
static void reset(void){base_reset();s_align_chain=s_align_kind=s_align_direction=0;s_align_distance=s_align_turn=0;
 g_source_align_command_seq=g_source_align_rejected=turn_calls=0;s_combined_turn=s_route_heading_turn_active=0;
 g_source_align_direction=1;g_source_align_distance_mm=10;g_source_align_turn_deg=-5;}
static void begin_turn(void){g_can_guard_ready=1;g_source_body_step_request=3;SourceBodyStep_Service(10);CK(g_source_body_step_state==2);}
static void running(unsigned session,unsigned tick){fake_body.session=session;fake_body.active=1;fake_status.state=1;fake_status.backend_active=1;SourceBodyStep_Service(tick);}
static void done(unsigned tick){fake_body.active=0;fake_status.state=MOTION_DONE;fake_status.backend_active=0;
 fake_status.backend_result=1;fake_status.stop_mask=15;SourceBodyStep_Service(tick);}
int main(void){
 reset();begin_turn();CK(g_source_align_command_seq==1 && turn_calls==1 && SourceBodyStep_SmallDistanceWindow());
 running(7,20);done(30);CK(g_source_body_step_state==3 && finish_calls==0 && s_motion_session_open && !SourceBodyStep_SmallDistanceWindow());
 g_source_align_direction=2;g_source_align_distance_mm=5;g_source_body_step_request=2;SourceBodyStep_Service(40);
 CK(g_source_align_command_seq==2 && g_source_body_step_state==2 && submit_calls==1);
 s_combined_turn=0;fake_status.distance_mode=1;fake_status.distance_reached=1;fake_status.distance_direction=2;fake_status.distance_target_cm=.5f;
 running(8,45);done(50);CK(g_source_body_step_state==3 && finish_calls==0);
 g_source_body_step_request=7;SourceBodyStep_Service(60);CK(finish_calls==1 && !s_align_chain && !s_motion_session_open);
 reset();g_source_align_distance_mm=.9f;g_source_body_step_request=2;SourceBodyStep_Service(10);
 CK(g_source_body_step_state==9 && g_source_body_step_reason==10 && submit_calls==0 && !SourceBodyStep_SmallDistanceWindow());
 g_source_body_step_request=3;SourceBodyStep_Service(20);CK(turn_calls==0 && g_source_body_step_state==9 && g_source_body_step_reason==10);
 reset();g_source_align_distance_mm=20.1f;g_source_body_step_request=2;SourceBodyStep_Service(10);CK(g_source_body_step_state==9);
 reset();g_source_align_turn_deg=4.9f;g_source_body_step_request=3;SourceBodyStep_Service(10);CK(g_source_body_step_state==9 && turn_calls==0);
 reset();begin_turn();g_source_align_turn_deg=10;g_source_body_step_request=3;SourceBodyStep_Service(15);
 CK(g_source_align_rejected==1 && g_source_align_command_seq==1 && turn_calls==1 && s_align_turn==-5);
 reset();begin_turn();running(4,20);g_it_cause=8;SourceBodyStep_Service(30);CK(stop_calls==1 && g_source_body_step_state==8);
 fake_body.active=0;fake_body.rotation_stop_mask=15;SourceBodyStep_Service(40);CK(g_source_body_step_state==9 && !SourceBodyStep_SmallDistanceWindow());
 reset();begin_turn();g_source_body_step_request=9;SourceBodyStep_Service(15);CK(stop_calls==1 && g_source_body_step_reason==9);
 reset();begin_turn();running(6,20);fake_body.session=7;done(30);CK(g_source_body_step_state==8 && g_source_body_step_reason==7);
 printf("PASS actual alignment owner: %u assertions; bounded steps, same-session DONE, explicit continuation, fault/STOP retained.\n",checks);return 0;
}
'''
source=E/'owner-check.c';exe=E/'owner-check.exe';source.write_text(prefix+extra+(P/'CHASSIS/USER/source_body_step.inc').read_text()+reset+tests)
p=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation',str(source),'-lm','-o',str(exe)],capture_output=True,text=True)
log=p.stdout+p.stderr
if p.returncode==0:
    p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10);log+=p.stdout+p.stderr
(E/'owner-check.log').write_text(log);print(log);assert p.returncode==0
base=R/'tmp/source_body_hold_20261111/project'
for name in ['CHASSIS/USER/integration13_immediate.inc','CHASSIS/USER/integration10_complete.inc','CHASSIS/SYSTEM/SAFETY/safety.c']:
    assert (P/name).read_bytes()==(base/name).read_bytes()
(E/'checks.json').write_text(json.dumps(dict(passed=True,actual_owner_transitions_checked=True,hardware_access=False,
    original_pzq_schedule_and_stop_and_hold_byte_identical=True,matlab_prior_passed=True),indent=2))
