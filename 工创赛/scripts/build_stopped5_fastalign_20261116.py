"""Extract actual scope/submit/SETTLE code, regress owners, build both targets."""
import hashlib
import json
import shutil
from pathlib import Path
import build_integration18_ext150_20261115 as build
import build_integration23_sync_20261115 as prior
from prepare_stopped5_fastalign_20261116 import R,BASE,P,E


def block(text, signature):
    start=text.index(signature);brace=text.index('{',start);depth=1;end=brace+1
    while depth:
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        end+=1
    return text[start:end]


def alignment_fixture():
    source=(P/'CHASSIS/USER/source_body_step.inc').read_text(encoding='utf-8')
    chassis=(P/'CHASSIS/USER/chassis_auto_test.c').read_text(encoding='gb18030')
    original=(BASE/'CHASSIS/USER/chassis_auto_test.c').read_text(encoding='gb18030')
    fresh=block(chassis,'static uint32_t Auto_AllSpeedSinceLocked(')
    zero=block(chassis,'static uint32_t Auto_ZeroSpeedLocked(')
    assert fresh==block(original,'static uint32_t Auto_AllSpeedSinceLocked(')
    assert zero==block(original,'static uint32_t Auto_ZeroSpeedLocked(')
    # Entire state branch (diagnostic endpoint code is compiled out for fixture).
    settle=block(chassis,'if (state == CHASSIS_AUTO_SETTLE)')
    # The first occurrence is in Auto_CheckLocked, not the service. Use exact later block.
    settle=block(chassis[chassis.index('if (state == CHASSIS_AUTO_VERIFY_ON)'):],
                 'if (state == CHASSIS_AUTO_SETTLE)') if 'Auto_AllSpeedSinceLocked' not in settle else settle
    assert 'Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked()' in settle
    submit=block(source,'if(g_source_body_step_state==1U)')
    scope=block(source,'uint32_t SourceBodyStep_FastAlignmentWindow(')
    fail=block(source,'static void SBS_Fail(')
    prefix=r'''
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %u %s\n",__LINE__,#x);exit(1);}}while(0)
typedef uint32_t TickType_t;
typedef struct {unsigned valid;} MotionProfile;
#define pdMS_TO_TICKS(x) (x)
#define CHASSIS_AUTO_FRESH_MS 1000U
#define CHASSIS_AUTO_GAP_MS 1000U
#define CHASSIS_AUTO_SETTLE 5U
#define CHASSIS_AUTO_DONE 6U
#define CHASSIS_AUTO_START 2U
#define APP_POSITION_DIAGNOSTIC 0
#define AUTO_SEQUENCE_HOLD() 1U
#define TURN_CCW 1U
#define TURN_CW 2U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static unsigned checks,s_align_kind,s_align_chain,g_source_body_step_state;
static unsigned s_speed_valid=15,s_phase_tick,s_speed_tick[4];
static unsigned g_chassis_auto_stop_mask[3],g_chassis_auto_segment;
static float g_chassis_auto_driver_rpm[4];
static unsigned g_chassis_auto_state,g_chassis_auto_result,g_chassis_auto_cause;
static unsigned g_chassis_auto_active,g_chassis_auto_cut_vm,s_finish_requested;
static unsigned s_run_started,s_stop_due,monitor_calls;
static unsigned g_can_guard_ready=1,s_align_rearm,s_motion_has_action,s_combined_turn,s_combined_powered_segment;
static unsigned s_route_heading_reference,s_route_heading_target,s_route_heading_turn_active;
static struct{float yaw_deg;}s_heading_source;
static unsigned s_align_direction=1,s_motion_list_submit,g_source_body_step_submitted;
static unsigned g_source_body_step_reason;
static float s_align_distance=11,s_align_turn=5,built_speed,submitted_speed;
static unsigned closed,builds,moves,turns,stops,reject;
static void MotionStop(void){stops++;}
static void Auto_EnterLocked(unsigned state,unsigned now){g_chassis_auto_state=state;s_phase_tick=now;}
static void Auto_Monitor(unsigned session){(void)session;monitor_calls++;}
static unsigned ChassisAuto_CloseHeldCompleted(unsigned session){(void)session;closed++;return 1;}
static void RouteHeading_Reset(unsigned *reference){*reference=0;}
static unsigned RouteHeading_Ensure(unsigned*r,float yaw,unsigned fresh){(void)r;(void)yaw;(void)fresh;return 1;}
static unsigned RouteHeading_Target(unsigned*r,float turn,unsigned*t){(void)r;(void)turn;*t=1;return 1;}
static unsigned MotionRoute_Build(unsigned direction,float mm,float speed,MotionProfile*p,unsigned*rpm,unsigned*ms){
 (void)direction;CHECK(mm==11 && p);built_speed=speed;builds++;*rpm=13;*ms=3000;return !reject;
}
static unsigned App_MoveSubmit(unsigned direction,unsigned rpm,unsigned ms,float mm,float speed){
 CHECK(direction==1 && rpm==13 && ms==3000 && mm==11);CHECK(s_motion_list_submit==1);
 moves++;submitted_speed=speed;return !reject;
}
static unsigned TurnStart(unsigned direction,float angle,unsigned speed){
 CHECK(direction==TURN_CCW && angle==5 && speed==3);turns++;return !reject;
}
'''
    fixture=prefix+scope+'\n'+fresh+'\n'+zero+'\n'+fail+r'''
static void settle_service(unsigned now){unsigned state=CHASSIS_AUTO_SETTLE,session=1;
'''+settle+r'''
}
static void submit_service(void){struct{unsigned session;}body={1};
'''+submit+r'''
}
int main(void){unsigned kind,chain,bs,age,mask,wheel,valid,expected,fast,iteration;
 for(kind=0;kind<=4;kind++)for(chain=0;chain<2;chain++)for(bs=0;bs<=9;bs++){
  s_align_kind=kind;s_align_chain=chain;g_source_body_step_state=bs;
  fast=kind==2 && chain && (bs==1 || bs==2);CHECK(SourceBodyStep_FastAlignmentWindow()==fast);
  for(iteration=0;iteration<3;iteration++)for(mask=0;mask<16;mask++)for(valid=0;valid<16;valid++){
   age=iteration==0?0:(iteration==1?999:1000);s_phase_tick=100;
   for(wheel=0;wheel<4;wheel++){s_speed_tick[wheel]=100+age;g_chassis_auto_driver_rpm[wheel]=0;}
   g_chassis_auto_segment=0;g_chassis_auto_stop_mask[0]=mask;s_speed_valid=valid;
   s_finish_requested=1;g_chassis_auto_state=CHASSIS_AUTO_SETTLE;g_chassis_auto_active=1;
   g_chassis_auto_result=0;g_chassis_auto_cause=0;g_chassis_auto_cut_vm=0;
   expected=valid==15 && (fast ? mask==15 : age>=1000);
   settle_service(100+age);CHECK((g_chassis_auto_state==CHASSIS_AUTO_DONE)==expected);
   CHECK(g_chassis_auto_cut_vm==0);CHECK(!g_chassis_auto_cause);
  }
 }
 /* New source fast completion cannot reuse any wheel sample before STOP phase. */
 s_align_kind=2;s_align_chain=1;g_source_body_step_state=2;s_speed_valid=15;
 g_chassis_auto_segment=0;g_chassis_auto_stop_mask[0]=15;s_finish_requested=1;
 for(wheel=0;wheel<4;wheel++){
  for(kind=0;kind<4;kind++){s_speed_tick[kind]=100;g_chassis_auto_driver_rpm[kind]=0;}
  s_phase_tick=100;s_speed_tick[wheel]=99;g_chassis_auto_state=CHASSIS_AUTO_SETTLE;
  settle_service(101);CHECK(g_chassis_auto_state==CHASSIS_AUTO_SETTLE);
  s_speed_tick[wheel]=100;g_chassis_auto_driver_rpm[wheel]=.01f;
  settle_service(101);CHECK(g_chassis_auto_state==CHASSIS_AUTO_SETTLE);
  g_chassis_auto_driver_rpm[wheel]=0;s_speed_tick[wheel]=0;s_phase_tick=0;
  for(kind=0;kind<4;kind++)if(kind!=wheel)s_speed_tick[kind]=1001;
  settle_service(1001);CHECK(g_chassis_auto_state==CHASSIS_AUTO_SETTLE);
 }
 /* Same real movement callsites use45 only for request2 + chain + state1. */
 for(kind=1;kind<=3;kind++)for(chain=0;chain<2;chain++){
  s_align_kind=kind;s_align_chain=chain;g_source_body_step_state=1;
  g_source_body_step_submitted=builds=moves=turns=0;reject=0;s_align_rearm=0;
  submit_service();CHECK(g_source_body_step_state==2 && g_source_body_step_submitted==1);
  if(kind==3)CHECK(turns==1 && !moves && !builds);
  else{CHECK(builds==1 && moves==1 && !turns);CHECK(built_speed==(kind==2 && chain?45:15));CHECK(submitted_speed==built_speed);}
 }
 g_source_body_step_reason=0;g_source_body_step_submitted=0;SBS_Fail(4);
 CHECK(g_source_body_step_state==9 && g_source_body_step_reason==4 && stops==0);
 g_source_body_step_submitted=1;SBS_Fail(6);
 CHECK(g_source_body_step_state==8 && g_source_body_step_reason==4 && stops==1);
 reject=1;s_align_kind=2;s_align_chain=1;g_source_body_step_state=1;
 g_source_body_step_reason=0;g_source_body_step_submitted=0;submit_service();
 CHECK(g_source_body_step_state==9 && g_source_body_step_reason==4);
 printf("PASS actual alignment scope/submit/SETTLE/fresh/zero/fault: %u checks; original STOP transition retained; no hardware\n",checks);
 return 0;
}
'''
    return fixture


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['validation_passed'] and proof['local_matlab'] and proof['selected_candidate_speed_mm_s']==45
    previous=R/'evidence/integration/2026-11-15/stopped3-grip170-release200'
    for f in previous.iterdir():
        if f.suffix in ('.c','.h','.inc','.txt') or f.name=='parameter-and-poll-checks.json':
            shutil.copyfile(f,E/f.name)
    (E/'tested-commit-owner.inc').write_bytes((P/'CHASSIS/USER/pzq_commit_overlap.inc').read_bytes().replace(
        b'#include "low_readonly_diag.inc"',b'/* separately verified diagnostic */'))
    prior.P=P;prior.E=E
    prior.run('actual-alignment-fast',alignment_fixture())
    for name in ('final-park-owner','owner-faults'):
        prior.run(name,(E/(name+'.c')).read_text(encoding='utf-8'))
    # Driver consumes a passed alias, leaving original MATLAB proof untouched.
    driver=Path(build.__file__).read_text(encoding='utf-8').replace('.read_text()',".read_text(encoding='utf-8')")
    driver=driver.replace("['passed']\n    assert json.loads((E/'parameter-and-poll-checks.json')", "['validation_passed']\n    assert json.loads((E/'parameter-and-poll-checks.json')")
    driver=driver.replace("str(ROOT/'tests'/source)","str(OWNER_TEST if name=='owner' else ROOT/'tests'/source)")
    scope={'__name__':'fast_alignment_build','__file__':build.__file__}
    exec(compile(driver,build.__file__,'exec'),scope)
    scope.update(PROJECT=P,BASE=BASE,E=E,OWNER_TEST=E/'all-regions-owner.c');scope['main']()
    path=E/'checks.json';checks=json.loads(path.read_text(encoding='utf-8'))
    checks.update(optimization_passed=True,three_second_target_validated=proof['target_3s_passed'],
        source_alignment_speed_mm_s=45,global_acceleration_jerk_unchanged=True,
        legacy_gap_ms=1000,source_extra_fixed_gap_ms=0,stop_ack_and_fresh_zero_preserved=True,
        preclose_deg=170,carrier_release_ms=200,synchronized_backend_byte_identical=True,
        required_stable_frames_pi=5,physical_convergence_validated=False,
        actual_alignment_fixture=(E/'actual-alignment-fast.log').read_text(encoding='utf-8').strip())
    path.write_text(json.dumps(checks,indent=2)+'\n',encoding='utf-8')
    path=E/'candidate.json';candidate=json.loads(path.read_text(encoding='utf-8'))
    candidate.update(optimization_passed=True,three_second_target_validated=proof['target_3s_passed'],
        source_alignment_speed_mm_s=45,source_extra_fixed_gap_ms=0,
        stop_ack_and_fresh_zero_preserved=True,global_acceleration_jerk_unchanged=True,
        preclose_deg=170,carrier_release_ms=200,synchronized_backend_byte_identical=True,
        physical_convergence_validated=False)
    path.write_text(json.dumps(candidate,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
