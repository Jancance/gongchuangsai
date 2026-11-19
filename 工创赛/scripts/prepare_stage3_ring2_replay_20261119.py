"""Build an isolated, opt-in Stage3 bench replay; no board access."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import ROOT, EVIDENCE, TARGETS, digest, extract_function

OUT = ROOT / "tmp/stage3_ring2_replay_v5_20261119"
BASE = ROOT / "tmp/stage3_ring2_touchdown_v3_20261119/stage3-ring2-touchdown-compatible.json"
MATLAB = EVIDENCE / "stage3-ring2-replay-matlab.log"
STOP_MATLAB = EVIDENCE / "stage3-ring2-replay-stop-queue-matlab.log"
ATOMIC_MATLAB = EVIDENCE / "stage3-ring2-replay-stop-atomic-matlab.log"

REPLAY = """
static uint32_t S3_ReplayNext(uint32_t value)
{return value==UINT32_MAX?1U:value+1U;}

static void S3_ReplayBegin(void)
{
 g_stage3_replay_step=0U;g_stage3_replay_state=0U;
 if(g_stage3_replay_enable==1U && g_region_dispatch_test_enable){
  g_stage3_replay_run=g_stage3_run;g_stage3_replay_state=1U;
 }
 g_stage3_replay_enable=0U;
}

static void S3_ReplayService(void)
{
 static const uint8_t direction[]={0U,0U,0U,2U,0U,2U,2U,0U,2U};
 static const uint8_t distance[]={7U,5U,2U,2U,1U,1U,1U,1U,1U};
 uint32_t step=g_stage3_replay_step;
 if(g_stage3_replay_run!=g_stage3_run || g_source_body_step_reason ||
    g_source_align_rejected!=s3_replay_rejected){S3_Fail(12U);return;}
 if(g_stage3_replay_state==2U){
  if(g_source_body_step_request)return;
  if(s3_replay_expected){
   uint32_t seq=g_source_align_command_seq;
   if(seq==s3_replay_previous)return;
   if(seq!=s3_replay_expected){S3_Fail(12U);return;}
   if(g_source_body_step_state!=3U || g_chassis_auto_active ||
      !g_source_body_step_session || g_source_body_step_session==s3_replay_session)return;
   g_stage3_replay_step=++step;s3_replay_expected=0U;
  }
  if(SourceBodyStep_Busy() || g_chassis_auto_active || g_chassis_auto_request)return;
  if(step==sizeof(distance)){
   if(!g_source_align_chain_active){S3_Fail(12U);return;}
   g_stage3_replay_state=3U;g_stage3_state=4U;
   g_source_body_step_request=7U;return;
  }
  if(step>sizeof(distance) || g_it_request || g_it_state!=2U){S3_Fail(12U);return;}
  s3_replay_session=g_source_body_step_session;
  s3_replay_previous=g_source_align_command_seq;
  s3_replay_expected=S3_ReplayNext(s3_replay_previous);
  g_source_align_direction=direction[step];g_source_align_distance_mm=(float)distance[step];
  g_stage3_body_steps++;g_source_body_step_request=2U;return;
 }
 if(g_source_body_step_request || SourceBodyStep_WheelWindow() ||
    g_chassis_auto_active || g_chassis_auto_request)return;
 if(g_low_wheel_hold_stage==99U){S3_Fail(12U);return;}
 if(!g_low_wheel_hold_ready || g_low_wheel_hold_stage!=13U ||
    g_low_wheel_hold_cycle==s3_replay_lock_cycle){
  if(g_stage3_replay_state==4U){S3_Fail(12U);return;}
  if(!g_low_wheel_hold_request && !g_low_wheel_hold_stage)g_low_wheel_hold_request=1U;
  return;
 }
 if(Buzzer_IsBusy())return;
 if(g_stage3_replay_state==3U){
  Buzzer_StartTimed(2U,80U,80U);g_stage3_replay_state=4U;return;
 }
 g_stage3_align_done=1U;g_stage3_state=7U;g_stage3_replay_state=5U;S3_Pose();
}
"""


def patch_sources(project, first_step_mm):
    return [
        edit(project, "stage3_cycle.h", (
            "uint32_t Stage3_Owns(void);",
            """uint32_t Stage3_Owns(void);
uint32_t Stage3_ReplayBodyQueueOwned(void);
uint32_t Stage3_ReplayStopPending(void);""")),
        edit(project, "source_body_step.h", (
            "extern volatile uint32_t g_source_body_step_reason,g_source_body_step_session;",
            """extern volatile uint32_t g_source_body_step_reason,g_source_body_step_session;
extern volatile uint32_t g_source_align_rejected;""")),
        edit(project, "stage3_cycle.inc", (
            "volatile uint32_t g_stage3_align_done,g_stage3_config_ready,g_stage3_rejected;",
            """volatile uint32_t g_stage3_align_done,g_stage3_config_ready,g_stage3_rejected;
volatile uint32_t g_stage3_replay_enable,g_stage3_replay_state,g_stage3_replay_step;
volatile uint32_t g_stage3_replay_run;
static uint32_t s3_replay_expected,s3_replay_previous,s3_replay_session,s3_replay_rejected,s3_replay_lock_cycle;"""), (
            " g_stage3_state=9U;s3_valid=0U;",
            """ g_stage3_state=9U;s3_valid=0U;
 if(g_stage3_replay_state>=1U && g_stage3_replay_state<=4U)g_stage3_replay_state=9U;"""), (
            "void Stage3_KeyEvent(void)",
            """uint32_t Stage3_ReplayBodyQueueOwned(void)
{
 return Stage3_Owns() && g_stage3_replay_state==2U && g_stage3_replay_run==g_stage3_run;
}
uint32_t Stage3_ReplayStopPending(void)
{
 extern volatile uint32_t g_vehicle_key;
 if(Stage3_ReplayBodyQueueOwned())
  return g_stage3_request==9U || g_source_body_step_request==9U ||
   g_it_request==9U || g_vehicle_key || !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
 /* Stage3 can mark a fault before the CAN task consumes the SBS STOP. */
 return Stage3_Owns() && g_stage3_replay_state==9U &&
  g_stage3_replay_run==g_stage3_run && g_source_body_step_request==9U;
}
void Stage3_KeyEvent(void)"""), (
            "static void Stage3_Service(uint32_t now)",
            REPLAY.replace("distance[]={7U,", f"distance[]={{{first_step_mm}U,")
            + "\nstatic void Stage3_Service(uint32_t now)"), (
            "  if(++g_stage3_run==0U)g_stage3_run=1U;",
            "  if(++g_stage3_run==0U)g_stage3_run=1U;\n  if(request==1U)S3_ReplayBegin();"), (
            " /* Explicit bench re-observation; clears only the previous visual result.",
            """ if(g_stage3_replay_state>=2U && g_stage3_replay_state<=4U){
  if(g_it_request==9U){S3_Fail(2U);return;}
  S3_ReplayService();return;
 }
 if(g_stage3_replay_state==5U && request){
  /* A replay is bench-only: no automatic state-5 path may open the jaw. */
  g_stage3_rejected++;return;
 }
 /* Explicit bench re-observation; clears only the previous visual result."""), (
            " if(request==30U){",
            " if(g_stage3_replay_state==1U && (request==20U || request==21U || request==30U)){\n  g_stage3_rejected++;return;\n }\n if(request==30U){"), (
            """  if(g_stage3_error_norm_um<=2000U && labs((long)g_stage3_error_yaw_mdeg)<=1000L){
   g_stage3_align_done=1U;S3_Pose();g_stage3_state=4U;""",
            """  if(g_stage3_error_norm_um<=2000U && labs((long)g_stage3_error_yaw_mdeg)<=1000L){
   if(g_stage3_replay_state==1U){
    s3_replay_expected=0U;s3_replay_rejected=g_source_align_rejected;s3_replay_lock_cycle=g_low_wheel_hold_cycle;
    g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
    S3_Pose();g_stage3_state=3U;g_stage3_replay_state=2U;return;
   }
   g_stage3_align_done=1U;S3_Pose();g_stage3_state=4U;""")),
        edit(project, "integration10_complete.inc", (
            " if(request==16U){",
            """ if(g_stage3_replay_state>=2U && g_stage3_replay_state<=4U && request){
  g_it_rejected=13U;request=0U;
 }
 if(request==16U){""")),
        edit(project, "source_body_step.inc", (
            "static void SourceBodyStep_Service(uint32_t now)",
            """/* Called within the CAN owner's critical section, before its next motion commits. */
static uint32_t SBS_CancelQueuedReplay(void)
{
 if(g_source_body_step_state!=2U || !g_source_body_step_submitted ||
    !Stage3_ReplayStopPending())return 0U;
 g_stage3_request=9U;
 g_vehicle_key=0U;
 SBS_Fail(9U);
 /* This call precedes the current move's backend commit. The preceding
  * move already acknowledged its own STOP before this one could be queued. */
 if(!g_chassis_auto_active)g_source_body_step_state=9U;
 return 1U;
}
static void SourceBodyStep_Service(uint32_t now)"""), (
            " g_source_body_step_request=0U;",
            """ if(Stage3_ReplayStopPending())request=9U;
 g_source_body_step_request=0U;"""), (
            """  s_motion_list_submit=1U;
  if(s_align_kind==3U)accepted=TurnStart(s_align_turn>0?TURN_CCW:TURN_CW,fabsf(s_align_turn),3U);
  else accepted=App_MoveSubmit(s_align_direction,rpm,run_ms,s_align_distance,speed);
  s_motion_list_submit=0U;
  if(!accepted){SBS_Fail(5U);return;}
  g_source_body_step_submitted=1U;g_source_body_step_state=2U;return;""",
            """  if(Stage3_ReplayStopPending()){SBS_Fail(9U);return;}
  if(s_align_kind==2U && Stage3_ReplayBodyQueueOwned()){
   s_motion_list_submit=1U;
   accepted=App_MoveSubmitOwned(s_align_direction,rpm,run_ms,s_align_distance,speed,1U);
   s_motion_list_submit=0U;
   if(!accepted)SBS_Fail(Stage3_ReplayStopPending()?9U:5U);
   return; /* Queue and SBS owner were published together by App_MoveSubmitOwned. */
  }
  s_motion_list_submit=1U;
  if(s_align_kind==3U)accepted=TurnStart(s_align_turn>0?TURN_CCW:TURN_CW,fabsf(s_align_turn),3U);
  else accepted=App_MoveSubmit(s_align_direction,rpm,run_ms,s_align_distance,speed);
  s_motion_list_submit=0U;
  if(!accepted){SBS_Fail(5U);return;}
  g_source_body_step_submitted=1U;g_source_body_step_state=2U;return;""")),
        edit(project, "main.c", (
            "static void App_ForwardHoldStep(void)",
            """static uint32_t SBS_CancelQueuedReplay(void);
static void App_ForwardHoldStep(void)"""), (
            "    request = g_forward_hold.request;\n    g_forward_hold.request = 0U;",
            """    request = g_forward_hold.request;
    if (request == 1U && g_forward_hold.phase == HR_IDLE && SBS_CancelQueuedReplay())
    { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
    g_forward_hold.request = 0U;"""), (
            """        begin_cycle = HWT101_TimingBegin();
        accepted = (!SourceBodyStep_WheelWindow()""",
            """        if (SBS_CancelQueuedReplay())
        { HWT101_TimingEnd(0U, critical_cycle); taskEXIT_CRITICAL(); return; }
        begin_cycle = HWT101_TimingBegin();
        accepted = (!SourceBodyStep_WheelWindow()"""), (
            "static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms, float distance_mm,float route_speed)",
            """static uint32_t App_MoveSubmitOwned(uint32_t direction, uint32_t rpm, uint32_t run_ms,
    float distance_mm,float route_speed,uint32_t replay)"""), (
            """    if (!g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())""",
            """    if ((!replay || (Stage3_ReplayBodyQueueOwned() && !Stage3_ReplayStopPending())) &&
        !g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())"""), (
            """        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;""",
            """        if (replay)
        {
            g_source_body_step_submitted = 1U;
            g_source_body_step_state = 2U;
        }
        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;"""), (
            """uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ return App_MoveSubmit(direction,rpm,run_ms,0.0f,0.0f); }""",
            """static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms,
    float distance_mm,float route_speed)
{return App_MoveSubmitOwned(direction,rpm,run_ms,distance_mm,route_speed,0U);}
uint32_t MoveStart(uint32_t direction, uint32_t rpm, uint32_t run_ms)
{ return App_MoveSubmit(direction,rpm,run_ms,0.0f,0.0f); }""")),
    ]


def check_fixture(project, out, first_step_mm):
    body = (project / "CHASSIS/USER/stage3_cycle.inc").read_bytes().decode("gbk", errors="surrogateescape")
    sbs = (project / "CHASSIS/USER/source_body_step.inc").read_bytes().decode("gbk", errors="surrogateescape")
    can_owner = (project / "CHASSIS/USER/main.c").read_bytes().decode("gbk", errors="surrogateescape")
    forward = extract_function(can_owner, "static void App_ForwardHoldStep(void)")
    assert forward.count("SBS_CancelQueuedReplay()") == 2
    assert forward.index("SBS_CancelQueuedReplay()") < forward.index("g_forward_hold.request = 0U;")
    assert forward.rindex("SBS_CancelQueuedReplay()") < forward.index("ChassisAuto_BeginPreparedTranslation")
    publish = extract_function(can_owner, "static uint32_t App_MoveSubmitOwned(")
    offsets = [publish.index(x) for x in ("taskENTER_CRITICAL();", "!Stage3_ReplayStopPending()",
               "g_source_body_step_submitted = 1U;", "g_source_body_step_state = 2U;",
               "g_forward_hold.request = 1U;", "taskEXIT_CRITICAL();")]
    assert offsets == sorted(offsets)
    prefix = r'''
#include <stdint.h>
#include <stdio.h>
static uint32_t checks;
#define CHECK(e) do{++checks;if(!(e)){fprintf(stderr,"line %d: %s\n",__LINE__,#e);return 1;}}while(0)
static uint32_t g_stage3_replay_enable,g_stage3_replay_state,g_stage3_replay_run,g_stage3_replay_step;
static uint32_t g_region_dispatch_test_enable=1,g_stage3_run=5,g_stage3_state,g_stage3_request;
static uint32_t g_stage3_fault,g_stage3_align_done,s3_valid,g_stage3_body_steps;
static uint32_t s3_replay_expected,s3_replay_previous,s3_replay_session,s3_replay_rejected,s3_replay_lock_cycle;
static uint32_t g_source_body_step_request,g_source_body_step_reason,g_source_body_step_state,g_source_body_step_session;
static uint32_t g_source_body_step_submitted,stop_calls,queue;
volatile uint32_t g_vehicle_key;
static uint32_t pin_high=1U;
#define GPIOC 1U
#define GPIO_Pin_13 13U
static uint32_t GPIO_ReadInputDataBit(uint32_t port,uint32_t pin){return port==1U && pin==13U && pin_high;}
static uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_rejected,g_source_align_chain_active;
static uint32_t g_chassis_auto_active,g_chassis_auto_request,g_low_wheel_hold_ready,g_low_wheel_hold_stage;
static uint32_t g_low_wheel_hold_cycle,g_low_wheel_hold_request,g_it_request,g_it_action,g_it_state=2,buzz,beeps,poses;
static float g_source_align_distance_mm;
static uint32_t Stage3_Owns(void){return 1;}
static uint32_t SourceBodyStep_Busy(void){return g_source_body_step_state==2;}
static uint32_t SourceBodyStep_WheelWindow(void){return g_source_align_chain_active;}
static uint32_t Buzzer_IsBusy(void){return buzz;}
static void Buzzer_StartTimed(uint32_t n,uint32_t on,uint32_t gap){if(n==2&&on==80&&gap==80)beeps++;buzz=1;}
static void S3_Pose(void){poses++;}
static void MotionStop(void){stop_calls++;queue=0;}
'''
    functions = [extract_function(body, n) for n in (
        "static void S3_Fail(", "uint32_t Stage3_ReplayBodyQueueOwned(", "uint32_t Stage3_ReplayStopPending(", "void Stage3_KeyEvent(",
        "static uint32_t S3_ReplayNext(", "static void S3_ReplayBegin(", "static void S3_ReplayService(")]
    functions += [extract_function(sbs, n) for n in (
        "static void SBS_Fail(", "static uint32_t SBS_CancelQueuedReplay(")]
    cases = r'''
static void boot(void){
 g_stage3_replay_enable=1;S3_ReplayBegin();g_stage3_replay_state=2;g_stage3_state=3;
 s3_replay_expected=g_source_body_step_request=g_source_body_step_reason=g_source_body_step_state=0;
 g_source_align_rejected=s3_replay_rejected=0;
 g_chassis_auto_active=g_source_align_chain_active=buzz=beeps=poses=0;
 g_stage3_request=g_stage3_fault=g_stage3_align_done=g_it_request=0;
 g_low_wheel_hold_stage=g_low_wheel_hold_ready=g_low_wheel_hold_request=0;
 s3_replay_lock_cycle=g_low_wheel_hold_cycle;
}
int main(void){
 const uint32_t d[]={0,0,0,2,0,2,2,0,2},m[]={7,5,2,2,1,1,1,1,1};
 uint32_t i,prior_session;
 g_stage3_replay_enable=1;S3_ReplayBegin();CHECK(g_stage3_replay_state==1&&!g_stage3_replay_enable);
 S3_ReplayBegin();CHECK(!g_stage3_replay_state);
 g_stage3_state=11;Stage3_KeyEvent();CHECK(g_stage3_request==1);
 boot();g_source_align_command_seq=UINT32_MAX-1;
 for(i=0;i<9;i++){
  S3_ReplayService();
  CHECK(g_source_body_step_request==2 && g_source_align_direction==d[i] && g_source_align_distance_mm==(float)m[i]);
  CHECK(!beeps && !g_stage3_align_done && g_stage3_replay_step==i);
  g_source_body_step_request=0;g_source_body_step_state=3;
  S3_ReplayService();CHECK(g_stage3_replay_step==i);
  g_source_align_command_seq=S3_ReplayNext(g_source_align_command_seq);
  prior_session=g_source_body_step_session;
  S3_ReplayService();CHECK(g_stage3_replay_step==i);
  g_source_body_step_session=prior_session+1;g_chassis_auto_active=1;
  S3_ReplayService();CHECK(g_stage3_replay_step==i);
  g_chassis_auto_active=0;g_source_align_chain_active=1;
  if(i+1<9){
   S3_ReplayService();CHECK(g_stage3_replay_step==i+1);
  }
 }
 S3_ReplayService();CHECK(g_stage3_replay_step==9 && g_source_body_step_request==7 && g_stage3_replay_state==3);
 g_source_body_step_request=0;S3_ReplayService();CHECK(!g_low_wheel_hold_request && !beeps);
 g_source_align_chain_active=0;S3_ReplayService();CHECK(g_low_wheel_hold_request && !beeps);
 g_low_wheel_hold_ready=1;g_low_wheel_hold_stage=13;S3_ReplayService();CHECK(!beeps);
 buzz=1;g_low_wheel_hold_cycle++;S3_ReplayService();CHECK(!beeps);
 buzz=0;S3_ReplayService();CHECK(beeps==1 && g_stage3_replay_state==4 && !g_stage3_align_done);
 S3_ReplayService();CHECK(beeps==1 && g_stage3_state==4 && !g_stage3_align_done);
 buzz=0;S3_ReplayService();CHECK(g_stage3_replay_state==5 && g_stage3_state==7 && g_stage3_align_done && poses==1);
 g_it_request=18;Stage3_KeyEvent();CHECK(g_stage3_request==9);
 g_it_request=0;g_it_action=18;g_it_state=1;Stage3_KeyEvent();CHECK(g_stage3_request==9);g_it_state=2;
 boot();S3_ReplayService();Stage3_KeyEvent();CHECK(g_stage3_request==9 && Stage3_ReplayStopPending());
 S3_Fail(2);CHECK(g_stage3_replay_state==9 && g_source_body_step_request==9 && !beeps);
 boot();S3_ReplayService();g_source_align_rejected++;S3_ReplayService();CHECK(g_stage3_replay_state==9 && !beeps);
 boot();S3_ReplayService();g_source_body_step_request=0;g_source_align_command_seq=S3_ReplayNext(s3_replay_expected);
 S3_ReplayService();CHECK(g_stage3_replay_state==9 && !beeps);
 boot();g_stage3_replay_state=3;g_low_wheel_hold_stage=99;S3_ReplayService();CHECK(g_stage3_replay_state==9 && !beeps);
 boot();g_stage3_replay_state=4;g_stage3_state=4;Stage3_KeyEvent();CHECK(g_stage3_request==9);
 S3_Fail(2);CHECK(g_stage3_replay_state==9 && !g_stage3_align_done);
 for(i=0;i<2;i++){
  uint32_t input;
  for(input=0;input<7;input++){
   boot();queue=1;stop_calls=0;g_vehicle_key=0;pin_high=1;g_source_body_step_state=2;g_source_body_step_submitted=1;
   if(i){CHECK(!SBS_CancelQueuedReplay() && queue);}
   if(input==1)g_stage3_request=9;
   if(input==2)g_it_request=9;
   if(input==3)g_source_body_step_request=9;
   if(input==4)g_vehicle_key=1;
   if(input==5)pin_high=0;
   if(input==6){g_stage3_replay_state=9;g_source_body_step_request=9;}
   if(input){
    CHECK(SBS_CancelQueuedReplay());
    CHECK(!queue && stop_calls==1 && g_stage3_request==9 && !g_vehicle_key);
    CHECK(g_source_body_step_state==9 && g_source_body_step_reason==9);
   }else CHECK(!SBS_CancelQueuedReplay() && queue && !stop_calls);
  }
 }
 boot();queue=1;stop_calls=0;g_vehicle_key=1;g_source_body_step_state=2;
 g_source_body_step_submitted=0;CHECK(!SBS_CancelQueuedReplay() && g_vehicle_key && queue);
 g_stage3_replay_state=0;g_source_body_step_submitted=1;
 CHECK(!SBS_CancelQueuedReplay() && g_vehicle_key && queue);
 printf("PASS actual Stage3 replay/key/fault/lock/beep and queued STOP helper: %u checks; mocked transport, no hardware\n",checks);
 return 0;
}
'''
    cases = cases.replace("m[]={7,", f"m[]={{{first_step_mm},")
    cfile, binary = out / "actual-replay.c", out / "actual-replay.exe"
    cfile.write_text(prefix + "\n".join(functions) + cases, encoding="ascii")
    subprocess.run(["gcc", "-std=c99", "-Werror", str(cfile), "-o", str(binary)], check=True, timeout=30)
    result = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
    log = out / "actual-replay.log"
    log.write_text(result.stdout + result.stderr, encoding="ascii")
    print(result.stdout + result.stderr, end="")
    result.check_returncode()
    return {"log": str(log), "sha256": digest(log)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--forward-mm", type=int, choices=(11, 16), default=16)
    args = parser.parse_args()
    out = OUT if args.forward_mm == 16 else ROOT / "tmp/stage3_ring2_replay_forward11_20261119"
    matlab = MATLAB if args.forward_mm == 16 else EVIDENCE / "stage3-ring2-replay-forward11-matlab.log"
    first_step_mm = args.forward_mm - 9
    matlab_text = matlab.read_text(encoding="utf-8")
    if ("PASS Stage3 ring2 replay compensation and release ordering: 3904 cases" not in matlab_text
            or (args.forward_mm == 11 and "Recipe: forward 11 mm, right 5 mm, nine steps" not in matlab_text)
            or "PASS replay STOP queue and CAN commit timing: 200 cases" not in STOP_MATLAB.read_text(encoding="utf-8")
            or "PASS replay STOP atomic publication and precommit timing: 56 cases" not in ATOMIC_MATLAB.read_text(encoding="utf-8")):
        raise RuntimeError("MATLAB prerequisite missing")
    manifest = json.loads(BASE.read_text(encoding="utf-8"))
    source = Path(manifest["project"]).resolve()
    project = out / "project"
    result_path = out / "stage3-ring2-replay-compatible.json"
    if not project.exists():
        for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
            if digest(source / name) != expected:
                raise ValueError(f"Baseline hash mismatch: {name}")
        shutil.copytree(source, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
        for name in patch_sources(project, first_step_mm):
            manifest["unchanged_sha256"].pop(name, None)
            manifest["changes"][name] = digest(project / name)
        manifest.update(project=str(project), images={}, build_passed=False, flashed=False,
                        matlab_sha256=digest(matlab), baseline_manifest=str(BASE),
                        stop_queue_matlab_sha256=digest(STOP_MATLAB),
                        stop_atomic_matlab_sha256=digest(ATOMIC_MATLAB),
                        replay_activation_symbol="g_stage3_replay_enable")
        manifest["capabilities"]["stage3_ring2_replay"] = True
        result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    else:
        manifest = json.loads(result_path.read_text(encoding="utf-8"))
        for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
            if digest(project / name) != expected:
                raise ValueError(f"Candidate hash mismatch: {name}")
    manifest["c_fixture"] = check_fixture(project, out, first_step_mm)
    manifest["replay_compensation_mm"] = {"forward": args.forward_mm, "right": 5}
    if args.build:
        for target in TARGETS:
            for directory in ("Objects", "Listings"):
                (project / directory / target).mkdir(parents=True, exist_ok=True)
            log = out / f"{target}.log"
            result = subprocess.run(["C:/Keil_v5/UV4/UV4.exe", "-r", str(project / "gongxun-f407.uvprojx"),
                                     "-t", target, "-j0", "-o", str(log)], timeout=180)
            if result.returncode or b"0 Error(s), 0 Warning(s)" not in log.read_bytes():
                raise RuntimeError(f"Build failed: {log}")
            image = project / "Objects" / target / f"{target}.axf"
            manifest["images"][target] = {"axf": str(image), "sha256": digest(image)}
            print(f"{target}: 0 errors, 0 warnings", flush=True)
        manifest["build_passed"] = True
    result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(result_path)


if __name__ == "__main__":
    main()
