"""Derive/build the isolated Stage3 offline development candidate; never flash."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence/integration/2026-11-18/stage3-resume"
BASE_MANIFEST = EVIDENCE / "stage3-return-settle-compatible.json"
OUT = ROOT / "tmp/stage3_speed_offline_v3_20261119"
TARGETS = ("Vehicle_LiftRuler", "Vehicle_LiftRuler_Offline")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def edit(project, filename, *replacements):
    relative = "CHASSIS/USER/" + filename
    path = project / relative
    data = path.read_bytes()
    newline = b"\r\n" if b"\r\n" in data else b"\n"
    for old, new in replacements:
        old, new = (x.encode("ascii").replace(b"\n", newline) for x in (old, new))
        if data.count(old) != 1:
            raise ValueError(f"{relative}: expected one anchor {old[:100]!r}")
        data = data.replace(old, new)
    path.write_bytes(data)
    return relative


def change_sources(project):
    changed = []
    changed.append(edit(project, "source_parallel_flow.h", (
        " uint32_t lift_limit_pulses;",
        " uint32_t lift_limit_pulses,stage3_business_transfer;")))
    changed.append(edit(project, "source_parallel_flow.c", (
        "  c->extension_rpm!=(c->lift_limit_pulses?SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||",
        """  (c->stage3_business_transfer && (c->stage3_business_transfer!=1U ||
   c->kind!=SPF_CMD_TRANSFER || c->lift_limit_pulses!=SPF_STAGE3_MAX_LIFT_PULSES)) ||
  c->extension_rpm!=(c->lift_limit_pulses && !c->stage3_business_transfer?SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||""")))
    changed.append(edit(project, "stage3_cycle.h", (
        "uint32_t Stage3_Owns(void);",
        """extern volatile uint32_t g_stage3_storage_only;
extern volatile float g_stage3_place_clearance_depth[3],g_stage3_recovery_clearance_depth[3];
uint32_t Stage3_Owns(void);
uint32_t Stage3_BusinessReturnActive(void);
uint32_t Stage3_LrcSharesXReturn(void);""")))
    changed.append(edit(project, "stage3_cycle.inc", (
        "volatile uint32_t g_stage3_placed_count,g_stage3_recovered_count;",
        """volatile uint32_t g_stage3_placed_count,g_stage3_recovered_count;
volatile uint32_t g_stage3_storage_only;
/* Physical ring order, absolute downward virtual pulses from mechanical top. */
volatile float g_stage3_place_clearance_depth[3]={-1.0f,-1.0f,-1.0f};
volatile float g_stage3_recovery_clearance_depth[3]={-1.0f,-1.0f,-1.0f};
static float s3_place_clearance[3],s3_recovery_clearance[3],s3_return_clearance;
static uint32_t s3_full_batch,s3_parallel_x_started;"""), (
        "uint32_t Stage3_FineTurnAllowed(void)",
        """uint32_t Stage3_BusinessReturnActive(void)
{return Stage3_Owns() && g_stage3_state==5U && s3_pending==4U &&
  g_it_state==1U && g_it_action==4U;}
uint32_t Stage3_LrcSharesXReturn(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 return Stage3_BusinessReturnActive() && s3_parallel_x_started &&
  VehicleSourceParallel_Owns() && vsp_group.command.axes_mask==SPF_EXT &&
  vsp_group.command.goal.extension_mm==0.0f && vsp_group.state<4U;
#endif
}
uint32_t Stage3_FineTurnAllowed(void)"""), (
        " g_stage3_task_run=task->run_id;g_stage3_task_batch=task->batch_id;return 1U;",
        """ g_stage3_storage_only=0U;
 for(i=0U;i<3U;i++){g_stage3_place_clearance_depth[i]=-1.0f;g_stage3_recovery_clearance_depth[i]=-1.0f;}
 g_stage3_task_run=task->run_id;g_stage3_task_batch=task->batch_id;return 1U;"""), (
        " if(g_stage3_task_run && (first!=0U || count!=3U ||",
        """ if(g_stage3_storage_only>1U ||
    (g_stage3_storage_only && (!g_stage3_task_run || g_stage3_task_run!=g_stage3_run)))return 0U;
 if(g_stage3_task_run && (first!=0U || count!=3U ||"""), (
        "  found=0U;for(j=0U;j<3U;j++)if(g_stage3_slot_colors[j]==g_stage3_colors[i])found++;",
        """  if(!g_stage3_task_run && first==1U && count==1U && i!=first)continue;
  found=0U;for(j=0U;j<3U;j++)if(g_stage3_slot_colors[j]==g_stage3_colors[i])found++;"""), (
        "  if(g_stage3_task_run && !(g_stage3_recovery_depth[j]>=3200.0f &&",
        "  if(g_stage3_task_run && !g_stage3_storage_only && !(g_stage3_recovery_depth[j]>=3200.0f &&"), (
        " s3_round_trip=g_stage3_task_run!=0U;",
        """ s3_full_batch=g_stage3_task_run!=0U;
 s3_round_trip=s3_full_batch && !g_stage3_storage_only;
 s3_return_clearance=-1.0f;s3_parallel_x_started=0U;"""), (
        "  s3_recovery_depth[i]=g_stage3_recovery_depth[i];",
        """  s3_recovery_depth[i]=g_stage3_recovery_depth[i];
  s3_place_clearance[i]=g_stage3_place_clearance_depth[i];
  s3_recovery_clearance[i]=g_stage3_recovery_clearance_depth[i];"""), (
        " if(s3_round_trip && g_stage3_state==5U && s3_pending==3U &&",
        " if(s3_full_batch && g_stage3_state==5U && s3_pending==3U &&"), (
        "static void S3_Axis(uint32_t axis,float value)",
        """static void S3_ReturnStart(uint32_t target,uint32_t recovery)
{
 float clear=recovery?s3_recovery_clearance[target]:s3_place_clearance[target];
 float contact=recovery?s3_recovery_depth[target]:s3_depth[target];
 /* Missing/invalid calibration never prevents the original serial return. */
 s3_parallel_x_started=0U;
 s3_return_clearance=s3_full_batch && s3_x[target]>0.0f &&
  clear>(float)SOURCE_HOME_BACKOFF_PULSES && clear<contact?clear:-1.0f;
 S3_Command(4U);
}
static void S3_Axis(uint32_t axis,float value)"""), (
        "/* Scoped permission for the dedicated Stage3 two-stepper transfer. */",
        """static void S3_ReturnX(void)
{
 if(s3_parallel_x_started){
  s3_parallel_x_started=0U;
  if(fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)<=SPF_STAGE3_TURN_ZERO_DEG){
   g_stage3_phase++;return;
  }
  /* The overlap already supplied the first settled attempt. Correct once. */
  s3_zero_correction_sent=1U;
 }
 S3_Axis(SPF_EXT,0.0f);
}
/* Scoped permission for the dedicated Stage3 two-stepper transfer. */"""), (
        " return Stage3_Owns() && s3_round_trip && g_stage3_state==5U &&",
        " return Stage3_Owns() && s3_full_batch && g_stage3_state==5U &&"), (
        "  if((p==1U || p==7U || p==12U) && it_at_top()){g_stage3_phase++;return;}",
        """  if((p==1U || p==7U || p==12U) && it_at_top()){g_stage3_phase++;return;}
  if(p==4U){
   if(!s3_preclose_started || (uint32_t)(now-s3_preclose_tick)<600U)return;
   g_stage3_phase=p=5U;
  }"""), (
        "  case 4U:if(s3_preclose_started && (uint32_t)(now-s3_preclose_tick)>=600U)g_stage3_phase++;break; /* DM done plus original settle. */\n",
        ""), (
        "   if(s3_round_trip){\n    g_it_axes=SPF_EXT|SPF_LIFT;",
        "   if(s3_full_batch){\n    g_it_axes=SPF_EXT|SPF_LIFT;"), (
        "   if(s3_round_trip)g_stage3_phase++;",
        "   if(s3_full_batch)g_stage3_phase++;"), (
        "  case 12U:S3_Command(4U);break;\n  case 13U:S3_Axis(SPF_EXT,0.0f);break;",
        "  case 12U:S3_ReturnStart(target,0U);break;\n  case 13U:S3_ReturnX();break;"), (
        "  case 21U:S3_Command(4U);break;\n  case 22U:S3_Axis(SPF_EXT,0.0f);break;",
        "  case 21U:S3_ReturnStart(target,1U);break;\n  case 22U:S3_ReturnX();break;")))
    changed.append(edit(project, "vehicle_source_parallel.inc", (
        "static uint32_t vsp_pick_sync,vsp_pick_queued;",
        """static uint32_t vsp_pick_sync,vsp_pick_queued,vsp_stage3_return_x;
/* LRC owns node 5 throughout. This alternate reference predicate is only
 * for the authorized X-only return during its normal reference-reuse leg. */
static uint32_t VSP_Stage3ReturnReferences(void)
{
 return g_lift_ruler_controller.top_valid && g_lift_ruler_controller.top_scale &&
  !g_lift_ruler_cause && g_startup_axes_state==4U && !g_startup_axes_cause &&
  g_extension_home_reference_valid && g_extension_hold_confirmed &&
  g_vm_test_request==1U && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;
}"""), (
        " if(!SingleTeach_ReferencesReady() && vsp_group.state!=4)SPF_GroupCancel(&vsp_group,8);",
        """ if(!(vsp_stage3_return_x?VSP_Stage3ReturnReferences():SingleTeach_ReferencesReady()) &&
    vsp_group.state!=4)SPF_GroupCancel(&vsp_group,8);"""), (
        "uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *command,uint32_t sweep_verified,uint32_t driving_stow_verified)",
        """static uint32_t VSP_Begin(const SPF_Command *command,uint32_t sweep_verified,
 uint32_t driving_stow_verified,uint32_t stage3_return_x)"""), (
        " (void)command;(void)sweep_verified;(void)driving_stow_verified;return 0;",
        " (void)command;(void)sweep_verified;(void)driving_stow_verified;(void)stage3_return_x;return 0;"), (
        " if(!command || VehicleArm_Busy() || vsp_owned || !SingleTeach_ReferencesReady() ||",
        """ if(!command || VehicleArm_Busy() || vsp_owned ||
  (stage3_return_x?(command->axes_mask!=SPF_EXT || command->goal.extension_mm!=0.0f ||
   !VSP_Stage3ReturnReferences()):!SingleTeach_ReferencesReady()) ||"""), (
        " vsp_pick_lead_ms=0U;memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;",
        """ vsp_stage3_return_x=stage3_return_x;
 vsp_pick_lead_ms=0U;memset(vsp_axis,0,sizeof(vsp_axis));vsp_dm_posted=0;vsp_fast_lift=0U;vsp_owned=1;"""), (
        "uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t address,const uint8_t *q,uint32_t n)",
        """uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *command,uint32_t sweep_verified,uint32_t driving_stow_verified)
{return VSP_Begin(command,sweep_verified,driving_stow_verified,0U);}

uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t address,const uint8_t *q,uint32_t n)""")))
    changed.append(edit(project, "integration10_complete.inc", (
        "#include \"stage3_cycle.inc\"\n",
        """#include "stage3_cycle.inc"
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t S3_ReturnClearanceReached(const LiftRulerController *z,float clearance,uint32_t now)
{
 float depth;
 /* Phase 12 follows the current move's validated node-5 position read.
  * Native homing refresh remains serial. No cached VSP Z or synthetic top. */
 if(!(clearance>0.0f) || z->state!=LRC_MOVING || z->operation!=2U ||
    z->phase!=12U || !z->reuse_reference || !z->top_valid ||
    !z->top_scale || z->scale!=z->top_scale || z->cause || z->stop_pending ||
    !z->moves_sent || (uint32_t)(now-z->last_position_ms)>=400U ||
    z->position_after_raw>=z->command_before_raw || z->position_after_raw<z->top_position_raw ||
    z->goal_raw>=z->command_before_raw)return 0U;
 depth=(float)(z->position_after_raw-z->top_position_raw)*(3200.0f/360.0f)/(float)z->top_scale;
 return depth<=clearance;
}
static void S3_ServiceBusinessReturn(uint32_t now)
{
 LiftRulerController *z=&g_lift_ruler_controller;
 SPF_Command c;
 if(g_lift_ruler_rejected){it_fail(5U);return;}
 if(s3_parallel_x_started && g_source_parallel_state==4U){it_fail(8U);return;}
 if(g_lift_ruler_samples!=it_lift_samples && g_lift_ruler_state==LRC_DONE){
  if(!s3_parallel_x_started ||
     (g_source_parallel_state==3U && (g_source_parallel_done_mask&SPF_EXT) && !VehicleSourceParallel_Owns())){
   if(it_at_top())it_done();
  }
  return;
 }
 if(s3_parallel_x_started || !g_low_wheel_hold_ready || !it_dm_fresh() ||
    !g_dm_posvel_holding || VehicleArm_Busy() ||
    !S3_ReturnClearanceReached(z,s3_return_clearance,now))return;
 memset(&c,0,sizeof(c));c.kind=SPF_CMD_TRANSFER;c.sequence=++g_it_sequence;
 c.axes_mask=SPF_EXT;c.lift_limit_pulses=SPF_STAGE3_MAX_LIFT_PULSES;c.stage3_business_transfer=1U;
 c.dm_deg_s=SPF_DM_DEG_S;c.extension_rpm=SPF_EXT_RPM;c.lift_rpm=1846.4f;
 c.stepper_accel_rpm_s=600U;c.keep_vm_on=1U;
 s3_parallel_x_started=VSP_Begin(&c,1U,0U,1U);
}
#endif
"""), (
        " if(g_lift_ruler_state>0U && g_lift_ruler_state<4U)g_lift_ruler_request=2U;",
        " if((g_lift_ruler_state>0U && g_lift_ruler_state<4U) || g_lift_ruler_request==3U)g_lift_ruler_request=2U;"), (
        "     it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=stage3_scope?SPF_STAGE3_EXT_RPM:",
        """     it_command.stage3_business_transfer=stage3_scope && g_stage3_state==5U && s3_pending==3U;
     it_command.dm_deg_s=SPF_DM_DEG_S;
     it_command.extension_rpm=it_command.stage3_business_transfer?SPF_EXT_RPM:stage3_scope?SPF_STAGE3_EXT_RPM:"""), (
        " case 1U:case 4U:\n  it_service_lift_command(now);break;",
        """ case 4U:
#if !APP_MOTOR_OFFLINE_ONLY
  if(Stage3_BusinessReturnActive()){S3_ServiceBusinessReturn((uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS);break;}
#endif
  it_service_lift_command(now);break;
 case 1U:
  it_service_lift_command(now);break;""")))
    changed.append(edit(project, "lift_ruler_adapter.c", (
        " && (g_vehicle_arm_state==0U || g_vehicle_arm_state==2U) &&",
        " && (g_vehicle_arm_state==0U || g_vehicle_arm_state==2U || Stage3_LrcSharesXReturn()) &&"), (
        """ if(request==3U && !(Stage3_Owns()?
  LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,now,ready,SPF_STAGE3_MAX_LIFT_PULSES):
  LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready)))g_lift_ruler_rejected=1U;""",
        """ if(request==3U && !(Stage3_BusinessReturnActive()?
  LiftRuler_StartBusinessReturn(&g_lift_ruler_controller,now,ready):
  (Stage3_Owns()?LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,now,ready,SPF_STAGE3_MAX_LIFT_PULSES):
  LiftRuler_StartReturn(&g_lift_ruler_controller,now,ready))))g_lift_ruler_rejected=1U;""")))
    changed.append(edit(project, "lift_ruler_calibration.h", (
        " uint32_t return_limit_pulses; /* Latched only for the explicit return. */",
        " uint32_t return_limit_pulses,stage3_business_return; /* Latched for the explicit return. */"), (
        "/* Explicit commissioning escape: confirmed empty at top; no origin created. */",
        """uint32_t LiftRuler_StartBusinessReturn(LiftRulerController*,uint32_t now,uint32_t ready);
/* Explicit commissioning escape: confirmed empty at top; no origin created. */""")))
    changed.append(edit(project, "lift_ruler_calibration.c", (
        " if(LiftRuler_Start(c,now,ready)){c->return_limit_pulses=limit;return 1U;}",
        " if(LiftRuler_Start(c,now,ready)){c->return_limit_pulses=limit;c->stage3_business_return=0U;return 1U;}"), (
        "uint32_t LiftRuler_StartRelease(LiftRulerController*c,uint32_t now,uint32_t ready)",
        """uint32_t LiftRuler_StartBusinessReturn(LiftRulerController*c,uint32_t now,uint32_t ready)
{
 if(!LiftRuler_StartReturnWithLimit(c,now,ready,SPF_STAGE3_MAX_LIFT_PULSES))return 0U;
 c->stage3_business_return=1U;return 1U;
}
uint32_t LiftRuler_StartRelease(LiftRulerController*c,uint32_t now,uint32_t ready)"""), (
        "   c->command_speed=REFERENCE_SPEED;c->command_accel=REFERENCE_ACCEL;",
        """   c->command_speed=REFERENCE_SPEED;
   c->command_accel=(c->operation==2U && c->stage3_business_return)?600U:REFERENCE_ACCEL;""")))
    return changed


def extract_function(text, signature):
    start = text.index(signature)
    body = text.index("{", start)
    depth = 0
    for i, ch in enumerate(text[body:], body):
        depth += (ch == "{") - (ch == "}")
        if depth == 0:
            return text[start:i + 1]
    raise ValueError(signature)


def run_fixture(project):
    user = project / "CHASSIS/USER"
    stage = (user / "stage3_cycle.inc").read_bytes().decode("gbk", errors="surrogateescape")
    backend = (user / "integration10_complete.inc").read_bytes().decode("gbk", errors="surrogateescape")
    selected = [(stage, n) for n in (
        "static uint32_t S3_Config(void)", "static void S3_LockConfig(void)",
        "static uint32_t S3_Complete(void)", "static void S3_ReturnStart(",
        "static void S3_ReturnX(", "static void S3_Axis(")] + [(backend, n) for n in (
        "static uint32_t S3_ReturnClearanceReached(", "static void S3_ServiceBusinessReturn(")]
    prefix = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
#define SOURCE_HOME_BACKOFF_PULSES 600U
static unsigned checks;
#define CHECK(e) do { ++checks; if(!(e)){fprintf(stderr,"line %d: %s\n",__LINE__,#e);return 1;} } while(0)
static uint32_t g_stage3_task_run,g_stage3_run=7,g_stage3_task_batch=2,g_stage3_storage_only;
static uint32_t g_stage3_test_first_item,g_stage3_test_count=3,g_stage3_item,g_stage3_state,g_stage3_phase;
static uint32_t g_stage3_placed_count,g_stage3_recovered_count;
static uint32_t g_stage3_colors[3]={2,1,5},g_stage3_positions[3]={1,2,3},g_stage3_slot_colors[3]={5,1,2};
static float g_stage3_pick_depth=29700,g_stage3_carrier_yaw=37;
static float g_stage3_target_yaw[3]={177,180,183},g_stage3_target_x[3]={10,0,15};
static float g_stage3_target_depth[3]={60100,60206,60300},g_stage3_recovery_depth[3]={-1,-1,-1};
static float g_stage3_place_clearance_depth[3]={48000,-1,-1},g_stage3_recovery_clearance_depth[3]={-1,-1,-1};
static uint32_t s3_first,s3_count,s3_colors[3],s3_positions[3],s3_slots[3];
static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3],s3_recovery_depth[3];
static float s3_place_clearance[3],s3_recovery_clearance[3],s3_return_clearance;
static uint32_t s3_full_batch,s3_round_trip,s3_parallel_x_started,s3_zero_correction_sent,s3_pending;
static uint32_t g_it_request,g_it_action,g_it_state,g_it_rejected,g_it_axes,g_it_sequence;
static float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses;
static float g_startup_extension_current_deg,g_extension_home_reference_deg;
static uint32_t g_lift_ruler_rejected,g_lift_ruler_samples,g_lift_ruler_state,it_lift_samples;
static uint32_t g_source_parallel_state,g_source_parallel_done_mask,g_low_wheel_hold_ready=1,g_dm_posvel_holding=1;
static uint32_t owns,top,fresh=1,busy,failed,finished,issued;
static LiftRulerController g_lift_ruler_controller;
static SPF_Command sent;
static void S3_Axis(uint32_t axis,float value);
static void S3_Command(uint32_t action){g_it_request=s3_pending=action;}
static void S3_Fail(uint32_t why){failed=why;}
static void it_fail(uint32_t why){failed=why;}
static void it_done(void){finished=1;}
static uint32_t it_at_top(void){return top;}
static uint32_t it_dm_fresh(void){return fresh;}
static uint32_t VehicleArm_Busy(void){return busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owns;}
static uint32_t prep(void *ctx,uint32_t bit,const SPF_Command *c){(void)ctx;(void)bit;(void)c;return 1;}
static uint32_t axis(void *ctx,uint32_t bit){(void)ctx;(void)bit;return 1;}
static uint32_t clock_ms(void *ctx){(void)ctx;return 1000;}
static uint32_t VSP_Begin(const SPF_Command *c,uint32_t sweep,uint32_t drive,uint32_t stage3){
 SPF_Group group={0};SPF_GroupIO io={0,prep,prep,axis,axis,clock_ms};
 assert(drive==0 && stage3==1);sent=*c;issued++;
 return SPF_GroupBegin(&group,c,&io,sweep,90,12000);
}
static uint32_t position_raw;
static uint32_t z_reply(void *ctx,const uint8_t *q,uint32_t n,uint32_t rn,uint8_t *r){
 (void)ctx;
 assert(n==2 && rn==7 && q[0]==0x36);
 r[0]=0x36;r[1]=0;r[2]=(uint8_t)(position_raw>>24);r[3]=(uint8_t)(position_raw>>16);
 r[4]=(uint8_t)(position_raw>>8);r[5]=(uint8_t)position_raw;r[6]=0x6b;return 1;
}
'''
    cases = r'''
static void sample(LiftRulerController *z){
 memset(z,0,sizeof(*z));z->state=LRC_MOVING;z->operation=2;z->phase=12;
 z->reuse_reference=z->top_valid=z->moves_sent=1;z->scale=z->top_scale=100;
 z->last_position_ms=1000;z->top_position_raw=1300;z->goal_raw=8050;
 z->position_after_raw=541300;z->command_before_raw=676300;
}
int main(void){
 LiftRulerController z;SPF_Group group={0};SPF_GroupIO io={0,prep,prep,axis,axis,clock_ms};
 g_stage3_test_first_item=1;g_stage3_test_count=1;
 g_stage3_slot_colors[0]=g_stage3_slot_colors[2]=0;
 CHECK(S3_Config());g_stage3_slot_colors[0]=1;CHECK(!S3_Config());g_stage3_slot_colors[0]=5;
 g_stage3_slot_colors[2]=2;g_stage3_test_first_item=0;g_stage3_test_count=3;
 g_stage3_storage_only=1;
 CHECK(!S3_Config());g_stage3_task_run=6;CHECK(!S3_Config());
 g_stage3_task_run=7;CHECK(S3_Config());S3_LockConfig();
 CHECK(s3_full_batch && !s3_round_trip && s3_count==3);
 g_stage3_storage_only=0;CHECK(!S3_Config());CHECK(!s3_round_trip);
 g_stage3_recovery_depth[0]=59400;g_stage3_recovery_depth[1]=61400;g_stage3_recovery_depth[2]=60600;
 CHECK(S3_Config());S3_LockConfig();CHECK(s3_full_batch && s3_round_trip);
 S3_ReturnStart(0,0);CHECK(s3_return_clearance==48000 && s3_pending==4);
 S3_ReturnStart(1,0);CHECK(s3_return_clearance<0);S3_ReturnStart(0,1);CHECK(s3_return_clearance<0);
 s3_place_clearance[0]=60100;S3_ReturnStart(0,0);CHECK(s3_return_clearance<0);
 s3_place_clearance[0]=48000;S3_ReturnStart(0,0);
 sample(&z);
 CHECK(S3_ReturnClearanceReached(&z,48000,1000));CHECK(!S3_ReturnClearanceReached(&z,47999,1000));
 CHECK(!S3_ReturnClearanceReached(&z,-1,1000));CHECK(!S3_ReturnClearanceReached(&z,48000,1400));
 z.reuse_reference=0;CHECK(!S3_ReturnClearanceReached(&z,48000,1000));z.reuse_reference=1;
 z.stop_pending=1;CHECK(!S3_ReturnClearanceReached(&z,48000,1000));z.stop_pending=0;
 z.scale=10;CHECK(!S3_ReturnClearanceReached(&z,48000,1000));z.scale=100;
 z.phase=11;CHECK(!S3_ReturnClearanceReached(&z,48000,1000));z.phase=12;
 z.moves_sent=0;CHECK(!S3_ReturnClearanceReached(&z,48000,1000));z.moves_sent=1;
 g_lift_ruler_controller=z;
 fresh=0;S3_ServiceBusinessReturn(1000);CHECK(!issued);fresh=1;
 S3_ServiceBusinessReturn(1000);CHECK(issued==1 && s3_parallel_x_started);
 CHECK(sent.extension_rpm==150 && sent.lift_rpm==1846.4f && sent.dm_deg_s==300 && sent.stepper_accel_rpm_s==600);
 S3_ServiceBusinessReturn(1000);CHECK(issued==1);
 sent.stage3_business_transfer=0;CHECK(!SPF_GroupBegin(&group,&sent,&io,1,90,12000));
 sent.extension_rpm=106;CHECK(SPF_GroupBegin(&group,&sent,&io,1,90,12000));
 memset(&group,0,sizeof(group));sent.lift_limit_pulses=0;sent.extension_rpm=150;
 CHECK(SPF_GroupBegin(&group,&sent,&io,1,90,12000));
 memset(&group,0,sizeof(group));sent.stage3_business_transfer=1;
 CHECK(!SPF_GroupBegin(&group,&sent,&io,1,90,12000));
 g_lift_ruler_samples=1;g_lift_ruler_state=LRC_DONE;top=1;
 S3_ServiceBusinessReturn(1010);CHECK(!finished);g_source_parallel_state=3;g_source_parallel_done_mask=SPF_EXT;
 owns=1;S3_ServiceBusinessReturn(1010);CHECK(!finished);owns=0;
 top=0;S3_ServiceBusinessReturn(1010);CHECK(!finished);top=1;
 S3_ServiceBusinessReturn(1010);CHECK(finished);
 g_it_request=0;g_stage3_phase=13;g_startup_extension_current_deg=3;
 S3_ReturnX();CHECK(!g_it_request && g_stage3_phase==14);
 s3_parallel_x_started=1;g_startup_extension_current_deg=3.1f;S3_ReturnX();
 CHECK(s3_zero_correction_sent && g_it_request==3 && g_it_axes==SPF_EXT);
 g_it_request=0;g_it_state=2;g_it_action=3;CHECK(!S3_Complete() && failed==6);
 failed=0;s3_zero_correction_sent=0;
 CHECK(!S3_Complete() && !failed && s3_zero_correction_sent && g_it_request==3);
 g_it_request=0;g_startup_extension_current_deg=3;CHECK(S3_Complete() && !s3_zero_correction_sent);
 s3_parallel_x_started=1;g_source_parallel_state=4;S3_ServiceBusinessReturn(1010);CHECK(failed==8);
 LiftRuler_Init(&z);z.state=LRC_DONE;z.top_valid=1;z.top_scale=z.scale=100;z.top_position_raw=1300;
 CHECK(LiftRuler_StartBusinessReturn(&z,1000,1));
 z.state=LRC_PREP;z.phase=7;z.prep_ms=1000;position_raw=676300;
 LiftRuler_Service(&z,1001,1,1,1,z_reply,0);
 CHECK(z.phase==8 && z.command_speed==18464 && z.command_accel==600 && z.goal_raw==8050);
 CHECK(z.move_frame[1]==1 && z.move_frame[2]==2 && z.move_frame[3]==88);
 z.state=LRC_DONE;CHECK(LiftRuler_StartReturnWithLimit(&z,2000,1,SPF_STAGE3_MAX_LIFT_PULSES));
 z.state=LRC_PREP;z.phase=7;z.prep_ms=2000;
 LiftRuler_Service(&z,2001,1,1,1,z_reply,0);
 CHECK(z.phase==8 && z.command_speed==18464 && z.command_accel==45055 && z.goal_raw==8050);
 z.state=LRC_DONE;CHECK(LiftRuler_StartBusinessReturn(&z,3000,1));
 z.state=LRC_PREP;z.phase=7;z.prep_ms=3000;z.backoff_active=1;position_raw=1300;
 LiftRuler_Service(&z,3001,1,1,1,z_reply,0);
 CHECK(z.phase==8 && z.command_speed==3000 && z.command_accel==600 && z.goal_raw==8050);
 puts("PASS actual Stage3 C guards, mode, return service and SPF_GroupBegin");
 printf("%u checks; simulated outcomes, no hardware\n",checks);
 return 0;
}
'''
    cfile = OUT / "actual-stage3-offline.c"
    binary = OUT / "actual-stage3-offline.exe"
    cfile.write_text(prefix + "\n".join(extract_function(text, signature) for text, signature in selected) +
                     cases, encoding="ascii")
    subprocess.run(["gcc", "-std=c99", "-Werror", "-I", str(user), str(cfile),
                    str(user / "source_parallel_flow.c"), str(user / "lift_ruler_calibration.c"),
                    "-o", str(binary)], check=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=30)
    (OUT / "actual-stage3-offline.log").write_text(result.stdout, encoding="ascii")
    print(result.stdout, end="")
    return {"log": str(OUT / "actual-stage3-offline.log"),
            "sha256": digest(OUT / "actual-stage3-offline.log")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--fixture", action="store_true")
    options = parser.parse_args()
    matlab = EVIDENCE / "stage3-charge-offline-matlab.log"
    if "PASS Stage3 offline ordering/feedback/stack arithmetic" not in matlab.read_text(encoding="utf-8"):
        raise RuntimeError("MATLAB prerequisite is missing")
    manifest = json.loads(BASE_MANIFEST.read_text(encoding="utf-8"))
    base = Path(manifest["project"]).resolve()
    project = OUT / "project"
    for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
        if digest(base / name) != expected:
            raise ValueError(f"Baseline source hash mismatch: {name}")
    if not project.exists():
        shutil.copytree(base, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
        changed = change_sources(project)
        for name in changed:
            manifest["unchanged_sha256"].pop(name, None)
            manifest["changes"][name] = digest(project / name)
        manifest.update(project=str(project), images={}, build_passed=False, flashed=False,
                        matlab_sha256=digest(matlab), baseline_manifest=str(BASE_MANIFEST),
                        capabilities={"stage3_storage_only": True, "stage3_clearance_depth": True,
                                      "stage3_single_center_single_slot": True})
        (OUT / "stage3-speed-offline-compatible.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    else:
        manifest = json.loads((OUT / "stage3-speed-offline-compatible.json").read_text(encoding="utf-8"))
        for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
            if digest(project / name) != expected:
                raise ValueError(f"Existing candidate hash mismatch: {name}")
    if options.fixture:
        manifest["c_fixture"] = run_fixture(project)
    if options.build:
        for target in TARGETS:
            for folder in ("Objects", "Listings"):
                (project / folder / target).mkdir(parents=True, exist_ok=True)
            log = OUT / f"{target}.log"
            result = subprocess.run(["C:/Keil_v5/UV4/UV4.exe", "-r", str(project / "gongxun-f407.uvprojx"),
                                     "-t", target, "-j0", "-o", str(log)], timeout=180)
            if result.returncode or b"0 Error(s), 0 Warning(s)" not in log.read_bytes():
                raise RuntimeError(f"Build failed: {log}")
            image = project / "Objects" / target / f"{target}.axf"
            manifest["images"][target] = {"axf": str(image), "sha256": digest(image)}
            print(f"{target}: 0 errors, 0 warnings", flush=True)
        manifest["build_passed"] = True
    if options.build or options.fixture:
        (OUT / "stage3-speed-offline-compatible.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(OUT / "stage3-speed-offline-compatible.json")


if __name__ == "__main__":
    main()
