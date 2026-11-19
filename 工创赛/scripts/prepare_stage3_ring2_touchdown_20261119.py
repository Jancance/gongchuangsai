"""Build the isolated node-5 bounded bench touchdown candidate; never flash."""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from prepare_stage3_speed_offline_20261119 import ROOT, EVIDENCE, TARGETS, digest, extract_function

OUT = ROOT / "tmp/stage3_ring2_touchdown_v3_20261119"
BASE = EVIDENCE / "stage3-beep-turn3-compatible.json"
MATLAB = EVIDENCE / "stage3-ring2-touchdown-matlab.log"


def edit(project, name, *patches):
    relative = "CHASSIS/USER/" + name
    path = project / relative
    data = path.read_bytes()
    for old, new in patches:
        pattern = rb"\r?\n".join(re.escape(line.encode("ascii")) for line in old.split("\n"))
        hits = list(re.finditer(pattern, data))
        if len(hits) != 1:
            raise ValueError(f"{name}: anchor count {len(hits)} for {old[:90]!r}")
        hit = hits[0]
        eol = re.search(rb"\r?\n", hit.group() + data[hit.end():])
        newline = eol.group() if eol else b"\r\n"
        data = data[:hit.start()] + new.encode("ascii").replace(b"\n", newline) + data[hit.end():]
    path.write_bytes(data)
    return relative


def patch_sources(project):
    return [
        edit(project, "source_parallel_flow.h", (
            " SPF_CMD_ROUTE_AND_STOW,SPF_CMD_STOP };",
            """ SPF_CMD_ROUTE_AND_STOW,SPF_CMD_STOP,SPF_CMD_STAGE3_TOUCHDOWN };
#define SPF_STAGE3_TOUCHDOWN_RPM 30.0f
#define SPF_STAGE3_TOUCHDOWN_MAX_PULSES 1200U""")),
        edit(project, "source_parallel_flow.c", (
            "  c->lift_rpm!=SPF_LIFT_RPM || c->stepper_accel_rpm_s!=SPF_STEPPER_ACCEL ||",
            """  (c->kind==SPF_CMD_STAGE3_TOUCHDOWN && (c->axes_mask!=SPF_LIFT ||
   c->lift_limit_pulses!=SPF_STAGE3_MAX_LIFT_PULSES || !(c->goal.lift_pulses>=3200.0f))) ||
  c->lift_rpm!=(c->kind==SPF_CMD_STAGE3_TOUCHDOWN?SPF_STAGE3_TOUCHDOWN_RPM:SPF_LIFT_RPM) ||
  c->stepper_accel_rpm_s!=SPF_STEPPER_ACCEL ||""")),
        edit(project, "vehicle_source_parallel.inc", (
            "static uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)",
            """static uint32_t S3_TouchdownAllowed(void);
static uint32_t VSP_TouchdownDelta(float delta,float down_sign,uint32_t scale)
{
 float down=delta*down_sign;
 float max_deg=(float)SPF_STAGE3_TOUCHDOWN_MAX_PULSES*360.0f/3200.0f;
 /* The two driver scales are fixed by node-5 status. Reject rounding to zero. */
 return (scale==10U || scale==100U) && down>0.0f && down<=max_deg &&
  down*(float)scale>=0.5f;
}
static uint32_t VSP_Prepare(void *context,uint32_t bit,const SPF_Command *c)"""), (
            "  if(vsp_fast_lift && address==5U &&",
            "  if((vsp_fast_lift || c->kind==SPF_CMD_STAGE3_TOUCHDOWN) && address==5U &&"), (
            "  delta=a->goal-a->current;",
            """  delta=a->goal-a->current;
  if(c->kind==SPF_CMD_STAGE3_TOUCHDOWN &&
     !VSP_TouchdownDelta(delta,VEHICLE_LIFT_DOWN_SIGN,(a->options&0x80U)?100U:10U))return 2;"""), (
            "static uint32_t VSP_Poll(void *context,uint32_t bit)",
            """static uint32_t VSP_TouchdownArrived(float before,float current,const uint8_t *move,uint32_t scale)
{
 uint32_t units=((uint32_t)move[8]<<24)|((uint32_t)move[9]<<16)|((uint32_t)move[10]<<8)|move[11];
 float moved=(current-before)*VEHICLE_LIFT_DOWN_SIGN;
 return units && moved>0.0f && fabsf(moved*(float)scale-(float)units)<=0.5f;
}
static uint32_t VSP_Poll(void *context,uint32_t bit)"""), (
            " if(vsp_pick_sync)return VSP_StartQueuedPick(bit,a,address);\n if(fabsf(a->goal-a->before)<=.2f)return 1;",
            """ if(c->kind==SPF_CMD_STAGE3_TOUCHDOWN){
  /* The relative frame was prepared from this origin. Read it again before
   * start and do not transmit if it changed or a STOP arrived in the query. */
  if(address!=5U || !S3_TouchdownAllowed() || !VSP_Position(address,a) ||
     a->current!=a->before ||
     !VSP_TouchdownDelta(a->goal-a->current,VEHICLE_LIFT_DOWN_SIGN,(a->options&0x80U)?100U:10U) ||
     !S3_TouchdownAllowed())return 0U;
 }else{
  if(vsp_pick_sync)return VSP_StartQueuedPick(bit,a,address);
  if(fabsf(a->goal-a->before)<=.2f)return 1;
 }"""), (
            " if(!VSP_Query(address,0x35,5) || (zero_return && s_rx.Data[1]>1U)){",
            " if(!VSP_Query(address,0x35,5) || ((zero_return || vsp_group.command.kind==SPF_CMD_STAGE3_TOUCHDOWN) && s_rx.Data[1]>1U)){"), (
            """    fabsf(a->current-a->goal)<=(zero_return?SOURCE_EXTENSION_ZERO_DONE_DEG:(address==6U?SOURCE_EXTENSION_DONE_DEG:22.5f)) &&
  (fabsf(a->goal-a->before)<=.2f || (a->flags&2)))a->samples++;else a->samples=0;""",
            """    (vsp_group.command.kind==SPF_CMD_STAGE3_TOUCHDOWN?
     ((a->flags&2) && VSP_TouchdownArrived(a->before,a->current,a->move,(a->options&0x80U)?100U:10U)):
     (fabsf(a->current-a->goal)<=(zero_return?SOURCE_EXTENSION_ZERO_DONE_DEG:(address==6U?SOURCE_EXTENSION_DONE_DEG:22.5f)) &&
      (fabsf(a->goal-a->before)<=.2f || (a->flags&2)))))a->samples++;else a->samples=0;""")),
        edit(project, "stage3_cycle.inc", (
            " else if(g_stage3_state==7U)g_stage3_request=2U;",
            """ else if(g_stage3_state==7U){
  if(g_it_request==18U || (g_it_action==18U && g_it_state==1U))g_stage3_request=9U;
  else g_stage3_request=2U;
 }""")),
        edit(project, "integration10_complete.inc", (
            '#include "region_dispatch.inc"\nvoid Integration10_Service(void)',
            """#include "region_dispatch.inc"
static uint32_t S3_TouchdownAllowed(void)
{
 return Stage3_Owns() && g_region_dispatch_test_enable && g_stage3_state==7U &&
  g_stage3_align_done && g_low_wheel_hold_ready && !g_stage3_fault &&
  !g_stage3_manual_state && !g_stage3_manual_request && !s3_pending &&
  !g_stage3_request && g_it_request!=9U && GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) &&
  !g_source_body_step_request &&
  !SourceBodyStep_Busy() && !SourceBodyStep_WheelWindow() && !g_source_align_chain_active &&
  g_lift_ruler_request!=LRC_REQUEST_STOP && !g_lift_ruler_controller.stop_pending &&
  g_lift_ruler_controller.top_scale && SingleTeach_ReferencesReady() && it_platform() &&
  !g_dm_jog_first_fault && !g_dm_jog_request && g_dm_posvel_holding;
}
void Integration10_Service(void)"""), (
            "(request>15U && !(request==17U && Stage3_Owns()))",
            "(request>15U && !((request==17U || request==18U) && Stage3_Owns()))"), (
            "   }else if(request==3U){\n    uint32_t axes=g_it_axes;",
            """   }else if(request==3U || request==18U){
    uint32_t touchdown=request==18U;
    uint32_t axes=g_it_axes;"""), (
            "    if(!SingleTeach_ReferencesReady() || !axes || (axes&~7U) ||",
            "    if((touchdown && (axes!=SPF_LIFT || !S3_TouchdownAllowed())) ||\n       !SingleTeach_ReferencesReady() || !axes || (axes&~7U) ||"), (
            "     it_command.kind=SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;",
            "     it_command.kind=touchdown?SPF_CMD_STAGE3_TOUCHDOWN:SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;"), (
            ";it_command.lift_rpm=1846.4f;\n     /* 2026-11-08:",
            ";it_command.lift_rpm=touchdown?SPF_STAGE3_TOUCHDOWN_RPM:1846.4f;\n     /* 2026-11-08:"), (
            " case 3U:\n  if(it_phase==1U){",
            " case 3U:case 18U:\n  if(it_phase==1U){"), (
            "   if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}",
            """   if(g_it_action==18U && !S3_TouchdownAllowed()){it_fail(2U);return;}
   if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U)){it_fail(7U);return;}""")),
    ]


def check_fixture(project):
    user = project / "CHASSIS/USER"
    vsp = (user / "vehicle_source_parallel.inc").read_bytes().decode("gbk", errors="surrogateescape")
    source = r'''
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "source_parallel_flow.h"
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);return 1;} } while(0)
#define VEHICLE_LIFT_DOWN_SIGN (1.0f)
typedef struct {float goal,before,current;uint32_t options,phase;uint8_t move[15];} VSP_Axis;
typedef struct {uint8_t Data[8];} CanRxMsg;
static VSP_Axis vsp_axis[2];
static CanRxMsg s_rx;
static uint32_t g_dm_stage2_sync,g_dm_parallel_group_go,vsp_pick_sync,stopped,reads,frames,query_ok=1,stop_in_query;
static float position;
static uint32_t S3_TouchdownAllowed(void){return !stopped;}
static uint32_t VSP_Position(uint8_t address,VSP_Axis *a){
 if(address!=5)return 0;
 ++reads;if(stop_in_query)stopped=1;
 a->current=position;return query_ok;
}
static uint32_t VSP_StartQueuedPick(uint32_t bit,VSP_Axis *a,uint8_t address){
 (void)bit;(void)a;(void)address;return 0;
}
static uint32_t VehicleArm_Exchange(uint8_t addr,uint8_t *q,uint8_t n,uint8_t rn,CanRxMsg *reply){
 if(addr!=5 || q[0]!=0xfd || n!=15 || rn!=3)return 0;
 ++frames;reply->Data[1]=2;return 1;
}
'''
    source += extract_function(vsp, "static uint32_t VSP_TouchdownDelta(") + "\n"
    # Use the actual start path; the earlier helper has a longer name.
    source += extract_function(vsp, "static uint32_t VSP_Start(") + "\n"
    source += extract_function(vsp, "static uint32_t VSP_TouchdownArrived(") + "\n"
    source += r'''
static uint32_t prep(void *ctx,uint32_t bit,const SPF_Command *c){(void)ctx;(void)bit;(void)c;return 1;}
static uint32_t op(void *ctx,uint32_t bit){(void)ctx;(void)bit;return 1;}
static uint32_t stamp(void *ctx){(void)ctx;return 1;}
int main(void){
 SPF_Group g={0};
 SPF_GroupIO io={0,prep,VSP_Start,op,op,stamp};
 SPF_Command c={0};
 uint32_t scale,n;
 c.kind=SPF_CMD_STAGE3_TOUCHDOWN;c.axes_mask=SPF_LIFT;c.goal.lift_pulses=61406;
 c.lift_limit_pulses=SPF_STAGE3_MAX_LIFT_PULSES;c.keep_vm_on=1;
 c.lift_rpm=30;c.dm_deg_s=300;c.extension_rpm=106;c.stepper_accel_rpm_s=600;
 CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
 memset(&g,0,sizeof(g));c.axes_mask=SPF_EXT|SPF_LIFT;
 CHECK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));c.axes_mask=SPF_LIFT;
 c.lift_rpm=1846.4f;CHECK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));
 c.kind=SPF_CMD_TRANSFER;CHECK(SPF_GroupBegin(&g,&c,&io,1,90,12000));
 memset(&g,0,sizeof(g));c.lift_rpm=30;CHECK(!SPF_GroupBegin(&g,&c,&io,1,90,12000));
 c.kind=SPF_CMD_STAGE3_TOUCHDOWN;
 CHECK(VSP_TouchdownDelta(135,1,10));CHECK(VSP_TouchdownDelta(135,1,100));
 CHECK(!VSP_TouchdownDelta(135.01f,1,100));CHECK(!VSP_TouchdownDelta(0,1,10));
 CHECK(!VSP_TouchdownDelta(-1,1,10));CHECK(!VSP_TouchdownDelta(0.004f,1,100));
 vsp_axis[1].before=position=6773.175f;vsp_axis[1].goal=position+135.0f;vsp_axis[1].move[0]=0xfd;
 stopped=1;CHECK(!VSP_Start(0,SPF_LIFT,&c) && !reads && !frames);stopped=0;
 stop_in_query=1;CHECK(!VSP_Start(0,SPF_LIFT,&c) && reads==1 && !frames);
 stop_in_query=0;stopped=0;position+=0.1f;
 CHECK(!VSP_Start(0,SPF_LIFT,&c) && !frames);position=vsp_axis[1].before;
 query_ok=0;CHECK(!VSP_Start(0,SPF_LIFT,&c) && !frames);query_ok=1;
 CHECK(VSP_Start(0,SPF_LIFT,&c) && frames==1);
 for(scale=10U;scale<=100U;scale*=10U)for(n=1U;n<=48U;n++){
  static const float origin=6773.2f;
  uint8_t move[15]={0};move[11]=(uint8_t)n;
  CHECK(!VSP_TouchdownArrived(origin,origin,move,scale));
  CHECK(!VSP_TouchdownArrived(origin,origin-1.0f/(float)scale,move,scale));
  CHECK(!VSP_TouchdownArrived(origin,origin+(float)(n-1U)/(float)scale,move,scale));
  CHECK(VSP_TouchdownArrived(origin,origin+(float)n/(float)scale,move,scale));
  CHECK(!VSP_TouchdownArrived(origin,origin+(float)(n+1U)/(float)scale,move,scale));
 }
 printf("PASS actual bounded node-5 prepare/start predicates and SPF admission: %u checks; no hardware\n",checks);
 return 0;
}
'''
    cfile, binary = OUT / "actual-touchdown.c", OUT / "actual-touchdown.exe"
    cfile.write_text(source, encoding="ascii")
    subprocess.run(["gcc", "-std=c99", "-Werror", "-I", str(user), str(cfile),
                    str(user / "source_parallel_flow.c"), "-o", str(binary)], check=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=30)
    log = OUT / "actual-touchdown.log"
    log.write_text(result.stdout, encoding="ascii")
    print(result.stdout, end="")
    return {"log": str(log), "sha256": digest(log)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    if "720 model cases" not in MATLAB.read_text(encoding="utf-8"):
        raise RuntimeError("MATLAB prerequisite missing")
    manifest = json.loads(BASE.read_text(encoding="utf-8"))
    source = Path(manifest["project"]).resolve()
    project = OUT / "project"
    result_path = OUT / "stage3-ring2-touchdown-compatible.json"
    if not project.exists():
        for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
            if digest(source / name) != expected:
                raise ValueError(f"Baseline source hash mismatch: {name}")
        shutil.copytree(source, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
        for name in patch_sources(project):
            manifest["unchanged_sha256"].pop(name, None)
            manifest["changes"][name] = digest(project / name)
        manifest.update(project=str(project), images={}, build_passed=False, flashed=False,
                        matlab_sha256=digest(MATLAB), baseline_manifest=str(BASE),
                        capabilities={"stage3_ring2_touchdown": True})
        result_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    else:
        manifest = json.loads(result_path.read_text(encoding="utf-8"))
        for name, expected in (manifest["unchanged_sha256"] | manifest["changes"]).items():
            if digest(project / name) != expected:
                raise ValueError(f"Candidate hash mismatch: {name}")
    manifest["c_fixture"] = check_fixture(project)
    if args.build:
        for target in TARGETS:
            for directory in ("Objects", "Listings"):
                (project / directory / target).mkdir(parents=True, exist_ok=True)
            log = OUT / f"{target}.log"
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
