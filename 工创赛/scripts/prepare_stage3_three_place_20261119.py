"""Build the isolated three-placement image from the 11 mm replay baseline."""
import json
from pathlib import Path
import shutil
import subprocess

from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import EVIDENCE, ROOT, TARGETS, digest


BASE = ROOT / "tmp/stage3_ring2_replay_forward11_20261119/stage3-ring2-replay-compatible.json"
OUT = ROOT / "tmp/stage3_three_place_dm150_20261119"
MATLAB = EVIDENCE / "stage3-three-place-matlab.log"


def patch(project):
    edit(project, "source_parallel_flow.h", (
        "#define SPF_DM_DEG_S 300.0f",
        "#define SPF_DM_DEG_S 300.0f\n#define SPF_STAGE3_DM_DEG_S (SPF_DM_DEG_S / 2.0f)"), (
        " SPF_CMD_ROUTE_AND_STOW,SPF_CMD_STOP,SPF_CMD_STAGE3_TOUCHDOWN };",
        " SPF_CMD_ROUTE_AND_STOW,SPF_CMD_STOP,SPF_CMD_STAGE3_TOUCHDOWN,SPF_CMD_STAGE3_FAST_TRANSFER };"))
    edit(project, "source_parallel_flow.c", (
        "  c->dm_deg_s!=SPF_DM_DEG_S ||",
        """  (c->dm_deg_s!=SPF_DM_DEG_S &&
   !(c->kind==SPF_CMD_TRANSFER && c->axes_mask==SPF_DM &&
     c->lift_limit_pulses==SPF_STAGE3_MAX_LIFT_PULSES && c->dm_deg_s==SPF_STAGE3_DM_DEG_S)) ||"""), (
        "  c->extension_rpm!=(c->lift_limit_pulses?SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||",
        """  c->extension_rpm!=((c->lift_limit_pulses && c->kind!=SPF_CMD_STAGE3_FAST_TRANSFER)?
     SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||
  (c->kind==SPF_CMD_STAGE3_FAST_TRANSFER &&
   (c->axes_mask!=(SPF_EXT|SPF_LIFT) || c->lift_limit_pulses!=SPF_STAGE3_MAX_LIFT_PULSES)) ||"""))
    edit(project, "stage3_cycle.inc", (
        "#define S3_SYNC_PLACE_PHASE 9U",
        "#define S3_SYNC_PLACE_PHASE 9U\n#define S3_SYNC_RETURN_PHASE 12U"), (
        "void Stage3_KeyEvent(void)",
        """uint32_t Stage3_FastTransferStopPending(void)
{
 extern volatile uint32_t g_vehicle_key;
 return !Stage3_Owns() || g_stage3_state!=5U || g_stage3_fault ||
     g_stage3_request==9U || g_it_request==9U ||
     g_vehicle_key || !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
}
void Stage3_KeyEvent(void)"""), (
        "    s3_pending!=3U || s3_preclose_started)return 1U;",
        "    s3_pending!=3U || s3_preclose_started || s3_round_trip)return 1U;"), (
        "  if(g_stage3_task_run && !(g_stage3_recovery_depth[j]>=3200.0f &&\n      g_stage3_recovery_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;",
        "  /* Placement-only mission has no ring recovery targets. */"), (
        "  g_stage3_phase==S3_SYNC_PLACE_PHASE && s3_pending==3U &&",
        "  (g_stage3_phase==S3_SYNC_PLACE_PHASE || g_stage3_phase==S3_SYNC_RETURN_PHASE) && s3_pending==3U &&"), (
        " g_stage3_align_done=1U;g_stage3_state=7U;g_stage3_replay_state=5U;S3_Pose();",
        """ g_stage3_align_done=1U;g_stage3_state=7U;g_stage3_replay_state=5U;S3_Pose();
 if(g_stage3_task_run==g_stage3_run && g_stage3_config_ready)S3_LockConfig();"""), (
        " if(g_stage3_replay_state==5U && request){",
        " if(g_stage3_replay_state==5U && request &&\n    !(request==2U && g_stage3_state==7U && g_stage3_task_run && g_stage3_config_ready)){"), (
        "  if((p==1U || p==7U || p==12U) && it_at_top()){g_stage3_phase++;return;}",
        "  if((p==1U || p==7U || (p==12U && !s3_round_trip)) && it_at_top()){g_stage3_phase++;return;}"), (
        "  case 2U:S3_Axis(SPF_EXT,0.0f);break;",
        """  case 2U:
   if(s3_round_trip){
    if(g_stage3_item==s3_first)S3_Command(17U);
    else g_stage3_phase++;
   }else S3_Axis(SPF_EXT,0.0f);
   break;"""), (
        "  case 4U:if(s3_preclose_started && (uint32_t)(now-s3_preclose_tick)>=600U)g_stage3_phase++;break; /* DM done plus original settle. */",
        """  case 4U:
   if(s3_round_trip){g_stage3_phase=5U;S3_Axis(SPF_LIFT,s3_pick);}
   else if(s3_preclose_started && (uint32_t)(now-s3_preclose_tick)>=600U)g_stage3_phase++;
   break;"""), (
        "  case 11U:S3_Command(6U);break;\n  case 12U:S3_Command(4U);break;\n  case 13U:S3_Axis(SPF_EXT,0.0f);break;",
        """  case 11U:S3_Command(s3_round_trip?17U:6U);break; /* Place-only release uses the 160-degree command. */
  case S3_SYNC_RETURN_PHASE:
   if(!s3_round_trip){S3_Command(4U);break;}
   g_it_axes=SPF_EXT|SPF_LIFT;
   g_it_extension_mm=0.0f;
   /* The existing lift controller retains the calibrated safe top target. */
   if(!g_lift_ruler_controller.top_scale){S3_Fail(12U);break;}
   g_it_lift_pulses=((float)(g_lift_ruler_controller.safe_position_raw-
       g_lift_ruler_controller.top_position_raw)/(float)g_lift_ruler_controller.top_scale)*
       VEHICLE_LIFT_DOWN_SIGN*(3200.0f/360.0f);
   S3_Command(3U);
   break;
  case 13U:
   if(s3_round_trip)S3_Command(17U);
   else S3_Axis(SPF_EXT,0.0f);
   break;"""), (
        "   if(s3_round_trip){g_stage3_item=0U;g_stage3_phase=S3_RECOVER_FIRST_PHASE;return;}",
        "   if(s3_round_trip){g_stage3_state=6U;S3_Pose();return;}"))

    edit(project, "integration10_complete.inc", (
        "    uint32_t stage3_sync=S3_SyncPlaceOwned() && axes==(SPF_EXT|SPF_LIFT);",
        """    uint32_t stage3_sync=S3_SyncPlaceOwned() && axes==(SPF_EXT|SPF_LIFT);
    uint32_t stage3_return=stage3_sync && g_stage3_phase==S3_SYNC_RETURN_PHASE;"""), (
        "       ((axes&3U) && ((!it_at_top() && !Low_XYAllowed()) || ((axes&4U) && !stage3_sync))) ||\n       ((axes&4U) && g_it_lift_pulses<3200) ||",
        """       ((axes&3U) && ((!it_at_top() && !Low_XYAllowed() && !stage3_return) ||
                     ((axes&4U) && !stage3_sync))) ||
       ((axes&4U) && g_it_lift_pulses<3200 && !stage3_return) ||"""), (
        "     it_command.kind=touchdown?SPF_CMD_STAGE3_TOUCHDOWN:SPF_CMD_TRANSFER;it_command.sequence=++g_it_sequence;",
        """     it_command.kind=touchdown?SPF_CMD_STAGE3_TOUCHDOWN:
                      (stage3_sync?SPF_CMD_STAGE3_FAST_TRANSFER:SPF_CMD_TRANSFER);
     it_command.sequence=++g_it_sequence;"""), (
        "     it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=stage3_scope?SPF_STAGE3_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM);it_command.lift_rpm=touchdown?SPF_STAGE3_TOUCHDOWN_RPM:1846.4f;",
        """     it_command.dm_deg_s=(stage3_scope && axes==SPF_DM)?SPF_STAGE3_DM_DEG_S:SPF_DM_DEG_S;
     it_command.extension_rpm=stage3_sync?SPF_EXT_RPM:
         (stage3_scope?SPF_STAGE3_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM));
     it_command.lift_rpm=touchdown?SPF_STAGE3_TOUCHDOWN_RPM:SPF_LIFT_RPM;"""), (
        "    g_lift_ruler_controller.backoff_valid=0U;g_lift_ruler_controller.operation=2U;g_lift_ruler_controller.samples++;",
        """    if(it_command.kind==SPF_CMD_STAGE3_FAST_TRANSFER && g_stage3_phase==S3_SYNC_RETURN_PHASE){
     if(VehicleSourceParallel_Owns() ||
        (g_source_parallel_done_mask&(SPF_EXT|SPF_LIFT))!=(SPF_EXT|SPF_LIFT) ||
        fabsf(vsp_axis[1].current-(float)g_lift_ruler_controller.safe_position_raw/
              (float)g_lift_ruler_controller.top_scale)>SOURCE_TOP_TOLERANCE_DEG){
      it_fail(8U);return;
     }
     g_lift_ruler_controller.backoff_valid=1U;
     g_lift_ruler_controller.operation=3U;
    }else{
     g_lift_ruler_controller.backoff_valid=0U;
     g_lift_ruler_controller.operation=2U;
    }
    g_lift_ruler_controller.samples++;"""))
    edit(project, "vehicle_source_parallel.inc", (
        "static uint32_t VSP_StartQueuedPick(uint32_t bit,VSP_Axis *a,uint8_t address)",
        """extern uint32_t Stage3_FastTransferStopPending(void);
static uint32_t VSP_StartQueuedPick(uint32_t bit,VSP_Axis *a,uint8_t address)"""), (
        " a->move[13]=1U;\n r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);",
        """ if(vsp_group.command.kind==SPF_CMD_STAGE3_FAST_TRANSFER && Stage3_FastTransferStopPending())return 0U;
 a->move[13]=1U;
 r=VehicleArm_Exchange(address,a->move,15U,3U,&s_rx);"""), (
        "  if(CAN1_StartQueuedSourceAxes()!=1U)return 0U;",
        """  if(vsp_group.command.kind==SPF_CMD_STAGE3_FAST_TRANSFER && Stage3_FastTransferStopPending())return 0U;
  if(CAN1_StartQueuedSourceAxes()!=1U)return 0U;"""))

def main():
    assert "PASS Stage3 three-place event ordering and height conversion: 2430 cases" in MATLAB.read_text()
    assert "PASS: 150 deg/s" in (EVIDENCE / "stage3-dm150-matlab.log").read_text()
    manifest = json.loads(BASE.read_text(encoding="utf-8"))
    base = Path(manifest["project"])
    project = OUT / "project"
    for name, expected in (manifest["changes"] | manifest["unchanged_sha256"]).items():
        assert digest(base / name) == expected, name
    shutil.copytree(base, project, ignore=shutil.ignore_patterns("Objects", "Listings"))
    patch(project)
    for filename in ("stage3_cycle.inc", "integration10_complete.inc", "source_parallel_flow.h",
                     "source_parallel_flow.c", "vehicle_source_parallel.inc"):
        name = "CHASSIS/USER/" + filename
        manifest["unchanged_sha256"].pop(name, None)
        manifest["changes"][name] = digest(project / name)
    manifest.update(project=str(project), images={}, flashed=False, build_passed=False,
                    baseline_manifest=str(BASE), matlab_sha256=digest(MATLAB))
    manifest["capabilities"]["stage3_three_place"] = True
    manifest["three_place_release_angle_deg"] = 160
    manifest["stage3_dm_speed_deg_s"] = 150
    for target in TARGETS:
        for folder in ("Objects", "Listings"):
            (project / folder / target).mkdir(parents=True, exist_ok=True)
        log = OUT / f"{target}.log"
        result = subprocess.run(["C:/Keil_v5/UV4/UV4.exe", "-r", str(project / "gongxun-f407.uvprojx"),
                                 "-t", target, "-j0", "-o", str(log)], timeout=180)
        if result.returncode or b"0 Error(s), 0 Warning(s)" not in log.read_bytes():
            raise RuntimeError(f"Build failed: {log}")
        axf = project / "Objects" / target / f"{target}.axf"
        manifest["images"][target] = {"axf": str(axf), "sha256": digest(axf)}
        print(f"{target}: 0 errors, 0 warnings", flush=True)
    manifest["build_passed"] = True
    (OUT / "stage3-three-place-compatible.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
