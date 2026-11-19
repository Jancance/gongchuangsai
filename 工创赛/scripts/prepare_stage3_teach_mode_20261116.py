"""Explicit Stage3 teach owner after MATLAB topology gate. No hardware."""
from pathlib import Path
import json,shutil
R=Path(__file__).resolve().parents[1]
BASE=R/'tmp/stage3_deep_z_20261116/project'
P=R/'tmp/stage3_teach_mode_20261116/project'
E=R/'evidence/integration/2026-11-16/stage3-teach-mode'
S=R/'src/stage3_teach_mode_20261116'
def main():
 proof=json.loads((E/'matlab-result.json').read_text());assert proof['passed'] and len(proof['checks'])==4
 assert not P.exists(),'Preserve previous candidate'
 shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
 name='CHASSIS/USER/stage3_cycle.inc';b=(P/name).read_bytes()
 pairs=[
 (" if((request==20U || request==21U) && g_stage3_state==2U){", """ if(request==30U){
  if(g_stage3_state!=2U || !S3_Platform() || s3_pending || g_it_request ||
     SourceBodyStep_Busy() || g_source_body_step_request){g_stage3_rejected++;return;}
  /* Explicit manual calibration is not a successful visual alignment. */
  g_stage3_align_done=0U;S3_Pose();
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
  g_stage3_state=4U;
  if(g_source_align_chain_active)g_source_body_step_request=7U;
  return;
 }
 if((request==20U || request==21U) && g_stage3_state==2U){"""),
 ("  if(g_stage3_phase==4U){g_it_axes=SPF_DM;g_it_dm_deg=180.0f;g_it_extension_mm=0.0f;}",
  "  if(g_stage3_phase==4U){g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;}"),
 ("  if(g_stage3_phase==5U){g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;}",
  "  if(g_stage3_phase==5U){g_it_axes=SPF_DM;g_it_dm_deg=180.0f;}"),
 ("  if(g_stage3_config_ready)S3_LockConfig();else g_stage3_state=7U;return;",
  "  if(g_stage3_align_done && g_stage3_config_ready)S3_LockConfig();else g_stage3_state=7U;return;"),
 ("  if(request==2U && g_stage3_config_ready && g_low_wheel_hold_ready && it_at_top())S3_LockConfig();",
 """  if(request==2U && !g_stage3_align_done){
   if(!S3_Platform() || s3_pending || g_it_request || SourceBodyStep_Busy() ||
      g_source_body_step_request){g_stage3_rejected++;return;}
   g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
   g_stage3_phase=0U;g_stage3_state=1U;S3_Pose();return;
  }
  if(request==2U && g_stage3_align_done && g_stage3_config_ready && g_low_wheel_hold_ready && it_at_top())S3_LockConfig();""")]
 for old,new in pairs:
  a=old.replace('\n','\r\n').encode('ascii');z=new.replace('\n','\r\n').encode('ascii')
  assert b.count(a)==1,(old,b.count(a));b=b.replace(a,z)
 (P/name).write_bytes(b);(S/name).parent.mkdir(parents=True,exist_ok=True);(S/name).write_bytes(b)
 print('Prepared Stage3 teach candidate; only stage3_cycle.inc changed from deep-Z')
if __name__=='__main__':main()
