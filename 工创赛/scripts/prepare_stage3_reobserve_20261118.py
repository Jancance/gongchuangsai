"""Isolated third-stage alignment restart and configurable observation angle."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import prepare_stage3_manual_release_20261118 as editing

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/stage3_manual_release_can6_20261118/project'
PROJECT=ROOT/'tmp/stage3_reobserve_20261118/project'
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'

def main():
    assert json.loads((E/'chassis-edge-reobserve-matlab.json').read_text())['passed']
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    editing.PROJECT=PROJECT
    name='CHASSIS/USER/stage3_cycle.inc'
    editing.edit(name,'volatile float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm;',
        'volatile float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm;\nvolatile float g_stage3_observe_yaw=180.0f;')
    old=' /* Explicit one-step calibration, using the same SBS executor and epoch fence.'
    new=''' /* Explicit bench re-observation; clears only the previous visual result.
  * Configured placement cannot start through this bench-only path. */
 if(request==31U){
  if(!g_region_dispatch_test_enable || g_stage3_state!=7U || g_stage3_config_ready ||
     g_stage3_manual_state || !S3_TeachReady() || !S3_Platform() ||
     s3_pending || !it_at_top() || !g_low_wheel_hold_ready ||
     !g_extension_home_reference_valid || !g_extension_hold_confirmed ||
     !g_dm_posvel_holding || g_dm_can2_motor_state!=1U ||
     !(g_stage3_observe_yaw>=0.0f && g_stage3_observe_yaw<=270.0f)){
   g_stage3_rejected++;return;
  }
  g_stage3_align_done=0U;
  g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0U;
  g_stage3_phase=0U;g_stage3_state=1U;S3_Pose();return;
 }
'''+old
    # S3_TeachReady checks the request field after Stage3_Service consumed it.
    editing.edit(name,old,new)
    editing.edit(name,'if(g_stage3_phase==5U){g_it_axes=SPF_DM;g_it_dm_deg=180.0f;}',
        'if(g_stage3_phase==5U){g_it_axes=SPF_DM;g_it_dm_deg=g_stage3_observe_yaw;}')
    manifest=json.loads((E/'manual-release-can6-compatible.json').read_text(encoding='utf-8'))
    manifest.update(project=str(PROJECT),changes={},unchanged_sha256={},images={},
                    stage3_reobserve_request=31,stage3_observe_yaw_parameter=True,flashed=False)
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):(PROJECT/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/('stage3-reobserve-'+target+'.log')
        run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(PROJECT/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        assert run.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes()
        axf=PROJECT/'Objects'/target/(target+'.axf')
        manifest['images'][target]={'axf':str(axf),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target,'0 errors, 0 warnings',flush=True)
    for path in PROJECT.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(PROJECT)
        if any(n in ('Objects','Listings') for n in rel.parts):continue
        before=BASE/rel
        key='unchanged_sha256' if before.is_file() and path.read_bytes()==before.read_bytes() else 'changes'
        manifest[key][rel.as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    (E/'stage3-reobserve-compatible.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')

if __name__=='__main__':main()
