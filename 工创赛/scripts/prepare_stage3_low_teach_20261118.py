"""Add explicit low-position bench teaching and return-before-rehome exit."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import prepare_stage3_manual_release_20261118 as editing

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/stage3_observe180p55_20261118/project'
PROJECT=ROOT/'tmp/stage3_low_teach_20261118/project'
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'

def main():
    assert json.loads((E/'low-teach-matlab.json').read_text())['passed']
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    editing.PROJECT=PROJECT
    source='CHASSIS/USER/stage3_cycle.inc'
    editing.edit(source,'  if(request!=1U || !g_region_dispatch_test_enable || g_stage3_state!=7U ||\n     !S3_TeachReady() || !it_platform() || !it_at_top() || !g_low_wheel_hold_ready ||',
        '''  if((request!=1U && request!=3U) || !g_region_dispatch_test_enable || g_stage3_state!=7U ||
     !S3_TeachReady() || !it_platform() ||
     (request==1U && !it_at_top()) ||
     (request==3U && (g_lift_ruler_state!=LRC_DONE || !g_vehicle_lift_feedback_valid ||
                     !g_lift_ruler_controller.top_valid || !g_lift_ruler_controller.top_scale ||
                     !(g_stage3_actual_depth>=3200.0f && g_stage3_actual_depth<=(float)SPF_STAGE3_MAX_LIFT_PULSES))) ||
     !g_low_wheel_hold_ready ||''')
    editing.edit(source,' }else if(g_stage3_manual_state==3U && request==2U){\n  g_stage3_manual_state=4U;\n  g_stage3_manual_x=3U;',
        ''' }else if(g_stage3_manual_state==3U && request==2U){
  if(it_at_top()){
   g_stage3_manual_state=4U;g_stage3_manual_x=3U;
  }else{
   /* Only the existing powered lift-return operation may run during exit. */
   g_stage3_manual_state=6U;g_it_request=4U;return 0U;
  }
 }else if(g_stage3_manual_state==6U){
  if(g_it_state==9U || g_it_cause || g_it_rejected || g_lift_ruler_cause){
   g_stage3_manual_state=5U;return 1U;
  }
  if(g_it_state==2U && g_it_action==4U && !g_it_request && it_at_top()){
   g_stage3_manual_state=4U;g_stage3_manual_x=3U;
  }else{
   if(g_it_request!=4U)g_it_request=0U;
   g_stage3_request=0U;g_stage3_teach_request=0U;return 0U;
  }''')
    editing.edit(source,'  if(g_stage3_manual_x==3U)g_stage3_manual_x=2U;',
        '''  if(g_stage3_manual_x==3U)g_stage3_manual_x=2U;
  if(g_stage3_manual_state==6U && g_lift_ruler_state!=LRC_DONE &&
     !g_lift_ruler_stop_pending)g_lift_ruler_request=2U;''')
    editing.edit('CHASSIS/USER/integration10_complete.inc',
        ' RegionDispatch_Service((uint32_t)xTaskGetTickCount());',
        ' if(g_stage3_manual_state!=6U)RegionDispatch_Service((uint32_t)xTaskGetTickCount());')
    manifest=json.loads((E/'observe180p55-compatible.json').read_text(encoding='utf-8'))
    manifest.update(project=str(PROJECT),changes={},unchanged_sha256={},images={},
                    stage3_low_teach_request=3,flashed=False)
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):(PROJECT/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/('low-teach-'+target+'.log')
        run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(PROJECT/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        assert run.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
        axf=PROJECT/'Objects'/target/(target+'.axf')
        manifest['images'][target]={'axf':str(axf),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target,'0 errors, 0 warnings',flush=True)
    for path in PROJECT.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(PROJECT)
        if any(x in ('Objects','Listings') for x in rel.parts):continue
        before=BASE/rel
        key='unchanged_sha256' if before.is_file() and before.read_bytes()==path.read_bytes() else 'changes'
        manifest[key][rel.as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    (E/'low-teach-compatible.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')

if __name__=='__main__':main()
