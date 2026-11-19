"""Fix lift-return admission for the explicitly disabled XY manual owner."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import prepare_stage3_manual_release_20261118 as editing

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/stage3_low_teach_20261118/project'
PROJECT=ROOT/'tmp/stage3_teach_return_fix_20261118/project'
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'

HELPER='''/* Only the real manual XY-disabled lift-return owner may omit X hold. */
static uint32_t ruler_manual_return(void)
{
#if VEHICLE_EXTENSION_HOME_ONLY && !APP_MOTOR_OFFLINE_ONLY
 extern volatile uint32_t g_stage3_manual_state,g_stage3_manual_x,g_stage3_state,g_region_dispatch_area;
 return g_region_dispatch_area==3U && g_stage3_state==7U &&
  g_stage3_manual_state==6U && g_stage3_manual_x==2U && g_startup_axes_state==4U &&
  !g_startup_axes_cause && !g_dm_posvel_holding && g_dm_can2_motor_state==0U &&
  g_lift_ruler_controller.top_valid && g_lift_ruler_controller.top_scale;
#else
 return 0U;
#endif
}
'''


def main():
    assert json.loads((E/'teach-return-ready-matlab.json').read_text())['passed']
    shutil.copytree(BASE,PROJECT,ignore=shutil.ignore_patterns('Objects','Listings'))
    editing.PROJECT=PROJECT
    source='CHASSIS/USER/lift_ruler_adapter.c'
    editing.edit(source,'static uint32_t ruler_ready(void)',HELPER+'\nstatic uint32_t ruler_ready(void)')
    editing.edit(source,
        '(g_startup_axes_state==0U || (g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_hold_confirmed)) &&',
        '(g_startup_axes_state==0U || (g_startup_axes_state==4U && !g_startup_axes_cause && g_extension_hold_confirmed) || ruler_manual_return()) &&')
    source='CHASSIS/USER/stage3_cycle.inc'
    editing.edit(source,
        'g_stage3_manual_state=6U;g_it_request=4U;return 0U;',
        '''g_stage3_manual_state=6U;
   /* Reuse diagnostic reads while DM is disabled, preserving the existing
    * lift controller's actual DM feedback freshness requirement. */
   if(!g_dm_can2_read_request && g_dm_can2_state!=1U && g_dm_can2_state!=2U && g_dm_can2_state!=3U)
    g_dm_can2_read_request=2U;
   g_it_request=4U;return 0U;''')
    editing.edit(source,
        ' }else if(g_stage3_manual_state==6U){',
        ''' }else if(g_stage3_manual_state==6U){
  if(!g_dm_can2_read_request && g_dm_can2_state!=1U && g_dm_can2_state!=2U && g_dm_can2_state!=3U)
   g_dm_can2_read_request=2U;''')
    manifest=json.loads((E/'low-teach-compatible.json').read_text(encoding='utf-8'))
    manifest.update(project=str(PROJECT),changes={},unchanged_sha256={},images={},flashed=False)
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):(PROJECT/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/('teach-return-fix-'+target+'.log')
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
    (E/'teach-return-fix-compatible.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':main()
