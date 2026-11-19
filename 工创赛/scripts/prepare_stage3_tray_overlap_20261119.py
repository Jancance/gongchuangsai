"""Preindex tray and overlap gripper preclose with Stage3 carrier yaw."""
import json
import shutil
import subprocess
from pathlib import Path
from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import ROOT, EVIDENCE, TARGETS, digest

BASE = ROOT/'tmp/stage3_grip_return_20261119/stage3-three-place-compatible.json'
OUT = ROOT/'tmp/stage3_tray_overlap_close200_20261119'


def main():
    proof = EVIDENCE/'stage3-tray-close200-matlab.log'
    assert 'PASS Stage3 tray/DM/preclose overlap: 648 cases' in proof.read_text()
    assert 'Close dwell: 200 ms' in proof.read_text()
    m = json.loads(BASE.read_text(encoding='utf-8'))
    base = Path(m['project']);project = OUT/'project'
    for n,h in (m['changes']|m['unchanged_sha256']).items():
        assert digest(base/n)==h,n
    shutil.copytree(base,project,ignore=shutil.ignore_patterns('Objects','Listings'))
    edit(project,'integration10_complete.inc',(
        '  /* PWM has no opening feedback. Stage3 close holds Z for a full 1200 ms. */',
        '  /* User-selected 200 ms from close PWM request; no opening feedback. */'),(
        '  if(g_it_elapsed_ms>=((Stage3_Owns() && g_it_action==5U)?1200U:600U))it_done();break;',
        '  if(g_it_elapsed_ms>=((Stage3_Owns() && g_it_action==5U)?200U:600U))it_done();break;'))
    edit(project,'stage3_cycle.inc',(
        'static uint32_t s3_preclose_started,s3_preclose_tick;',
        '''static uint32_t s3_preclose_started,s3_preclose_tick;
volatile uint32_t g_stage3_tray_prefetch_item,g_stage3_tray_prefetch_writes;
static uint32_t s3_tray_valid,s3_tray_tick;'''),(
        'static void Stage3_Service(uint32_t now)',
        '''static uint32_t S3_PrefetchNextTray(uint32_t item,uint32_t now)
{
 uint32_t slot=0U;
 if(item>=s3_first+s3_count)return 1U;
 if(s3_tray_valid && g_stage3_tray_prefetch_item==item)return 1U;
 while(slot<3U && s3_slots[slot]!=s3_colors[item])slot++;
 if(slot>=3U || CarrierPwm_Write(500U+slot*120U*2000U/270U)!=1U){S3_Fail(9U);return 0U;}
 s3_tray_tick=now;s3_tray_valid=1U;
 g_stage3_tray_prefetch_item=item;g_stage3_tray_prefetch_writes++;
 return 1U;
}
static void Stage3_Service(uint32_t now)'''),(
        '  s3_pending=s3_loader_slot=s3_loader_dwell=0U;',
        '  s3_pending=s3_loader_slot=s3_loader_dwell=0U;\n  s3_tray_valid=g_stage3_tray_prefetch_writes=0U;'),(
        '  if(g_stage3_phase==4U){g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;}',
        '''  if(g_stage3_phase==2U && g_stage3_task_run && g_stage3_config_ready){
   uint32_t slot=0U,item=g_stage3_test_first_item;
   while(slot<3U && g_stage3_slot_colors[slot]!=g_stage3_colors[item])slot++;
   if(slot>=3U){S3_Fail(9U);return;}
   g_it_carrier_deg=(float)(slot*120U);
   g_stage3_tray_prefetch_item=item;s3_tray_tick=now;s3_tray_valid=1U;
   g_stage3_tray_prefetch_writes++;S3_Command(10U);return;
  }
  if(g_stage3_phase==4U){g_it_axes=SPF_EXT;g_it_extension_mm=0.0f;}'''),(
        '  case 0U:S3_IndexItemSlot();break;',
        '''  case 0U:
   if(s3_round_trip && s3_tray_valid && g_stage3_tray_prefetch_item==g_stage3_item){
    if((uint32_t)(now-s3_tray_tick)>=700U)g_stage3_phase++;
   }else S3_IndexItemSlot();
   break;'''),(
        '''  case 2U:
   if(s3_round_trip){
    if(g_stage3_item==s3_first)S3_Command(17U);
    else g_stage3_phase++;
   }else S3_Axis(SPF_EXT,0.0f);
   break;
  case 3U:s3_preclose_started=0U;s3_preclose_tick=0U;S3_Axis(SPF_DM,s3_carrier);break;''',
        '''  case 2U:
   if(s3_round_trip)g_stage3_phase++;
   else S3_Axis(SPF_EXT,0.0f);
   break;
  case 3U:
   s3_preclose_started=0U;s3_preclose_tick=0U;
   if(s3_round_trip){
    if(jiazhua(160.0f)!=1U){S3_Fail(7U);return;}
    s3_preclose_started=1U;s3_preclose_tick=now;
   }
   S3_Axis(SPF_DM,s3_carrier);break;'''),(
        '   if(s3_round_trip){g_stage3_phase=5U;S3_Axis(SPF_LIFT,s3_pick);}',
        '''   if(s3_round_trip){
    if((uint32_t)(now-s3_preclose_tick)<600U)break;
    g_stage3_phase=5U;S3_Axis(SPF_LIFT,s3_pick);
   }'''),(
        '''  case S3_SYNC_PLACE_PHASE:
   if(s3_round_trip){
    g_it_axes=SPF_EXT|SPF_LIFT;''',
        '''  case S3_SYNC_PLACE_PHASE:
   if(s3_round_trip){
    if(!S3_PrefetchNextTray(g_stage3_item+1U,now))return;
    g_it_axes=SPF_EXT|SPF_LIFT;'''))
    key='CHASSIS/USER/stage3_cycle.inc';m['unchanged_sha256'].pop(key,None);m['changes'][key]=digest(project/key)
    key='CHASSIS/USER/integration10_complete.inc';m['unchanged_sha256'].pop(key,None);m['changes'][key]=digest(project/key)
    m.update(project=str(project),images={},build_passed=False,flashed=False,
             baseline_manifest=str(BASE),matlab_tray_overlap_sha256=digest(proof),
             tray_preindex_before_vision=True,preclose_parallel_carrier_yaw=True,
             tray_prefetch_on_place_descent=True,stage3_close_dwell_ms=200)
    # Earlier evidence describes the baseline. New phase fixture is added after build.
    m.pop('validation_logs',None);m.pop('actual_phase_fixture',None)
    for target in TARGETS:
        for d in ('Objects','Listings'):(project/d/target).mkdir(parents=True,exist_ok=True)
        log=OUT/(target+'.log')
        run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(project/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        assert run.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
        axf=project/'Objects'/target/(target+'.axf')
        m['images'][target]=dict(axf=str(axf),sha256=digest(axf))
        print(target,'0 errors, 0 warnings',flush=True)
    m['build_passed']=True
    (OUT/'stage3-three-place-compatible.json').write_text(json.dumps(m,indent=2),encoding='utf-8')


if __name__=='__main__':main()
