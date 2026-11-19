"""Build the Stage3 round-trip candidate from the last flashed baseline."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import prepare_stage3_manual_release_20261118 as edit

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/stage3_disabled_dm_return_20261118/project'
PROJECT = ROOT / 'tmp/stage3_full_cycle_20261119/project'
EVIDENCE = ROOT / 'evidence/integration/2026-11-18/stage3-resume'


def main():
    assert json.loads((EVIDENCE / 'full-cycle-matlab.json').read_text())['passed']
    shutil.copytree(BASE, PROJECT, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    edit.PROJECT = PROJECT
    stage = 'CHASSIS/USER/stage3_cycle.inc'
    it = 'CHASSIS/USER/integration10_complete.inc'
    edit.edit(stage, 'static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3];', '''static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3];
/* Rough-ring recovery Z is separate from the loaded ring release Z. */
volatile float g_stage3_recovery_depth[3]={59400.0f,61400.0f,60600.0f};
volatile uint32_t g_stage3_placed_count,g_stage3_recovered_count;
static uint32_t s3_round_trip;
static float s3_recovery_depth[3];
#define S3_SYNC_PLACE_PHASE 9U
#define S3_RECOVER_FIRST_PHASE 17U''')
    edit.edit(stage, ' if(first>=3U || count<1U || first+count>3U ||', ''' if(g_stage3_task_run && (first!=0U || count!=3U ||
    g_stage3_task_batch<1U || g_stage3_task_batch>2U))return 0U;
 if(first>=3U || count<1U || first+count>3U ||''')
    edit.edit(stage, '  j=g_stage3_positions[i]-1U;', '''  j=g_stage3_positions[i]-1U;
  if(g_stage3_task_run && !(g_stage3_recovery_depth[j]>=3200.0f &&
      g_stage3_recovery_depth[j]<=(float)SPF_STAGE3_MAX_LIFT_PULSES))return 0U;''')
    edit.edit(stage, ' s3_pick=g_stage3_pick_depth;s3_carrier=g_stage3_carrier_yaw;', ''' s3_pick=g_stage3_pick_depth;s3_carrier=g_stage3_carrier_yaw;
 s3_round_trip=g_stage3_task_run!=0U;
 g_stage3_placed_count=g_stage3_recovered_count=0U;''')
    edit.edit(stage, '  s3_yaw[i]=g_stage3_target_yaw[i];s3_x[i]=g_stage3_target_x[i];s3_depth[i]=g_stage3_target_depth[i];', '''  s3_yaw[i]=g_stage3_target_yaw[i];s3_x[i]=g_stage3_target_x[i];s3_depth[i]=g_stage3_target_depth[i];
  s3_recovery_depth[i]=g_stage3_recovery_depth[i];''')
    edit.edit(stage, 'static uint32_t S3_Fresh(uint32_t now)', '''/* Scoped permission for the dedicated Stage3 two-stepper transfer. */
static uint32_t S3_SyncPlaceOwned(void)
{
 return Stage3_Owns() && s3_round_trip && g_stage3_state==5U &&
  g_stage3_phase==S3_SYNC_PLACE_PHASE && s3_pending==3U &&
  !g_stage3_fault && g_low_wheel_hold_ready;
}
static void S3_IndexItemSlot(void)
{
 uint32_t slot=0U;
 while(slot<3U && s3_slots[slot]!=s3_colors[g_stage3_item])slot++;
 if(slot==3U){S3_Fail(12U);return;}
 g_it_carrier_deg=(float)(slot*120U);
 S3_Command(10U);
}
static uint32_t S3_Fresh(uint32_t now)''')
    old = '''  case 0U:{
   uint32_t slot=0U;
   while(slot<3U && s3_slots[slot]!=s3_colors[g_stage3_item])slot++;
   /* Config validation plus locked inventory guarantee a unique color match. */
   g_it_carrier_deg=(float)(slot*120U);S3_Command(10U);break;
  }'''
    edit.edit(stage, old, '  case 0U:S3_IndexItemSlot();break;')
    edit.edit(stage, '  case 9U:S3_Axis(SPF_EXT,s3_x[target]);break;', '''  case S3_SYNC_PLACE_PHASE:
   if(s3_round_trip){
    g_it_axes=SPF_EXT|SPF_LIFT;
    g_it_extension_mm=s3_x[target];
    g_it_lift_pulses=s3_depth[target];
    S3_Command(3U);
   }else S3_Axis(SPF_EXT,s3_x[target]);
   break;''')
    edit.edit(stage, '  case 10U:S3_Axis(SPF_LIFT,s3_depth[target]);break;', '''  case 10U:
   if(s3_round_trip)g_stage3_phase++;
   else S3_Axis(SPF_LIFT,s3_depth[target]);
   break;''')
    edit.edit(stage, '''  case 14U:
   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}
   S3_Axis(SPF_DM,0.0f);break;''', '''  case 14U:
   g_stage3_placed_count++;
   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}
   if(s3_round_trip){g_stage3_item=0U;g_stage3_phase=S3_RECOVER_FIRST_PHASE;return;}
   S3_Axis(SPF_DM,0.0f);break;''')
    edit.edit(stage, '''  case 16U:S3_Command(6U);break;
  default:g_stage3_state=6U;S3_Pose();break;''', '''  case 16U:S3_Command(6U);break;
  /* Recover only after the three placement operations complete. */
  case S3_RECOVER_FIRST_PHASE:
   if(!s3_round_trip){g_stage3_state=6U;S3_Pose();break;}
   S3_Axis(SPF_DM,s3_yaw[target]);break;
  case 18U:S3_Axis(SPF_EXT,s3_x[target]);break;
  case 19U:S3_Axis(SPF_LIFT,s3_recovery_depth[target]);break;
  case 20U:S3_Command(5U);break;
  case 21U:S3_Command(4U);break;
  case 22U:S3_Axis(SPF_EXT,0.0f);break;
  case 23U:S3_Axis(SPF_DM,s3_carrier);break;
  case 24U:S3_IndexItemSlot();break;
  case 25U:S3_Axis(SPF_LIFT,s3_pick);break;
  case 26U:S3_Command(6U);break;
  case 27U:S3_Command(4U);break;
  case 28U:
   g_stage3_recovered_count++;
   if(++g_stage3_item<3U)g_stage3_phase=S3_RECOVER_FIRST_PHASE;
   else {g_stage3_state=6U;S3_Pose();}
   break;
  default:S3_Fail(12U);break;''')
    edit.edit(it, '    uint32_t stage3_scope=Stage3_Owns();', '''    uint32_t stage3_scope=Stage3_Owns();
    uint32_t stage3_sync=S3_SyncPlaceOwned() && axes==(SPF_EXT|SPF_LIFT);''')
    edit.edit(it, '((axes&3U) && ((!it_at_top() && !Low_XYAllowed()) || (axes&4U)))', '((axes&3U) && ((!it_at_top() && !Low_XYAllowed()) || ((axes&4U) && !stage3_sync)))')
    edit.edit(it, '''   if(g_source_cycle_state==1U && g_source_cycle_prep==4U &&''', '''   if(S3_SyncPlaceOwned() && it_command.axes_mask==(SPF_EXT|SPF_LIFT))vsp_pick_sync=1U;
   if(g_source_cycle_state==1U && g_source_cycle_prep==4U &&''')

    manifest = json.loads((EVIDENCE / 'disabled-dm-return-compatible.json').read_text())
    manifest.update(project=str(PROJECT), images={}, changes={}, unchanged_sha256={},
                    flashed=False, stage3_round_trip=True)
    for target in ('Vehicle_LiftRuler', 'Vehicle_LiftRuler_Offline'):
        for folder in ('Objects', 'Listings'):
            (PROJECT / folder / target).mkdir(parents=True, exist_ok=True)
        log = EVIDENCE / f'full-cycle-{target}.log'
        result = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(PROJECT / 'gongxun-f407.uvprojx'),
                                 '-t', target, '-j0', '-o', str(log)], timeout=180)
        assert result.returncode == 0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(), log
        axf = PROJECT / 'Objects' / target / f'{target}.axf'
        manifest['images'][target] = {'axf': str(axf), 'sha256': hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target, '0 errors, 0 warnings', flush=True)
    for path in PROJECT.rglob('*'):
        if not path.is_file():
            continue
        relative = path.relative_to(PROJECT)
        if any(name in ('Objects', 'Listings') for name in relative.parts):
            continue
        previous = BASE / relative
        key = 'unchanged_sha256' if previous.is_file() and previous.read_bytes() == path.read_bytes() else 'changes'
        manifest[key][relative.as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    (EVIDENCE / 'full-cycle-compatible.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
