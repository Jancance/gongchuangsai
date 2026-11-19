"""Offline repair candidate: grip dwell, second-stage speeds, ordered returns."""
import json
import shutil
import subprocess
from pathlib import Path
from prepare_stage3_ring2_touchdown_20261119 import edit
from prepare_stage3_speed_offline_20261119 import ROOT, EVIDENCE, TARGETS, digest

BASE = ROOT / 'tmp/stage3_three_place_dm150_verify_20261119/stage3-three-place-compatible.json'
OUT = ROOT / 'tmp/stage3_grip_return_20261119'


def main():
    proof = EVIDENCE / 'stage3-grip-return-matlab.log'
    assert 'PASS Stage3 grip and ordered return: 2430 cases' in proof.read_text()
    m = json.loads(BASE.read_text(encoding='utf-8'))
    base = Path(m['project'])
    project = OUT / 'project'
    for n, h in (m['changes'] | m['unchanged_sha256']).items():
        assert digest(base / n) == h, n
    shutil.copytree(base, project, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    edit(project, 'integration10_complete.inc', (
        '     it_command.dm_deg_s=(stage3_scope && axes==SPF_DM)?SPF_STAGE3_DM_DEG_S:SPF_DM_DEG_S;',
        '     it_command.dm_deg_s=SPF_DM_DEG_S;'), (
        '         (stage3_scope?SPF_STAGE3_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM));',
        '         (stage3_scope?SPF_EXT_RPM:(g_it_ext_rpm_test?(float)g_it_ext_rpm_test:SPF_EXT_RPM));'), (
        '    float turn_extension_zero_tolerance_deg = stage3_scope ? 3.0f : 2.0f;',
        '    float turn_extension_zero_tolerance_deg = stage3_scope ? SOURCE_EXTENSION_ZERO_DONE_DEG : 2.0f;'), (
        '   if(S3_SyncPlaceOwned() && it_command.axes_mask==(SPF_EXT|SPF_LIFT))vsp_pick_sync=1U;',
        '''   if(Stage3_Owns() && (it_command.axes_mask&SPF_LIFT) &&
      it_command.kind!=SPF_CMD_STAGE3_TOUCHDOWN)vsp_fast_lift=1U;
   if(S3_SyncPlaceOwned() && it_command.axes_mask==(SPF_EXT|SPF_LIFT))vsp_pick_sync=1U;'''), (
        ' case 5U:case 6U:case 17U:\n  if(g_it_elapsed_ms>=600U)it_done();break;',
        ''' case 5U:case 6U:case 17U:
  /* PWM has no opening feedback. Stage3 close holds Z for a full 1200 ms. */
  if(g_it_elapsed_ms>=((Stage3_Owns() && g_it_action==5U)?1200U:600U))it_done();break;'''))
    # All Stage3 command structs use the same second-stage extension speed.
    edit(project, 'source_parallel_flow.c', (
        '     SPF_STAGE3_EXT_RPM:SPF_EXT_RPM) ||',
        '     SPF_EXT_RPM:SPF_EXT_RPM) ||'))
    edit(project, 'stage3_cycle.inc', (
        '   if(!s3_round_trip){S3_Command(4U);break;}\n   g_it_axes=SPF_EXT|SPF_LIFT;',
        '''   if(!s3_round_trip || target!=1U){S3_Command(4U);break;}
   g_it_axes=SPF_EXT|SPF_LIFT;'''), (
        '''  case 13U:
   if(s3_round_trip)S3_Command(17U);
   else S3_Axis(SPF_EXT,0.0f);
   break;
  case 14U:
   g_stage3_placed_count++;''',
        '''  case 13U:S3_Axis(SPF_EXT,0.0f);break;
  case 14U:
   if(s3_round_trip){g_stage3_phase=29U;S3_Command(17U);break;}
   g_stage3_placed_count++;'''), (
        '  case 15U:g_it_carrier_deg=240.0f;S3_Command(10U);break;',
        '''  case 30U:
   g_stage3_placed_count++;
   if(++g_stage3_item<s3_first+s3_count){g_stage3_phase=0U;return;}
   g_stage3_state=6U;S3_Pose();return;
  case 15U:g_it_carrier_deg=240.0f;S3_Command(10U);break;'''))
    for n in ('stage3_cycle.inc', 'integration10_complete.inc', 'source_parallel_flow.c'):
        key = 'CHASSIS/USER/' + n
        m['unchanged_sha256'].pop(key, None)
        m['changes'][key] = digest(project / key)
    m.update(project=str(project), images={}, build_passed=False, flashed=False,
             baseline_manifest=str(BASE), stage3_dm_speed_deg_s=300,
             stage3_close_dwell_ms=1200, matlab_grip_return_sha256=digest(proof),
             stage3_extension_rpm=150, stage3_lift_rpm=1846.4,
             stage3_lift_accel_rpm_s=45055,
             side_ring_return='lift_top_then_extension_zero_then_rotate',
             rotation_zero_window='SOURCE_EXTENSION_ZERO_DONE_DEG')
    for target in TARGETS:
        for d in ('Objects','Listings'):
            (project / d / target).mkdir(parents=True, exist_ok=True)
        log = OUT / (target + '.log')
        run = subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(project/'gongxun-f407.uvprojx'),
                              '-t',target,'-j0','-o',str(log)], timeout=180)
        assert run.returncode == 0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(), log
        axf = project/'Objects'/target/(target+'.axf')
        m['images'][target] = dict(axf=str(axf), sha256=digest(axf))
        print(target, '0 errors, 0 warnings', flush=True)
    m['build_passed'] = True
    (OUT/'stage3-three-place-compatible.json').write_text(json.dumps(m,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
