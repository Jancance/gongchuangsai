"""Add one bounded supervised empty move to the existing isolated candidate."""
import hashlib
import json
from pathlib import Path
from prepare_stage2_fast_candidate_20261109 import replace

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-07/stage2-mainline'
P = R / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER'
assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text())['gate_cases'] == 181
p = P / 'integration10_complete.inc'
d = p.read_bytes()
helper = '''/* Supervised EMPTY commissioning only: accepted at the taught left yaw,
 * referenced safe top and retracted X. No tray, clamp or body operation. */
static uint32_t it_empty_overlap_begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#else
 if(!SingleTeach_ReferencesReady() || !it_at_top() || !it_dm_fresh() ||
  fabsf(g_dm_can2_logical_position_deg-195.0f)>3.0f ||
  fabsf(g_startup_extension_current_deg-g_extension_home_reference_deg)>1.0f ||
  !(g_it_extension_mm>=0.0f && g_it_extension_mm<=156.0f) ||
  g_it_lift_pulses!=118800.0f || g_lift_ruler_request || g_lift_ruler_stop_pending ||
  VehicleArm_Busy())return 0U;
 memset(&it_command,0,sizeof(it_command));
 it_command.sequence=++g_it_sequence;it_command.kind=SPF_CMD_PICK;
 it_command.axes_mask=SPF_EXT|SPF_LIFT;
 it_command.goal.dm_deg=195.0f;it_command.goal.extension_mm=g_it_extension_mm;
 it_command.goal.lift_pulses=g_it_lift_pulses;
 it_command.dm_deg_s=SPF_DM_DEG_S;it_command.extension_rpm=SPF_EXT_RPM;
 it_command.lift_rpm=SPF_LIFT_RPM;it_command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;
 it_command.keep_vm_on=1U;
 if(!VehicleArm_BeginSourceParallel(&it_command,1U,0U))return 0U;
 vsp_fast_lift=1U;return 1U;
#endif
}

'''
d = replace(d, 'void Integration10_Service(void)\r\n', helper.replace('\n','\r\n')+'void Integration10_Service(void)\r\n')
d = replace(d, 'request>13U', 'request>14U')
d = replace(d, '   }else if(request==13U){\r\n',
            '   }else if(request==14U){\r\n'
            '    if(it_empty_overlap_begin())g_it_state=1U;else g_it_rejected=10U;\r\n'
            '   }else if(request==13U){\r\n')
d = replace(d, ' case 5U:case 6U:\r\n',
            ' case 14U:\r\n'
            '  if(g_source_parallel_state==4U){it_fail(8U);return;}\r\n'
            '  if(g_source_parallel_state==3U && !VehicleSourceParallel_Owns() &&\r\n'
            '   (g_source_parallel_done_mask&6U)==6U)it_done();break;\r\n'
            ' case 5U:case 6U:\r\n')
p.write_bytes(d)
f = E / 'stage2-fast-candidate-20261109.json'
m = json.loads(f.read_text())
m['changed'][p.name] = hashlib.sha256(d).hexdigest()
m['images'] = {}
m['empty_test_entry'] = {'it_request': 14, 'yaw_deg': 195, 'max_extension_mm': 156,
                         'depth_virtual_pulses': 118800, 'no_body_no_carrier_no_clamp': True}
f.write_text(json.dumps(m,indent=2)+'\n')
print('Prepared IT14 empty commissioning move; no device access.')
