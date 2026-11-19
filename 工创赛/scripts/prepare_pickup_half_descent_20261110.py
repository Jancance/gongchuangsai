"""Prepare half-descent release of the same single-speed extension move.

No hardware access. Do not overwrite the diagnostic files owned by the
concurrent extension-diagnostics task after the initial project copy.
"""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/pickup_second_diag_20261110/project'
P = R / 'tmp/pickup_half_descent_20261110/project'
E = R / 'evidence/integration/2026-11-10/half-descent'


def main():
    E.mkdir(parents=True, exist_ok=True)
    if not P.exists():
        shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    rel = 'CHASSIS/USER/integration10_complete.inc'
    p = P / rel
    data = p.read_bytes()
    before = hashlib.sha256((B / rel).read_bytes()).hexdigest()

    def edit(old, new):
        nonlocal data
        old, new = old.encode('utf-8'), new.encode('utf-8')
        if data.count(new) == 1:
            return
        assert data.count(old) == 1, old[:90]
        data = data.replace(old, new, 1)

    edit('static uint32_t it_empty_overlap_begin(void)',
         '''volatile uint32_t g_it_half_extension_released,g_it_half_extension_release_tick;
volatile float g_it_half_lift_before,g_it_half_lift_goal,g_it_half_lift_at_release;
static uint32_t it_empty_overlap_begin(void)''')
    edit(' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;',
         ''' vsp_fast_lift=1U;g_it_carrier_auto_done=0U;g_it_lead5_grip_done=0U;
 g_it_half_extension_released=g_it_half_extension_release_tick=0U;
 g_it_half_lift_before=g_it_half_lift_goal=g_it_half_lift_at_release=0.0f;
 if(it_command.axes_mask&SPF_EXT){
  /* Only lift is prepared/started first. Extension keeps its one FD move. */
  vsp_group.delayed_mask=SPF_EXT;vsp_group.release_mask=SPF_LIFT;
 }''')
    edit('static void it_empty_overlap_publish(void)',
         '''static void it_half_extension_gate(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1];float span;
 if(g_it_state!=1U || g_it_action!=14U || !VehicleSourceParallel_Owns() ||
  vsp_group.state!=2U || vsp_group.command.kind!=SPF_CMD_PICK ||
  vsp_group.command.sequence!=it_command.sequence ||
  vsp_group.delayed_mask!=SPF_EXT || !(vsp_group.command.axes_mask&SPF_EXT) ||
  !(vsp_group.started&SPF_LIFT) || (vsp_group.release_mask&SPF_EXT))return;
 if(!z->position_cycle || ((uint32_t)(now-z->position_tick)>150U &&
    !(vsp_group.done&SPF_LIFT)))return;
 span=z->goal-z->before;
 /* Comparisons reject NaN/Inf and wrong-direction/zero strokes. Current-group
  * lift DONE is a real settled endpoint, not an old reference or a timer. */
 if(!(span>0.0f && span<=(float)SPF_MAX_LIFT_PULSES*360.0f/3200.0f+0.1f) ||
  !(z->current>=z->before-60.0f && z->current<=z->goal+60.0f)){
  it_fail(8U);return;
 }
 if((z->current-z->before)*2.0f<span)return;
 g_it_half_lift_before=z->before;g_it_half_lift_goal=z->goal;
 g_it_half_lift_at_release=z->current;g_it_half_extension_release_tick=now;
 vsp_group.release_mask|=SPF_EXT;g_it_half_extension_released=1U;
#else
 (void)now;
#endif
}
static void it_empty_overlap_publish(void)''')
    edit(' if(VehicleSourceParallel_Owns())VehicleArm_Service();',
         ' if(VehicleSourceParallel_Owns())VehicleArm_Service();\r\n'
         ' it_half_extension_gate((uint32_t)xTaskGetTickCount());')
    p.write_bytes(data)
    profile = P / 'CHASSIS/USER/pickup_trial_profile.h'
    header = profile.read_bytes()
    for old, new in ((b'g_pick_trial_profile==2U?300.0f', b'g_pick_trial_profile==2U?200.0f'),
                     (b'g_pick_trial_profile==2U?1200U', b'g_pick_trial_profile==2U?600U')):
        if old in header:
            assert header.count(old) == 1
            header = header.replace(old, new, 1)
        else:
            assert header.count(new) == 1
    header = header.replace(
        b'/* 0=pzq baseline,1..3=the user\'s paired trials. Selected only while idle and',
        b'/* 0=pzq baseline; profile2 revised to lift2500/ext200, extension600/600. Selected while idle and')
    header = header.replace(
        ' * 仅调整第一档；0档和第二、三档沿用原加减速。实物超调改善待验。 */'.encode('utf-8'),
        ' * 第二档按用户最新要求为200 RPM、加减速600 RPM/s，升降仍2500/45055；0档和第三档不改。 */'.encode('utf-8'))
    profile.write_bytes(header)
    (E / 'half-descent-source-change.json').write_text(json.dumps({
        'base': B.relative_to(R).as_posix(),
        'project': P.relative_to(R).as_posix(),
        'changed_file': rel,
        'before_sha256': before,
        'after_sha256': hashlib.sha256(data).hexdigest(),
        'changed_files_by_this_task': [rel, 'CHASSIS/USER/pickup_trial_profile.h'],
        'profile2_lift_rpm': 2500,
        'profile2_lift_accel_decel_rpm_s': 45055,
        'profile2_extension_rpm': 200,
        'profile2_extension_accel_decel_rpm_s': 600,
        'target_or_window_changes': False,
        'extension_prepare_delayed_until_half_feedback': True,
        'hardware_access': False,
    }, indent=2) + '\n', encoding='utf-8')
    print('Prepared half-descent gate and requested profile2 2500/200, extension600/600; no hardware access.')


if __name__ == '__main__':
    main()
