"""Isolated source-only faster alignment after local MATLAB validation; no IO."""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
BASE = R/'tmp/source_stopped3_grip170_release200_20261115/project'
P = R/'tmp/source_stopped5_fastalign_20261116/project'
E = R/'evidence/integration/2026-11-16/source-alignment-fast'


def replace_once(path, before, after):
    data=path.read_bytes()
    assert data.count(before)==1, str(path)
    path.write_bytes(data.replace(before,after,1))


def main():
    proof=json.loads((E/'matlab-result.json').read_text(encoding='utf-8'))
    assert proof['validation_passed'] and proof['local_matlab']
    speed=proof['selected_candidate_speed_mm_s']
    assert speed==45
    assert not P.exists(),'Preserve previous candidates'
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    replace_once(P/'CHASSIS/USER/source_body_step.h',
                 b'uint32_t SourceBodyStep_SmallDistanceWindow(void);',
                 b'uint32_t SourceBodyStep_SmallDistanceWindow(void);\r\nuint32_t SourceBodyStep_FastAlignmentWindow(void);')
    inc=P/'CHASSIS/USER/source_body_step.inc'
    replace_once(inc,b'uint32_t SourceBodyStep_SmallDistanceWindow(void)',
                 b'/* Source axis correction only; legacy/turn/other owners keep their gap. */\r\n'
                 b'uint32_t SourceBodyStep_FastAlignmentWindow(void){return s_align_kind==2U && s_align_chain && (g_source_body_step_state==1U || g_source_body_step_state==2U);}\r\n\r\n'
                 b'uint32_t SourceBodyStep_SmallDistanceWindow(void)')
    replace_once(inc,b'MotionProfile profile;uint32_t rpm,run_ms,accepted;',
                 b'MotionProfile profile;uint32_t rpm,run_ms,accepted;\r\n'
                 b'  float speed=SourceBodyStep_FastAlignmentWindow()?45.0f:15.0f;')
    replace_once(inc,b'MotionRoute_Build(s_align_direction,s_align_distance,15.0f,',
                 b'MotionRoute_Build(s_align_direction,s_align_distance,speed,')
    replace_once(inc,b'App_MoveSubmit(s_align_direction,rpm,run_ms,s_align_distance,15.0f)',
                 b'App_MoveSubmit(s_align_direction,rpm,run_ms,s_align_distance,speed)')
    replace_once(P/'CHASSIS/USER/chassis_auto_test.c',
                 b'if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&\r\n'
                 b'            ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS)))',
                 b'if (Auto_AllSpeedSinceLocked(now) && Auto_ZeroSpeedLocked() &&\r\n'
                 b'            (SourceBodyStep_FastAlignmentWindow() ?\r\n'
                 b'             (g_chassis_auto_stop_mask[g_chassis_auto_segment] == 0x0FU) :\r\n'
                 b'             ((TickType_t)(now - s_phase_tick) >= pdMS_TO_TICKS(CHASSIS_AUTO_GAP_MS))))')
    changes={}
    for f in P.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(P);data=f.read_bytes();old=(BASE/rel).read_bytes()
        if data!=old:
            # The source already contains mixed endings; preserve every old LF.
            assert data.count(b'\n')-data.count(b'\r\n') == old.count(b'\n')-old.count(b'\r\n')
            changes[rel.as_posix()]=hashlib.sha256(data).hexdigest()
    assert set(changes)=={'CHASSIS/USER/source_body_step.h','CHASSIS/USER/source_body_step.inc','CHASSIS/USER/chassis_auto_test.c'}
    (E/'prepared.json').write_text(json.dumps(dict(project=P.relative_to(R).as_posix(),changes=changes,
        speed_mm_s=speed,global_acceleration_jerk_unchanged=True,only_source_gap_removed=True,
        three_second_target_validated=proof['target_3s_passed'],matlab_prior_to_implementation=True,
        hardware_access=False,flashed=False),indent=2)+'\n',encoding='utf-8')
    print('Prepared isolated source alignment 45 mm/s / fresh-zero + STOP ACK, no global tuning or deployment')


if __name__=='__main__':main()
