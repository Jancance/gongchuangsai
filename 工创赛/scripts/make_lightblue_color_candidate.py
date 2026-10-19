"""Candidate tuning from the measured lightblue sample, not full calibration."""
import copy


def make_candidate(config):
    candidate=copy.deepcopy(config)
    by_id={int(c['color_id']):c for c in candidate['colors']}
    blue=by_id[3]['ranges'];pale=by_id[6]['ranges']
    if len(blue)!=1 or len(pale)!=1:
        raise ValueError('Candidate requires the measured single-range blue configurations')
    blue[0]['lower'][1]=150
    pale[0]['upper'][1]=149
    candidate['_candidate_update']={
        'version':'lightblue-saturation-candidate-2026-10-06',
        'basis':'Lightblue top saturation median 140, deepblue mostly 255; replay checked.',
        'blue_s_min':150,'lightblue_s_max':149,'requires_live_validation':True}
    # 保留相机/其余颜色/未完成正式标定标记，不冒充全六色标定通过。
    return candidate
