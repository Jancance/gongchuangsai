"""复核已存遮挡证据；保留原候选统计，独立标出质量拒绝，不访问相机或串口。"""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'evidence/p2-camera/2026-10-27/ring-occlusion'
RUN_NAME = '20261027-192637-095054'
APP = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app/code'


def main():
    archive = BASE / (RUN_NAME + '.zip')
    run = BASE / ('run-' + RUN_NAME)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert Path(name).name == name
            assert z.read(name) == (run / name).read_bytes()
    hashes = json.loads((run / 'module_source_hashes.json').read_text())
    source_matches = {
        name: value == sha(ROOT / 'output/p2_ring_pixel_audit_20261027.py'
                           if name == 'diagnostic_entry' else APP / name)
        for name, value in hashes.items()
    }
    assert all(source_matches.values())
    configs = {}
    for name in ('ring_station_detector.json', 'camera_processing.json', 'project_profile.json'):
        source = json.loads((run / (name + '.source.json')).read_text())
        configs[name] = {
            'release_hash_matches': source['sha256'] == sha(APP / 'config' / name),
            'effective_matches': json.loads((run / name).read_text()) ==
                                 json.loads((APP / 'config' / name).read_text()),
        }
        assert all(configs[name].values())
    rows = [json.loads(s) for s in (run / 'observations.jsonl').read_text(encoding='utf8').splitlines()]
    summary = json.loads((run / 'run.json').read_text(encoding='utf8'))
    assert len(rows) == summary['frames']
    stamps = np.array([r['host_read_time'] for r in rows])
    assert np.all(np.diff(stamps) > 0)
    assert all(not r['metric_valid'] and not r['motion_allowed'] and r['serial'] == 'DISABLED' for r in rows)
    for r in rows:
        valid = (r['detected'] and len(r['centers_px']) == 3 and
                 np.isfinite(r['centers_px']).all() and
                 all(0 <= x < 1920 and 0 <= y < 1080 for x, y in r['centers_px']) and
                 0 <= r['result_host_age_ms'] <= 150)
        assert bool(valid) == r['usable_for_pixel_statistics']
        assert r['usable_centers_px'] == (r['centers_px'] if valid else [])
    counts = dict(Counter(r['state'] for r in rows))
    assert counts == summary['states']
    usable = [r for r in rows if r['usable_for_pixel_statistics']]
    assert len(usable) == summary['usable_frames']
    points = np.array([r['usable_centers_px'] for r in usable])
    for i, expected in enumerate(summary['ring_statistics']):
        p = points[:, i, :]
        med = np.median(p, axis=0)
        assert expected['median_xy_px'] == med.tolist()
        assert expected['range_xy_px'] == np.ptp(p, axis=0).tolist()
        assert expected['p95_abs_deviation_from_median_xy_px'] == np.percentile(np.abs(p-med), 95, axis=0).tolist()
    segments = []
    for r in rows:
        t = r['host_read_time'] - stamps[0]
        if not segments or segments[-1]['state'] != r['state']:
            segments.append({'state': r['state'], 'first_frame': r['frame_index'],
                             'last_frame': r['frame_index'], 'start_s': t, 'end_s': t, 'count': 1})
        else:
            segments[-1]['last_frame'] = r['frame_index']
            segments[-1]['end_s'] = t
            segments[-1]['count'] += 1
    # 旧记录器没有stable字段。仅按已核对来源的两个互斥detail分支推断，明确不冒充现场遥测。
    stable_details = {'工位偏差超出验收精度', '工位视觉残差已达到验收精度'}
    inferred_stable = [r for r in usable if r['detail'] in stable_details]
    minimum_confidence = json.loads((run / 'ring_station_detector.json').read_text())['reference']['minimum_confidence']
    low = [r for r in usable if r['confidence'] < minimum_confidence]
    lost = [r for r in rows if not r['usable_for_pixel_statistics']]
    first_lost, last_lost = lost[0], lost[-1]
    before = [r for r in inferred_stable if r['frame_index'] < first_lost['frame_index']]
    after = [r for r in inferred_stable if r['frame_index'] > last_lost['frame_index']]
    def statistics(records):
        a = np.array([r['centers_px'] for r in records])
        return {'count': len(records), 'first_frame': records[0]['frame_index'],
                'last_frame': records[-1]['frame_index'], 'median_xy_px': np.median(a, axis=0).tolist(),
                'range_xy_px': np.ptp(a, axis=0).tolist()}
    pre, post = statistics(before), statistics(after)
    delta = np.array(post['median_xy_px']) - np.array(pre['median_xy_px'])
    before_median = np.array(pre['median_xy_px'])
    max_low_offset = max(float(np.linalg.norm(np.array(r['centers_px'])[2] - before_median[2])) for r in low)
    report = {
        'archive': archive.name, 'sha256': sha(archive), 'crc_and_member_bytes_match': True,
        'source_matches': source_matches, 'config_matches': configs,
        'frames': len(rows), 'states': counts, 'host_read_span_s': float(stamps[-1]-stamps[0]),
        'segments': segments, 'detail_counts': dict(Counter(r['detail'] for r in rows)),
        'original_fresh_candidate_frames': len(usable),
        'minimum_confidence': minimum_confidence,
        'low_confidence_candidate_frames': [r['frame_index'] for r in low],
        'right_ring_max_low_confidence_offset_from_pre_median_px': max_low_offset,
        'stable_frames_inferred_from_audited_source_detail': len(inferred_stable),
        'stable_not_recorded_directly': True,
        'before_inferred_stable': pre, 'after_inferred_stable': post,
        'post_minus_pre_median_xy_px': delta.tolist(),
        'no_three_centers_have_empty_usable_coordinates': all(not r['usable_centers_px'] for r in lost),
        'read_gap_p50_p95_max_ms': np.percentile(np.diff(stamps)*1000, [50,95,100]).tolist(),
        'result_host_age_p50_p95_max_ms': np.percentile([r['result_host_age_ms'] for r in rows], [50,95,100]).tolist(),
        'detector_processing_p50_p95_max_ms': np.percentile([r['detector_processing_ms'] for r in rows], [50,95,100]).tolist(),
        'visual_review': {
            'first-three-centers-overlay.png': 'All three complete rings; green crosses near centers; printed left/middle/right 3/2/1.',
            'first-unusable-overlay.png': 'Right ring substantially covered by held printed sheet; no three-center crosses.',
            'first-reappearance-overlay.png': 'Right ring still substantially covered; cross is a low-confidence candidate, not stable recovery.'},
        'conclusion': 'This scene loses three-center output and later regains a sustained high-quality output segment. Invalid records clear usable coordinates. Partial-occlusion robustness is not accepted: six low-confidence candidates enter the old pixel-statistics stream.',
        'limits': ['Manual cover/removal timestamps are absent; the 9.1s loss interval is not removal-to-recovery latency.',
                   'Only initial/loss/first-candidate-return raw frames were saved; no raw frame of the later stable return.',
                   'Original recorder marks fresh finite triplets usable regardless of detector stable/quality. Preserve these as candidate statistics.',
                   'Stable detail is inferred from matching source; metric wording reflects placeholder geometry, not real millimeter verification.'],
        'metric_valid': False, 'motion_allowed': False,
    }
    (BASE / 'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
