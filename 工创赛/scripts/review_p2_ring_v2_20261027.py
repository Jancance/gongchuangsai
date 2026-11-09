"""复核v2圆环像素证据；不访问板端、不修改应用或现场记录。"""
from collections import Counter
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app/code'


def main(run_name, label):
    # 参数仅可作为单层文件名，不允许拼接任意目标路径。
    assert Path(run_name).name == run_name and '/' not in run_name and '\\' not in run_name
    assert label in ('ring-rotation', 'ring-shadow', 'ring-empty')
    base = ROOT / 'evidence/p2-camera/2026-10-27' / label
    base.mkdir(exist_ok=True)
    source = ROOT / (run_name + '.zip')
    archive = base / source.name
    if archive.exists():
        assert archive.read_bytes() == source.read_bytes()
    else:
        shutil.copyfile(source, archive)
    run = base / ('run-' + run_name)
    run.mkdir(exist_ok=True)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert Path(name).name == name and '/' not in name and '\\' not in name
            target = run / name
            data = z.read(name)
            if target.exists():
                assert target.read_bytes() == data
            else:
                target.write_bytes(data)
            assert target.read_bytes() == data
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = json.loads((run / 'module_source_hashes.json').read_text())
    matches = {name: value == sha(ROOT / 'output/p2_ring_pixel_audit_v2_20261027.py'
                                 if name == 'diagnostic_entry' else APP / name)
               for name, value in hashes.items()}
    assert all(matches.values())
    configs = {}
    for name in ('ring_station_detector.json', 'camera_processing.json', 'project_profile.json'):
        provenance = json.loads((run / (name + '.source.json')).read_text())
        configs[name] = provenance['sha256'] == sha(APP / 'config' / name) and (
            json.loads((run / name).read_text()) == json.loads((APP / 'config' / name).read_text()))
    assert all(configs.values())
    config = json.loads((run / 'ring_station_detector.json').read_text())
    threshold = config['reference']['minimum_confidence']
    rows = [json.loads(s) for s in (run / 'observations.jsonl').read_text(encoding='utf8').splitlines()]
    summary = json.loads((run / 'run.json').read_text(encoding='utf8'))
    stamps = np.array([r['host_read_time'] for r in rows])
    assert np.all(np.diff(stamps) > 0)
    assert len(rows) == summary['frames'] and summary['recorder_version'] == 2
    assert dict(Counter(r['state'] for r in rows)) == summary['states']
    for r in rows:
        centers = r['centers_px']
        valid = bool(r['detected'] and len(centers) == 3 and np.isfinite(centers).all()
                     and all(0 <= x < 1920 and 0 <= y < 1080 for x, y in centers)
                     and 0 <= r['result_host_age_ms'] <= 150)
        ready = bool(valid and r['detector_stable'] and r['confidence'] >= threshold)
        assert r['minimum_confidence'] == threshold
        assert r['usable_for_pixel_statistics'] == valid and r['pixel_ready'] == ready
        assert r['usable_centers_px'] == (centers if valid else [])
        assert r['stable_centers_px'] == (centers if ready else [])
        assert not r['metric_valid'] and not r['motion_allowed'] and r['serial'] == 'DISABLED'
    def stats(selected):
        if not selected:
            return []
        a = np.array([r['centers_px'] for r in selected])
        return [{'index': i+1, 'median_xy_px': np.median(a[:, i], axis=0).tolist(),
                 'range_xy_px': np.ptp(a[:, i], axis=0).tolist(),
                 'p95_abs_deviation_from_median_xy_px': np.percentile(
                     np.abs(a[:, i]-np.median(a[:, i], axis=0)), 95, axis=0).tolist()}
                for i in range(3)]
    candidates = [r for r in rows if r['usable_for_pixel_statistics']]
    stable = [r for r in rows if r['pixel_ready']]
    assert len(candidates) == summary['usable_frames']
    assert len(stable) == summary['pixel_ready_frames']
    assert stats(candidates) == summary['ring_statistics']
    assert stats(stable) == summary['stable_ring_statistics']
    segments = []
    for r in rows:
        key = [r['state'], r['pixel_ready']]
        t = r['host_read_time']-stamps[0]
        if not segments or segments[-1]['key'] != key:
            segments.append(dict(key=key, first_frame=r['frame_index'], last_frame=r['frame_index'],
                                 start_s=t, end_s=t, count=1))
        else:
            segments[-1]['last_frame'] = r['frame_index']
            segments[-1]['end_s'] = t
            segments[-1]['count'] += 1
    first = [r for r in stable if r['host_read_time']-stamps[0] <= 5]
    tail = [r for r in stable if stamps[-1]-r['host_read_time'] <= 10]
    first_stats, tail_stats = stats(first), stats(tail)
    first_xy = np.array([s['median_xy_px'] for s in first_stats])
    tail_xy = np.array([s['median_xy_px'] for s in tail_stats])
    angle = lambda p: float(np.degrees(np.arctan2(*(p[2]-p[0])[::-1])))
    first_angle = angle(first_xy) if len(first_xy) == 3 else None
    tail_angle = angle(tail_xy) if len(tail_xy) == 3 else None
    report = {
        'run_name': run_name, 'label': label, 'zip_sha256': sha(archive),
        'crc_and_member_bytes_match': True, 'source_matches': matches, 'config_matches': configs,
        'frames': len(rows), 'fresh_candidate_frames': len(candidates), 'pixel_ready_frames': len(stable),
        'not_ready_frames': len(rows)-len(stable), 'states': summary['states'],
        'all_fresh_candidate_statistics': stats(candidates),
        'all_ready_statistics': stats(stable),
        'detail_counts': dict(Counter(r['detail'] for r in rows)),
        'read_completion_span_s': float(stamps[-1]-stamps[0]), 'segments': segments,
        'first_5s_ready_count': len(first), 'first_5s_ready_statistics': first_stats,
        'last_10s_ready_count': len(tail), 'last_10s_ready_statistics': tail_stats,
        'last_10s_total_count': sum(bool(stamps[-1]-r['host_read_time'] <= 10) for r in rows),
        'tail_minus_initial_median_xy_px': (tail_xy-first_xy).tolist() if len(first_xy)==len(tail_xy)==3 else None,
        'initial_image_line_angle_deg': first_angle, 'tail_image_line_angle_deg': tail_angle,
        'image_line_angle_change_deg': tail_angle-first_angle if first_angle is not None and tail_angle is not None else None,
        'not_ready_always_empty_stable_centers': True,
        'read_interval_p50_p95_max_ms': np.percentile(np.diff(stamps)*1000, [50,95,100]).tolist(),
        'host_result_age_p50_p95_max_ms': np.percentile([r['result_host_age_ms'] for r in rows], [50,95,100]).tolist(),
        'detector_processing_p50_p95_max_ms': np.percentile([r['detector_processing_ms'] for r in rows], [50,95,100]).tolist(),
        'limits': ['PixelReady is original short-window stability plus freshness/quality, not an independent stationary detector.',
                   'If paper pose changes, global statistics cannot measure stationary jitter; inspect phases separately.',
                   'No measured hand-motion or settle timestamps; no settle-to-recovery latency.',
                   'First stable reappearance image may precede the final settled pose; inspect its matching JSON.',
                   'Host timestamps are not camera exposure timestamps; no metric or physical motion acceptance.'],
        'metric_valid': False, 'motion_allowed': False,
    }
    (base / 'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-name', default='20261027-193422-279783')
    parser.add_argument('--label', choices=('ring-rotation', 'ring-shadow', 'ring-empty'), default='ring-rotation')
    args = parser.parse_args()
    main(args.run_name, args.label)
