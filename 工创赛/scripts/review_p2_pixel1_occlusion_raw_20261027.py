"""复核pixel1遮挡原包并定位原算法拒绝层；不改应用、不访问硬件。"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from zipfile import ZipFile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic135_pixel1_20261027/vision_app/code'
BASE = ROOT / 'evidence/p2-camera/2026-10-27/pixel1-material-occlusion'
RUN = '20261027-205050-650002'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def groups(observations, config, module):
    # 严格复现原聚类的连通分组，同时保留被原函数过滤掉的组供诊断。
    parent = list(range(len(observations)))
    for i in range(len(observations)):
        for j in range(i):
            if np.linalg.norm(observations[i].center-observations[j].center) <= config.center_cluster_radius_px:
                a, b = module._find(parent, i), module._find(parent, j)
                if a != b:
                    parent[b] = a
    result = {}
    for i, item in enumerate(observations):
        result.setdefault(module._find(parent, i), []).append(item)
    report = []
    for items in result.values():
        bands = []
        for d in sorted(item.major_axis for item in items):
            if not bands or d-bands[-1] > config.diameter_band_gap_px:
                bands.append(d)
        report.append({
            'median_center_px': np.median([item.center for item in items], axis=0).tolist(),
            'support': len(items), 'diameter_bands': len(bands),
            'passes_cluster_gate': len(items) >= config.min_cluster_support and len(bands) >= config.min_diameter_bands,
        })
    return report


def main():
    BASE.mkdir(parents=True, exist_ok=True)
    archive = ROOT / (RUN+'.zip')
    data = archive.read_bytes()
    copy = BASE / archive.name
    if copy.exists():
        assert copy.read_bytes() == data
    else:
        copy.write_bytes(data)
    dest = BASE / ('run-'+RUN)
    dest.mkdir(exist_ok=True)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert Path(name).name == name and '/' not in name and '\\' not in name
            target, payload = dest/name, z.read(name)
            if target.exists():
                assert target.read_bytes() == payload
            else:
                target.write_bytes(payload)
    hashes = json.loads((dest/'module_source_hashes.json').read_text(encoding='utf8'))
    matches = {n: h == sha((ROOT/'output/p2_ring_pixel_audit_v2_20261027.py' if n == 'diagnostic_entry' else APP/n).read_bytes()) for n, h in hashes.items()}
    assert all(matches.values())
    configs = {}
    for name in ('camera_processing.json', 'project_profile.json', 'ring_station_detector.json'):
        provenance = json.loads((dest/(name+'.source.json')).read_text(encoding='utf8'))
        configs[name] = provenance['sha256'] == sha((APP/'config'/name).read_bytes()) and json.loads((dest/name).read_text(encoding='utf8')) == json.loads((APP/'config'/name).read_text(encoding='utf8'))
    assert all(configs.values())
    sys.path.insert(0, str(APP))
    import ring_station_detector as module
    cv2.ocl.setUseOpenCL(False)
    config = module.load_ring_station_config(dest/'ring_station_detector.json')
    rows = [json.loads(line) for line in (dest/'observations.jsonl').read_text(encoding='utf8').splitlines()]
    summary = json.loads((dest/'run.json').read_text(encoding='utf8'))
    assert len(rows) == summary['frames'] == 55
    assert dict(Counter(r['state'] for r in rows)) == summary['states']
    stamps = np.array([r['host_read_time'] for r in rows])
    assert np.all(np.diff(stamps)>0)
    assert [r['frame_index'] for r in rows] == list(range(1, len(rows)+1))
    for r in rows:
        valid = bool(r['detected'] and len(r['centers_px']) == 3 and np.isfinite(r['centers_px']).all() and all(0 <= x < 1920 and 0 <= y < 1080 for x,y in r['centers_px']) and 0 <= r['result_host_age_ms'] <= 150)
        ready = bool(valid and r['detector_stable'] and r['confidence'] >= config.minimum_confidence)
        assert valid == r['usable_for_pixel_statistics']
        assert ready == r['pixel_ready']
        assert r['minimum_confidence'] == config.minimum_confidence
        assert r['usable_centers_px'] == (r['centers_px'] if valid else [])
        assert r['stable_centers_px'] == (r['centers_px'] if ready else [])
        assert not r['metric_valid'] and not r['motion_allowed'] and r['serial'] == 'DISABLED'
    candidates = [r for r in rows if r['detected']]
    assert len(candidates) == summary['usable_frames'] == 4
    assert not any(r['pixel_ready'] for r in rows) and summary['pixel_ready_frames'] == 0
    snapshots = {}
    for stem in ('first-frame', 'first-three-centers', 'first-unusable', 'first-reappearance'):
        frame = cv2.imdecode(np.fromfile(dest/(stem+'-raw.png'), np.uint8), cv2.IMREAD_COLOR)
        assert frame.shape == (1080, 1920, 3)
        observations = module._extract_observations(frame, config)
        clusters = module._cluster_observations(observations, config)
        group_report = groups(observations, config, module)
        assert sum(g['passes_cluster_gate'] for g in group_report) == len(clusters)
        raw = module._raw_detection(frame, config)
        recorded = json.loads((dest/(stem+'.json')).read_text(encoding='utf8'))
        assert bool(raw) == recorded['detected']
        if raw:
            assert np.allclose(raw['centers'], recorded['centers_px'], atol=0.05, rtol=0)
            assert abs(raw['confidence']-recorded['confidence']) < 1e-5
        snapshots[stem] = {'frame_index': recorded['frame_index'], 'ellipses': len(observations), 'groups': group_report,
                           'qualified_candidates': len(clusters), 'raw_result': raw,
                           'rejection_stage': 'confidence' if raw and raw['confidence'] < config.minimum_confidence else 'cluster_or_triple'}
        # 黄框是原候选，不是稳定输出；画出各轮廓分组便于复核拒绝位置。
        for g in group_report:
            x,y = (int(round(v)) for v in g['median_center_px'])
            color = (0,190,255) if g['passes_cluster_gate'] else (0,0,255)
            cv2.drawMarker(frame, (x,y), color, cv2.MARKER_CROSS, 20, 2)
            cv2.putText(frame, f"support={g['support']} bands={g['diameter_bands']}", (x-90,y-20), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
        cv2.putText(frame, 'OFFLINE CANDIDATES ONLY - NO STABLE OUTPUT', (30,50), cv2.FONT_HERSHEY_SIMPLEX, .8, (0,0,255), 2)
        cv2.imencode('.png', frame)[1].tofile(BASE/(stem+'-rejection-diagnostic.png'))
    report = {'run': RUN, 'zip_sha256': sha(data), 'archive_crc_and_bytes_match': True,
              'source_matches_pixel1': matches, 'config_matches_pixel1': configs,
              'source_coverage_limit': 'project_ring_pixel.py and imported diagnostic helper hashes were not recorded by this recorder; no claim of full deployment match.',
              'opencv_local': cv2.__version__, 'local_replay_cpu_only': True,
              'frames': len(rows), 'states': summary['states'], 'candidate_frame_indices': [r['frame_index'] for r in candidates],
              'candidate_confidences': [r['confidence'] for r in candidates],
              'minimum_confidence': config.minimum_confidence, 'pixel_ready_frames': 0,
              'all_stable_centers_empty': all(not r['stable_centers_px'] for r in rows),
              'host_result_age_p50_p95_max_ms': np.percentile([r['result_host_age_ms'] for r in rows], [50,95,100]).tolist(),
              'snapshots': snapshots,
              'limits': ['Only archived raw snapshots can be replayed, not all 55 input images.',
                         'No same-pose ground truth; candidate error is unmeasured.',
                         'Some original detail strings contain replacement characters; numeric fields remain primary evidence.',
                         'first-reappearance is a candidate reappearance, not stable recovery.',
                         'No threshold, application, camera parameter, metric gate or actuator changed.']}
    (BASE/'raw-review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({k: report[k] for k in ('frames','states','candidate_frame_indices','candidate_confidences','pixel_ready_frames','source_matches_pixel1','config_matches_pixel1')}, indent=2))
    for name, value in snapshots.items():
        print(name, json.dumps({'candidates': value['qualified_candidates'], 'groups': value['groups']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
