"""复核已取回的 blue2 台架证据；不访问相机或执行器。"""
import collections
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'evidence/p2-camera/2026-10-27/blue2'
RUN = BASE / 'run-20261027-175625-750469'
APP = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app/code'
DIAG = ROOT / 'output/p2_generic135_rejection_audit_20261026'


def main(zip_name='20261027-175625-750469.zip', report_filename='review.json'):
    rows = [json.loads(s) for s in (RUN / 'observations.jsonl').read_text(encoding='utf-8').splitlines()]
    good = [r for r in rows if r['state'] != 'STALE_RESULT']
    stale = [r for r in rows if r['state'] == 'STALE_RESULT']
    report = {
        'source_zip': zip_name,
        'zip_sha256': hashlib.sha256((BASE / zip_name).read_bytes()).hexdigest(),
        'frames': len(rows), 'states': dict(collections.Counter(r['state'] for r in rows)),
        'read_completion_span_s': rows[-1]['timestamp'] - rows[0]['timestamp'],
        'scope': 'Desktop host-read-completion pixel evidence; no exposure latency or physical acceptance.',
        'target_color_counts': dict(collections.Counter('+'.join(sorted(t['color'] for t in r['targets'])) or 'NONE' for r in rows)),
        'timing_percentiles_p50_p95_max_ms': {
            k: np.percentile([r[k] for r in rows], [50, 95, 100]).tolist()
            for k in ('detector_call_ms', 'result_host_age_ms', 'diagnostic_io_ms')
        },
        'colors': {}, 'module_matches': {}, 'config_matches': {},
        'stale_outputs_all_empty': all(not r['targets'] for r in stale),
        'stale_min_age_ms': min(r['result_host_age_ms'] for r in stale),
        'normal_max_age_ms': max(r['result_host_age_ms'] for r in good),
        'no_motion_output': all(not r['motion_allowed'] and not r['ready'] for r in rows),
        'normal_duplicate_detector_frames': [r['frame_index'] for r in good if any(
            sum(t['label'] == c for t in r['detector_regions']) > 1 for c in ('blue', 'lightblue'))],
    }
    with zipfile.ZipFile(BASE / report['source_zip']) as z:
        report['zip_crc_bad_member'] = z.testzip()
    for color in ('blue', 'lightblue'):
        targets = [t for r in good for t in r['targets'] if t['color'] == color]
        report['colors'][color] = {
            'normal_missing_frames': [r['frame_index'] for r in good if not any(t['color'] == color for t in r['targets'])],
            'proposal_sources': dict(collections.Counter(t['proposal_source'] for t in targets)),
            'center_range_xy_px': np.ptp(np.array([t['center_px'] for t in targets]), axis=0).tolist(),
        }
    for name, expected in json.loads((RUN / 'module_source_hashes.json').read_text()).items():
        path = APP / name if name.endswith('.py') else DIAG / {
            'diagnostic_entry': 'p2_color_follow_audit.py',
            'diagnostic_base': 'p2_integrated_follow_audit.py',
        }[name]
        report['module_matches'][name] = hashlib.sha256(path.read_bytes()).hexdigest() == expected
    for name in ('camera_processing.json', 'material_color_thresholds.json', 'project_profile.json'):
        source = json.loads((RUN / (name + '.source.json')).read_text())
        report['config_matches'][name] = {
            'source_hash_matches_release': hashlib.sha256((APP / 'config' / name).read_bytes()).hexdigest() == source['sha256'],
            'effective_matches_release': json.loads((RUN / name).read_text(encoding='utf-8')) == json.loads((APP / 'config' / name).read_text(encoding='utf-8')),
        }
    assert len(rows) == json.loads((RUN / 'run.json').read_text())['frames']
    assert all(report['module_matches'].values())
    assert all(all(v.values()) for v in report['config_matches'].values())
    assert report['zip_crc_bad_member'] is None
    (BASE / report_filename).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
