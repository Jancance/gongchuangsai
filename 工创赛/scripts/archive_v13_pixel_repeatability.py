"""Describe observed pixel-centre variation; no metric or grasp accuracy claim."""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'evidence/20261008-130216-931830.zip'
out = ROOT / 'output/v13-pixel-repeatability-20261008'
out.mkdir(parents=True, exist_ok=True)
with ZipFile(source) as z:
    assert z.testzip() is None
    rows = [json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    assert all(len(r['targets']) == 1 and r['targets'][0]['color'] == 'green' and
               r['targets'][0]['id'] == 'T001' and not r['ready'] and not r['motion_allowed'] for r in rows)
    targets = [r['targets'][0] for r in rows]
    xy = np.array([t['center_px'] for t in targets])
    times = np.array([r['timestamp'] for r in rows])
    median = np.median(xy, axis=0)
    deviations = np.abs(xy - median)
    blocks = []
    for begin in range(0, 240, 30):
        selected = (times - times[0] >= begin) & (times - times[0] < begin + 30)
        if selected.any():
            blocks.append({'start_s': begin, 'count': int(selected.sum()),
                           'median_center_px': np.median(xy[selected], axis=0).tolist()})
    early = np.median(xy[times - times[0] < 30], axis=0)
    late = np.median(xy[times[-1] - times < 30], axis=0)
    velocity = [t['velocity_px_s'] for t in targets if t['velocity_px_s'] is not None]
    report = {
        'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'count': len(rows), 'duration_s': float(times[-1] - times[0]),
        'median_center_px': median.tolist(),
        'axis_abs_deviation_px_p50_p95_max': np.percentile(deviations, [50, 95, 100], axis=0).tolist(),
        'axis_peak_to_peak_px': np.ptp(xy, axis=0).tolist(),
        'radial_deviation_px_p95_max': np.percentile(np.linalg.norm(xy - median, axis=1), [95, 100]).tolist(),
        'first30s_to_last30s_median_shift_px': (late - early).tolist(),
        'raw_speed_px_s_p50_p95_max': np.percentile(np.linalg.norm(velocity, axis=1), [50, 95, 100]).tolist(),
        'blocks30s': blocks,
        'scope': 'Ground A4 white-paper isolated test; static according to test instructions, not independently measured ground truth. Combined-silhouette centre, not verified top-face/TCP grasp point. Deviations from whole-run median describe output repeatability, not bias, distortion, mm accuracy or physical speed. Y p95 exceeds the CAL-04 candidate 1px target; no formal precision gate closed.'
    }
(out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: report[k] for k in ('count', 'median_center_px', 'axis_abs_deviation_px_p50_p95_max', 'first30s_to_last30s_median_shift_px')}, ensure_ascii=False))
