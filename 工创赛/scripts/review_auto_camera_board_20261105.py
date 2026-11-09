"""Review the mixed manual/auto camera run without assigning an unknown switch time."""
from pathlib import Path
from collections import Counter
import hashlib
import importlib.util
import json
import sys
import zipfile
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
E = ROOT/'evidence/p2-camera/2026-11-05/auto-camera'
archive = E/'20261105-161032-971488.zip'
APP = ROOT/'output/hsv-live-trial-20261105/vision_app/code'
sys.path.insert(0, str(APP))
import project_detector as detector
cv2.setNumThreads(2)
z = zipfile.ZipFile(archive)
assert z.testzip() is None
rows = [json.loads(line) for line in z.read('observations.jsonl').decode().splitlines()]
run = json.loads(z.read('run.json'))
manifest = json.loads(z.read('module_source_hashes.json'))
for name in ('project_detector.py', 'project_follow.py', 'project_tracker.py', 'project_camera.py'):
    assert hashlib.sha256((APP/name).read_bytes()).hexdigest() == manifest[name], name
assert len(rows) == run['frames']
fresh = [r for r in rows if r['state'] != 'STALE_RESULT']
stale = [r for r in rows if r['state'] == 'STALE_RESULT']
assert all(not r['targets'] and r['result_host_age_ms'] >= 150 for r in stale)
def percentiles(values):
    return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50,95,100]))))
summary = dict(zip_bytes=archive.stat().st_size, zip_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
               frames=len(rows), fresh=len(fresh), stale=len(stale), stale_fraction=len(stale)/len(rows),
               states=dict(Counter(r['state'] for r in rows)),
               fresh_target_sets=dict(Counter('+'.join(sorted(t['color'] for t in r['targets'])) or 'none' for r in fresh)),
               fresh_color_presence={c:sum(any(t['color']==c for t in r['targets']) for r in fresh)
                                     for c in ('red','yellow','blue','green','black','lightblue')},
               age_ms=percentiles([r['result_host_age_ms'] for r in rows]),
               detector_call_ms=percentiles([r['detector_call_ms'] for r in rows]),
               diagnostic_io_ms=percentiles([r['diagnostic_io_ms'] for r in rows if 'diagnostic_io_ms' in r]),
               run=run, exact_four_module_hashes=True,
               helper_hash_in_archive='project_hsv_masks.py' in manifest,
               current_controls_user_reported=dict(exposure_auto=3, exposure_auto_priority=0,
                    exposure_absolute=50, gain=32, white_balance_temperature_auto=1, white_balance_temperature=4600),
               controls_note='Post-run readback, not per-frame sensor exposure. Switch time unknown; mixed settings.',
               acceptance=False)
(E/'board-review.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k != 'run'}, indent=2), flush=True)

cfg = json.loads(z.read('material_color_thresholds.json'))
lines = Path(detector.__file__).read_text(encoding='utf-8').splitlines()
trace_rows = []
current = None
def trace(frame, event, arg):
    if event == 'line' and frame.f_code.co_name == 'chromatic_fallback':
        line = lines[frame.f_lineno-1]
        if ('if circularity<.60 or solidity<.90' in line or 'if ranking[0]!=label' in line
                or "if (projected_blue or" in line):
            d = frame.f_locals
            if d.get('color_id') == 3 and 250 < d.get('rx', 0) < 850:
                values = {k:d[k] for k in ('rx','ry','rw','rh','circularity','solidity','scores','projected_blue') if k in d}
                values['perimeter_pass'] = bool((d.get('blue_settings') or {}).get('_perimeter_pass',False))
                trace_rows.append(dict(image=current, line=frame.f_lineno, values=values))
    return trace
replays = []
for current in ('sample-01-raw.png', 'sample-02-raw.png', 'sample-03-raw.png', 'sample-04-raw.png',
                'first-blue-visible-raw.png', 'first-blue-loss-raw.png', 'longest-blue-loss-raw.png',
                'first-duplicate-lightblue-raw.png'):
    image = cv2.imdecode(np.frombuffer(z.read(current),np.uint8),cv2.IMREAD_COLOR)
    sys.settrace(trace)
    try:
        result, _ = detector.analyze(image, cfg, draw_overlay=False)
    finally:
        sys.settrace(None)
    replays.append(dict(image=current, regions=result['regions']))
(E/'blue-fallback-trace.json').write_text(json.dumps(trace_rows, indent=2), encoding='utf-8')
(E/'offline-regions.json').write_text(json.dumps(replays, indent=2), encoding='utf-8')
print('TRACE',json.dumps(trace_rows, indent=2),flush=True)
