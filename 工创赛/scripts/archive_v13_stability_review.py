"""Archive the observed stability span without inflating it to five minutes."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'evidence/20261008-130216-931830.zip'
out = ROOT / 'output/v13-stability-review-20261008'
out.mkdir(parents=True, exist_ok=True)
with ZipFile(source) as z, ZipFile(ROOT / 'output/black_overlap_patch_v13_2026-10-07.zip') as package:
    assert z.testzip() is None
    rows = [json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes = json.loads(z.read('module_source_hashes.json'))
    manifest = json.loads(package.read('PATCH_MANIFEST.json'))
    matched = {name: hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest() == entry['after_sha256']
               if name.endswith('.json') else hashes[Path(name).name] == entry['after_sha256']
               for name, entry in manifest['files'].items()}
    assert all(matched.values()), matched
    previous = json.loads((ROOT / 'output/v13-reconnect-review-20261008/module_source_hashes.json').read_text(encoding='utf-8'))
    assert hashes == previous
    assert all(r['ready'] is False and r['motion_allowed'] is False for r in rows)
    duration = rows[-1]['timestamp'] - rows[0]['timestamp']
    gaps = [{'previous_frame': i + 1, 'next_frame': i + 2, 'duration_s': b['timestamp'] - a['timestamp']}
            for i, (a, b) in enumerate(zip(rows, rows[1:]))]
    changes = [{'previous_frame': i + 1, 'next_frame': i + 2,
                'previous_ids': [t['id'] for t in a['targets']], 'next_ids': [t['id'] for t in b['targets']]}
               for i, (a, b) in enumerate(zip(rows, rows[1:]))
               if [t['id'] for t in a['targets']] != [t['id'] for t in b['targets']]]
    report = {
        'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'matches_v13': matched, 'all_module_hashes_match_reconnect_run': hashes == previous,
        'run': json.loads(z.read('run.json')), 'result_count': len(rows), 'duration_s': duration,
        'full_five_minute_observation_span': duration >= 300,
        'states': dict(Counter(r['state'] for r in rows)),
        'target_combinations': [{'labels': list(k), 'count': v} for k, v in
                                Counter(tuple(t['color'] for t in r['targets']) for r in rows).items()],
        'ids': dict(Counter(t['id'] for r in rows for t in r['targets'])),
        'id_transitions': changes,
        'host_age_ms_p50_p95_max': np.percentile([r['result_host_age_ms'] for r in rows], [50, 95, 100]).tolist(),
        'detector_call_ms_p50_p95_max': np.percentile([r['detector_call_ms'] for r in rows], [50, 95, 100]).tolist(),
        'largest_observation_gaps': sorted(gaps, key=lambda x: x['duration_s'], reverse=True)[:10],
        'console_log_present': (ROOT / 'evidence/follow-stability-console-20261008.log').exists(),
        'scope': 'Observed 239-second ground A4 white-paper stability only, not a five-minute completion. No long gap in detection rows; V4L/JPEG/connection events need separate console. Static successful detections do not prove unique sensor frames or exposure latency. Actions disabled.'
    }
    for name in ('material_color_thresholds.json', 'camera_processing.json', 'project_profile.json', 'module_source_hashes.json', 'run.json', 'first-frame-raw.png', 'sample-08-raw.png'):
        (out / name).write_bytes(z.read(name))
console = ROOT / 'evidence/follow-stability-console-20261008.log'
if console.exists():
    data = console.read_bytes()
    lines = data.decode('utf-8-sig').splitlines()
    starts = [i for i, line in enumerate(lines) if line.startswith('Diagnostic folder: ')]
    selected = [i for i in starts if lines[i].endswith('/' + source.stem)]
    assert len(selected) == 1
    start = selected[0]
    end = next((i for i in starts if i > start), len(lines))
    segment = lines[start:end]
    warnings = [line for line in segment if re.search(r'warn|error|corrupt|jpeg|failed|invalid', line, re.I)]
    connected = sum(line.startswith('Camera connected:') for line in segment)
    unavailable = sum(line.startswith('Camera unavailable ') for line in segment)
    waiting = sum('Camera=WAITING' in line for line in segment)
    modes = sorted(set(re.findall(r'Mode=(\w+)', '\n'.join(segment))))
    changes = [line for line in segment if line.startswith('Mode changed:')]
    assert connected == 1 and unavailable == 0 and waiting == 0
    assert modes == ['FOLLOW'] and not changes and not warnings
    report['console_review'] = {
        'source': str(console), 'sha256': hashlib.sha256(data).hexdigest(),
        'first_source_line': start + 1, 'last_source_line': end,
        'connected_count': connected, 'unavailable_count': unavailable, 'waiting_count': waiting,
        'modes': modes, 'mode_change_lines': changes, 'warning_lines': warnings,
        'logged_target_counts': sorted(set(re.findall(r'targets=(\d+)', '\n'.join(segment)))),
        'limit': 'No warning/event found in selected console segment; no sensor timestamp, unique frame or full 300-second claim.'
    }
    (out / 'console-selected-run.log').write_text('\n'.join(segment) + '\n', encoding='utf-8')
(out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: report[k] for k in ('result_count', 'duration_s', 'ids', 'full_five_minute_observation_span', 'console_log_present')}, ensure_ascii=False))
print(out / 'report.json')
