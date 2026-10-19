"""Archive FOLLOW reconnect evidence; observation gaps are not recovery timings."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'evidence/20261007-071416-797897.zip'
out = ROOT / 'output/v13-reconnect-review-20261008'
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
    previous_hashes = json.loads((ROOT / 'output/v13-yellow-black-review-20261007/module_source_hashes.json').read_text(encoding='utf-8'))
    module_matches = {name: value == previous_hashes.get(name) for name, value in hashes.items()}
    assert all(module_matches.values()), module_matches
    assert all(r['ready'] is False and r['motion_allowed'] is False for r in rows)
    transitions = []
    gaps = []
    for i, (a, b) in enumerate(zip(rows, rows[1:])):
        dt = b['timestamp'] - a['timestamp']
        old = [t['id'] for t in a['targets']]
        new = [t['id'] for t in b['targets']]
        item = {'previous_frame': i + 1, 'next_frame': i + 2, 'observation_gap_s': dt,
                'previous_ids': old, 'next_ids': new, 'next_targets': b['targets']}
        if dt > .5:
            gaps.append(item)
        if old != new:
            transitions.append(item)
    report = {
        'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'matches_v13': matched, 'module_matches_previous_v13': module_matches,
        'run': json.loads(z.read('run.json')), 'result_count': len(rows),
        'duration_s': rows[-1]['timestamp'] - rows[0]['timestamp'],
        'states': dict(Counter(r['state'] for r in rows)),
        'target_combinations': [{'labels': list(k), 'count': v} for k, v in
                                Counter(tuple(t['color'] for t in r['targets']) for r in rows).items()],
        'ids': dict(Counter(t['id'] for r in rows for t in r['targets'])),
        'host_age_ms_p50_p95_max': np.percentile([r['result_host_age_ms'] for r in rows], [50, 95, 100]).tolist(),
        'detector_call_ms_p50_p95_max': np.percentile([r['detector_call_ms'] for r in rows], [50, 95, 100]).tolist(),
        'gaps_over_half_second': gaps, 'id_transitions': transitions,
        'last_segment_duration_s': rows[-1]['timestamp'] - rows[gaps[-1]['next_frame'] - 1]['timestamp'] if gaps else None,
        'console_log_present': (ROOT / 'evidence/camera-reconnect-console-20261007.log').exists(),
        'scope': 'Ground A4 white-paper background. Green output recovery supported by user, screenshot and restarted candidate after a gap. Waiting states absent from observations; gap is not replug-to-recovery time. Earlier ID transition cause and same-run waiting/mode/USB events require missing console log. No actuator or metric calibration acceptance.'
    }
    for name in ('material_color_thresholds.json', 'camera_processing.json', 'project_profile.json', 'module_source_hashes.json', 'run.json'):
        (out / name).write_bytes(z.read(name))
console = ROOT / 'evidence/camera-reconnect-console-20261007.log'
if console.exists():
    data = console.read_bytes()
    lines = data.decode('utf-8-sig').splitlines()
    starts = [i for i, line in enumerate(lines) if line.startswith('Diagnostic folder: ')]
    run_id = source.stem
    selected = [i for i in starts if lines[i].rstrip().endswith('/' + run_id)]
    assert len(selected) == 1, 'Console must contain exactly one matching run'
    start = selected[0]
    end = next((i for i in starts if i > start), len(lines))
    segment = lines[start:end]
    connected = [i for i, line in enumerate(segment) if line.startswith('Camera connected:')]
    unavailable = [i for i, line in enumerate(segment) if line.startswith('Camera unavailable ')]
    waiting = [line for line in segment if 'Camera=WAITING' in line]
    changes = [line for line in segment if line.startswith('Mode changed:')]
    assert len(connected) == 2 and unavailable and connected[0] < unavailable[0] < connected[-1]
    assert waiting and all('Mode=FOLLOW ' in line for line in waiting)
    assert not changes and not any('Mode=TURNTABLE' in line for line in segment)
    report['console_review'] = {
        'source': str(console), 'sha256': hashlib.sha256(data).hexdigest(),
        'run_id': run_id, 'first_source_line': start + 1, 'last_source_line': end,
        'connected_count': len(connected), 'unavailable_count': len(unavailable),
        'waiting_count': len(waiting), 'mode_change_lines': changes,
        'mode_stays_follow_in_logged_status': True,
        'v4l2_no_device_warning_lines': [line for line in segment if 'errno=19' in line],
        'recovered_follow_status': next(line for line in segment[connected[-1] + 1:] if line.startswith('Mode=FOLLOW ')),
        'time_limit': 'Console has no line timestamps or physical replug marker; exact recovery latency remains unmeasured.',
        'early_id_reset_limit': 'No mode-change or camera event explains T001 to T002 before disconnect. Repeated F can reset same mode without printing Mode changed; cause not proven.'
    }
    (out / 'console-selected-run.log').write_text('\n'.join(segment) + '\n', encoding='utf-8')
(out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: report[k] for k in ('result_count', 'duration_s', 'ids', 'last_segment_duration_s', 'console_log_present')}, ensure_ascii=False))
print(out / 'report.json')
