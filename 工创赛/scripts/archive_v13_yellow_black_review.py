"""Archive yellow/black cross-placement evidence without accepting handling losses."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'evidence/20261007-063948-136571.zip'
out = ROOT / 'output/v13-yellow-black-review-20261007'
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
    start = None
    episodes = []
    for i, row in enumerate(rows):
        good = sorted(t['color'] for t in row['targets']) == ['black', 'yellow']
        if good and start is None:
            start = i
        if start is not None and (not good or i == len(rows) - 1):
            end = i - 1 if not good else i
            episodes.append({'first_frame': start + 1, 'last_frame': end + 1,
                             'duration_s': rows[end]['timestamp'] - rows[start]['timestamp']})
            start = None
    report = {
        'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'matches_v13': matched, 'run': json.loads(z.read('run.json')),
        'result_count': len(rows), 'duration_s': rows[-1]['timestamp'] - rows[0]['timestamp'],
        'observed_labels': dict(Counter(t['color'] for r in rows for t in r['targets'])),
        'target_combinations': [{'colours': list(k), 'count': v} for k, v in
                                Counter(tuple(sorted(t['color'] for t in r['targets'])) for r in rows).items()],
        'complete_pair_episodes': episodes,
        'last_positions': [{k: t[k] for k in ('color', 'id', 'center_px')} for t in rows[-1]['targets']],
        'result_host_age_ms_p50_p95_max': np.percentile([r['result_host_age_ms'] for r in rows], [50, 95, 100]).tolist(),
        'scope': 'Separated static cross-placement supported. Black handling losses, red hand false positives, glare and identity remain unresolved; no whole-run or grasp acceptance.'
    }
    for name in ('material_color_thresholds.json', 'camera_processing.json', 'project_profile.json', 'module_source_hashes.json'):
        (out / name).write_bytes(z.read(name))
(out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({k: report[k] for k in ('result_count', 'duration_s', 'observed_labels', 'complete_pair_episodes')}, ensure_ascii=False))
print(out / 'report.json')
