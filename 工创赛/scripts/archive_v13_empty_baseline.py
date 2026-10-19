"""Archive pure empty-scene evidence; no claim about hands or all backgrounds."""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261007-061026-046572.zip'
out=ROOT/'output/v13-empty-baseline-20261007';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/black_overlap_patch_v13_2026-10-07.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'));manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matched={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256'] for name,entry in manifest['files'].items()}
    matched.update({name:hashes[Path(name).name]==digest for name,digest in manifest['preserve_sha256'].items()})
    run=json.loads(z.read('run.json'))
    checks={'v13_matches':all(matched.values()),'no_valid_target_any_colour':all(not r['targets'] for r in rows),
            'all_empty_states_searching':all(r['state']=='SEARCHING' for r in rows),
            'all_host_ages_within_150ms':all(0<=r['result_host_age_ms']<=150 for r in rows),
            'no_action':all(not r['ready'] and not r['motion_allowed'] for r in rows) and run['serial']=='DISABLED'}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'run':run,
            'module_matches':matched,'checks':checks,'duration_s':rows[-1]['timestamp']-rows[0]['timestamp'],
            'result_host_age_ms_p50_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
            'detector_ms_p50_p95_max':np.percentile([r['detector_call_ms'] for r in rows],[50,95,100]).tolist(),
            'scope':'Pure current desk background/lighting with hands absent. Not hand-only, other backgrounds, cross-colour or grasp acceptance.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(checks.values()),checks
print('PASS pure empty scene:',len(rows),'results; max age ms:',report['result_host_age_ms_p50_p95_max'][-1])
print(out/'report.json')
