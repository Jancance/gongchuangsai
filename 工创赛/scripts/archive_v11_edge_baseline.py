"""Record the successful independent-blue gate separately from pale limits."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261006-233915-492000.zip'
out=ROOT/'output/v11-independent-blue-baseline-20261006';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/independent_blue_patch_v11_2026-10-06.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'))
    manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matches={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256']
             for name,entry in manifest['files'].items()}
    blue_ids=sorted({t['id'] for r in rows for t in r['targets'] if t['color']=='blue'})
    checks={'v11_package_matches':all(matches.values()),
            'blue_present_in_every_result_after_startup':all(sum(t['color']=='blue' for t in r['targets'])==1 for r in rows[1:]),
            'blue_has_one_id':len(blue_ids)==1,
            'all_results_after_startup_fresh':all(r['state']=='FOLLOWING_PIXEL' and 0<=r['result_host_age_ms']<=150 for r in rows[1:]),
            'all_action_gates_off':all(not r['ready'] and not r['motion_allowed'] for r in rows)}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'run':json.loads(z.read('run.json')),'matches':matches,'checks':checks,
            'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'blue_ids':blue_ids,'initial_result_state':rows[0]['state'],
            'blue_results_after_startup':len(rows)-1,
            'duration_s':rows[-1]['timestamp']-rows[0]['timestamp'],
            'result_host_age_ms_p50_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
            'post_startup_result_host_age_ms_max':max(r['result_host_age_ms'] for r in rows[1:]),
            'detector_ms_p50_p95_max':np.percentile([r['detector_call_ms'] for r in rows],[50,95,100]).tolist(),
            'scope':'Independent fixed blue remains observed during pale edge test. Not acceptance of all pale losses, blue motion or real grasp.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(checks.values()),checks
print('PASS independent blue after startup:',len(rows)-1,'results, IDs',blue_ids)
print('Post-startup max host age ms:',report['post_startup_result_host_age_ms_max'])
print(out/'report.json')
