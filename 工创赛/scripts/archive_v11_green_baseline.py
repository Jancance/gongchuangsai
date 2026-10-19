"""Archive green continuity while retaining the extra hand candidate."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261007-002007-035470.zip'
out=ROOT/'output/v11-green-moving-baseline-20261007';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/independent_blue_patch_v11_2026-10-06.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'));manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matches={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256'] for name,entry in manifest['files'].items()}
    labels=Counter(t['color'] for r in rows for t in r['targets'])
    ids=sorted({t['id'] for r in rows for t in r['targets'] if t['color']=='green'})
    checks={'v11_matches':all(matches.values()),'one_green_every_result':all(sum(t['color']=='green' for t in r['targets'])==1 for r in rows),
            'one_green_id':len(ids)==1,'all_results_fresh':all(r['state']=='FOLLOWING_PIXEL' and 0<=r['result_host_age_ms']<=150 for r in rows),
            'all_action_gates_off':all(not r['ready'] and not r['motion_allowed'] for r in rows)}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'run':json.loads(z.read('run.json')),
            'matches':matches,'checks':checks,'observed_labels':dict(labels),'green_ids':ids,
            'duration_s':rows[-1]['timestamp']-rows[0]['timestamp'],
            'result_host_age_ms_p50_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
            'detector_ms_p50_p95_max':np.percentile([r['detector_call_ms'] for r in rows],[50,95,100]).tolist(),
            'remaining_negative':'One additional red hand candidate; not a zero-false-positive claim.',
            'scope':'Current tabletop green movement/lighting only. Not all glare/edge/multi-instance or physical identity/grasp acceptance.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(checks.values()),checks
print('PASS green desk continuity:',len(rows),'results, IDs',ids)
print(out/'report.json')
