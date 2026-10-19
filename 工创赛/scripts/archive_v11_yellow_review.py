"""Retain yellow working feedback without turning short losses into a pass."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261007-003307-733760.zip'
out=ROOT/'output/v11-yellow-moving-review-20261007';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/independent_blue_patch_v11_2026-10-06.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'));manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matches={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256'] for name,entry in manifest['files'].items()}
    gaps=[];changes=[];start=None;last=None
    for i,r in enumerate(rows):
        targets=[t for t in r['targets'] if t['color']=='yellow']
        if not targets and start is None:start=i
        if targets:
            if start is not None:
                gaps.append({'first_missing_frame':start+1,'return_frame':i+1,'duration_s':r['timestamp']-rows[start]['timestamp']});start=None
            if last and last['id']!=targets[0]['id']:
                changes.append({'frame':i+1,'previous_id':last['id'],'current_id':targets[0]['id'],
                                'observed_reference_distance_px':float(np.linalg.norm(np.array(last['center_px'])-targets[0]['center_px']))})
            last=targets[0]
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'matches_v11':matches,
            'run':json.loads(z.read('run.json')),'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'duration_s':rows[-1]['timestamp']-rows[0]['timestamp'],'yellow_gaps':gaps,'yellow_id_changes':changes,
            'result_host_age_ms_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[95,100]).tolist(),
            'scope':'Current tabletop yellow output review; short losses and hand/glare remain unresolved. No zero-loss, scene accuracy or grasp acceptance.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(matches.values()),matches
print('V11 matched; yellow loss episodes:',len(gaps),'ID changes:',len(changes))
print(out/'report.json')
