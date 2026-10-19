"""Record complete-pair episodes without claiming all handling frames passed."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261007-061820-099415.zip'
out=ROOT/'output/v13-red-green-review-20261007';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/black_overlap_patch_v13_2026-10-07.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'));manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matched={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256'] for name,entry in manifest['files'].items()}
    start=None;episodes=[]
    for i,row in enumerate(rows):
        complete=sorted(t['color'] for t in row['targets'])==['green','red']
        if complete and start is None:start=i
        if start is not None and (not complete or i==len(rows)-1):
            end=i-1 if not complete else i
            episodes.append({'first_frame':start+1,'last_frame':end+1,'duration_s':rows[end]['timestamp']-rows[start]['timestamp']})
            start=None
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'matches_v13':matched,
            'run':json.loads(z.read('run.json')),'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'complete_pair_results':sum(sorted(t['color'] for t in r['targets'])==['green','red'] for r in rows),
            'green_only_results':sum([t['color'] for t in r['targets']]==['green'] for r in rows),
            'complete_pair_episodes':episodes,'last_positions':[{k:t[k] for k in ('color','id','center_px')} for t in rows[-1]['targets']],
            'result_host_age_ms_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[95,100]).tolist(),
            'scope':'Current separated red/green recognition and complete-pair spans. Handling losses are not automatically labelled static or motion; no whole-run zero-loss/identity/grasp acceptance.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(matched.values()),matched
print('V13 matched; complete red/green pair results:',report['complete_pair_results'],'/',len(rows))
print(out/'report.json')
