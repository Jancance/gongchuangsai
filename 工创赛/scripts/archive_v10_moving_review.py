"""Archive observed improvement feedback separately from unresolved desk gates."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import types
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261006-204150-736167.zip'
out=ROOT/'output/v10-moving-review-20261006';out.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'vision_app/code'))
with ZipFile(ROOT/'output/dark_pale_patch_v10_2026-10-06.zip') as package:
    manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    detector=types.ModuleType('returned_v10')
    exec(package.read('vision_app/code/project_detector.py'),detector.__dict__)
with ZipFile(source) as z:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'))
    cfg=json.loads(z.read('material_color_thresholds.json'))
    matches={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256']
             for name,entry in manifest['files'].items()}
    trace=[]
    original=detector.hinted_color_circles
    def traced(*args,**kwargs):
        hints=args[4]
        if hints and hints[0].get('proposal_source')=='current_component_search':
            diagnostics=[];kwargs['diagnostics']=diagnostics
            result=original(*args,**kwargs)
            trace.append({'current_component_bbox':hints[0]['bbox_px'],'diagnostics':diagnostics,'accepted':len(result)})
            return result
        return original(*args,**kwargs)
    detector.hinted_color_circles=traced
    replay=[]
    for name in ('first-lightblue-loss-raw.png','longest-lightblue-loss-before-raw.png','longest-lightblue-loss-raw.png'):
        trace.clear()
        frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
        detector.analyze(frame,cfg,draw_overlay=False)
        replay.append({'image':name,'current_component_search_trace':list(trace),
                       'scope':'Independent PC current-image search trace, not camera replay.'})
    changes=[];last=None;gaps=[];start=None
    for i,row in enumerate(rows):
        ids=tuple(t['id'] for t in row['targets'] if t['color']=='lightblue')
        if not ids and start is None:start=i
        if ids:
            if start is not None:
                gaps.append({'first_missing_frame':start+1,'return_frame':i+1,
                             'duration_s':row['timestamp']-rows[start]['timestamp']})
                start=None
            if last and last!=ids:changes.append({'frame':i+1,'previous':last,'current':ids})
            last=ids
    first_gap=next((i for i,r in enumerate(rows) if sorted(t['color'] for t in r['targets'])!=['blue','lightblue']),len(rows))
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'matches_delivered_v10':matches,'run':json.loads(z.read('run.json')),
            'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'initial_continuous_pair_results':first_gap,
            'initial_pair_duration_s':rows[first_gap-1]['timestamp']-rows[0]['timestamp'] if first_gap else 0,
            'duration_s':rows[-1]['timestamp']-rows[0]['timestamp'],
            'lightblue_gaps':gaps,'lightblue_id_changes':changes,
            'result_host_age_ms_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[95,100]).tolist(),
            'current_image_failure_trace':replay,
            'user_feedback':'User reports much better effect; retain as qualitative feedback, not zero-loss acceptance.',
            'scope':'Desk output evidence only. Runs differ in motion/shadow and cannot establish a comparative accuracy score.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(matches.values()),matches
print('Archived V10 matched:',len(rows),'results; first stable pair',first_gap,'results')
print(out/'report.json')
