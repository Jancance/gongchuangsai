"""Archive blue audit statistics and replay events using past-only hints."""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze,VERSION
source=ROOT/'evidence/20261006-085047-455357.zip'
out=ROOT/'output/blue-highlight-v5-analysis-20261006'
out.mkdir(parents=True,exist_ok=True)
with ZipFile(ROOT/'output/follow_touch_patch_v4_2026-10-06.zip') as archive:
    old=type(sys)('frozen_v4_detector')
    exec(compile(archive.read('vision_app/code/project_detector.py'),'frozen_v4_detector','exec'),old.__dict__)
with ZipFile(source) as archive:
    assert archive.testzip() is None
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode('utf-8').splitlines()]
    cfg=json.loads(archive.read('material_color_thresholds.json'))
    replay=[]
    for name in sorted(archive.namelist()):
        if not name.endswith('-raw.png'):continue
        event=name.removesuffix('-raw.png').removesuffix('-before')
        metadata=json.loads(archive.read(event+'.json'))
        observation=metadata['previous'] if '-before-' in name else metadata['current']
        prior={}
        for row in rows:
            if row['frame_index']>=observation['frame_index']:break
            for target in row['targets']:prior[target['color']]=copy.deepcopy(target)
        hints=[target for target in prior.values() if 0<=observation['timestamp']-target['read_time']<=.5]
        frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
        before,_=old.analyze(frame,cfg,hints);after,drawing=analyze(frame,cfg,hints)
        known=lambda r:[a for a in r['regions'] if a['label']!='UNKNOWN']
        replay.append({'file':name,'causal_hints':hints,'v4_known':known(before),
                       'v5_known':known(after),'circle_diagnostics':after['circle_diagnostics']})
        if name=='first-blue-loss-raw.png':
            cv2.imencode('.png',drawing)[1].tofile(out/'first-blue-loss-v5-overlay.png')
    timing={}
    for key in ('result_host_age_ms','detector_call_ms','diagnostic_io_ms'):
        a=sorted(row[key] for row in rows)
        timing[key]={'p95':a[int(.95*(len(a)-1))],'max':max(a)}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'candidate_version':VERSION,'run':json.loads(archive.read('run.json')),'timing':timing,
        'module_hashes':json.loads(archive.read('module_source_hashes.json')),
        'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
        'unexpected_color_rows':[r for r in rows if any(t['color']!='blue' for t in r['targets'])],
        'event_replay':replay,
        'scope':'Saved event development replay only; no board V5 runtime or all-frame acceptance.'}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved:',out/'report.json')
