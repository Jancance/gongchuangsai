"""Replay V3 event frames with only causally available search hints."""
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze,VERSION
source=ROOT/'evidence/20261006-055844-166424.zip'
out=ROOT/'output/follow-touch-v4-analysis-20261006'
out.mkdir(parents=True,exist_ok=True)
with ZipFile(ROOT/'output/follow_touch_patch_v3_2026-10-06.zip') as archive:
    v3=type(sys)('v3_detector')
    exec(compile(archive.read('vision_app/code/project_detector.py'),'v3_detector','exec'),v3.__dict__)
with ZipFile(source) as archive:
    assert archive.testzip() is None
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode('utf-8').splitlines()]
    cfg=json.loads(archive.read('material_color_thresholds.json'))
    run=json.loads(archive.read('run.json'))
    hashes=json.loads(archive.read('module_source_hashes.json'))
    replay=[]
    for name in sorted(archive.namelist()):
        if not name.endswith('-raw.png'):continue
        event=name.removesuffix('-raw.png').removesuffix('-before')
        meta=json.loads(archive.read(event+'.json'))
        observation=meta['previous'] if '-before-' in name else meta['current']
        prior={}
        for row in rows:
            if row['frame_index']>=observation['frame_index']:break
            for target in row['targets']:prior[target['color']]=target
        hints=[]
        for target in prior.values():
            if 0<=observation['timestamp']-target['read_time']<=.5:
                hint=copy.deepcopy(target)
                if hint.get('proposal_source')=='chromatic_circle':
                    hint['radius_px']=math.sqrt(hint['area_px2']/math.pi)
                hints.append(hint)
        frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
        old,_=v3.analyze(frame,cfg,hints)
        new,drawing=analyze(frame,cfg,hints)
        known=lambda result:[r for r in result['regions'] if r['label']!='UNKNOWN']
        replay.append({'file':name,'frame_index':observation['frame_index'],
            'recorded_runtime_state':observation['state'],'causal_hints':hints,
            'v3_known':known(old),'v4_known':known(new),'circle_diagnostics':new['circle_diagnostics']})
        if name=='first-red-loss-raw.png':
            cv2.imencode('.png',drawing)[1].tofile(out/'first-loss-v4-overlay.png')
    timing={}
    for key in ('result_host_age_ms','detector_call_ms','diagnostic_io_ms'):
        values=sorted(r[key] for r in rows)
        timing[key]={'p95':values[int(.95*(len(values)-1))],'max':max(values)}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'candidate_version':VERSION,'recorded_module_hashes':hashes,'run':run,'timing':timing,
        'states':dict(Counter(r['state'] for r in rows)),'event_replay':replay,
        'scope':'Only event PNGs, no future hints. Image classification replay does not '
                'remove recorded stale-frame gates or prove continuous board IDs.'}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved:',out/'report.json')
for row in replay:print(row['file'],[r['label'] for r in row['v3_known']],
                      '->',[r['label'] for r in row['v4_known']],'hints:',len(row['causal_hints']))
