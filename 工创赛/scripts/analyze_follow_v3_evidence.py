"""Compare the deployed v2 detector and v3 candidate on both event archives."""
from collections import Counter
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
with ZipFile(ROOT/'output/follow_touch_patch_2026-10-06.zip') as archive:
    v2=type(sys)('v2_detector')
    exec(compile(archive.read('vision_app/code/project_detector.py'),'v2_detector','exec'),v2.__dict__)
out=ROOT/'output/follow-touch-v3-analysis-20261006'
out.mkdir(parents=True,exist_ok=True)
reports=[]
for name in ('20261006-051539-184251','20261006-053817-238086'):
    source=ROOT/'evidence'/(name+'.zip')
    with ZipFile(source) as archive:
        assert archive.testzip() is None
        rows=[json.loads(s) for s in archive.read('observations.jsonl').decode('utf-8').splitlines()]
        cfg=json.loads(archive.read('material_color_thresholds.json'))
        hints=json.loads(archive.read('first-red-visible.json'))['current']['targets']
        replay=[]
        for image in sorted(archive.namelist()):
            if not image.endswith('-raw.png'):continue
            frame=cv2.imdecode(np.frombuffer(archive.read(image),np.uint8),cv2.IMREAD_COLOR)
            old,_=v2.analyze(frame,cfg)
            candidate,overlay=analyze(frame,cfg,hints)
            known=lambda result:[r for r in result['regions'] if r['label']!='UNKNOWN']
            replay.append({'file':image,'v2_known':known(old),'v3_known':known(candidate),
                           'hint_scope':'Offline development search using first-visible bbox; '
                                        'live age limited to 500ms by MaterialFollower.'})
            if name.endswith('238086') and image=='first-red-loss-raw.png':
                cv2.imencode('.png',overlay)[1].tofile(out/'same-color-touch-candidate-overlay.png')
        stats={}
        for key in ('detector_call_ms','result_host_age_ms','diagnostic_io_ms'):
            values=sorted(row[key] for row in rows)
            stats[key]={'p95':values[int(.95*(len(values)-1))],'max':max(values)}
        reports.append({'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'run':json.loads(archive.read('run.json')),'timing':stats,
            'states':dict(Counter(row['state'] for row in rows)),
            'proposal_sources':dict(Counter(r.get('proposal_source','ABSENT') for row in rows for r in row['detector_regions'])),
            'event_frame_replay':replay})
(out/'report.json').write_text(json.dumps({'candidate_version':VERSION,'runs':reports,
    'scope':'Event-frame development replay, not continuous replay, board runtime or grasp acceptance.'},
    ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved:',out/'report.json')
