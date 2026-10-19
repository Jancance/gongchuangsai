"""Reproduce contact-loss analysis against the immutable deployed baseline."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics
import sys
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze

source=ROOT/'evidence/20261006-051539-184251.zip'
out=ROOT/'output/follow-touch-analysis-20261006'
out.mkdir(parents=True,exist_ok=True)
with ZipFile(ROOT/'output/vision_app_decxin_2026-10-03.zip') as baseline:
    old=type(sys)('baseline_detector')
    exec(compile(baseline.read('vision_app/code/project_detector.py'),'baseline_detector','exec'),old.__dict__)
with ZipFile(source) as archive:
    assert archive.testzip() is None
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode('utf-8').splitlines()]
    cfg=json.loads(archive.read('material_color_thresholds.json'))
    run=json.loads(archive.read('run.json'))
    images=[]
    for name in sorted(archive.namelist()):
        if not name.endswith('-raw.png'):continue
        frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),cv2.IMREAD_COLOR)
        before,drawing_before=old.analyze(frame,cfg)
        after,drawing_after=analyze(frame,cfg)
        known=lambda v:[r for r in v['regions'] if r['label']!='UNKNOWN']
        images.append({'file':name,'old_known':known(before),'new_known':known(after),
                       'old_regions':before['regions'],'new_regions':after['regions']})
        if name=='first-red-loss-raw.png':
            cv2.imencode('.png',drawing_before)[1].tofile(out/'first-loss-old-overlay.png')
            cv2.imencode('.png',drawing_after)[1].tofile(out/'first-loss-candidate-overlay.png')
    first_loss=json.loads(archive.read('first-red-loss.json'))
episodes=[]
lost=None
previous_red=False
for row in rows:
    red=any(t['color']=='red' for t in row['targets'])
    if previous_red and not red:lost=row
    if red and lost is not None:
        episodes.append({'start_frame':lost['frame_index'],'reappearance_frame':row['frame_index'],
                         'duration_s':row['timestamp']-lost['timestamp'],
                         'reappearance_ids':[t['id'] for t in row['targets']]})
        lost=None
    previous_red=red
timing={}
for key in ('detector_call_ms','result_host_age_ms','diagnostic_io_ms'):
    values=sorted(r[key] for r in rows)
    timing[key]={'median':statistics.median(values),'p95':values[int(.95*(len(values)-1))],'max':max(values)}
report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'run':run,'states':dict(Counter(r['state'] for r in rows)),
        'rejection_counts':dict(Counter(r['reason'] for row in rows for r in row['detector_regions'])),
        'first_loss':first_loss,'loss_episodes':episodes,'timing':timing,'saved_frame_replay':images,
        'scope':'Seven event PNGs are development replay, not a replay of all 430 captured frames. '
                'No sensor latency, board candidate performance or physical grasp acceptance.'}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved:',out/'report.json')
