"""Record the static misclassification and replay the measured candidate."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'vision_app/code'))
from make_lightblue_color_candidate import make_candidate
from project_detector import analyze
source=ROOT/'evidence/20261006-092130-937051.zip'
out=ROOT/'output/lightblue-v6-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as archive:
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode().splitlines()]
    original=json.loads(archive.read('material_color_thresholds.json'))
    candidate=make_candidate(original);replay=[]
    for name in archive.namelist():
        if not name.endswith('-raw.png'):continue
        frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
        result,drawing=analyze(frame,candidate)
        replay.append({'file':name,'known_regions':[r for r in result['regions'] if r['label']!='UNKNOWN']})
        if name=='first-other-color-candidate-raw.png':
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)[290:430,570:710].reshape(-1,3)
            sample=hsv[(hsv[:,2]>=60)&(hsv[:,2]<250)]
            percentiles=np.percentile(sample,[10,25,50,75,90],axis=0).tolist()
            cv2.imencode('.png',drawing)[1].tofile(out/'lightblue-v6-overlay.png')
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'run':json.loads(archive.read('run.json')),
            'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'development_top_roi_xywh':[570,290,140,140],'top_hsv_percentiles':percentiles,
            'candidate_blue_s_min':150,'candidate_lightblue_s_max':149,'saved_frame_replay':replay,
            'scope':'Three development images from one static scene, not live 223-frame acceptance.'}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Saved:',out/'report.json')
