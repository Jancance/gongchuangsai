"""Direct blue-label event compared to the measured V6 baseline."""
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
source=ROOT/'evidence/20261006-183128-046231.zip'
out=ROOT/'output/round-face-v7-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
with ZipFile(ROOT/'output/lightblue_patch_v6_2026-10-06.zip') as package:
    old=type(sys)('v6_detector')
    exec(compile(package.read('vision_app/code/project_detector.py'),'v6_detector','exec'),old.__dict__)
with ZipFile(source) as archive:
    rows=[json.loads(s) for s in archive.read('observations.jsonl').decode().splitlines()]
    cfg=json.loads(archive.read('material_color_thresholds.json'))
    filename='first-lightblue-with-blue-raw.png'
    image=cv2.imdecode(np.frombuffer(archive.read(filename),np.uint8),1)
    before,_=old.analyze(image,cfg)
    after,overlay=analyze(image,cfg)
    known=lambda r:[a for a in r['regions'] if a['label']!='UNKNOWN']
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'run':json.loads(archive.read('run.json')),'module_hashes':json.loads(archive.read('module_source_hashes.json')),
        'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
        'image':filename,'candidate_version':VERSION,'v6_known':known(before),'v7_known':known(after),
        'scope':'Current-image colour test without history hints. Not 831-frame replay or board acceptance.'}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
cv2.imencode('.png',overlay)[1].tofile(out/'direct-lightblue-v7-overlay.png')
print('Saved:',out/'report.json')
