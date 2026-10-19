"""Archive the V7 board result and explicitly selected top-face color samples."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261006-191939-893382.zip'
out=ROOT/'output/v7-board-color-analysis-20261006'
out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as archive:
    if archive.testzip() is not None:raise RuntimeError('CRC failure')
    rows=[json.loads(line) for line in archive.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(archive.read('module_source_hashes.json'))
    run=json.loads(archive.read('run.json'))
    samples=[]
    # 人工从已查看原图选取同物料顶面内圆；不是自动顶面/抓取点验证。
    for name,x,y,r in [('first-lightblue-with-blue',700,168,75),
                       ('first-lightblue-visible',705,373,75),
                       ('first-lightblue-with-red',628,177,75)]:
        im=cv2.imdecode(np.frombuffer(archive.read(name+'-raw.png'),np.uint8),1)
        hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
        yy,xx=np.ogrid[:im.shape[0],:im.shape[1]]
        mask=(xx-x)**2+(yy-y)**2<r*r
        values=hsv[mask]
        samples.append({'image':name+'-raw.png','manually_selected_disk_px':[x,y,r],
                        'HSV_p10_p50_p90':np.percentile(values,[10,50,90],axis=0).tolist(),
                        'fraction_S_below_150':float((values[:,1]<150).mean())})
    versions={name:hashlib.sha256((ROOT/'vision_app/code'/name).read_bytes()).hexdigest()==hashes[name]
              for name in ('project_detector.py','project_tracker.py')}
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'crc_passed':True,'current_v7_hash_matches':versions,'run':run,
            'observed_labels':dict(Counter(t['color'] for row in rows for t in row['targets'])),
            'observed_ids':dict(Counter(t['color']+':'+t['id'] for row in rows for t in row['targets'])),
            'result_host_age_ms_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[95,100]).tolist(),
            'manual_top_samples':samples,
            'scope':'Saved raw samples and logged outputs only, not full image replay or calibrated scene accuracy.',
            'limitations':['Watched color is diagnostic intent, not a general ground-truth label.',
                           'Longest watched loss only measures completed loss/reappearance episodes; initial and final absence excluded.',
                           'Lighting cause cannot be separated from position/view change without paired capture.',
                           'Hand red false positive remains unresolved. Metric/action/field acceptance unavailable.']}
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(out/'report.json')
