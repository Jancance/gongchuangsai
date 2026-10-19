"""Replay saved paired/legacy images; never claim unsaved-frame accuracy."""
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
SETTINGS={'version':'paired-face-candidate-v8-20261006','lightblue_s_max':189,
          'requires_live_validation':True}

def candidate(config):
    result=copy.deepcopy(config)
    result['round_face_color_candidate']=dict(SETTINGS)
    return result

def main():
    out=ROOT/'output/paired-color-v8-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
    with ZipFile(ROOT/'output/round_face_patch_v7_2026-10-06.zip') as package:
        old=type(sys)('v7');exec(compile(package.read('vision_app/code/project_detector.py'),'v7','exec'),old.__dict__)
    datasets=[]
    for suffix in ('083236-468596','085047-455357','091047-121270','092130-937051',
                   '183128-046231','191939-893382','193600-046816'):
        path=next((ROOT/'evidence').glob('*'+suffix+'.zip'))
        with ZipFile(path) as archive:
            assert archive.testzip() is None
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            rows=[json.loads(s) for s in archive.read('observations.jsonl').decode().splitlines()]
            images=[]
            for name in archive.namelist():
                if not name.endswith('-raw.png'):continue
                hints=[]
                metadata=name.replace('-before-raw.png','.json').replace('-raw.png','.json')
                if metadata in archive.namelist():
                    packet=json.loads(archive.read(metadata))
                    current=packet.get('previous' if '-before-' in name else 'current')
                    if current:
                        previous={}
                        for row in rows:
                            if row['frame_index']>=current['frame_index']:break
                            for t in row['targets']:previous[t['color']]=t
                        hints=[t for t in previous.values() if 0<=current['timestamp']-t['read_time']<=.5]
                frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
                before,_=old.analyze(frame,cfg,hints)
                after,drawing=analyze(frame,candidate(cfg),hints)
                known=lambda r:sorted(x['label'] for x in r['regions'] if x['label']!='UNKNOWN')
                images.append({'file':name,'v7':known(before),'v8':known(after),
                               'v8_regions':after['regions'],'fresh_recorded_hint_count':len(hints)})
                if suffix=='193600-046816' and name in ('sample-03-raw.png','sample-09-raw.png'):
                    cv2.imencode('.png',drawing)[1].tofile(out/(name.replace('-raw','-v8-overlay')))
            datasets.append({'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                             'run':json.loads(archive.read('run.json')),
                             'observed_labels':dict(Counter(t['color'] for row in rows for t in row['targets'])),
                             'images':images})
    report={'version':VERSION,'settings':SETTINGS,'datasets':datasets,
            'scope':'Selected saved development images with original recorded fresh hints; not full frame replay or hardware acceptance.'}
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for data in datasets:
        changes=[(r['file'],r['v7'],r['v8']) for r in data['images'] if r['v7']!=r['v8']]
        print(data['file'],len(data['images']),'changes:',changes)
    print(out/'report.json')

if __name__=='__main__':main()
