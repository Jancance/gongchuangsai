"""Current-frame dark pale recovery; compare archived images against V9."""
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze,VERSION

def candidate(config):
    cfg=copy.deepcopy(config)
    cfg['round_face_color_candidate']['current_pale_component']=True
    cfg['round_face_color_candidate']['component_candidate_version']='v10-current-frame-pale-component'
    return cfg

def main():
    out=ROOT/'output/dark-pale-v10-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
    with ZipFile(ROOT/'output/follow_speed_patch_v9_2026-10-06.zip') as z:
        old=types.ModuleType('v9');exec(z.read('vision_app/code/project_detector.py'),old.__dict__)
    baseline=ROOT/'evidence/20261006-202015-651610.zip'
    with ZipFile(baseline) as z:cfg=json.loads(z.read('material_color_thresholds.json'))
    datasets=[]
    for suffix in ('083236-468596','085047-455357','091047-121270','191939-893382',
                   '193600-046816','195642-102168','201311-289701','202015-651610'):
        path=next((ROOT/'evidence').glob('*'+suffix+'.zip'))
        with ZipFile(path) as z:
            assert z.testzip() is None
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
            images=[]
            for name in z.namelist():
                if not name.endswith('-raw.png'):continue
                meta=name.replace('-before-raw.png','.json').replace('-raw.png','.json')
                packet=json.loads(z.read(meta)) if meta in z.namelist() else {}
                current=packet.get('previous' if '-before-' in name else 'current');hints=[]
                if current and current['frame_index']>1:
                    prev=rows[current['frame_index']-2]
                    hints=[t for t in prev.get('internal_tracks_not_actionable',[]) if 0<=current['timestamp']-t['read_time']<=.5]
                im=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                before,_=old.analyze(im,cfg,hints,draw_overlay=False)
                after,drawing=analyze(im,candidate(cfg),hints)
                known=lambda r:sorted(a['label'] for a in r['regions'] if a['label']!='UNKNOWN')
                images.append({'image':name,'v9':known(before),'v10':known(after),'current_regions':after['regions']})
                if suffix=='202015-651610' and name in ('first-lightblue-loss-raw.png','longest-lightblue-loss-before-raw.png'):
                    cv2.imencode('.png',drawing)[1].tofile(out/(name.replace('-raw','-v10-overlay')))
            datasets.append({'source':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                             'run':json.loads(z.read('run.json')),
                             'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
                             'images':images})
    report={'version':VERSION,'settings':candidate(cfg)['round_face_color_candidate'],'datasets':datasets,
            'scope':'Saved image development replay using prior recorded fresh hints; not all 964 frames or physical identity acceptance.'}
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for data in datasets:
        print(data['source'],len(data['images']),[(r['image'],r['v9'],r['v10']) for r in data['images'] if r['v9']!=r['v10']],flush=True)

if __name__=='__main__':main()
