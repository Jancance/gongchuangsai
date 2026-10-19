"""Verify independence without force-labeling or choosing a blue by history."""
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
    cfg['round_face_color_candidate']['reject_unverified_blue_family']=True
    cfg['round_face_color_candidate']['conflict_candidate_version']='v11-current-color-conflict-rejection'
    return cfg

def main():
    out=ROOT/'output/independent-blue-v11-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
    with ZipFile(ROOT/'output/dark_pale_patch_v10_2026-10-06.zip') as package:
        old=types.ModuleType('v10');exec(package.read('vision_app/code/project_detector.py'),old.__dict__)
    with ZipFile(ROOT/'evidence/20261006-204150-736167.zip') as z:
        cfg=json.loads(z.read('material_color_thresholds.json'))
    results=[]
    for suffix in ('083236-468596','085047-455357','091047-121270','201311-289701','202015-651610','204150-736167'):
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
                a,_=old.analyze(im,cfg,hints,draw_overlay=False);b,drawing=analyze(im,candidate(cfg),hints)
                known=lambda r:sorted(x['label'] for x in r['regions'] if x['label']!='UNKNOWN')
                images.append({'file':name,'v10':known(a),'v11':known(b),'regions':b['regions']})
                if suffix=='204150-736167' and name=='first-lightblue-loss-raw.png':
                    cv2.imencode('.png',drawing)[1].tofile(out/'independent-blue-overlay.png')
            results.append({'source':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'images':images})
    report={'version':VERSION,'datasets':results,'scope':'Saved-image development comparison only; unresolved pale circle remains UNKNOWN, no frame-wide tracker relaxation.'}
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in results:print(r['source'],len(r['images']),[(i['file'],i['v10'],i['v11']) for i in r['images'] if i['v10']!=i['v11']],flush=True)

if __name__=='__main__':main()
