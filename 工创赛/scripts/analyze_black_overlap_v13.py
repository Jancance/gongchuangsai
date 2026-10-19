"""Verify black/colour overlap removal against immutable V12 evidence."""
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
    cfg['black_edge_recovery_candidate']['disjoint_colour_evidence']=True
    cfg['black_edge_recovery_candidate']['version']='v13-black-colour-overlap-candidate'
    return cfg

def main():
    out=ROOT/'output/black-overlap-v13-analysis-20261007';out.mkdir(parents=True,exist_ok=True)
    with ZipFile(ROOT/'output/black_edge_patch_v12_2026-10-07.zip') as package:
        old=types.ModuleType('v12');exec(package.read('vision_app/code/project_detector.py'),old.__dict__)
    with ZipFile(ROOT/'evidence/20261007-012619-825807.zip') as z:
        cfg=json.loads(z.read('material_color_thresholds.json'))
    datasets=[]
    for suffix in ('083236-468596','091047-121270','233915-492000','000604-409723',
                   '002007-035470','003307-733760','004517-208447','012619-825807'):
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
                frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                before,_=old.analyze(frame,cfg,hints,draw_overlay=False)
                after,drawing=analyze(frame,candidate(cfg),hints)
                known=lambda r:sorted(a['label'] for a in r['regions'] if a['label']!='UNKNOWN')
                images.append({'file':name,'v12':known(before),'v13':known(after),'regions':after['regions']})
                if suffix=='012619-825807' and name=='first-black-loss-raw.png':
                    cv2.imencode('.png',drawing)[1].tofile(out/'first-touch-v13-overlay.png')
            datasets.append({'source':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'images':images})
    report={'version':VERSION,'settings':candidate(cfg)['black_edge_recovery_candidate'],'datasets':datasets,
            'scope':'Saved image comparison with same original past hints. No full temporal replay, board timing or physical acceptance.'}
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    for d in datasets:print(d['source'],len(d['images']),[(r['file'],r['v12'],r['v13']) for r in d['images'] if r['v12']!=r['v13']],flush=True)

if __name__=='__main__':main()
