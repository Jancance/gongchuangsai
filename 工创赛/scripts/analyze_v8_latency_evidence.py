"""Separate result expiration from visible-region rejection; benchmark PC only."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import types
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze,VERSION

def main():
    source=ROOT/'evidence/20261006-195642-102168.zip'
    out=ROOT/'output/v8-latency-v9-analysis-20261006';out.mkdir(parents=True,exist_ok=True)
    with ZipFile(ROOT/'output/paired_face_patch_v8_2026-10-06.zip') as package:
        old=types.ModuleType('v8');exec(package.read('vision_app/code/project_detector.py'),old.__dict__)
    with ZipFile(source) as z:
        assert z.testzip() is None
        rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        cfg=json.loads(z.read('material_color_thresholds.json'))
        stale=[r for r in rows if r['state']=='STALE_RESULT']
        stale_two_colors=sum({'blue','lightblue'} <= {a['label'] for a in r['detector_regions']} for r in stale)
        comparisons=[]
        for name in z.namelist():
            if not name.endswith('-raw.png'):continue
            meta=name.replace('-before-raw.png','.json').replace('-raw.png','.json')
            packet=json.loads(z.read(meta));current=packet.get('previous' if '-before-' in name else 'current')
            hints=[]
            if current and current['frame_index']>1:
                previous=rows[current['frame_index']-2]
                hints=[t for t in previous.get('internal_tracks_not_actionable',[]) if 0<=current['timestamp']-t['read_time']<=.5]
            frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
            expected,_=old.analyze(frame,cfg,hints)
            actual,_=analyze(frame,cfg,hints,draw_overlay=False)
            expected.pop('version');actual.pop('version');assert expected==actual,name
            comparisons.append({'file':name,'exact_regions_and_diagnostics_match':True})
        frame=cv2.imdecode(np.frombuffer(z.read('first-lightblue-loss-raw.png'),np.uint8),1)
        old.analyze(frame,cfg);analyze(frame,cfg,draw_overlay=False)
        timings={'v8':[],'v9':[]}
        for i in range(30):
            calls=[('v8',lambda:old.analyze(frame,cfg)),('v9',lambda:analyze(frame,cfg,draw_overlay=False))]
            if i%2:calls.reverse()
            for name,call in calls:
                start=time.perf_counter();call();timings[name].append((time.perf_counter()-start)*1000)
        report={'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                'module_hashes':json.loads(z.read('module_source_hashes.json')),
                'candidate':VERSION,'run':json.loads(z.read('run.json')),
                'stale_results_with_both_blue_and_lightblue_already_detected':stale_two_colors,
                'age_ms_p50_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[50,95,100]).tolist(),
                'detector_ms_p50_p95_max':np.percentile([r['detector_call_ms'] for r in rows],[50,95,100]).tolist(),
                'saved_image_equivalence':comparisons,
                'pc_benchmark_ms_p50_p95':{k:np.percentile(v,[50,95]).tolist() for k,v in timings.items()},
                'scope':'Saved image equivalence and PC timing only. Board timing and occlusion remain unverified.'}
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Stale already detected both:',stale_two_colors,'/',len(stale))
    print('PC timing:',report['pc_benchmark_ms_p50_p95'])
    print(out/'report.json')

if __name__=='__main__':main()
