"""Archive blue motion feedback and measured glare failures without acceptance inflation."""
from collections import Counter
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
source=ROOT/'evidence/20261007-000604-409723.zip'
out=ROOT/'output/v11-blue-moving-review-20261007';out.mkdir(parents=True,exist_ok=True)
with ZipFile(source) as z,ZipFile(ROOT/'output/independent_blue_patch_v11_2026-10-06.zip') as package:
    assert z.testzip() is None
    rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
    hashes=json.loads(z.read('module_source_hashes.json'))
    manifest=json.loads(package.read('PATCH_MANIFEST.json'))
    matches={name:hashlib.sha256(z.read('material_color_thresholds.json')).hexdigest()==entry['after_sha256']
             if name.endswith('.json') else hashes[Path(name).name]==entry['after_sha256']
             for name,entry in manifest['files'].items()}
    first=next(i for i,r in enumerate(rows) if any(t['color']=='blue' for t in r['targets']))
    gaps=[];start=None;last=None;changes=[]
    for i,r in enumerate(rows):
        targets=[t for t in r['targets'] if t['color']=='blue']
        if not targets and start is None:start=i
        if targets:
            if start is not None:
                gaps.append({'first_missing_frame':start+1,'return_frame':i+1,
                             'duration_s':r['timestamp']-rows[start]['timestamp'],
                             'before_first_acquisition':start<first})
                start=None
            if last and last['id']!=targets[0]['id']:
                changes.append({'frame':i+1,'previous_id':last['id'],'current_id':targets[0]['id'],
                                'observed_center_displacement_px':float(np.linalg.norm(np.array(last['center_px'])-targets[0]['center_px']))})
            last=targets[0]
    manual=[]
    for filename,x,y,radius in [('first-frame-raw.png',643,545,80),('first-blue-loss-raw.png',638,596,80)]:
        image=cv2.imdecode(np.frombuffer(z.read(filename),np.uint8),1)
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);yy,xx=np.ogrid[:image.shape[0],:image.shape[1]]
        pixels=hsv[(xx-x)**2+(yy-y)**2<radius**2]
        manual.append({'image':filename,'manually_selected_disk_px':[x,y,radius],
                       'clipped_fraction_V_ge_250':float((pixels[:,2]>=250).mean()),
                       'scope':'Manual development ROI, not a full-object or scene quality score.'})
    report={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'matches_v11':matches,'run':json.loads(z.read('run.json')),
            'observed_labels':dict(Counter(t['color'] for r in rows for t in r['targets'])),
            'first_blue_frame':first+1,'initial_no_blue_duration_s':rows[first]['timestamp']-rows[0]['timestamp'],
            'blue_missing_episodes':gaps,'blue_id_changes':changes,
            'result_host_age_ms_p95_max':np.percentile([r['result_host_age_ms'] for r in rows],[95,100]).tolist(),
            'manual_glare_samples':manual,
            'scope':'Desk motion development evidence. Ordinary-motion feedback retained; strong glare, identity and grasp not accepted.'}
    for name in ('material_color_thresholds.json','camera_processing.json','project_profile.json','module_source_hashes.json'):
        (out/name).write_bytes(z.read(name))
(out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(matches.values()),matches
print('V11 matched; first blue frame:',first+1,'post-acquisition losses:',sum(not g['before_first_acquisition'] for g in gaps))
print(out/'report.json')
