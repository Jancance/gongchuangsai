from pathlib import Path
import sys,json,importlib.util,time,zipfile,hashlib
import cv2,numpy as np
R=Path(__file__).resolve().parents[1];A=R/'output/blue-perimeter-trial-20261105/vision_app/code';O=R/'output/hsv-mask-candidate-20261105';sys.path[:0]=[str(O),str(A)]
from project_hsv_masks import masks_from_split
import importlib
oldmask=importlib.import_module('颜色标定')._mask_from_hsv
cfg=json.loads((A/'config/material_color_thresholds.json').read_text(encoding='utf-8'));cv2.setNumThreads(2)
checks=0
for start in range(0,256,8):
 h=np.repeat(np.arange(start,start+8,dtype=np.uint8),256*256).reshape(-1,256)
 s=np.tile(np.repeat(np.arange(256,dtype=np.uint8),256),8).reshape(-1,256)
 v=np.tile(np.arange(256,dtype=np.uint8),8*256).reshape(-1,256)
 hsv=cv2.merge([h,s,v]);m,union=masks_from_split(h,s,v,cfg);expected=np.zeros(h.shape,np.uint8)
 for c in cfg['colors']:
  mask=oldmask(hsv,c['ranges']);assert np.array_equal(mask,m[c['color_id']]);expected|=mask;checks+=1
 assert np.array_equal(union,expected);checks+=1
s=(A/'project_detector.py').read_text(encoding='utf-8')
old="""        calibrated_masks={int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in color_config['colors']}
        mask=next(iter(calibrated_masks.values())).copy()
        for m in list(calibrated_masks.values())[1:]:cv2.bitwise_or(mask,m,dst=mask)"""
new="""        from project_hsv_masks import masks_from_split
        prepared=masks_from_split(h,s,v,color_config)
        if prepared is None:
            calibrated_masks={int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in color_config['colors']}
            mask=next(iter(calibrated_masks.values())).copy()
            for m in list(calibrated_masks.values())[1:]:cv2.bitwise_or(mask,m,dst=mask)
        else:calibrated_masks,mask=prepared"""
assert old in s;s=s.replace(old,new,1);(O/'project_detector.py').write_text(s,encoding='utf-8')
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
old=load('before_hsv_bits',A/'project_detector.py');new=load('after_hsv_bits',O/'project_detector.py')
rows=[];seen=set();cases=[]
for folder in ['2026-10-26','2026-10-27','2026-11-05']:
 for p in (R/'evidence/p2-camera'/folder).rglob('*raw.png'):
  data=p.read_bytes();hh=hashlib.sha256(data).hexdigest()
  if hh not in seen:seen.add(hh);cases.append((str(p.relative_to(R)),data))
z=zipfile.ZipFile(R/'evidence/p2-camera/2026-11-05/blue-perimeter-board/20261105-144531-087584.zip')
for n in z.namelist():
 if n.endswith('-raw.png'):
  data=z.read(n);hh=hashlib.sha256(data).hexdigest()
  if hh not in seen:seen.add(hh);cases.append(('board-zip/'+n,data))
for i,(label,data) in enumerate(cases):
 im=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
 t=time.perf_counter();a,_=old.analyze(im,cfg,draw_overlay=False);ta=time.perf_counter()-t
 t=time.perf_counter();b,_=new.analyze(im,cfg,draw_overlay=False);tb=time.perf_counter()-t
 assert a==b,label;checks+=1;rows.append(dict(image=label,before_ms=ta*1000,after_ms=tb*1000))
 if (i+1)%40==0:print('CHECKED',i+1,flush=True)
E=R/'evidence/p2-camera/2026-11-05/hsv-mask-candidate';E.mkdir(exist_ok=True)
report=dict(passed=True,HSV_values_exhaustively_checked=256**3,mask_checks=224,images=len(rows),total_checks=checks,PC_timing_only=True,median_before_ms=float(np.median([r['before_ms'] for r in rows])),median_after_ms=float(np.median([r['after_ms'] for r in rows])),results=rows)
(E/'replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print('SUMMARY',len(rows),report['median_before_ms'],report['median_after_ms'],flush=True)
