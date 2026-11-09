from pathlib import Path
import json,hashlib,zipfile,sys,time,importlib.util,copy
import cv2,numpy as np
R=Path(__file__).resolve().parents[1];A=R/'output/blue-perimeter-trial-20261105/vision_app/code';O=R/'output/blue-cache-candidate-20261105';O.mkdir(exist_ok=True)
s=(A/'project_detector.py').read_text(encoding='utf-8');s=s.replace('import argparse','import copy\nimport argparse',1)
old='''        contours,_=sparse_external_contours(geometry)
        for contour in contours:'''
new='''        contour_key=('fallback_contours',id(geometry))
        cached=pale_cache.get(contour_key) if pale_cache is not None else None
        if cached is None:
            contours,_=sparse_external_contours(geometry)
            if pale_cache is not None:pale_cache[contour_key]=contours
        else:contours=cached
        for contour in contours:'''
assert old in s;s=s.replace(old,new,1)
old='''            pad=4
            local=np.zeros((h+2*pad,w+2*pad),np.uint8)
            local[pad:pad+h,pad:pad+w]=geometry[y:y+h,x:x+w]
            local=cv2.morphologyEx(local,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
            shapes,_=cv2.findContours(local,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            refined=max(shapes,key=cv2.contourArea)+np.array([[[x-pad,y-pad]]],dtype=np.int32)'''
new='''            refine_key=('fallback_refined',id(geometry),x,y,w,h)
            refined=pale_cache.get(refine_key) if pale_cache is not None else None
            if refined is None:
                pad=4
                local=np.zeros((h+2*pad,w+2*pad),np.uint8)
                local[pad:pad+h,pad:pad+w]=geometry[y:y+h,x:x+w]
                local=cv2.morphologyEx(local,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
                shapes,_=cv2.findContours(local,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                refined=max(shapes,key=cv2.contourArea)+np.array([[[x-pad,y-pad]]],dtype=np.int32)
                if pale_cache is not None:pale_cache[refine_key]=refined'''
assert old in s;s=s.replace(old,new,1)
old='''                face=pale_face_candidate(frame,masks,refined,value,face_settings)
                if face is None:face=current_pale_circle(frame,masks,refined,hsv,value,cache=pale_cache)'''
new='''                face_key=('fallback_face',refined.tobytes())
                if pale_cache is not None and face_key in pale_cache:
                    face=copy.deepcopy(pale_cache[face_key])
                else:
                    face=pale_face_candidate(frame,masks,refined,value,face_settings)
                    if face is None:face=current_pale_circle(frame,masks,refined,hsv,value,cache=pale_cache)
                    if pale_cache is not None:pale_cache[face_key]=copy.deepcopy(face)'''
assert old in s;s=s.replace(old,new,1)
(O/'project_detector.py').write_text(s,encoding='utf-8');sys.path.insert(0,str(A));cv2.setNumThreads(2)
def load(name,p):
 sp=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);return m
old=load('perimeter_before_cache',A/'project_detector.py');new=load('perimeter_after_cache',O/'project_detector.py')
cases=[];seen=set()
for folder in ['2026-10-26','2026-10-27','2026-11-05']:
 for p in (R/'evidence/p2-camera'/folder).rglob('*raw.png'):
  data=p.read_bytes();h=hashlib.sha256(data).hexdigest()
  if h in seen:continue
  seen.add(h);cases.append((str(p.relative_to(R)),data,h))
z=zipfile.ZipFile(R/'evidence/p2-camera/2026-11-05/blue-perimeter-board/20261105-144531-087584.zip')
for n in z.namelist():
 if n.endswith('-raw.png'):
  data=z.read(n);h=hashlib.sha256(data).hexdigest()
  if h not in seen:seen.add(h);cases.append(('board-zip/'+n,data,h))
cfg=json.loads(z.read('material_color_thresholds.json'));results=[]
for i,(label,data,h) in enumerate(cases):
 im=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
 t=time.perf_counter();a,_=old.analyze(im,cfg,draw_overlay=False);ta=time.perf_counter()-t
 t=time.perf_counter();b,_=new.analyze(im,cfg,draw_overlay=False);tb=time.perf_counter()-t
 assert a==b,label
 results.append(dict(image=label,sha256=h,before_ms=ta*1000,after_ms=tb*1000))
 if (i+1)%40==0:print('CHECKED',i+1,flush=True)
E=R/'evidence/p2-camera/2026-11-05/blue-cache-candidate';E.mkdir(exist_ok=True)
report=dict(passed=True,images=len(results),exact_result_equality=True,hardware_accessed=False,PC_timing_not_board_validation=True,cache_scope='local dict discarded at each analyze() return',median_before_ms=float(np.median([r['before_ms'] for r in results])),median_after_ms=float(np.median([r['after_ms'] for r in results])),results=results)
(E/'replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print('SUMMARY',len(results),report['median_before_ms'],report['median_after_ms'],flush=True)
