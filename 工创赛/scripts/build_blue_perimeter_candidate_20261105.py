from pathlib import Path
import json,copy,sys,importlib.util,hashlib,zipfile,time
import numpy as np,cv2
R=Path(__file__).resolve().parents[1];A=R/'output/p2_vision_generic135_lightblue1_20261027/vision_app/code'
E=R/'evidence/p2-camera/2026-11-05/three-color-audit';O=R/'output/blue-perimeter-candidate-20261105';O.mkdir(exist_ok=True)
sys.path.insert(0,str(A));cv2.setNumThreads(2)
def load(name,p):
 sp=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);return m
s=(A/'project_detector.py').read_text(encoding='utf-8')
needle='            area=cv2.contourArea(refined)'
assert s.count(needle)==1
s=s.replace(needle,'''            if blue_settings and blue_settings.get('_perimeter_pass',False):
                refined=cv2.approxPolyDP(refined,1.0,True)
'''+needle)
needle="            if projected_blue and (scores['blue']<.90 or scores['blue']-scores[ranking[1]]<.65):"
assert needle in s
s=s.replace(needle,"            if (projected_blue or (blue_settings and blue_settings.get('_perimeter_pass',False))) and (scores['blue']<.90 or scores['blue']-scores[ranking[1]]<.65):")
needle='        # 只有原有全部颜色/圆弧分支仍无浅蓝时，尝试本帧投影分量。'
assert needle in s
s=s.replace(needle,'''        if blue_settings and blue_settings.get('perimeter_denoise_candidate',False) and not any(r['label']=='blue' for r in records):
            for row in chromatic_fallback(frame,calibrated_masks,rejected,accepted,
                    calibrated_masks,color_config.get('round_face_color_candidate'),hsv=hsv,blue_settings=dict(blue_settings,_perimeter_pass=True),pale_cache=pale_cache):
                row['proposal_source']='perimeter_denoised_blue_candidate'
                records.append(row)
                if draw_overlay:
                    x,y,w,ht=row['bbox_px'];cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                    cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
'''+needle)
(O/'project_detector.py').write_text(s,encoding='utf-8');old=load('blue_old',A/'project_detector.py');new=load('blue_new',O/'project_detector.py')
z=zipfile.ZipFile(E/'20261105-142718-167305.zip');cfg=json.loads(z.read('material_color_thresholds.json'));on=copy.deepcopy(cfg);on['blue_projected_silhouette_candidate']['perimeter_denoise_candidate']=True
checks=0;results=[];seen=set();files=[]
for p in (E/'raw').glob('*raw.png'):files.append(p)
for folder in ['2026-10-26','2026-10-27','2026-11-05']:
 for p in (R/'evidence/p2-camera'/folder).rglob('*raw.png'):
  if 'three-color-audit' not in str(p):files.append(p)
for p in files:
 data=p.read_bytes();h=hashlib.sha256(data).hexdigest()
 if h in seen:continue
 seen.add(h);im=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
 a,_=old.analyze(im,cfg,draw_overlay=False);off,_=new.analyze(im,cfg,draw_overlay=False);b,_=new.analyze(im,on,draw_overlay=False)
 checks+=1;assert a==off,p
 nonblue=lambda r:[x for x in r['regions'] if x['label']!='blue']
 checks+=1;assert nonblue(a)==nonblue(b),p
 ab=[x for x in a['regions'] if x['label']=='blue'];bb=[x for x in b['regions'] if x['label']=='blue']
 if ab:checks+=1;assert ab==bb,p
 added=len(bb)>len(ab)
 results.append(dict(file=str(p.relative_to(R)),sha256=h,blue_before=len(ab),blue_after=len(bb),added=added,boxes=[r['bbox_px'] for r in bb]))
 print(len(results),p.name,'added' if added else '',flush=True) if added else None
report=dict(passed=True,images=len(results),checks=checks,added_images=[r for r in results if r['added']],results=results,source_installed=False,hardware_accessed=False,configuration_flag='blue_projected_silhouette_candidate.perimeter_denoise_candidate',limitations='saved development images only; no board speed or live acceptance')
(E/'blue-perimeter-replay.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print('SUMMARY',len(results),checks,len(report['added_images']),flush=True)
