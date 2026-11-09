"""Read-only offline audit; no camera or motor access."""
from pathlib import Path
import zipfile,json,collections,hashlib
import cv2,numpy as np
R=Path(__file__).resolve().parents[1];E=R/'evidence/p2-camera/2026-11-05/yellow-audit'
z=zipfile.ZipFile(E/'20261105-140230-265860.zip');assert z.testzip() is None
rows=[json.loads(x) for x in z.read('observations.jsonl').decode().splitlines()]
states=collections.Counter(x['state'] for x in rows)
colors=collections.Counter(t['color'] for x in rows for t in x.get('targets',[]))
reasons=collections.Counter(t['reason'] for x in rows for t in x.get('detector_regions',[]) if t.get('bbox_px',[0])[0]>1300)
measure=[]
for n in ['sample-01','sample-02','sample-03','sample-04']:
 im=cv2.imdecode(np.frombuffer(z.read(n+'-raw.png'),np.uint8),cv2.IMREAD_COLOR)
 row=json.loads(z.read(n+'.json'));row=row.get('current',row)
 candidates=[q for q in row.get('detector_regions',[]) if q.get('bbox_px',[0])[0]>1300 and q.get('reason')=='INSUFFICIENT_NONCLIPPED_PIXELS']
 for q in candidates:
  x,y,w,h=q['bbox_px'];roi=im[y:y+h,x:x+w];hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
  measure.append(dict(sample=n,bbox=q['bbox_px'],recorded_usable_fraction=q.get('usable_fraction'),bbox_HSV_median=np.median(hsv.reshape(-1,3),axis=0).tolist(),bbox_v_ge250_fraction=float((hsv[:,:,2]>=250).mean())))
app=zipfile.ZipFile(R/'output/p2_vision_generic135_lightblue1_20261027.zip');manifest=json.loads(z.read('module_source_hashes.json'));verified={}
for n,h in manifest.items():
 names=[x for x in app.namelist() if x.endswith('/'+n)]
 if names:
  verified[n]=hashlib.sha256(app.read(names[0])).hexdigest()==h
assert all(verified.values()),verified
report=dict(frames=len(rows),states=dict(states),accepted_colors=dict(colors),right_side_candidate_reasons=dict(reasons),samples=measure,packaged_source_matches_recorded_modules=verified,exposure_actual_per_frame_not_recorded=True,serial_disabled=True,motion_allowed=False,remote_configuration_changed=False)
(E/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False,indent=2))
