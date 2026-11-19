"""Read saved empty-disk15frames and extract the three drawn circle boundaries.
Diagnostic regions are selected from this image; no runtime detector changes.
"""
from pathlib import Path
from collections import Counter
import json,hashlib
import cv2,numpy as np
from measure_source_paper50_20261111 import read
R=Path(__file__).resolve().parents[1];B=R/'evidence/integration/2026-11-11/source-view-entry'
S=B/'source-observe-20261111-041409';O=B/'clean-disk-review'
def main():
    O.mkdir(exist_ok=True)
    session=json.loads((S/'session.json').read_text(encoding='utf-8'))
    rows=[json.loads(x) for x in (S/'observations.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    ds=[r['detection'] for r in rows];assert len(rows)==session['frames']==15 and session['error'] is None
    assert session['serial']=='DISABLED' and all(not r['motion_allowed'] and not r['replay'] for r in rows)
    im=read(S/'last-raw.png');g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);overlay=im.copy();circles=[]
    for name,cx,cy,radius in [('far_left',680,214,102),('far_right',1385,269,102),('near',984,849,102)]:
        points=[]
        for angle in np.deg2rad(np.arange(360)):
            radii=np.arange(radius-8,radius+8.01,.25)
            x=cx+radii*np.cos(angle);y=cy+radii*np.sin(angle)
            val=cv2.remap(g,x.astype('float32')[None],y.astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
            idx=int(np.argmin(val));contrast=np.percentile(val,80)-val[idx]
            if 2<idx<len(radii)-3 and contrast>6:
                points.append([float(x[idx]),float(y[idx])])
        assert len(points)>240,(name,len(points))
        circles.append(dict(id=name,boundary_image_px=points))
        for p in points[::3]:cv2.circle(overlay,tuple(np.rint(p).astype(int)),1,(0,0,255),1)
        cv2.putText(overlay,name,(cx-65,cy),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),1)
    before=cv2.cvtColor(read(B/'source-observe-20261111-033718/last-raw.png'),cv2.COLOR_BGR2GRAY)
    patches=[]
    for x,y in [(1003,366),(942,412),(1012,441),(952,486),(1021,517),(1073,395),(995,279)]:
        a=cv2.matchTemplate(g[y-21:y+22,x-21:x+22],before[y-15:y+16,x-15:x+16],cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(a);patches.append(dict(before_px=[x,y],correlation=score,integer_shift_px=[where[0]-6,where[1]-6]))
    summary=dict(frames=len(rows),detected_frames=sum(d['detected'] for d in ds),stable_frames=sum(d['stable'] for d in ds),
        tracking_counts=dict(Counter(d['tracking_mode'] for d in ds)),
        unstable_sequences=[r['frame_sequence'] for r in rows if not r['detection']['stable']],
        configs=session['configs'],serial=session['serial'],sensor_exposure_age_verified=False)
    for key in ['center_x_px','center_y_px','radius_px','fit_rms_px','visible_arc_deg']:
        a=np.array([d[key] for d in ds if d['detected']]);summary[key]=dict(median=float(np.median(a)),peak_to_peak=float(np.ptp(a)))
    data=dict(image_sha256=hashlib.sha256((S/'last-raw.png').read_bytes()).hexdigest(),image_size=[1920,1080],
        summary=summary,circles=circles,frame_boundary_points_px=[r['boundary_points_px'] for r in rows],
        fixed_surface_patches=patches,
        plane_model='paper50-geometry/model-comparison.json:similarity',
        ruler_verification='ruler-review/matlab-result.json',
        circles_are_drawn_static_markers_not_material_detections=True,motion_allowed=False)
    (O/'measurement-input.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    (O/'boundary-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    print(json.dumps(dict(summary=summary,circle_boundary_counts=[len(c['boundary_image_px']) for c in circles],fixed_surface_patches=patches),indent=2))
if __name__=='__main__':main()
