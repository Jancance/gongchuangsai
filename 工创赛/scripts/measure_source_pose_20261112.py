"""Extract current contact-plane evidence for one MATLAB pose computation.
No detector/config/actuator changes. Empty drawn circles are search priors.
"""
from pathlib import Path
import json,hashlib
import cv2,numpy as np
from measure_source_paper50_20261111 import read
R=Path(__file__).resolve().parents[1];B=R/'evidence/integration/2026-11-11/source-view-entry'
S=B/'source-observe-20261112-023027';O=B/'pose-current-20261112'

def main():
    O.mkdir(exist_ok=True)
    reference=read(B/'source-observe-20261112-010846/last-raw.png');im=read(S/'last-raw.png')
    a=cv2.cvtColor(reference,cv2.COLOR_BGR2GRAY);g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    patches=[]
    for x,y in [(1005,330),(945,374),(1014,403),(953,449),(1021,480),(1074,359),(996,239)]:
        response=cv2.matchTemplate(g[y-62:y+63,x-62:x+63],a[y-12:y+13,x-12:x+13],cv2.TM_CCOEFF_NORMED)
        _,score,_,p=cv2.minMaxLoc(response)
        patches.append(dict(reference_px=[x,y],current_px=[x+p[0]-50,y+p[1]-50],correlation=score))
    shift=np.median([np.array(x['current_px'])-x['reference_px'] for x in patches],axis=0)
    circles=[];arcs=[];overlay=im.copy()
    for name,cx,cy,r in [('far_left',680,173,101),('far_right',1378,230,101),('near',981,801,101)]:
        pts=[]
        for ang in np.deg2rad(np.arange(360)):
            radii=np.arange(r-8,r+8.01,.25);x=cx+radii*np.cos(ang);y=cy+radii*np.sin(ang)
            v=cv2.remap(a,x.astype('float32')[None],y.astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
            i=int(np.argmin(v))
            if 2<i<len(radii)-3 and np.percentile(v,80)-v[i]>6:pts.append([float(x[i]),float(y[i])])
        circles.append(dict(id=name,boundary_px=pts))
        center=np.array([cx,cy])+shift;direction=np.arctan2(540-center[1],960-center[0]);pts=[]
        for ang in direction+np.deg2rad(np.linspace(-60,60,241)):
            radii=np.arange(r-18,r+18.01,.25);x=center[0]+radii*np.cos(ang);y=center[1]+radii*np.sin(ang)
            v=cv2.remap(g,x.astype('float32')[None],y.astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
            grad=(v[8:]-v[:-8])/2;i=int(np.argmax(grad))+4
            if 6<i<len(radii)-7 and grad[i-4]>10:pts.append([float(x[i]),float(y[i])])
        arcs.append(dict(id=name,boundary_px=pts,prior_current_px=center.tolist(),prior_used_for_search=True))
        for p in pts[::3]:cv2.circle(overlay,tuple(np.rint(p).astype(int)),2,(0,255,255),1)
        cv2.putText(overlay,name,tuple(np.rint(center).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
    d=dict(source_session=S.name,patches=patches,empty_circles=circles,current_arcs=arcs,
        task_code='215+213+512+321',task_source='user supplied',current_colors_by_region=[2,1,5],
        empty_prior_is_not_material_presence=True,motion_allowed=False,
        image_sha256=hashlib.sha256((S/'last-raw.png').read_bytes()).hexdigest())
    (O/'measurement-input.json').write_text(json.dumps(d,indent=2),encoding='utf-8')
    (O/'contact-arcs.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    print(json.dumps(dict(patches=patches,empty_counts=[len(x['boundary_px']) for x in circles],arc_counts=[len(x['boundary_px']) for x in arcs]),indent=2))
if __name__=='__main__':main()
