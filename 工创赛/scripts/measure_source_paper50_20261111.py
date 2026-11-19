"""Extract the user's current50mm paper and static A evidence for MATLAB.
No camera, serial, calibration activation or control writes.
"""
from pathlib import Path
from collections import Counter
import hashlib,json
import cv2
import numpy as np
R=Path(__file__).resolve().parents[1]
S=R/'evidence/integration/2026-11-11/source-view-entry/source-observe-20261111-033718'
O=S.parent/'paper50-geometry'

def read(p):
    return cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv2.IMREAD_COLOR)

def line(gray,a,b):
    a=np.array(a,float);b=np.array(b,float);v=b-a
    normal=np.array([-v[1],v[0]])/np.linalg.norm(v)
    points=[]
    for t in np.arange(.15,.85,1/np.linalg.norm(v)):
        q=a+t*v+np.arange(-6,6.01,.25)[:,None]*normal
        values=cv2.remap(gray,q[:,0].astype('float32')[None],q[:,1].astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
        contrast=np.maximum(0,np.percentile(values,80)-values-8)
        if contrast.sum()>30:points.append(np.average(q,weights=contrast,axis=0))
    p=np.array(points);assert len(p)>80
    keep=np.ones(len(p),bool)
    for _ in range(5):
        center=p[keep].mean(0);_,_,v=np.linalg.svd(p[keep]-center)
        normal=v[-1];res=(p-center)@normal
        keep=np.abs(res)<max(.8,3*1.4826*np.median(np.abs(res[keep])))
    return np.r_[normal,-center@normal],p[keep],float(np.sqrt(np.mean(res[keep]**2)))

def paper(image):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    initial=[[1120,466],[1319,495],[1288,696],[1087,667]]
    lines=[line(gray,initial[i],initial[(i+1)%4]) for i in range(4)]
    corners=[]
    for i in range(4):
        p=np.cross(lines[(i-1)%4][0],lines[i][0]);corners.append((p[:2]/p[2]).tolist())
    return corners,lines

def main():
    O.mkdir(exist_ok=True)
    session=json.loads((S/'session.json').read_text(encoding='utf-8'))
    rows=[json.loads(x) for x in (S/'observations.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    detections=[r['detection'] for r in rows];valid=[d for d in detections if d['detected']]
    assert len(rows)==session['frames'] and session['error'] is None
    assert session['serial']=='DISABLED' and all(not r['motion_allowed'] and not r['replay'] for r in rows)
    summary=dict(frames=len(rows),detected_frames=len(valid),stable_frames=sum(d['stable'] for d in detections),
        tracking_counts=dict(Counter(d['tracking_mode'] for d in detections)),
        unstable_sequences=[r['frame_sequence'] for r in rows if not r['detection']['stable']],
        metric_status_counts=dict(Counter(r['metric_preview']['status'] for r in rows)),
        sequence_contiguous=[r['frame_sequence'] for r in rows]==list(range(1,len(rows)+1)),
        serial=session['serial'],sensor_exposure_age_verified=False)
    for key in ['center_x_px','center_y_px','radius_px','fit_rms_px','visible_arc_deg','processing_ms']:
        a=np.array([d[key] for d in valid]);summary[key]=dict(minimum=float(a.min()),median=float(np.median(a)),maximum=float(a.max()),peak_to_peak=float(np.ptp(a)))
    first=read(S/'first-raw.png');last=read(S/'last-raw.png')
    corners,lines=paper(last);first_corners,_=paper(first)
    before=read(S.parent/'source-observe-20261111-032846/last-raw.png')
    gray0=cv2.cvtColor(before,cv2.COLOR_BGR2GRAY);gray1=cv2.cvtColor(last,cv2.COLOR_BGR2GRAY)
    patches=[]
    for x,y in [(1003,366),(942,412),(1012,441),(952,486),(1021,517),(1073,395),(995,279),(882,540)]:
        response=cv2.matchTemplate(gray1[y-21:y+22,x-21:x+22],gray0[y-15:y+16,x-15:x+16],cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(response)
        patches.append(dict(before_px=[x,y],correlation=score,integer_shift_px=[where[0]-6,where[1]-6]))
    result=dict(schema_version=1,paper_width_mm=50,paper_height_mm=50,
        size_basis='User explicitly made and placed50x50mm square; not historic100mm paper.',
        image_size=[last.shape[1],last.shape[0]],
        image_sha256=hashlib.sha256((S/'last-raw.png').read_bytes()).hexdigest(),
        corners_ABCD_px=corners,first_corners_ABCD_px=first_corners,
        corner_method='robust pen-line centerline intersections excluding labels/endpoints; image-selected initial bounds',
        border_line_support_counts=[len(l[1]) for l in lines],border_line_rms_px=[l[2] for l in lines],
        independent_known_metric_check_points=[],independent_field_accuracy_verified=False,
        paper_nominal_size_uncertainty_measured=False,
        frame_boundary_points_px=[r['boundary_points_px'] for r in rows],
        detection_centers_px=[[d['center_x_px'],d['center_y_px']] for d in valid],
        fixed_surface_patches=patches,summary=summary,configs=session['configs'],
        yaw_feedback_deg=180.00811767578125,extension_command_mm=65,
        user_approximate_outer_gap_mm=75,motion_allowed=False)
    (O/'measurement-input.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    (O/'pixel-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    overlay=last.copy()
    for name,p in zip('ABCD',corners):
        xy=tuple(np.rint(p).astype(int));cv2.circle(overlay,xy,5,(0,0,255),2)
        cv2.putText(overlay,name,(xy[0]+8,xy[1]-8),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
    (O/'paper-corners-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    (O/'paper-corners-detail.png').write_bytes(cv2.imencode('.png',overlay[440:725,1050:1350])[1].tobytes())
    print(json.dumps(dict(summary=summary,corners_ABCD_px=corners,border_line_rms_px=result['border_line_rms_px'],
        first_last_corner_shift_px=np.linalg.norm(np.array(corners)-first_corners,axis=1).tolist(),fixed_surface_patches=patches),indent=2))
if __name__=='__main__':main()
