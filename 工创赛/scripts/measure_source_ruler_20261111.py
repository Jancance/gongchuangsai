"""Read current static ruler tick coordinates for a MATLAB length check."""
from pathlib import Path
import json,hashlib
import cv2,numpy as np
from measure_source_paper50_20261111 import read,line
R=Path(__file__).resolve().parents[1]
B=R/'evidence/integration/2026-11-11/source-view-entry'
S=B/'source-observe-20261111-035604';O=B/'ruler-review'

def main():
    O.mkdir(exist_ok=True);im=read(S/'last-raw.png');gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    # Printed centimetre labels establish identity; fit the engraved long tick,
    # never use the text centre as a tick. 14..16cm excluded for glare.
    initials={2:42,3:83,4:124,5:165,6:206,7:248,8:289,9:331,10:372,11:414,
              12:455,13:497,17:662,18:703,19:743,20:785,21:826,22:867,23:909,24:951,25:992}
    ticks=[];overlay=im.copy()
    for cm,guess in initials.items():
        roi=gray[guess-3:guess+4,817:828].astype(float)
        prof=roi.mean(axis=1);idx=int(np.argmin(prof))
        # One-dimensional quadratic refinement of the engraved-line minimum.
        offset=0.0
        if 0<idx<len(prof)-1:
            denom=prof[idx-1]-2*prof[idx]+prof[idx+1]
            if denom>1e-6:offset=float(np.clip(.5*(prof[idx-1]-prof[idx+1])/denom,-.5,.5))
        y=float(guess-3+idx+offset)
        ticks.append(dict(label_cm=cm,position_mm=cm*10,point_px=[822.0,y],initial_y_px=guess,
                          search_boundary_hit=idx in (0,len(prof)-1)))
        cv2.line(overlay,(795,round(y)),(828,round(y)),(0,0,255),1)
        cv2.putText(overlay,str(cm)+'cm',(906,round(y)+5),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,255),1)
    # Current paper moved independently. Confirm the camera against FIXED screws.
    previous=read(B/'source-observe-20261111-033718/last-raw.png')
    g0=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY)
    patches=[]
    for x,y in [(1003,366),(942,412),(1012,441),(952,486),(1021,517),(1073,395),(995,279)]:
        a=cv2.matchTemplate(gray[y-21:y+22,x-21:x+22],g0[y-15:y+16,x-15:x+16],cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(a)
        patches.append(dict(before_px=[x,y],correlation=score,integer_shift_px=[where[0]-6,where[1]-6]))
    initial=[[1124,451],[1332,464],[1318,670],[1113,657]]
    borders=[line(gray,initial[i],initial[(i+1)%4]) for i in range(4)]
    corners=[]
    for i in range(4):
        p=np.cross(borders[(i-1)%4][0],borders[i][0]);corners.append((p[:2]/p[2]).tolist())
    session=json.loads((S/'session.json').read_text(encoding='utf-8'))
    result=dict(schema_version=1,image_sha256=hashlib.sha256((S/'last-raw.png').read_bytes()).hexdigest(),
        image_size=[im.shape[1],im.shape[0]],ticks=ticks,excluded_cm=[14,15,16],
        exclusion_basis='specular glare across ruler',tick_selection_uncertainty_px_declared=2,
        ruler_reference_x_px=822,method='visually identified centimetre long ticks; local dark-line minima',
        fixed_surface_patches=patches,current_corners_ABCD_px=corners,paper_moved_in_image=True,
        paper_moved_independently_does_not_reset_fixed_old_plane_origin=True,
        old_models='paper50-geometry/model-comparison.json',
        session_configs=session['configs'],serial=session['serial'],frames=session['frames'],
        empty_source_disk_user_confirmed=True,source_disk_was_not_commanded_to_rotate=True,
        ruler_surface_height_above_disk_mm=None,perpendicular_ruler_axis_checked=False,motion_allowed=False)
    (O/'measurement-input.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    (O/'ruler-ticks-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    (O/'ruler-ticks-detail.png').write_bytes(cv2.imencode('.png',overlay[:,775:975])[1].tobytes())
    print(json.dumps(dict(ticks=ticks,fixed_surface_patches=patches,current_corners_ABCD_px=corners),indent=2))
if __name__=='__main__':main()
