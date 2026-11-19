"""Extract visible centimetre engravings for independent MATLAB checking."""
from pathlib import Path
import json,hashlib
import cv2,numpy as np
from measure_source_paper50_20261111 import read
R=Path(__file__).resolve().parents[1]
B=R/'evidence/integration/2026-11-11/source-view-entry'
S=B/'source-observe-20261112-011409';O=B/'ruler-current-20261112'

def main():
    O.mkdir(exist_ok=True);im=read(S/'last-raw.png');g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    ticks=[];overlay=im.copy()
    # Labels identify the long tick just before each printed centimetre number.
    # 16..18 are excluded before evaluation because of specular glare.
    lower_guesses={19:645,20:687,21:728,22:770,23:811,24:853,25:894,26:936,27:977}
    for cm in list(range(4,16))+list(range(19,28)):
        # Lower tick search windows updated from enlarged raw image, before
        # any MATLAB metric comparison. Uniform initial spacing drifted.
        guess=lower_guesses.get(cm,round(30+(cm-4)*40.8))
        ys=np.arange(guess-4,guess+5);xs=1114-.036*ys
        xx=xs[:,None]+np.arange(24,35)[None,:]
        yy=np.repeat(ys[:,None],xx.shape[1],axis=1)
        val=cv2.remap(g,xx.astype('float32'),yy.astype('float32'),cv2.INTER_LINEAR).mean(axis=1).astype(float)
        idx=int(np.argmin(val));offset=0.
        if 0<idx<len(val)-1:
            den=val[idx-1]-2*val[idx]+val[idx+1]
            if den>1e-6:offset=float(np.clip(.5*(val[idx-1]-val[idx+1])/den,-.5,.5))
        y=float(ys[idx]+offset);x=1114-.036*y+29
        ticks.append(dict(label_cm=cm,position_mm=cm*10,point_px=[x,y],initial_y_px=guess,search_boundary_hit=idx in (0,len(val)-1),
            accepted_for_metric=cm<=15 and idx not in (0,len(val)-1),
            rejection_reason='lower shadowed engravings uncertain in enlarged raw image' if cm>=19 else ('search boundary' if idx in (0,len(val)-1) else None)))
        cv2.line(overlay,(round(x-24),round(y)),(round(x+12),round(y)),(0,0,255),1)
        cv2.putText(overlay,str(cm),(1235,round(y)+4),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    before=cv2.cvtColor(read(B/'source-observe-20261112-010846/last-raw.png'),cv2.COLOR_BGR2GRAY)
    patches=[]
    for x,y in [(1005,330),(945,374),(1014,403),(953,449),(1021,480),(996,239)]:
        a=cv2.matchTemplate(g[y-24:y+25,x-24:x+25],before[y-12:y+13,x-12:x+13],cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(a);patches.append(dict(before_px=[x,y],correlation=score,integer_shift_px=[where[0]-12,where[1]-12]))
    session=json.loads((S/'session.json').read_text(encoding='utf-8'))
    data=dict(ticks=ticks,excluded_cm=[16,17,18],exclusion_basis='specular glare identified before evaluation',
        tick_selection_method='visible numbered centimetre long engravings, local dark-line minimum',
        fixed_surface_patches=patches,image_size=[im.shape[1],im.shape[0]],frames=session['frames'],serial=session['serial'],error=session['error'],
        ruler_surface_height_above_disk_mm=None,perpendicular_axis_checked=False,motion_allowed=False,
        sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in S.iterdir() if f.is_file()})
    (O/'measurement-input.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    (O/'ticks-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    (O/'ticks-detail.png').write_bytes(cv2.imencode('.png',overlay[:,1060:1320])[1].tobytes())
    print(json.dumps(dict(ticks=ticks,patches=patches),indent=2))
if __name__=='__main__':main()
