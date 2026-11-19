"""Independent left-side ruler image extraction, no hardware access."""
from pathlib import Path
import hashlib,json
import cv2,numpy as np
from measure_source_paper50_20261111 import read
R=Path(__file__).resolve().parents[1];B=R/'evidence/integration/2026-11-11/source-view-entry'
S=B/'source-observe-20261112-012041';O=B/'ruler-left-20261112'

def main():
    O.mkdir(exist_ok=True);im=read(S/'last-raw.png');g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);overlay=im.copy();ticks=[]
    # Image-selected long-tick search windows, before metric evaluation.
    for cm in range(4,25):
        # The raw-image upper long ticks give about40.75px/cm; the first
        # 41.55 estimate drifted below the lower numbered long ticks.
        guess=round(81.7+(cm-4)*40.75)
        ys=np.arange(guess-8,guess+9);left=681-.064*ys
        xx=left[:,None]+np.arange(12,31)[None,:];yy=np.repeat(ys[:,None],xx.shape[1],axis=1)
        values=cv2.remap(g,xx.astype('float32'),yy.astype('float32'),cv2.INTER_LINEAR).mean(axis=1).astype(float)
        idx=int(np.argmin(values));offset=0.
        if 0<idx<len(values)-1:
            den=values[idx-1]-2*values[idx]+values[idx+1]
            if den>1e-6:offset=float(np.clip(.5*(values[idx-1]-values[idx+1])/den,-.5,.5))
        y=float(ys[idx]+offset);x=681-.064*y+21;boundary=idx in (0,len(values)-1)
        ticks.append(dict(label_cm=cm,position_mm=cm*10,point_px=[x,y],initial_y_px=guess,
            search_boundary_hit=boundary,accepted_for_metric=not boundary and cm!=16,
            rejection_reason='bright reflection stripe' if cm==16 else ('search boundary' if boundary else None)))
        cv2.line(overlay,(round(x-18),round(y)),(round(x+20),round(y)),(0,0,255),1)
        cv2.putText(overlay,str(cm),(795,round(y)+4),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
    before=cv2.cvtColor(read(B/'source-observe-20261112-010846/last-raw.png'),cv2.COLOR_BGR2GRAY);patches=[]
    for x,y in [(1005,330),(945,374),(1014,403),(953,449),(1021,480),(996,239)]:
        a=cv2.matchTemplate(g[y-24:y+25,x-24:x+25],before[y-12:y+13,x-12:x+13],cv2.TM_CCOEFF_NORMED)
        _,score,_,where=cv2.minMaxLoc(a);patches.append(dict(before_px=[x,y],correlation=score,integer_shift_px=[where[0]-12,where[1]-12]))
    s=json.loads((S/'session.json').read_text(encoding='utf-8'))
    data=dict(ticks=ticks,excluded_cm=[16],fixed_surface_patches=patches,image_size=[im.shape[1],im.shape[0]],
        frames=s['frames'],error=s['error'],serial=s['serial'],motion_allowed=False,
        ruler_pose='left side, near vertical; photographed pose is authoritative',
        sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in S.iterdir() if f.is_file()})
    (O/'measurement-input.json').write_text(json.dumps(data,indent=2),encoding='utf-8')
    (O/'ticks-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    (O/'ticks-detail.png').write_bytes(cv2.imencode('.png',overlay[:,600:850])[1].tobytes())
    print(json.dumps(dict(ticks=ticks,patches=patches),indent=2))
if __name__=='__main__':main()
