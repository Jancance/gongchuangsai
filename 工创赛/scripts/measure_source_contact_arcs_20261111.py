"""Extract visible material contact-arc evidence in archived real images.
Uses the empty-image circle locations as tracking/search priors: this is an
offline feasibility check, not a tested unknown-phase acquisition algorithm.
"""
from pathlib import Path
import json,hashlib
import cv2,numpy as np
from measure_source_paper50_20261111 import read
R=Path(__file__).resolve().parents[1];B=R/'evidence/integration/2026-11-11/source-view-entry'
O=B/'triangle-feasibility'
def main():
    O.mkdir(exist_ok=True)
    ground=json.loads((B/'clean-disk-review/matlab-result.json').read_text(encoding='utf-8'))
    circles=ground['circles'];samples=[]
    for folder in ['source-observe-20261111-032846','source-observe-20261111-033718']:
        path=B/folder/'last-raw.png';im=read(path);g=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);overlay=im.copy();arcs=[]
        for c in circles:
            center=np.array(c['center_image_px'],float)-[1,0]
            direction=np.arctan2(540-center[1],960-center[0])
            points=[];strengths=[]
            radius=c['radius_mm']/np.hypot(.2410467291222243,.036166778783864782)
            for angle in direction+np.deg2rad(np.linspace(-60,60,241)):
                r=np.arange(radius-18,radius+18.01,.25)
                x=center[0]+r*np.cos(angle);y=center[1]+r*np.sin(angle)
                values=cv2.remap(g,x.astype('float32')[None],y.astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
                grad=(values[8:]-values[:-8])/2
                idx=int(np.argmax(grad))+4
                if 6<idx<len(r)-7 and grad[idx-4]>10:
                    points.append([float(x[idx]),float(y[idx])]);strengths.append(float(grad[idx-4]))
            arcs.append(dict(id=c['id'],boundary_image_px=points,
                radial_gradient_strength=strengths,empty_circle_prior_px=center.tolist(),
                visible_sector_deg=120,prior_used_for_search=True))
            for p in points[::3]:cv2.circle(overlay,tuple(np.rint(p).astype(int)),2,(0,255,255),1)
            cv2.putText(overlay,c['id'],tuple(np.rint(center).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
        sample=dict(image_path=path.relative_to(R).as_posix(),image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),arcs=arcs)
        samples.append(sample)
        (O/(folder+'-contact-arcs.png')).write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
        print(folder,[(a['id'],len(a['boundary_image_px'])) for a in arcs])
    data=dict(schema_version=1,samples=samples,empty_ground_truth=circles,
        ground_truth_source='clean-disk-review/matlab-result.json',
        image_registration_material_to_empty_px=[1,0],
        key_limitations=['search priors from empty circles','only stationary same-phase images',
            'no live colour/mask/provider validation','observed contact arc may include marker edge'],
        motion_allowed=False)
    (O/'contact-arc-input.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__':main()
