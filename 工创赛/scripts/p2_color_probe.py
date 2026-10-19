"""Offline six-color development probe for supplied isolated tabletop samples.

NOT a robot detector: no camera, serial port, motion target or calibrated pose.
Pixel centers describe candidate silhouettes (which may include sidewalls/shadow).
Thresholds are provisional; development-set agreement is not an accuracy claim.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

VERSION = 'tabletop-color-probe-v0'

def analyze(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h,s,v = cv2.split(hsv)
    colorful = ((s>=90)&(v>=35)&((h<=12)|(h>=165)|((h>=18)&(h<=85))|((h>=100)&(h<=135))))
    pale_blue = (h>=90)&(h<=115)&(s>=25)&(s<=180)&(v>=100)
    mask = ((colorful | pale_blue | (gray<=60)).astype(np.uint8))*255
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    records, drawing = [], frame.copy()
    for contour in sorted(contours,key=cv2.contourArea,reverse=True):
        area = cv2.contourArea(contour)
        if area<1000:
            continue
        x,y,w,ht=cv2.boundingRect(contour)
        row={'bbox_px':[x,y,w,ht],'area_px2':area,'label':'UNKNOWN','reason':''}
        if x<=1 or y<=1 or x+w>=frame.shape[1]-1 or y+ht>=frame.shape[0]-1:
            row['reason']='TOUCHES_BORDER';records.append(row);continue
        perimeter=cv2.arcLength(contour,True)
        circularity=4*np.pi*area/max(perimeter**2,1)
        solidity=area/max(cv2.contourArea(cv2.convexHull(contour)),1)
        if circularity<.35 or solidity<.7 or max(w/ht,ht/w)>2:
            row['reason']='SHAPE_UNCERTAIN';records.append(row);continue
        region=np.zeros(gray.shape,np.uint8)
        cv2.drawContours(region,[contour],-1,255,-1)
        region=cv2.erode(region,np.ones((7,7),np.uint8))>0
        valid=region & (v<250)
        usable=float(valid.sum()/max(region.sum(),1))
        row['usable_fraction']=usable
        if usable<.35 or valid.sum()<500:
            row['reason']='INSUFFICIENT_NONCLIPPED_PIXELS';records.append(row);continue
        H,S,V=h[valid],s[valid],v[valid]
        votes={
            'red': ((H<=12)|(H>=165))&(S>=90)&(V>=35),
            'yellow': (H>=18)&(H<35)&(S>=90)&(V>=35),
            'green': (H>=35)&(H<=85)&(S>=60)&(V>=35),
            'blue': (H>=100)&(H<=135)&(S>=140)&(V>=60),
            'lightblue': (H>=90)&(H<=115)&(S>=25)&(S<140)&(V>=100),
        }
        scores={k:float(val.mean()) for k,val in votes.items()}
        dark=float((V<45).mean())
        ranking=sorted(scores,key=scores.get,reverse=True)
        best,second=ranking[:2]
        if dark>=.70 and scores[best]<.08:
            row.update(label='black',reason='DARK_WITH_LITTLE_CHROMATIC_EVIDENCE')
        elif scores[best]>=.30 and scores[best]-scores[second]>=.15:
            row.update(label=best,reason='DOMINANT_COLOR_EVIDENCE')
        else:
            row['reason']='AMBIGUOUS_COLOR'
        row.update(color_fractions=scores,dark_fraction=dark)
        m=cv2.moments(contour)
        center=[m['m10']/m['m00'],m['m01']/m['m00']]
        row['silhouette_center_px']=center
        records.append(row)
        cv2.drawContours(drawing,[contour],-1,(0,255,0),2)
        cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
    return {'version':VERSION,'regions':records,
            'scope':'OFFLINE DEVELOPMENT ONLY; no metric coordinates or grasp target'},drawing

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',required=True,type=Path)
    p.add_argument('--output-dir',required=True,type=Path)
    args=p.parse_args();args.output_dir.mkdir(parents=True,exist_ok=True)
    summary=[];tiles=[]
    for path in sorted(args.input_dir.glob('*.png')):
        frame=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None: raise RuntimeError(f'Cannot decode {path}')
        result,drawing=analyze(frame);result['file']=path.name
        (args.output_dir/(path.stem+'.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
        ok,data=cv2.imencode('.png',drawing)
        if not ok: raise RuntimeError('Encode failed')
        data.tofile(args.output_dir/(path.stem+'-overlay.png'))
        summary.append(result)
        desc=[r['label']+':'+r['reason'] for r in result['regions']]
        print(path.name,desc or ['NO_CANDIDATE'])
        tile=cv2.resize(drawing,(480,360));footer=np.full((50,480,3),245,np.uint8)
        cv2.putText(footer,path.name,(8,18),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,0,0),1)
        labels=[r['label'] for r in result['regions'] if r['reason']!='TOUCHES_BORDER']
        cv2.putText(footer,','.join(labels) or 'NO INTERIOR CANDIDATE',(8,40),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
        tiles.append(np.vstack([tile,footer]))
    if not summary: raise RuntimeError('No PNG inputs')
    (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    if len(tiles)%2: tiles.append(np.full_like(tiles[0],245))
    grid=np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)])
    ok,data=cv2.imencode('.png',grid)
    if not ok: raise RuntimeError('Encode failed')
    data.tofile(args.output_dir/'overview.png')

if __name__=='__main__':
    main()
