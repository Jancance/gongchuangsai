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

VERSION = 'tabletop-color-probe-v7-round-face-color-candidate'


def bright_round_face(frame,masks,parent,value):
    """当前轮廓内提取明亮圆面；颜色仍使用原蒙版，不用历史标签。"""
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    masks=dict(masks)
    masks[6]=cv2.inRange(hsv,(90,25,100),(115,189,255))
    masks[3]=cv2.bitwise_and(masks[3],cv2.bitwise_not(masks[6]))
    x,y,w,h=cv2.boundingRect(parent)
    parent_mask=np.zeros((h,w),np.uint8)
    cv2.drawContours(parent_mask,[parent-np.array([[[x,y]]],np.int32)],-1,255,-1)
    v=value[y:y+h,x:x+w]
    samples=v[(parent_mask>0)&(v<250)]
    if samples.size<500:return None
    cutoff=max(60.,float(np.percentile(samples,60)))
    bright=((parent_mask>0)&(v>=cutoff)).astype(np.uint8)*255
    bright=cv2.morphologyEx(bright,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    contours,_=cv2.findContours(bright,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
    candidates=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<1000 or area/cv2.contourArea(parent)<.20:continue
        rx,ry,rw,rh=cv2.boundingRect(contour)
        perimeter=cv2.arcLength(contour,True)
        circularity=4*np.pi*area/max(perimeter**2,1)
        solidity=area/max(cv2.contourArea(cv2.convexHull(contour)),1)
        if circularity<.60 or solidity<.90 or max(rw/rh,rh/rw)>1.4:continue
        face=np.zeros((h,w),np.uint8)
        cv2.drawContours(face,[contour],-1,255,-1)
        face=cv2.erode(face,np.ones((7,7),np.uint8))>0
        valid=face & (v<250)
        usable=float(valid.sum()/max(face.sum(),1))
        if usable<.35 or valid.sum()<500:continue
        scores={names[k]:float((m[y:y+h,x:x+w][valid]>0).mean()) for k,m in masks.items()}
        ranking=sorted(scores,key=scores.get,reverse=True)
        label=ranking[0]
        if label not in ('blue','lightblue') or scores[label]<.65 or scores[label]-scores[ranking[1]]<.25:continue
        moment=cv2.moments(contour)
        candidates.append({'label':label,'bbox_px':[x+rx,y+ry,rw,rh],
            'area_px2':area,'silhouette_center_px':[x+moment['m10']/moment['m00'],y+moment['m01']/moment['m00']],
            'color_fractions':scores,'usable_fraction':usable,
            'circularity':float(circularity),'solidity':float(solidity),
            'face_value_cutoff':cutoff,'face_parent_area_fraction':area/cv2.contourArea(parent),
            'proposal_source':'bright_round_face','reason':'CURRENT_ROUND_FACE_COLOR',
            'center_reference':'bright_face_pixels_not_calibrated_grasp_point'})
    return candidates[0] if len(candidates)==1 else None


def hinted_color_circles(frame,masks,rejected,known,hints,value,diagnostics=None,geometry_masks=None):
    """旧框只限定搜索，必须有当前帧颜色与至少75%周向边界证据。"""
    rows=[]
    names={1:'red',2:'yellow',3:'blue',4:'green',6:'lightblue'}
    occupied={r['label'] for r in known if r['label']!='UNKNOWN'}
    height,width=frame.shape[:2]
    for hint in hints or []:
        color=hint['color']
        color_id=next((k for k,v in names.items() if v==color),None)
        if color_id is None or color in occupied:
            continue
        x,y,w,h=hint['bbox_px']
        margin=max(20,int(min(w,h)*.4))
        left,top=max(0,x-margin),max(0,y-margin)
        right,bottom=min(width,x+w+margin),min(height,y+h+margin)
        geometry=(geometry_masks or masks)[color_id]
        mask=geometry[top:bottom,left:right]
        if mask.size==0:
            continue
        small=cv2.resize(mask,None,fx=.5,fy=.5,interpolation=cv2.INTER_NEAREST)
        small=cv2.GaussianBlur(small,(5,5),1)
        circles=cv2.HoughCircles(small,cv2.HOUGH_GRADIENT,1,max(8,min(w,h)*.05),
            param1=80,param2=20,minRadius=max(8,int(min(w,h)*.15)),
            maxRadius=max(9,int(min(w,h)*.30)))
        proposals=[]
        if hint.get('proposal_source')=='chromatic_circle' and hint.get('radius_px'):
            hx,hy=hint['center_px']
            proposals.append((float(hx),float(hy),float(hint['radius_px']),'revalidated_previous_circle'))
        if circles is not None:
            proposals.extend((float(cx)*2+left,float(cy)*2+top,float(radius)*2,'hough')
                             for cx,cy,radius in circles[0])
        if not proposals:
            if diagnostics is not None:diagnostics.append({'color':color,'reason':'NO_CIRCLE_PROPOSAL'})
            continue
        supported=[]
        for cx,cy,radius,origin in proposals:
            detail={'color':color,'center_px':[cx,cy],'radius_px':radius,'circle_proposal':origin}
            if cx-radius<=1 or cy-radius<=1 or cx+radius>=width-1 or cy+radius>=height-1:
                if diagnostics is not None:diagnostics.append(dict(detail,reason='CIRCLE_TOUCHES_FRAME'))
                continue
            rx,ry=int(np.floor(cx-radius)),int(np.floor(cy-radius))
            rw,rh=int(np.ceil(cx+radius))-rx+1,int(np.ceil(cy+radius))-ry+1
            yy,xx=np.ogrid[ry:ry+rh,rx:rx+rw]
            disk=(xx-cx)**2+(yy-cy)**2<=(radius*.87)**2
            parent_region=np.zeros((rh,rw),np.uint8)
            for parent in rejected:
                cv2.drawContours(parent_region,[parent-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
            parent_fraction=float(((parent_region>0)&disk).sum()/max(disk.sum(),1))
            # 圆心可落在高光缺口中，以圆面重叠支持代替单像素归属检查。
            if parent_fraction<.5:
                if diagnostics is not None:diagnostics.append(dict(detail,reason='INSUFFICIENT_PARENT_OVERLAP',parent_fraction=parent_fraction))
                continue
            # 逐角度检查内圈为该色、外圈为背景；连接手指的角度不能算圆弧证据。
            angles=np.arange(72)*2*np.pi/72
            inside=[];outside=[];in_bounds=np.ones(72,dtype=bool)
            for scale,destination in ((.62,inside),(.72,inside),(.82,inside),(.90,inside),(.96,inside),
                                      (1.20,outside),(1.30,outside)):
                xx=np.rint(cx+radius*scale*np.cos(angles)).astype(int)
                yy=np.rint(cy+radius*scale*np.sin(angles)).astype(int)
                valid=(xx>=0)&(xx<width)&(yy>=0)&(yy<height)
                in_bounds &= valid
                sampled=np.zeros(72,bool)
                sampled[valid]=geometry[yy[valid],xx[valid]]>0
                destination.append(sampled)
            arc_fraction=float((in_bounds & (np.sum(inside,axis=0)>=2)
                                & ~np.all(outside,axis=0)).mean())
            if arc_fraction<.75:
                if diagnostics is not None:diagnostics.append(dict(detail,reason='INSUFFICIENT_ARC',arc_fraction=arc_fraction))
                continue
            valid=disk & (value[ry:ry+rh,rx:rx+rw]<250)
            usable=float(valid.sum()/max(disk.sum(),1))
            if usable<.35 or valid.sum()<500:
                if diagnostics is not None:diagnostics.append(dict(detail,reason='INSUFFICIENT_NONCLIPPED_PIXELS',usable_fraction=usable))
                continue
            labels={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
            scores={labels[k]:float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean()) for k,m in masks.items()}
            ranking=sorted(scores,key=scores.get,reverse=True)
            if ranking[0]!=color or scores[color]<.65 or scores[color]-scores[ranking[1]]<.25:
                if diagnostics is not None:diagnostics.append(dict(detail,reason='CIRCLE_COLOR_AMBIGUOUS',color_fractions=scores))
                continue
            if diagnostics is not None:diagnostics.append(dict(detail,reason='CURRENT_FRAME_VALIDATED',arc_fraction=arc_fraction))
            supported.append({'bbox_px':[rx,ry,rw,rh],'area_px2':float(np.pi*radius**2),
                'label':color,'reason':'CURRENT_COLOR_ARC_SUPPORT',
                'proposal_source':'chromatic_circle','silhouette_center_px':[cx,cy],
                'center_reference':'current_color_circle_not_calibrated_grasp_point',
                'arc_fraction':arc_fraction,'usable_fraction':usable,'color_fractions':scores,
                'radius_px':radius,'circle_proposal':origin,'parent_fraction':parent_fraction})
        # 同一圆的近邻提议合并；两个不同的可支持圆仍为歧义。
        distinct=[]
        for row in sorted(supported,key=lambda r:r['arc_fraction'],reverse=True):
            if any(np.linalg.norm(np.array(row['silhouette_center_px'])-np.array(other['silhouette_center_px']))
                   <.45*min(row['radius_px'],other['radius_px'])
                   and max(row['radius_px'],other['radius_px'])/min(row['radius_px'],other['radius_px'])<1.3
                   for other in distinct):
                continue
            distinct.append(row)
        if len(distinct)==1:
            rows.extend(distinct)
        elif len(distinct)>1 and diagnostics is not None:
            diagnostics.append({'color':color,'reason':'MULTIPLE_SUPPORTED_CIRCLES','count':len(distinct)})
    return rows


def chromatic_fallback(frame, masks, rejected, accepted,geometry_masks=None):
    """从被合并轮廓拒绝的区域中寻找独立彩色圆面，不解除碰边/质量门。"""
    if not rejected:
        return []
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    value=hsv[:,:,2]
    names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
    candidates=[]
    for color_id in (1,2,3,4,6):
        geometry=(geometry_masks or masks)[color_id]
        contours,_=cv2.findContours(geometry,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour)<1000:
                continue
            x,y,w,h=cv2.boundingRect(contour)
            if x<=1 or y<=1 or x+w>=frame.shape[1]-1 or y+h>=frame.shape[0]-1:
                continue
            # 必须位于被拒绝的父轮廓内，不能成为无关联的宽松候选。
            moment=cv2.moments(contour)
            point=(moment['m10']/moment['m00'],moment['m01']/moment['m00'])
            if not any(cv2.pointPolygonTest(parent,point,False)>=0 for parent in rejected):
                continue
            if any(cv2.pointPolygonTest(parent,point,False)>=0 for parent in accepted):
                continue
            # 仅在候选自身包围框内闭合同色细小缺口，不引入框外手指。
            pad=4
            local=np.zeros((h+2*pad,w+2*pad),np.uint8)
            local[pad:pad+h,pad:pad+w]=geometry[y:y+h,x:x+w]
            local=cv2.morphologyEx(local,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
            shapes,_=cv2.findContours(local,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            refined=max(shapes,key=cv2.contourArea)+np.array([[[x-pad,y-pad]]],dtype=np.int32)
            area=cv2.contourArea(refined)
            perimeter=cv2.arcLength(refined,True)
            circularity=4*np.pi*area/max(perimeter**2,1)
            solidity=area/max(cv2.contourArea(cv2.convexHull(refined)),1)
            rx,ry,rw,rh=cv2.boundingRect(refined)
            if circularity<.60 or solidity<.90 or max(rw/rh,rh/rw)>1.4:
                continue
            roi=np.zeros((rh,rw),np.uint8)
            cv2.drawContours(roi,[refined-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
            roi=cv2.erode(roi,np.ones((7,7),np.uint8))>0
            valid=roi & (value[ry:ry+rh,rx:rx+rw]<250)
            usable=float(valid.sum()/max(roi.sum(),1))
            if usable<.35 or valid.sum()<500:
                continue
            scores={names[k]:float((mask[ry:ry+rh,rx:rx+rw][valid]>0).mean())
                    for k,mask in masks.items()}
            ranking=sorted(scores,key=scores.get,reverse=True)
            label=names[color_id]
            if ranking[0]!=label or scores[label]<.65 or scores[label]-scores[ranking[1]]<.25:
                continue
            moment=cv2.moments(refined)
            center=[moment['m10']/moment['m00'],moment['m01']/moment['m00']]
            if any(np.linalg.norm(np.array(center)-np.array(r['silhouette_center_px']))<min(rw,rh)*.4
                   for r in candidates):
                continue
            if label=='blue':
                face=bright_round_face(frame,masks,refined,value)
                if face is not None and face['label']=='lightblue':
                    candidates.append(face);continue
            candidates.append({'bbox_px':[rx,ry,rw,rh],'area_px2':area,'label':label,
                'reason':'ISOLATED_CHROMATIC_COMPONENT','proposal_source':'chromatic_component',
                'center_reference':'chromatic_region_not_calibrated_grasp_point',
                'circularity':float(circularity),'solidity':float(solidity),
                'usable_fraction':usable,'color_fractions':scores,
                'dark_fraction':float((value[ry:ry+rh,rx:rx+rw][valid]<45).mean()),
                'silhouette_center_px':center})
    return candidates

def analyze(frame,color_config=None,prior_regions=None):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h,s,v = cv2.split(hsv)
    colorful = ((s>=90)&(v>=35)&((h<=12)|(h>=165)|((h>=18)&(h<=85))))
    # Include dim blue sidewalls in the proposal; reliable class votes below
    # still require V>=60. This avoids a V=45..59 hole in the silhouette.
    strong_blue = (h>=100)&(h<=135)&(s>=140)&(v>=35)
    pale_blue = (h>=90)&(h<=115)&(s>=25)&(s<=180)&(v>=100)
    calibrated_masks=None
    if color_config is not None:
        from 颜色标定 import make_all_color_masks
        calibrated_masks=make_all_color_masks(frame,color_config)
        mask=np.zeros(gray.shape,np.uint8)
        for m in calibrated_masks.values():mask=cv2.bitwise_or(mask,m)
    else:
        mask = ((colorful | strong_blue | pale_blue | (v<45)).astype(np.uint8))*255
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    records, drawing = [], frame.copy()
    rejected=[];accepted=[];circle_diagnostics=[]
    for contour in sorted(contours,key=cv2.contourArea,reverse=True):
        area = cv2.contourArea(contour)
        if area<1000:
            continue
        x,y,w,ht=cv2.boundingRect(contour)
        row={'bbox_px':[x,y,w,ht],'area_px2':area,'label':'UNKNOWN','reason':'',
             'proposal_source':'combined_silhouette'}
        if x<=1 or y<=1 or x+w>=frame.shape[1]-1 or y+ht>=frame.shape[0]-1:
            row['reason']='TOUCHES_BORDER';records.append(row);rejected.append(contour);continue
        perimeter=cv2.arcLength(contour,True)
        circularity=4*np.pi*area/max(perimeter**2,1)
        solidity=area/max(cv2.contourArea(cv2.convexHull(contour)),1)
        if circularity<.35 or solidity<.7 or max(w/ht,ht/w)>2:
            row['reason']='SHAPE_UNCERTAIN';records.append(row);rejected.append(contour);continue
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
        if calibrated_masks is not None:
            names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
            scores={names[k]:float((m[valid]>0).mean()) for k,m in calibrated_masks.items()}
        dark=float((V<45).mean())
        ranking=sorted(scores,key=scores.get,reverse=True)
        best,second=ranking[:2]
        if calibrated_masks is None and dark>=.70 and scores[best]<.08:
            row.update(label='black',reason='DARK_WITH_LITTLE_CHROMATIC_EVIDENCE')
        elif scores[best]>=.30 and scores[best]-scores[second]>=.15:
            row.update(label=best,reason='DOMINANT_COLOR_EVIDENCE')
        else:
            row['reason']='AMBIGUOUS_COLOR'
            # 蓝/浅蓝与暗侧面混票时，允许原有严格圆面分支再检查。
            if best in ('blue','lightblue','black') and second in ('blue','lightblue','black'):
                rejected.append(contour)
        row.update(color_fractions=scores,dark_fraction=dark)
        m=cv2.moments(contour)
        center=[m['m10']/m['m00'],m['m01']/m['m00']]
        row['silhouette_center_px']=center
        # 蓝/浅蓝整体低置信混票时以当前圆面复核，侧面不覆盖已证实的面色。
        if calibrated_masks is not None and row['label']=='blue' \
                and scores['blue']<.65 and scores.get('lightblue',0)>=.15:
            face=bright_round_face(frame,calibrated_masks,contour,v)
            row['silhouette_color_fractions']=scores
            if face is None:
                row.update(label='UNKNOWN',reason='BLUE_FAMILY_FACE_UNVERIFIED')
            else:
                row.update(face)
        if calibrated_masks is not None and (row['label']=='blue' or row['reason']=='AMBIGUOUS_COLOR'):
            face=bright_round_face(frame,calibrated_masks,contour,v)
            if face is not None and face['label']=='lightblue':row.update(face)
        records.append(row)
        if row['label']!='UNKNOWN':accepted.append(contour)
        if row['proposal_source']=='bright_round_face':
            dx,dy,dw,dh=row['bbox_px'];cv2.rectangle(drawing,(dx,dy),(dx+dw,dy+dh),(0,255,0),2)
        else:
            cv2.drawContours(drawing,[contour],-1,(0,255,0),2)
        cv2.putText(drawing,row['label'],(row['bbox_px'][0],max(25,row['bbox_px'][1]-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
    if calibrated_masks is not None:
        # 蓝色高光可变白/低饱和，只辅助轮廓提议；分类票数仍取原六色蒙版。
        geometry_masks=dict(calibrated_masks)
        highlight=((v>=220)&(s<=140))|(v>=250)
        geometry_masks[3]=cv2.bitwise_or(calibrated_masks[3],calibrated_masks[6])
        geometry_masks[3]=cv2.bitwise_or(geometry_masks[3],highlight.astype(np.uint8)*255)
        for row in chromatic_fallback(frame,calibrated_masks,rejected,accepted,geometry_masks):
            records.append(row)
            x,y,w,ht=row['bbox_px']
            cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
            cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
        for row in hinted_color_circles(frame,calibrated_masks,rejected,records,prior_regions,v,circle_diagnostics,geometry_masks):
            records.append(row)
            x,y,w,ht=row['bbox_px']
            cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
    return {'version':VERSION,'regions':records,'circle_diagnostics':circle_diagnostics,
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
