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

VERSION = 'tabletop-color-probe-v13-disjoint-black-colour-evidence-candidate'


def bright_round_face(frame,masks,parent,value,value_percentile=60,diagnostics=None,prepared_parent=None,value_cutoff=None,proposal_mask=None):
    """当前轮廓内提取明亮圆面；颜色仍使用原蒙版，不用历史标签。"""
    if prepared_parent is None:
        x,y,w,h=cv2.boundingRect(parent)
        parent_mask=np.zeros((h,w),np.uint8)
        cv2.drawContours(parent_mask,[parent-np.array([[[x,y]]],np.int32)],-1,255,-1)
        v=value[y:y+h,x:x+w]
    else:
        x,y,w,h,parent_mask,v=prepared_parent
    if value_cutoff is None:
        samples=v[(parent_mask>0)&(v<250)]
        if samples.size<500:return None
        cutoff=max(60.,float(np.percentile(samples,value_percentile)))
    else:cutoff=value_cutoff
    parent_area=cv2.contourArea(parent)
    if proposal_mask is None:
        bright=((parent_mask>0)&(v>=cutoff)).astype(np.uint8)*255
    else:
        bright=((parent_mask>0)&(proposal_mask>0)).astype(np.uint8)*255
    bright=cv2.morphologyEx(bright,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
    contours,_=cv2.findContours(bright,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
    candidates=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<1000 or area/parent_area<.20:continue
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
            'face_value_cutoff':cutoff,'face_parent_area_fraction':area/parent_area,
            'proposal_source':'bright_round_face','reason':'CURRENT_ROUND_FACE_COLOR',
            'center_reference':'bright_face_pixels_not_calibrated_grasp_point'})
    if diagnostics is not None:diagnostics['supported_faces']=len(candidates)
    return candidates[0] if len(candidates)==1 else None


def pale_face_candidate(frame,masks,parent,value,settings):
    """仅在当前圆面内复核浅蓝；原六色蒙版继续负责几何，不改主颜色范围。"""
    if settings is None:return None
    if settings.get('lightblue_s_max')!=189:
        raise ValueError('Unsupported round-face color candidate')
    x,y,w,h=cv2.boundingRect(parent)
    crop=frame[y:y+h,x:x+w]
    hsv=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
    face_masks={k:m[y:y+h,x:x+w] for k,m in masks.items()}
    face_masks[6]=cv2.inRange(hsv,(90,25,100),(115,189,255))
    face_masks[3]=cv2.bitwise_and(face_masks[3],cv2.bitwise_not(face_masks[6]))
    local_value=value[y:y+h,x:x+w]
    # 已有门要求至少500有效像素且浅蓝票>=65%，少于325个浅蓝
    # 非削顶像素不可能通过；此处只跳过必定失败的重复切分。
    if cv2.countNonZero(cv2.bitwise_and(face_masks[6],(local_value<250).astype(np.uint8)*255))<325:return None
    local_parent=parent-np.array([[[x,y]]],np.int32)
    parent_mask=np.zeros((h,w),np.uint8)
    cv2.drawContours(parent_mask,[local_parent],-1,255,-1)
    samples=local_value[(parent_mask>0)&(local_value<250)]
    if samples.size<500:return None
    cutoffs=np.maximum(60.,np.percentile(samples,(50,55,60)))
    prepared=(0,0,w,h,parent_mask,local_value)
    candidates=[]
    for percentile,cutoff in zip((50,55,60),cutoffs):
        diagnosis={}
        face=bright_round_face(crop,face_masks,local_parent,local_value,percentile,diagnosis,prepared,float(cutoff))
        if diagnosis.get('supported_faces',0)>1:return None
        if face is not None:
            face['bbox_px'][0]+=x;face['bbox_px'][1]+=y
            face['silhouette_center_px'][0]+=x;face['silhouette_center_px'][1]+=y
            face['face_value_percentile']=percentile
            candidates.append(face)
    if settings.get('current_pale_component',False):
        # 暗处亮度切分会截掉圆面；另用当前浅蓝像素提出完整区域，
        # 仍复用同一圆形度/父区域/有效像素/色票/多面歧义检查。
        diagnosis={}
        face=bright_round_face(crop,face_masks,local_parent,local_value,diagnostics=diagnosis,
                               prepared_parent=prepared,value_cutoff=100.,proposal_mask=face_masks[6])
        if diagnosis.get('supported_faces',0)>1:return None
        if face is not None:
            face['bbox_px'][0]+=x;face['bbox_px'][1]+=y
            face['silhouette_center_px'][0]+=x;face['silhouette_center_px'][1]+=y
            face.update(face_value_percentile='CURRENT_HSV_COMPONENT',proposal_source='pale_component_face')
            candidates.append(face)
    if not candidates or any(r['label']!='lightblue' for r in candidates):return None
    best=max(candidates,key=lambda r:r['area_px2'])
    bx,by,bw,bh=best['bbox_px']
    for row in candidates:
        x,y,w,h=row['bbox_px']
        if np.linalg.norm(np.array(row['silhouette_center_px'])-best['silhouette_center_px'])>.4*min(w,h,bw,bh):return None
        if max(min(w,h),min(bw,bh))/min(min(w,h),min(bw,bh))>1.5:return None
    best.update(reason='CURRENT_PALE_FACE_COLOR',face_color_candidate='S189_CURRENT_FACE_ONLY',
                face_agreeing_cutoffs=[r['face_value_percentile'] for r in candidates])
    return best


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


def current_pale_circle(frame,masks,parent,hsv,value):
    """当前蓝族分量只限定搜索，圆面位置/颜色/周向证据均来自本帧。"""
    x,y,w,h=cv2.boundingRect(parent)
    palette=dict(masks)
    palette[6]=cv2.inRange(hsv,(90,25,100),(115,189,255))
    if cv2.countNonZero(palette[6][y:y+h,x:x+w])<325:return None
    palette[3]=cv2.bitwise_and(masks[3],cv2.bitwise_not(palette[6]))
    hints=[{'color':'lightblue','bbox_px':[x,y,w,h],'proposal_source':'current_component_search'}]
    faces=hinted_color_circles(frame,palette,[parent],[],hints,value)
    if len(faces)!=1:return None
    face=faces[0]
    face.update(reason='CURRENT_PALE_COMPONENT_ARC',proposal_source='current_pale_circle',
                search_reference='current_frame_component_bbox',face_color_candidate='S189_CURRENT_FACE_ONLY')
    return face


def conflicting_pale_evidence(frame,parent,value,settings):
    """当前像素有显著浅蓝证据但没有合法圆面时，不强归为第二个蓝色。"""
    if not settings or not settings.get('reject_unverified_blue_family',False):return None
    x,y,w,h=cv2.boundingRect(parent)
    hsv=cv2.cvtColor(frame[y:y+h,x:x+w],cv2.COLOR_BGR2HSV)
    region=np.zeros((h,w),np.uint8)
    cv2.drawContours(region,[parent-np.array([[[x,y]]],np.int32)],-1,255,-1)
    region=cv2.erode(region,np.ones((7,7),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)>0
    valid=region & (value[y:y+h,x:x+w]<250)
    count=int(valid.sum())
    if count<500:return None
    pale=cv2.inRange(hsv,(90,25,100),(115,189,249))
    pixels=int(((pale>0)&valid).sum())
    fraction=pixels/count
    # 30%复用既有显著色票门，是工程拒绝阈值，不证明物料为浅蓝。
    if pixels>=500 and fraction>=.30:
        return {'label':'UNKNOWN','reason':'PALE_COLOR_CONFLICT_WITHOUT_VALID_FACE',
                'conflicting_pale_fraction':fraction,'conflicting_pale_pixels':pixels,
                'note':'Current colour evidence is ambiguous; no material identity asserted.'}
    return None


def black_edge_recovery(frame,masks,rejected,accepted,hints,value,diagnostics,settings=None):
    """旧框只限搜索；当前边缘、暗色、父区域及唯一解均验证后才输出。"""
    if not rejected:return []
    height,width=frame.shape[:2]
    searches=[dict(hint,search_reference='fresh_black_bbox') for hint in hints or [] if hint['color']=='black']
    if not searches:
        # 首次检出只搜索当前独立紧凑拒绝分量，不从整幅手影假定物料。
        for parent in rejected:
            x,y,w,h=cv2.boundingRect(parent)
            if x<=1 or y<=1 or x+w>=width-1 or y+h>=height-1:continue
            if min(w,h)<80 or max(w,h)>500 or max(w/h,h/w)>1.8 or cv2.contourArea(parent)<5000:continue
            searches.append({'color':'black','bbox_px':[x,y,w,h],'search_reference':'current_compact_dark_region'})
            if len(searches)>=2:break
    names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
    angles=np.arange(72)*2*np.pi/72;cos=np.cos(angles);sin=np.sin(angles)
    supported=[];colour_rejections=0
    disjoint=bool(settings and settings.get('disjoint_colour_evidence',False))
    for hint in searches:
        x,y,w,h=hint['bbox_px'];margin=max(20,int(min(w,h)*.4))
        left,top=max(0,x-margin),max(0,y-margin)
        right,bottom=min(width,x+w+margin),min(height,y+h+margin)
        crop=frame[top:bottom,left:right]
        if crop.size==0:continue
        gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
        gray=cv2.GaussianBlur(gray,(5,5),1)
        gx=cv2.Sobel(gray,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(gray,cv2.CV_32F,0,1,ksize=3)
        offsets=np.linspace(-max(3.,min(w,h)*.5*.06),max(3.,min(w,h)*.5*.06),9)
        for dx in (-w*.08,0.,w*.08):
            for dy in (-h*.08,0.,h*.08):
                for scale in (.9,1.,1.1):
                    cx=x+w*.5+dx;cy=y+h*.5+dy;ax=w*.5*scale;ay=h*.5*scale
                    if cx-ax<=1 or cy-ay<=1 or cx+ax>=width-1 or cy+ay>=height-1:continue
                    if any(cv2.pointPolygonTest(p,(cx,cy),False)>=0 for p in accepted):continue
                    nx,ny=cos/ax,sin/ay;length=np.hypot(nx,ny);nx=nx/length;ny=ny/length
                    scores=[]
                    for delta in offsets:
                        xx=np.rint(cx+ax*cos+delta*nx).astype(int)-left
                        yy=np.rint(cy+ay*sin+delta*ny).astype(int)-top
                        good=(xx>=0)&(xx<gx.shape[1])&(yy>=0)&(yy<gx.shape[0])
                        sx=np.zeros(72);sy=np.zeros(72)
                        sx[good]=gx[yy[good],xx[good]];sy[good]=gy[yy[good],xx[good]]
                        magnitude=np.hypot(sx,sy)
                        alignment=np.abs(sx*nx+sy*ny)/np.maximum(magnitude,1.)
                        scores.append(np.where(good&(magnitude>=40)&(alignment>=.7),magnitude*alignment,0.))
                    scores=np.array(scores);edge=scores.max(axis=0)>0;fraction=float(edge.mean())
                    residual=float(np.sqrt(np.mean(offsets[scores[:,edge].argmax(axis=0)]**2))) if edge.any() else 999.
                    if fraction<.75 or residual>max(2.,min(ax,ay)*.04):continue
                    rx,ry=int(np.floor(cx-ax)),int(np.floor(cy-ay))
                    rw,rh=int(np.ceil(cx+ax))-rx+1,int(np.ceil(cy+ay))-ry+1
                    yy,xx=np.ogrid[ry:ry+rh,rx:rx+rw]
                    disk=((xx-cx)/(ax*.87))**2+((yy-cy)/(ay*.87))**2<=1.
                    parent_mask=np.zeros((rh,rw),np.uint8)
                    for parent in rejected:cv2.drawContours(parent_mask,[parent-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
                    parent_fraction=float(((parent_mask>0)&disk).sum()/max(disk.sum(),1))
                    if parent_fraction<.5:continue
                    valid=disk&(value[ry:ry+rh,rx:rx+rw]<250)
                    usable=float(valid.sum()/max(disk.sum(),1))
                    if usable<.35 or valid.sum()<500:continue
                    fractions={names[k]:float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean()) for k,m in masks.items()}
                    if disjoint:
                        # 黑范围与其他色V35..44重叠；这些同一像素不能重复
                        # 算作竞争色证据。保留原票供核对，竞争票只数黑范围外。
                        nonblack=masks[5][ry:ry+rh,rx:rx+rw]==0
                        conflicts={names[k]:float(((m[ry:ry+rh,rx:rx+rw]>0)&nonblack)[valid].mean())
                                   for k,m in masks.items() if k!=5}
                    else:conflicts={k:v for k,v in fractions.items() if k!='black'}
                    if fractions['black']<.65 or max(conflicts.values())>.10:
                        colour_rejections+=1;continue
                    row={'label':'black','proposal_source':'black_edge_ellipse','reason':'CURRENT_DARK_ELLIPSE_SUPPORT',
                        'bbox_px':[rx,ry,rw,rh],'area_px2':float(np.pi*ax*ay),'silhouette_center_px':[cx,cy],
                        'ellipse_axes_px':[ax,ay],'edge_fraction':fraction,'edge_fit_residual_px':residual,
                        'color_fractions':fractions,'usable_fraction':usable,'parent_fraction':parent_fraction,
                        'center_reference':'current_dark_silhouette_not_calibrated_grasp_point',
                        'search_reference':hint['search_reference']}
                    if disjoint:row.update(chromatic_nonblack_fractions=conflicts,colour_vote_policy='black_range_pixels_counted_once')
                    supported.append(row)
    distinct=[]
    for row in sorted(supported,key=lambda r:(-r['edge_fraction'],r['edge_fit_residual_px'])):
        if any(np.linalg.norm(np.array(row['silhouette_center_px'])-old['silhouette_center_px'])<.45*min(*row['ellipse_axes_px'],*old['ellipse_axes_px'])
               and max(np.sqrt(row['area_px2']/old['area_px2']),np.sqrt(old['area_px2']/row['area_px2']))<1.3 for old in distinct):continue
        distinct.append(row)
    diagnostics.append({'color':'black','reason':'CURRENT_BLACK_EDGE_VALIDATED' if len(distinct)==1 else 'BLACK_EDGE_NO_UNIQUE_SUPPORT',
                        'supported_ellipse_count':len(supported),'distinct_ellipse_count':len(distinct),
                        'colour_rejection_count':colour_rejections})
    return distinct if len(distinct)==1 else []


def chromatic_fallback(frame, masks, rejected, accepted,geometry_masks=None,face_settings=None):
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
            if color_id==3 and face_settings and face_settings.get('current_pale_component',False):
                face=pale_face_candidate(frame,masks,refined,value,face_settings)
                if face is None:face=current_pale_circle(frame,masks,refined,hsv,value)
                if face is not None:
                    if any(np.linalg.norm(np.array(face['silhouette_center_px'])-np.array(row['silhouette_center_px']))
                           <min(face['bbox_px'][2:])*.4 for row in candidates):continue
                    candidates.append(face);continue
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
                face=pale_face_candidate(frame,masks,refined,value,face_settings)
                if face is not None:
                    candidates.append(face);continue
            row={'bbox_px':[rx,ry,rw,rh],'area_px2':area,'label':label,
                'reason':'ISOLATED_CHROMATIC_COMPONENT','proposal_source':'chromatic_component',
                'center_reference':'chromatic_region_not_calibrated_grasp_point',
                'circularity':float(circularity),'solidity':float(solidity),
                'usable_fraction':usable,'color_fractions':scores,
                'dark_fraction':float((value[ry:ry+rh,rx:rx+rw][valid]<45).mean()),
                'silhouette_center_px':center}
            if label=='blue':
                conflict=conflicting_pale_evidence(frame,refined,value,face_settings)
                if conflict is not None:row.update(conflict)
            candidates.append(row)
    return candidates

def analyze(frame,color_config=None,prior_regions=None,draw_overlay=True):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h,s,v = cv2.split(hsv)
    calibrated_masks=None
    if color_config is not None:
        # 复用当前帧HSV与原标定工具的同一蒙版函数，避免再转换整图。
        from 颜色标定 import validate_color_config,_mask_from_hsv
        validate_color_config(color_config)
        calibrated_masks={int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in color_config['colors']}
        mask=np.zeros(frame.shape[:2],np.uint8)
        for m in calibrated_masks.values():mask=cv2.bitwise_or(mask,m)
    else:
        colorful = ((s>=90)&(v>=35)&((h<=12)|(h>=165)|((h>=18)&(h<=85))))
        strong_blue = (h>=100)&(h<=135)&(s>=140)&(v>=35)
        pale_blue = (h>=90)&(h<=115)&(s>=25)&(s<=180)&(v>=100)
        mask = ((colorful | strong_blue | pale_blue | (v<45)).astype(np.uint8))*255
    mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    records, drawing = [], frame.copy() if draw_overlay else None
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
        # 候选框外原本全部为0；显式零边界腐蚀保持原整图像素集合。
        region=np.zeros((ht,w),np.uint8)
        cv2.drawContours(region,[contour-np.array([[[x,y]]],np.int32)],-1,255,-1)
        region=cv2.erode(region,np.ones((7,7),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)>0
        local_value=v[y:y+ht,x:x+w]
        valid=region & (local_value<250)
        usable=float(valid.sum()/max(region.sum(),1))
        row['usable_fraction']=usable
        if usable<.35 or valid.sum()<500:
            row['reason']='INSUFFICIENT_NONCLIPPED_PIXELS';records.append(row);continue
        V=local_value[valid]
        if calibrated_masks is None:
            H,S=h[y:y+ht,x:x+w][valid],s[y:y+ht,x:x+w][valid]
            votes={
            'red': ((H<=12)|(H>=165))&(S>=90)&(V>=35),
            'yellow': (H>=18)&(H<35)&(S>=90)&(V>=35),
            'green': (H>=35)&(H<=85)&(S>=60)&(V>=35),
            'blue': (H>=100)&(H<=135)&(S>=140)&(V>=60),
            'lightblue': (H>=90)&(H<=115)&(S>=25)&(S<140)&(V>=100),
            }
            scores={k:float(val.mean()) for k,val in votes.items()}
        else:
            names={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}
            scores={names[k]:float((m[y:y+ht,x:x+w][valid]>0).mean()) for k,m in calibrated_masks.items()}
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
        if calibrated_masks is not None and (row['label']=='blue' or row['reason'] in ('AMBIGUOUS_COLOR','BLUE_FAMILY_FACE_UNVERIFIED')):
            if best in ('blue','lightblue','black'):
                face=pale_face_candidate(frame,calibrated_masks,contour,v,color_config.get('round_face_color_candidate'))
                if face is not None:
                    row['silhouette_color_fractions']=scores
                    row.update(face)
        if calibrated_masks is not None and row['label']=='blue':
            conflict=conflicting_pale_evidence(frame,contour,v,color_config.get('round_face_color_candidate'))
            if conflict is not None:
                row.update(conflict)
                rejected.append(contour)
        records.append(row)
        if row['label']!='UNKNOWN':accepted.append(contour)
        if draw_overlay:
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
        for row in chromatic_fallback(frame,calibrated_masks,rejected,accepted,geometry_masks,color_config.get('round_face_color_candidate')):
            records.append(row)
            x,y,w,ht=row['bbox_px']
            if draw_overlay:
                cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
        for row in hinted_color_circles(frame,calibrated_masks,rejected,records,prior_regions,v,circle_diagnostics,geometry_masks):
            records.append(row)
            x,y,w,ht=row['bbox_px']
            if draw_overlay:cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
        if color_config.get('black_edge_recovery_candidate',{}).get('enabled',False) and not any(r['label']=='black' for r in records):
            for row in black_edge_recovery(frame,calibrated_masks,rejected,accepted,prior_regions,v,circle_diagnostics,color_config['black_edge_recovery_candidate']):
                records.append(row)
                if draw_overlay:
                    x,y,w,ht=row['bbox_px'];cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                    cv2.putText(drawing,'black',(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
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
