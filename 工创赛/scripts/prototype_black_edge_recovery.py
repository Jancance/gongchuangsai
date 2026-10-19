"""Bounded current-edge black-circle development prototype, no camera or serial."""
import cv2
import numpy as np

NAMES={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}

def recover(frame,masks,rejected,accepted,hints,diagnostics):
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV);value=hsv[:,:,2]
    height,width=gray.shape
    searches=[dict(h) for h in hints if h['color']=='black']
    if not searches:
        for parent in rejected:
            x,y,w,h=cv2.boundingRect(parent)
            if x<=1 or y<=1 or x+w>=width-1 or y+h>=height-1:continue
            if min(w,h)<80 or max(w,h)>500 or max(w/h,h/w)>1.8:continue
            searches.append({'color':'black','bbox_px':[x,y,w,h],'proposal_source':'current_compact_dark_region'})
            if len(searches)>=2:break
    supported=[]
    for hint in searches:
        x,y,w,h=hint['bbox_px'];margin=max(20,int(min(w,h)*.4))
        left,top=max(0,x-margin),max(0,y-margin)
        right,bottom=min(width,x+w+margin),min(height,y+h+margin)
        roi=gray[top:bottom,left:right]
        small=cv2.resize(roi,None,fx=.5,fy=.5,interpolation=cv2.INTER_AREA)
        small=cv2.GaussianBlur(small,(5,5),1)
        circles=cv2.HoughCircles(small,cv2.HOUGH_GRADIENT,1,max(8,min(w,h)*.05),
            param1=80,param2=20,minRadius=max(8,int(min(w,h)*.15)),maxRadius=max(9,int(min(w,h)*.30)))
        if circles is None:
            diagnostics.append({'color':'black','reason':'NO_CURRENT_EDGE_CIRCLE'});continue
        blurred=cv2.GaussianBlur(roi,(5,5),1)
        gx=cv2.Sobel(blurred,cv2.CV_32F,1,0,ksize=3)
        gy=cv2.Sobel(blurred,cv2.CV_32F,0,1,ksize=3)
        for cx,cy,radius in circles[0]:
            cx=float(cx)*2+left;cy=float(cy)*2+top;radius=float(radius)*2
            detail={'color':'black','center_px':[cx,cy],'radius_px':radius}
            if cx-radius<=1 or cy-radius<=1 or cx+radius>=width-1 or cy+radius>=height-1:continue
            if any(cv2.pointPolygonTest(parent,(cx,cy),False)>=0 for parent in accepted):continue
            rx,ry=int(np.floor(cx-radius)),int(np.floor(cy-radius))
            rw,rh=int(np.ceil(cx+radius))-rx+1,int(np.ceil(cy+radius))-ry+1
            yy,xx=np.ogrid[ry:ry+rh,rx:rx+rw];disk=(xx-cx)**2+(yy-cy)**2<=(radius*.87)**2
            parent_region=np.zeros((rh,rw),np.uint8)
            for parent in rejected:cv2.drawContours(parent_region,[parent-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
            parent_fraction=float(((parent_region>0)&disk).sum()/max(disk.sum(),1))
            if parent_fraction<.5:continue
            angles=np.arange(72)*2*np.pi/72;cos=np.cos(angles);sin=np.sin(angles)
            offsets=np.linspace(-max(3.,radius*.06),max(3.,radius*.06),9)
            scores=[]
            for delta in offsets:
                xx=np.rint(cx+(radius+delta)*cos).astype(int)-left
                yy=np.rint(cy+(radius+delta)*sin).astype(int)-top
                inside=(xx>=0)&(xx<gx.shape[1])&(yy>=0)&(yy<gx.shape[0])
                sx=np.zeros(72);sy=np.zeros(72);sx[inside]=gx[yy[inside],xx[inside]];sy[inside]=gy[yy[inside],xx[inside]]
                magnitude=np.hypot(sx,sy);alignment=np.abs(sx*cos+sy*sin)/np.maximum(magnitude,1.)
                scores.append(np.where(inside&(magnitude>=40)&(alignment>=.7),magnitude*alignment,0.))
            scores=np.array(scores);peaks=scores.max(axis=0);edge_angles=peaks>0
            edge_fraction=float(edge_angles.mean())
            residual=float(np.sqrt(np.mean(offsets[scores[:,edge_angles].argmax(axis=0)]**2))) if edge_angles.any() else float('inf')
            if edge_fraction<.75 or residual>max(2.,radius*.04):
                diagnostics.append(dict(detail,reason='INSUFFICIENT_RADIAL_EDGE',edge_fraction=edge_fraction,fit_residual_px=residual));continue
            valid=disk&(value[ry:ry+rh,rx:rx+rw]<250)
            usable=float(valid.sum()/max(disk.sum(),1))
            if usable<.35 or valid.sum()<500:continue
            fractions={NAMES[k]:float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean()) for k,m in masks.items()}
            chromatic=max(fractions[k] for k in ('red','yellow','blue','green','lightblue'))
            if fractions['black']<.65 or chromatic>.10:
                diagnostics.append(dict(detail,reason='BLACK_COLOR_UNCERTAIN',color_fractions=fractions));continue
            supported.append({'label':'black','reason':'CURRENT_BLACK_EDGE_AND_DARK_SUPPORT',
                'proposal_source':'black_edge_circle','bbox_px':[rx,ry,rw,rh],
                'area_px2':float(np.pi*radius**2),'silhouette_center_px':[cx,cy],
                'center_reference':'current_edge_circle_not_calibrated_grasp_point',
                'radius_px':radius,'edge_fraction':edge_fraction,'fit_residual_px':residual,
                'usable_fraction':usable,'parent_fraction':parent_fraction,'color_fractions':fractions,
                'search_reference':hint.get('proposal_source','recent_black_bbox')})
    distinct=[]
    for row in sorted(supported,key=lambda r:r['edge_fraction'],reverse=True):
        if any(np.linalg.norm(np.array(row['silhouette_center_px'])-old['silhouette_center_px'])<.45*min(row['radius_px'],old['radius_px']) and max(row['radius_px'],old['radius_px'])/min(row['radius_px'],old['radius_px'])<1.3 for old in distinct):continue
        distinct.append(row)
    return distinct if len(distinct)==1 else []
