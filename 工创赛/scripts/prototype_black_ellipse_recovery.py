"""Current-edge ellipse validation bounded by a fresh black search box."""
import cv2
import numpy as np

NAMES={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'lightblue'}

def recover(frame,masks,rejected,accepted,hints,diagnostics):
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY);value=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)[:,:,2]
    height,width=gray.shape;supported=[]
    for hint in hints:
        if hint['color']!='black':continue
        x,y,w,h=hint['bbox_px'];margin=max(20,int(min(w,h)*.4))
        left,top=max(0,x-margin),max(0,y-margin);right,bottom=min(width,x+w+margin),min(height,y+h+margin)
        roi=cv2.GaussianBlur(gray[top:bottom,left:right],(5,5),1)
        gx=cv2.Sobel(roi,cv2.CV_32F,1,0,ksize=3);gy=cv2.Sobel(roi,cv2.CV_32F,0,1,ksize=3)
        angles=np.arange(72)*2*np.pi/72;cos=np.cos(angles);sin=np.sin(angles)
        for dx in (-w*.08,0.,w*.08):
         for dy in (-h*.08,0.,h*.08):
          for scale in (.9,1.,1.1):
            cx=x+w*.5+dx;cy=y+h*.5+dy;ax=w*.5*scale;ay=h*.5*scale
            if cx-ax<=1 or cy-ay<=1 or cx+ax>=width-1 or cy+ay>=height-1:continue
            if any(cv2.pointPolygonTest(p,(cx,cy),False)>=0 for p in accepted):continue
            nx,ny=cos/ax,sin/ay;length=np.hypot(nx,ny);nx/=length;ny/=length
            offsets=np.linspace(-max(3.,min(ax,ay)*.06),max(3.,min(ax,ay)*.06),9);scores=[]
            for delta in offsets:
                xx=np.rint(cx+ax*cos+delta*nx).astype(int)-left;yy=np.rint(cy+ay*sin+delta*ny).astype(int)-top
                good=(xx>=0)&(xx<gx.shape[1])&(yy>=0)&(yy<gx.shape[0]);sx=np.zeros(72);sy=np.zeros(72)
                sx[good]=gx[yy[good],xx[good]];sy[good]=gy[yy[good],xx[good]]
                magnitude=np.hypot(sx,sy);alignment=np.abs(sx*nx+sy*ny)/np.maximum(magnitude,1.)
                scores.append(np.where(good&(magnitude>=40)&(alignment>=.7),magnitude*alignment,0.))
            scores=np.array(scores);edge=scores.max(axis=0)>0;fraction=float(edge.mean())
            residual=float(np.sqrt(np.mean(offsets[scores[:,edge].argmax(axis=0)]**2))) if edge.any() else 999.
            if fraction<.75 or residual>max(2.,min(ax,ay)*.04):continue
            rx,ry=int(np.floor(cx-ax)),int(np.floor(cy-ay));rw,rh=int(np.ceil(cx+ax))-rx+1,int(np.ceil(cy+ay))-ry+1
            yy,xx=np.ogrid[ry:ry+rh,rx:rx+rw];disk=((xx-cx)/(ax*.87))**2+((yy-cy)/(ay*.87))**2<=1.
            pm=np.zeros((rh,rw),np.uint8)
            for p in rejected:cv2.drawContours(pm,[p-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
            parent_fraction=float(((pm>0)&disk).sum()/max(disk.sum(),1))
            if parent_fraction<.5:continue
            valid=disk&(value[ry:ry+rh,rx:rx+rw]<250);usable=float(valid.sum()/max(disk.sum(),1))
            if usable<.35 or valid.sum()<500:continue
            fractions={NAMES[k]:float((m[ry:ry+rh,rx:rx+rw][valid]>0).mean()) for k,m in masks.items()}
            if fractions['black']<.65 or max(v for k,v in fractions.items() if k!='black')>.10:continue
            supported.append({'label':'black','proposal_source':'black_edge_ellipse','reason':'CURRENT_DARK_ELLIPSE_SUPPORT',
                'bbox_px':[rx,ry,rw,rh],'area_px2':float(np.pi*ax*ay),'silhouette_center_px':[cx,cy],
                'ellipse_axes_px':[ax,ay],'edge_fraction':fraction,'fit_residual_px':residual,
                'color_fractions':fractions,'usable_fraction':usable,'parent_fraction':parent_fraction,
                'center_reference':'current_dark_silhouette_not_calibrated_grasp_point'})
    supported.sort(key=lambda r:(-r['edge_fraction'],r['fit_residual_px']))
    diagnostics.append({'color':'black','supported_ellipse_count':len(supported),'best_edge_fraction':supported[0]['edge_fraction'] if supported else 0})
    return supported[:1]
