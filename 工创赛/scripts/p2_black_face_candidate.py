"""Source fragment for isolated black1 detector; no capture or motion access."""
import cv2
import numpy as np


def black_face_supported_hull(frame, masks, rejected, accepted, value, settings, diagnostics):
    """实心暗圆面只证明当前黑色候选；凸包中心仍不是顶面抓取点。"""
    if not settings or not settings.get('enabled', False):
        return []
    height, width = frame.shape[:2]
    parents = []
    for parent in sorted(rejected, key=cv2.contourArea, reverse=True):
        x, y, w, h = cv2.boundingRect(parent)
        if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
            continue
        if min(w, h) < 80 or max(w, h) > 500 or max(w/h, h/w) > 2:
            continue
        if cv2.contourArea(parent) < 5000:
            continue
        parents.append(parent)
        if len(parents) >= 2:
            break
    supported = []
    angles = np.arange(72)*2*np.pi/72
    co, si = np.cos(angles), np.sin(angles)
    for parent in parents:
        x, y, w, h = cv2.boundingRect(parent)
        gray = cv2.cvtColor(frame[y:y+h, x:x+w], cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 1)
        small = cv2.resize(gray, None, fx=.5, fy=.5)
        circles = cv2.HoughCircles(small, cv2.HOUGH_GRADIENT, 1, max(12, min(w,h)*.12),
            param1=80, param2=20, minRadius=max(10, int(min(w,h)*.07)),
            maxRadius=max(11, int(min(w,h)*.35)))
        if circles is None:
            continue
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        for cx0, cy0, r0 in circles[0][:12]:
            cx, cy, radius = float(cx0)*2+x, float(cy0)*2+y, float(r0)*2
            if cx-radius <= 1 or cy-radius <= 1 or cx+radius >= width-1 or cy+radius >= height-1:
                continue
            if any(cv2.pointPolygonTest(c, (cx,cy), False) >= 0 for c in accepted):
                continue
            rx, ry = int(np.floor(cx-radius)), int(np.floor(cy-radius))
            rw, rh = int(np.ceil(cx+radius))-rx+1, int(np.ceil(cy+radius))-ry+1
            yy, xx = np.ogrid[ry:ry+rh, rx:rx+rw]
            disk = (xx-cx)**2+(yy-cy)**2 <= (radius*.87)**2
            local_value = value[ry:ry+rh, rx:rx+rw]
            valid = disk & (local_value < 250)
            usable = float(valid.sum()/max(disk.sum(), 1))
            if usable < .35 or valid.sum() < 500:
                continue
            black = masks[5][ry:ry+rh, rx:rx+rw] > 0
            black_fraction = float(black[valid].mean())
            if black_fraction < .65:
                continue
            conflicts = {str(k):float(((m[ry:ry+rh,rx:rx+rw]>0)&~black)[valid].mean())
                         for k,m in masks.items() if k != 5}
            if max(conflicts.values(), default=0) > .10:
                continue
            parent_mask = np.zeros((rh,rw), np.uint8)
            cv2.drawContours(parent_mask,[parent-np.array([[[rx,ry]]],np.int32)],-1,255,-1)
            overlap = float(((parent_mask>0)&disk).sum()/max(disk.sum(),1))
            if overlap < .5:
                continue
            offsets = np.linspace(-max(3., radius*.06), max(3., radius*.06), 9)
            xx = np.rint(cx+(radius+offsets[:,None])*co).astype(int)-x
            yy = np.rint(cy+(radius+offsets[:,None])*si).astype(int)-y
            good = (xx>=0)&(yy>=0)&(xx<w)&(yy<h)
            sx, sy = np.zeros(xx.shape), np.zeros(yy.shape)
            sx[good], sy[good] = gx[yy[good],xx[good]], gy[yy[good],xx[good]]
            magnitude = np.hypot(sx,sy)
            alignment = np.abs(sx*co+sy*si)/np.maximum(magnitude,1.)
            scores = np.where(good&(magnitude>=40)&(alignment>=.7), magnitude*alignment, 0.)
            edge = scores.max(axis=0)>0
            fraction = float(edge.mean())
            residual = float(np.sqrt(np.mean(offsets[scores[:,edge].argmax(axis=0)]**2))) if edge.any() else 999.
            if fraction < .75 or residual > max(2.,radius*.04):
                continue
            hull = cv2.convexHull(parent)
            moments = cv2.moments(hull)
            center = [moments['m10']/moments['m00'], moments['m01']/moments['m00']]
            supported.append({'label':'black', 'reason':'CURRENT_SOLID_DARK_FACE_SUPPORT',
                'proposal_source':'black_face_supported_hull','bbox_px':[x,y,w,h],
                'area_px2':float(cv2.contourArea(hull)),'silhouette_center_px':center,
                'center_reference':'current_parent_hull_not_calibrated_grasp_point',
                'face_evidence':{'center_px':[cx,cy],'radius_px':radius,
                    'black_fraction':black_fraction,'usable_fraction':usable,
                    'parent_fraction':overlap,'edge_fraction':fraction,
                    'edge_fit_residual_px':residual,'chromatic_nonblack_fractions':conflicts}})
    distinct = []
    for row in supported:
        e = row['face_evidence']
        if any(np.linalg.norm(np.array(e['center_px'])-r['face_evidence']['center_px']) <
               .45*min(e['radius_px'],r['face_evidence']['radius_px']) for r in distinct):
            continue
        distinct.append(row)
    if distinct:
        diagnostics.append({'color':'black','reason':'SOLID_DARK_FACE_UNIQUE' if len(distinct)==1
                            else 'SOLID_DARK_FACE_NOT_UNIQUE','supported_face_count':len(distinct)})
    return distinct if len(distinct)==1 else []
