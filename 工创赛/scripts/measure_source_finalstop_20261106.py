"""Extract image evidence for MATLAB; no controller or camera connections."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'evidence/integration/2026-11-06/source-vision-new-pose'
SOURCE = BASE / 'source-paper-finalstop-20261106'
OUT = BASE / 'finalstop-geometry'


def line(gray, a, b):
    a, b = np.array(a, float), np.array(b, float)
    tangent = (b-a) / np.linalg.norm(b-a)
    normal = np.array([-tangent[1], tangent[0]])
    ts = np.arange(.12, .88, 1/np.linalg.norm(b-a))
    points = []
    for t in ts:
        q = a + t*(b-a) + np.arange(-7, 7.01, .5)[:, None]*normal
        pix = cv2.remap(gray, q[:, 0].astype('float32')[None], q[:, 1].astype('float32')[None], cv2.INTER_LINEAR)[0]
        dark = np.maximum(0, 95-pix.astype(float))
        if dark.sum() > 30:
            points.append(np.average(q, weights=dark, axis=0))
    points = np.array(points)
    keep = np.ones(len(points), bool)
    for _ in range(5):
        p = points[keep]
        center = np.mean(p, axis=0)
        _, _, v = np.linalg.svd(p-center)
        n = v[-1]
        residual = (points-center) @ n
        keep = abs(residual) < max(1.2, 3*1.4826*np.median(abs(residual[keep])))
    return np.r_[n, -center @ n], int(keep.sum())


def intersection(a, b):
    p = np.cross(a, b)
    return p[:2]/p[2]


def main():
    OUT.mkdir(exist_ok=True)
    image = cv2.imdecode(np.frombuffer((SOURCE/'last-raw.png').read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    initial = [[480,230],[887,201],[913,617],[501,640]]
    borders = [line(gray, initial[i], initial[(i+1)%4]) for i in range(4)]
    corners = [intersection(borders[(i-1)%4][0], borders[i][0]) for i in range(4)]
    diagonal_ac, _ = line(gray, initial[0], initial[2])
    diagonal_db, _ = line(gray, initial[3], initial[1])
    m = intersection(diagonal_ac, diagonal_db)
    circles = []
    for name, cx, cy, radius in [('far_left',647,98,103),('far_right',1356,142,103),('near',964,730,104)]:
        points = []
        for angle in np.deg2rad(np.arange(360)):
            radii = np.arange(radius-9, radius+9.01, .25)
            x, y = cx+radii*np.cos(angle), cy+radii*np.sin(angle)
            if min(x)<2 or max(x)>1917 or min(y)<2 or max(y)>1077:
                continue
            values = cv2.remap(gray, x.astype('float32')[None], y.astype('float32')[None], cv2.INTER_LINEAR)[0]
            idx = int(np.argmin(values))
            if values[idx] < 165 and 2<idx<len(radii)-3:
                points.append([float(x[idx]),float(y[idx])])
        circles.append({'id':name,'boundary_image_px':points})
    session = json.loads((SOURCE/'session.json').read_text(encoding='utf-8'))
    result = {
        'paper_image_sha256':hashlib.sha256((SOURCE/'last-raw.png').read_bytes()).hexdigest(),
        'image_size':[image.shape[1],image.shape[0]],'paper_width_mm':100,'paper_height_mm':100,
        'size_basis':'same previously ruler-measured paper; no new dimensions assumed',
        'corners_ABCD_px':np.array(corners).tolist(),
        'border_line_support_counts':[b[1] for b in borders],
        'corner_method':'robust dark-border centerline intersections, excluding endpoints and labels',
        'heldout_MEF_px':[m.tolist(),[587,324],[785,311]],
        'heldout_basis':'M independent diagonal intersection; E/F visible quarter-diagonal ticks, manually selected within3px',
        'heldout_point_selection_uncertainty_px':3,
        'circles':circles,'rim_boundary_image_px':session['last']['boundary_points_px'],
        'serial':session['serial'],'motion_allowed':False,
        'single_frame_not_temporal_stability':True,
    }
    (OUT/'measurement-input.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    overlay = image.copy()
    for name, p in zip('ABCD', corners):
        q=tuple(np.rint(p).astype(int));cv2.circle(overlay,q,5,(0,0,255),2)
        cv2.putText(overlay,name,q,cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),1)
    for p in result['heldout_MEF_px']:
        cv2.circle(overlay,tuple(np.rint(p).astype(int)),5,(255,0,255),1)
    for circle in circles:
        for p in circle['boundary_image_px'][::3]:
            cv2.circle(overlay,tuple(np.rint(p).astype(int)),1,(0,0,255),1)
    (OUT/'measurement-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
    (OUT/'heldout-EF-detail.png').write_bytes(cv2.imencode('.png',image[270:355,540:830])[1].tobytes())
    print(json.dumps({k:result[k] for k in ['paper_image_sha256','corners_ABCD_px','heldout_MEF_px','border_line_support_counts']}))
    print('circle sample counts', [len(c['boundary_image_px']) for c in circles])


if __name__ == '__main__':
    main()
