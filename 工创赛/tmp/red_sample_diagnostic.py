"""Offline, single-image exploratory segmentation; never emits motion commands."""
from pathlib import Path
import argparse
import json
import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--color', choices=['red', 'yellow', 'blue', 'green', 'black'], default='red')
color = parser.parse_args().color
filename = f'{color}-day-e50-g30.png' if color in ('green', 'black') else f'{color}-e50-g30.png'
source = Path(r'C:\Users\Tenderness\Pictures') / filename
out = Path(r'D:\Codex\工创赛\evidence') / f'{color}_sample_diagnostic'
out.mkdir(parents=True, exist_ok=True)
frame = cv2.imdecode(np.fromfile(source, dtype=np.uint8), cv2.IMREAD_COLOR)
if frame is None:
    raise RuntimeError('Image decode failed')
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
if color == 'red':
    mask = cv2.inRange(hsv, (0, 90, 35), (12, 255, 255))
    mask |= cv2.inRange(hsv, (165, 90, 35), (179, 255, 255))
elif color == 'yellow':
    mask = cv2.inRange(hsv, (18, 90, 35), (38, 255, 255))
elif color == 'blue':
    mask = cv2.inRange(hsv, (100, 100, 35), (135, 255, 255))
elif color == 'green':
    mask = cv2.inRange(hsv, (35, 60, 35), (85, 255, 255))
else:
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 0, 60)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
candidates = [c for c in contours if cv2.contourArea(c) >= 500]
result = {'status': 'UNKNOWN', 'color': color, 'candidate_count': len(candidates),
          'scope': 'one supplied image only; thresholds unvalidated',
          'coordinate_frame': 'image: origin upper-left, x right, y down, pixels',
          'warning': 'Color-region centroid is neither calibrated top center nor grasp target.'}
if color == 'black':
    result['warning'] += ' Dark mask alone cannot classify black versus dark blue.'
overlay = frame.copy()
if len(candidates) == 1:
    c = candidates[0]
    x, y, w, h = cv2.boundingRect(c)
    if x > 0 and y > 0 and x+w < frame.shape[1] and y+h < frame.shape[0]:
        m = cv2.moments(c)
        center = (m['m10']/m['m00'], m['m01']/m['m00'])
        result.update(status='SINGLE_SAMPLE_CANDIDATE', center_px=center,
                      outer_contour_area_px2=cv2.contourArea(c), bbox_px=[x,y,w,h])
        cv2.drawContours(overlay, [c], -1, (0,255,0), 2)
        cv2.drawMarker(overlay, tuple(round(v) for v in center), (0,255,255),
                       cv2.MARKER_CROSS, 22, 2)
        label = 'DARK' if color == 'black' else color.upper()
        cv2.putText(overlay, label + ' REGION (%.1f, %.1f) px' % center,
                    (max(5,x-90),max(30,y-15)),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2)
for name, image in [(f'{color}-overlay.png',overlay),(f'{color}-mask.png',mask)]:
    ok, encoded = cv2.imencode('.png',image)
    if not ok:
        raise RuntimeError('Encode failed')
    encoded.tofile(out/name)
(out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
