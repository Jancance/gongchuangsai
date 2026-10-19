"""Exploratory cross-check, not a production detector or acceptance test.

Every rule sees every image without a supplied class. Expected labels are
used only to describe the evaluation rows. Never emits robot commands.
"""
from pathlib import Path
import hashlib
import json
import cv2
import numpy as np

OUT = Path(r'D:\Codex\工创赛\evidence\six_color_crosscheck_2026-09-24')
SRC = Path(r'C:\Users\Tenderness\Pictures')
SAMPLES = [
    ('red-e50-g30.png', 'red', 'night'),
    ('yellow-e50-g30.png', 'yellow', 'night'),
    ('blue-e50-g30.png', 'blue', 'night'),
    ('blue-day-e50-g30.png', 'blue', 'day'),
    ('lightblue-e50-g30.png', 'lightblue', 'day'),
    ('green-day-e50-g30.png', 'green', 'day'),
    ('black-day-e50-g30.png', 'black', 'day'),
    ('empty-day-e50-g30.png', 'empty', 'day'),
]
COLORS = ['red', 'yellow', 'blue', 'green', 'black', 'lightblue']
RANGES = {
    'red': [((0,90,35),(12,255,255)), ((165,90,35),(179,255,255))],
    'yellow': [((18,90,35),(38,255,255))],
    'blue': [((100,100,35),(135,255,255))],
    'green': [((35,60,35),(85,255,255))],
    'lightblue': [((90,25,100),(115,180,255))],
}

def candidates(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    output = {}
    for color in COLORS:
        if color == 'black':
            mask = cv2.inRange(gray, 0, 60)
        else:
            mask = np.zeros(gray.shape, np.uint8)
            for low, high in RANGES[color]:
                mask |= cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output[color] = [c for c in cs if cv2.contourArea(c) >= 500]
    return output

OUT.mkdir(parents=True, exist_ok=True)
(OUT/'inputs').mkdir(exist_ok=True)
rows, tiles = [], []
for filename, expected, lighting in SAMPLES:
    raw = (SRC/filename).read_bytes()
    (OUT/'inputs'/filename).write_bytes(raw)
    image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(filename)
    found = candidates(image)
    counts = {k:len(v) for k,v in found.items()}
    total = sum(counts.values())
    # Even one rule hit is only a candidate. Ambiguity must remain explicit.
    status = 'NO_CANDIDATE' if total == 0 else 'ONE_RULE_CANDIDATE' if total == 1 else 'UNKNOWN_MULTIPLE_RULE_HITS'
    rows.append(dict(file=filename, expected=expected, lighting=lighting,
                     sha256=hashlib.sha256(raw).hexdigest(), counts=counts,status=status))
    tile = cv2.resize(image,(480,360))
    footer = np.full((70,480,3),245,np.uint8)
    cv2.putText(footer,filename,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,0),1)
    text = ' '.join(f'{c}:{counts[c]}' for c in COLORS)
    cv2.putText(footer,text,(8,43),cv2.FONT_HERSHEY_SIMPLEX,.39,(0,0,0),1)
    cv2.putText(footer,status,(8,62),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
    tiles.append(np.vstack([tile,footer]))
    print(filename,counts,status)
grid = np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)])
ok,encoded=cv2.imencode('.png',grid)
if not ok:
    raise RuntimeError('Encode failed')
encoded.tofile(OUT/'crosscheck.png')
report = dict(scope='Development samples only; no accuracy claim. Independent rule hits are NOT object counts.',
              method='All rules applied to every full image; no center crop or expected-color input.',
              ranges_hsv=RANGES,black_gray_max=60,min_contour_area=500,
              close_kernel=5,lightblue_rule='New provisional rule; not validated.',samples=rows)
(OUT/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
