import json, sys
import cv2, numpy as np

IMG = sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame_now.png'
CFG = sys.argv[2] if len(sys.argv) > 2 else '/home/orangepi/p2-source-observer-20261106/vision_app/code/config/material_color_thresholds.json'
doc = json.load(open(CFG, encoding='utf-8'))
frame = cv2.imdecode(np.fromfile(IMG, dtype=np.uint8), cv2.IMREAD_COLOR)
print('image', frame.shape)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
kernel_o = np.ones((5, 5), np.uint8)
kernel_c = np.ones((11, 11), np.uint8)
for c in doc['colors']:
    mask = None
    for r in c['ranges']:
        m = cv2.inRange(hsv, np.array(r['lower'], np.uint8), np.array(r['upper'], np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_o)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_c)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for ct in cnts:
        a = cv2.contourArea(ct)
        if a < 1200:
            continue
        x, y, w, h = cv2.boundingRect(ct)
        M = cv2.moments(ct)
        cx = int(M['m10'] / M['m00']) if M['m00'] else x + w // 2
        cy = int(M['m01'] / M['m00']) if M['m00'] else y + h // 2
        blobs.append((a, cx, cy, x, y, w, h))
    blobs.sort(reverse=True)
    print('--- color %d %-8s blobs=%d total_px=%d' % (c['color_id'], c['name'], len(blobs), int(np.count_nonzero(mask))))
    for a, cx, cy, x, y, w, h in blobs[:4]:
        print('    area=%7.0f center=(%4d,%4d) bbox=(%4d,%4d,%4d,%4d)' % (a, cx, cy, x, y, w, h))
