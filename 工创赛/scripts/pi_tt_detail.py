import os, sys
CODE = '/home/orangepi/p2-source-observer-20261106/vision_app/code'
os.chdir(CODE); sys.path.insert(0, CODE)
sys.argv = ['main.py']
import cv2, numpy as np
import main as app
from project_camera import DisabledSerialLink
app.TurntableSerialLink = DisabledSerialLink
args = app.parse_args()
app.initialize(args)
frame = cv2.imdecode(np.fromfile('/tmp/frame_now.png', dtype=np.uint8), cv2.IMREAD_COLOR)
for i in range(3):
    d = app.turntable_detector.detect(frame)
print('detected=%s stable=%s conf=%.4f' % (d.detected, d.stable, d.confidence))
for k in sorted(vars(d)):
    print('  %-22s %s' % (k, getattr(d, k)))
