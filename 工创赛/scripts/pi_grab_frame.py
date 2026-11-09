import cv2, time
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
print('opened', cap.isOpened())
ok = False
for i in range(30):
    ok, frame = cap.read()
    if ok and frame is not None and frame.shape[:2] == (1080, 1920):
        break
    time.sleep(0.05)
print('read', ok, None if frame is None else frame.shape)
if ok:
    cv2.imwrite('/tmp/frame_now.png', frame)
    print('saved /tmp/frame_now.png')
cap.release()
