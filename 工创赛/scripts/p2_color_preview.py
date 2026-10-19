"""Visual-only camera preview for the provisional tabletop color probe.

No serial port, robot commands, calibration, tracking or action verification.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
import cv2
from p2_color_probe import analyze

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',default='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0')
    p.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/v0-main/live-color'))
    args=p.parse_args()
    cap=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
    try:
        if not cap.isOpened(): raise RuntimeError('Camera could not open')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,960)
        cap.set(cv2.CAP_PROP_FPS,30)
        ok,frame=cap.read()
        if not ok: raise RuntimeError('First frame read failed')
        for value in ('exposure_auto=1','gain=30','white_balance_temperature_auto=0',
                      'white_balance_temperature=4600','exposure_absolute=50'):
            subprocess.run(['v4l2-ctl','-d',args.device,'--set-ctrl='+value],check=True)
        for _ in range(90):
            ok,frame=cap.read()
            if not ok: raise RuntimeError('Warm-up read failed')
        subprocess.run(['v4l2-ctl','-d',args.device,
                        '--get-ctrl=exposure_auto,exposure_absolute,gain,white_balance_temperature_auto,white_balance_temperature'],check=True)
        print('VISUAL TEST ONLY. S: save raw/overlay/JSON; Q: quit.',flush=True)
        while True:
            start=time.monotonic()
            ok,frame=cap.read()
            if not ok: raise RuntimeError('Frame read failed; preview stopped')
            result,drawing=analyze(frame)
            counts=Counter(r['label'] for r in result['regions'])
            text=' '.join(f'{k}:{v}' for k,v in sorted(counts.items())) or 'NO CANDIDATE'
            cv2.rectangle(drawing,(0,0),(drawing.shape[1]-1,85),(0,0,0),-1)
            cv2.putText(drawing,'VISUAL TEST ONLY - NO ROBOT CONTROL',(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
            cv2.putText(drawing,text,(12,58),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
            result['read_and_processing_ms']=round((time.monotonic()-start)*1000,2)
            cv2.imshow('P2 color preview - S save / Q quit',cv2.resize(drawing,(960,720)))
            key=cv2.waitKey(1)&0xff
            if key in (ord('q'),ord('Q'),27): break
            if key in (ord('s'),ord('S')):
                args.output_dir.mkdir(parents=True,exist_ok=True)
                stem=datetime.now().strftime('color-%Y%m%d-%H%M%S-%f')
                for suffix,im in (('raw',frame),('overlay',drawing)):
                    path=args.output_dir/f'{stem}-{suffix}.png'
                    if not cv2.imwrite(str(path),im): raise RuntimeError(f'Cannot save {path}')
                (args.output_dir/f'{stem}.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
                print('Saved:',args.output_dir/stem,flush=True)
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__=='__main__':
    main()
