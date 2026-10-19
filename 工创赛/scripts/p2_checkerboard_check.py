"""Single static tabletop board check; does NOT calibrate the camera."""
import argparse
import json
from pathlib import Path
import subprocess
import cv2
import numpy as np

PATTERN=(9,6)

def inspect_board(frame):
    # This standalone diagnostic uses CPU: the supplied Orange Pi log shows
    # an OpenCL warp_affine build failure (depth macro/name collision).
    # Do not change the system driver or other project programs.
    cv2.ocl.setUseOpenCL(False)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    if hasattr(cv2,'findChessboardCornersSB'):
        found,corners=cv2.findChessboardCornersSB(gray,PATTERN,cv2.CALIB_CB_NORMALIZE_IMAGE)
        method='findChessboardCornersSB'
    else:
        found,corners=cv2.findChessboardCorners(gray,PATTERN,
            cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
        method='findChessboardCorners'
        if found:
            corners=cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),
                (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_MAX_ITER,40,.001))
    valid=bool(found and corners is not None and len(corners)==54)
    drawing=frame.copy()
    report=dict(found_all_54_corners=valid,pattern_size=list(PATTERN),
        square_size_mm=15.0,printed_five_squares_mm_user_report=[75,75],
        image_size=[frame.shape[1],frame.shape[0]],method=method,
        note='One tabletop image check only; no camera matrix, distortion fit or metric accuracy acceptance.')
    if valid:
        cv2.drawChessboardCorners(drawing,PATTERN,corners,True)
        report['corners_px']=corners.reshape(-1,2).tolist()
        pts=corners.reshape(6,9,2)
        spacing=np.r_[np.linalg.norm(np.diff(pts,axis=0),axis=2).ravel(),
                      np.linalg.norm(np.diff(pts,axis=1),axis=2).ravel()]
        report['median_neighbor_spacing_px']=float(np.median(spacing))
    cv2.putText(drawing,'54 CORNERS FOUND' if valid else 'BOARD NOT FULLY DETECTED',
                (15,30),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,170,0) if valid else (0,0,255),2)
    return report,drawing

def save_png(path,frame):
    ok,data=cv2.imencode('.png',frame)
    if not ok:raise RuntimeError('Cannot encode output')
    data.tofile(path)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',default='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0')
    p.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/v0-main/checkerboard-check'))
    p.add_argument('--image',type=Path,help='Optional offline image; never opens camera in this mode')
    args=p.parse_args();controls=None
    if args.image:
        frame=cv2.imdecode(np.fromfile(args.image,dtype=np.uint8),1)
        if frame is None:raise RuntimeError('Image cannot be decoded')
    else:
        cap=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
        try:
            if not cap.isOpened():raise RuntimeError('Camera could not open; close other previews')
            cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,960)
            cap.set(cv2.CAP_PROP_FPS,30);cap.set(cv2.CAP_PROP_BUFFERSIZE,4)
            ok,frame=cap.read()
            if not ok:raise RuntimeError('Initial read failed')
            for value in ('exposure_auto=1','gain=30','white_balance_temperature_auto=0',
                          'white_balance_temperature=4600','exposure_absolute=50'):
                subprocess.run(['v4l2-ctl','-d',args.device,'--set-ctrl='+value],check=True,timeout=5)
            for _ in range(90):
                ok,frame=cap.read()
                if not ok:raise RuntimeError('Frame read failed')
            if frame.shape[:2]!=(960,1280):raise RuntimeError('Unexpected image size')
            controls=subprocess.run(['v4l2-ctl','-d',args.device,
                '--get-ctrl=exposure_auto,exposure_absolute,gain,white_balance_temperature_auto,white_balance_temperature'],
                check=True,capture_output=True,text=True,timeout=5).stdout
        finally:cap.release()
    report,drawing=inspect_board(frame)
    report.update(controls_readback=controls,opencv_version=cv2.__version__,
                  opencl_enabled=bool(cv2.ocl.useOpenCL()),processing_backend='CPU')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    save_png(args.output_dir/'board-raw.png',frame)
    save_png(args.output_dir/'board-corners.png',drawing)
    (args.output_dir/'board-check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('found_all_54_corners:',report['found_all_54_corners'])
    print('processing_backend: CPU')
    print('Saved:',args.output_dir,flush=True)
    if not report['found_all_54_corners']:
        print('Keep raw image for review; do not change board dimensions in code to force a match.')

if __name__=='__main__':main()
