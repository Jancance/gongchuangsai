"""Measure actual three-ring centers with the released detector; offline only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'output/stage3_ring_pi_20261116'))
from stage3 import Pipeline, read_image, write_image, cv2, RingStationDetector

def measure(image, output):
    frame=read_image(image)
    if frame is None:raise ValueError('Image cannot be read')
    config=ROOT/'evidence/integration/2026-11-18/stage3-resume/image-analysis/before-search-config.json'
    pipe=Pipeline(config,tuple(frame.shape[1::-1]))
    result=RingStationDetector(pipe.ring).detect(frame)
    if not result.detected or len(result.centers_px)!=3:
        raise ValueError('Original detector did not find three actual rings')
    centers=result.centers_px
    yaw=math.degrees(math.atan2(centers[2][1]-centers[0][1],centers[2][0]-centers[0][0]))
    report={'image_path':str(image.resolve()),'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
            'centers_px':centers,'center_px':centers[1],'row_yaw_deg':yaw,
            'image_size':list(frame.shape[1::-1]),'single_frame_stable':False,
            'calibration_verified':False,'algorithm':'Released RingStationDetector, actual centers; no estimated side centers',
            'detector_sha256':hashlib.sha256((ROOT/'output/stage3_ring_pi_20261116/vendor/ring_station_detector.py').read_bytes()).hexdigest()}
    with output.open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2)
    for i,c in enumerate(centers):
        point=tuple(round(v) for v in c)
        cv2.drawMarker(frame,point,(0,255,0),cv2.MARKER_CROSS,24,2)
        cv2.putText(frame,str(i+1),(point[0]+16,point[1]-15),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,180,0),2)
    cv2.line(frame,tuple(round(v) for v in centers[0]),tuple(round(v) for v in centers[2]),(0,255,0),2)
    cv2.drawMarker(frame,(960,540),(0,0,255),cv2.MARKER_CROSS,32,2)
    cv2.putText(frame,f'row yaw {yaw:.4f} deg; target 960,540',(30,45),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
    write_image(output.with_suffix('.png'),frame)
    print(json.dumps(report))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();measure(args.image,args.output)
