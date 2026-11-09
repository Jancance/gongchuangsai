"""运行原项目R模式，记录新鲜三圆环像素坐标；不改算法/配置，不输出毫米许可。"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED

import cv2
import numpy as np


class RingPixelRecorder:
    def __init__(self, folder, seconds=10., minimum_confidence=.9):
        self.folder=Path(folder)
        self.folder.mkdir(parents=True,exist_ok=False)
        self.log=(self.folder/'observations.jsonl').open('w',encoding='utf8')
        self.seconds=seconds;self.started=None;self.frames=0;self.last_stamp=None
        self.counts=Counter();self.points=[];self.last_row=None;self.saved=set()
        self.last_usable=None;self.closed=False
        self.minimum_confidence=minimum_confidence
        self.ready_points=[];self.last_ready=None;self.seen_ready=False

    def write(self,path,value):
        path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')

    def observe(self,raw,overlay,result,read_time,finished):
        if self.started is None:self.started=finished
        self.frames+=1
        centers=getattr(result,'centers_px',())
        valid_centers=(bool(getattr(result,'detected',False)) and len(centers)==3
                       and all(len(p)==2 and all(math.isfinite(float(v)) for v in p) for p in centers))
        if valid_centers:
            valid_centers=all(0<=float(x)<raw.shape[1] and 0<=float(y)<raw.shape[0] for x,y in centers)
        valid_stamp=isinstance(read_time,(int,float)) and math.isfinite(read_time)
        age=(finished-read_time)*1000 if valid_stamp else None
        increasing=valid_stamp and (self.last_stamp is None or read_time>self.last_stamp)
        if increasing:self.last_stamp=read_time
        fresh=bool(increasing and 0<=age<=150)
        reason=('NONINCREASING_OR_MISSING_READ_TIMESTAMP' if not increasing else
                'STALE_RESULT' if not fresh else 'THREE_CENTERS' if valid_centers else 'NO_THREE_CENTERS')
        usable=bool(fresh and valid_centers)
        # 原usable仅表示新鲜候选，保留字段兼容旧证据；稳定坐标必须通过原检测器质量/时序门。
        detector_stable=bool(getattr(result,'stable',False))
        confidence=float(getattr(result,'confidence',0.))
        pixel_ready=bool(usable and detector_stable and math.isfinite(confidence)
                         and confidence>=self.minimum_confidence)
        row={'frame_index':self.frames,'state':reason,'detected':bool(getattr(result,'detected',False)),
             'centers_px':[[float(x),float(y)] for x,y in centers] if valid_centers else [],
             'usable_centers_px':[[float(x),float(y)] for x,y in centers] if usable else [],
             'usable_for_pixel_statistics':usable,'timestamp_source':'host_read_completion',
             'detector_stable':detector_stable,'pixel_ready':pixel_ready,
             'stable_centers_px':[[float(x),float(y)] for x,y in centers] if pixel_ready else [],
             'minimum_confidence':self.minimum_confidence,
             'host_read_time':float(read_time) if valid_stamp else None,'result_host_age_ms':age,
             'detector_processing_ms':float(getattr(result,'processing_ms',0.)),
             'confidence':float(getattr(result,'confidence',0.)),
             'detail':str(getattr(result,'detail','')),
             'units':'pixels','metric_valid':False,'motion_allowed':False,'serial':'DISABLED'}
        self.counts[reason]+=1
        if usable:self.points.append(row['usable_centers_px'])
        if pixel_ready:self.ready_points.append(row['stable_centers_px'])
        self.log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        if self.frames%24==0:self.log.flush()
        names=[]
        if self.frames==1:names.append('first-frame')
        if usable:names.append('first-three-centers')
        if self.last_usable is True and not usable:names.append('first-unusable')
        if self.last_usable is False and usable:names.append('first-reappearance')
        if self.seen_ready and self.last_ready is False and pixel_ready:names.append('first-stable-reappearance')
        # 一种事件只存第一次，原图/叠加图总量有界；编码时间不计作检测耗时。
        for name in names:
            if name in self.saved:continue
            self.saved.add(name)
            for suffix,image in (('raw',raw),('overlay',overlay)):
                ok,data=cv2.imencode('.png',image)
                if not ok:raise RuntimeError('Cannot encode diagnostic image')
                data.tofile(self.folder/(name+'-'+suffix+'.png'))
            self.write(self.folder/(name+'.json'),row)
        self.last_usable=usable;self.last_row=row
        self.last_ready=pixel_ready;self.seen_ready=self.seen_ready or pixel_ready
        return row

    def done(self,now):
        return self.started is not None and now-self.started>=self.seconds

    def close(self,status):
        if self.closed:return
        self.closed=True;self.log.close()
        summary={'frames':self.frames,'states':dict(self.counts),'usable_frames':len(self.points),
                 'requested_seconds':self.seconds,'exit_code':status,'units':'pixels','metric_valid':False,
                 'motion_allowed':False,'serial':'DISABLED',
                 'scope':'Original R algorithm, host-read completion age <=150ms. Saved frames can affect timing. No camera exposure latency or metric acceptance.',
                 'ordering':'Configured ring1_side order; not digit OCR.',
                 'recorder_version':2,'candidate_statistics_include_unstable':True,
                 'pixel_ready_frames':len(self.ready_points),'stable_ring_statistics':[],
                 'completed_duration':self.started is not None and time.monotonic()-self.started>=self.seconds,
                 'ring_statistics':[]}
        if self.points:
            a=np.asarray(self.points)
            for i in range(3):
                pts=a[:,i,:];median=np.median(pts,axis=0)
                summary['ring_statistics'].append({'index':i+1,'median_xy_px':median.tolist(),
                    'range_xy_px':np.ptp(pts,axis=0).tolist(),
                    'p95_abs_deviation_from_median_xy_px':np.percentile(np.abs(pts-median),95,axis=0).tolist()})
        if self.ready_points:
            ready=np.asarray(self.ready_points)
            for i in range(3):
                pts=ready[:,i,:];median=np.median(pts,axis=0)
                summary['stable_ring_statistics'].append({'index':i+1,'median_xy_px':median.tolist(),
                    'range_xy_px':np.ptp(pts,axis=0).tolist(),
                    'p95_abs_deviation_from_median_xy_px':np.percentile(np.abs(pts-median),95,axis=0).tolist()})
        self.write(self.folder/'run.json',summary)
        archive=self.folder.with_suffix('.zip')
        with ZipFile(archive,'x',ZIP_DEFLATED) as z:
            for p in sorted(self.folder.iterdir()):
                if p.is_file():z.write(p,p.name)
        print('Run saved:',self.folder)
        print('Evidence ZIP:',archive)
        return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/camera135-ring-pixel-diagnostics'))
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 3<=args.seconds<=60:parser.error('--seconds must be 3..60')
    code=args.app.resolve()/'code'
    profile=json.loads((code/'config/project_profile.json').read_text(encoding='utf8'))
    if profile.get('serial_enabled') is not False:raise RuntimeError('Require existing disabled-serial profile')
    sys.path.insert(0,str(code))
    workbench=importlib.import_module('main')
    sys.argv=[str(code/'main.py')]+(['--check'] if args.check else [])
    if args.check:return workbench.main()
    folder=args.output_dir/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    ring_config=json.loads((code/'config/ring_station_detector.json').read_text(encoding='utf8'))
    recorder=RingPixelRecorder(folder,args.seconds,ring_config['reference']['minimum_confidence'])
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    hashes={n:sha(code/n) for n in ('main.py','ring_station_detector.py','project_camera.py')}
    hashes['diagnostic_entry']=sha(Path(__file__))
    recorder.write(folder/'module_source_hashes.json',hashes)
    for name in ('ring_station_detector.json','project_profile.json','camera_processing.json'):
        p=code/'config'/name;(folder/name).write_bytes(p.read_bytes())
        recorder.write(folder/(name+'.source.json'),{'file':str(p),'sha256':sha(p)})
    original_initialize=workbench.initialize
    def initialize(parsed):
        original_initialize(parsed)
        workbench.set_mode(workbench.Mode.RING_STATION)
    workbench.initialize=initialize
    original_process=workbench.process_ring_station
    def process(frame):
        raw=frame.copy()
        result=original_process(frame)
        finished=time.monotonic()
        recorder.observe(raw,frame,result,getattr(workbench.camera,'last_read_completed_time',None),finished)
        return result
    workbench.process_ring_station=process
    original_print=workbench.print_runtime_status
    def print_status(result,fps,frame_count):
        row=recorder.last_row
        if workbench.current_mode==workbench.Mode.RING_STATION and row:
            print('Mode=RING_STATION FPS=%.1f Units=px Centers=%s PixelFresh=%s Metric=UNVERIFIED Serial=DISABLED' %
                  (fps,json.dumps(row['stable_centers_px']),row['usable_for_pixel_statistics']))
            print('PixelStable=%s CandidateCenters=%s' %
                  (row['pixel_ready'],json.dumps(row['centers_px'])))
        else:original_print(result,fps,frame_count)
    workbench.print_runtime_status=print_status
    original_key=workbench.handle_key
    def handle_key(key):
        if recorder.done(time.monotonic()):return False
        # 诊断只运行R，允许Q/Esc退出，不切换到其他功能以污染统计。
        return original_key(key) if key in (ord('q'),ord('Q'),27) else True
    workbench.handle_key=handle_key
    status=1
    try:
        status=workbench.main()
        return status
    finally:recorder.close(status)


if __name__=='__main__':
    raise SystemExit(main())
