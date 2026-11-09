"""10秒独立实时像素取证；保留原应用，不导入主程序或串口，不发送动作。"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from queue import Queue,Empty,Full
import sys
from threading import Thread,Event
import time
from zipfile import ZipFile,ZIP_DEFLATED

import cv2
import numpy as np

from p2_partial_ring_live_gate import LivePixelGate


def write_json(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')


def draw(frame,status):
    canvas=frame.copy()
    color=(0,255,0) if status['pixel_ready'] else (0,190,255)
    for i,p in enumerate(status['stable_centers_px'] if status['pixel_ready'] else status['candidate_centers_px']):
        point=tuple(round(v) for v in p)
        cv2.drawMarker(canvas,point,color,cv2.MARKER_CROSS,24,2)
        cv2.putText(canvas,'slot%d (NOT OCR)'%(i+1),(point[0]+8,point[1]-12),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
    lines=['LIVE PIXEL AUDIT | '+status['state']+' | '+status['reason'],
           'Centers='+str([[round(v,2) for v in p] for p in status['stable_centers_px']]),
           'Method='+str(status.get('method'))+' Age(ms)='+str(round(status['result_host_age_ms'],1) if status['result_host_age_ms'] is not None else None),
           'Window stability only | Stationary UNVERIFIED | Metric UNVERIFIED | Serial DISABLED']
    cv2.rectangle(canvas,(10,10),(1560,145),(0,0,0),-1)
    for i,line in enumerate(lines):
        cv2.putText(canvas,line,(20,38+i*29),cv2.FONT_HERSHEY_SIMPLEX,.65,color,1,cv2.LINE_AA)
    return canvas


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/partial-ring-live-diagnostics'))
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 3<=args.seconds<=30:
        parser.error('--seconds must be 3..30')
    base=Path(__file__).resolve().parent
    manifest=json.loads((base/'LIVE_MANIFEST.json').read_text(encoding='utf8'))
    for name,digest in manifest['files'].items():
        path=(base/name).resolve()
        assert path.is_relative_to(base) and hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
    code=base/'vision_app/code'
    profile=json.loads((code/'config/project_profile.json').read_text(encoding='utf8'))
    assert profile['serial_enabled'] is False and profile['geometry_verified'] is False
    sys.path.insert(0,str(code))
    import ring_station_detector as module
    from p2_partial_ring_candidate import detect_candidate
    config=module.load_ring_station_config(code/'config/ring_station_detector.json')
    gate=LivePixelGate(config.stable_frame_count,config.maximum_center_jitter_px,150)
    if args.check:
        print('source_and_profile_hashes=PASS camera_opened=False serial=DISABLED motion_allowed=False maximum_host_age_ms=150')
        return
    from project_camera import LatestProjectCamera
    cv2.ocl.setUseOpenCL(False)
    folder=args.output_dir/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True,exist_ok=False)
    write_json(folder/'source.json',dict(manifest_sha256=hashlib.sha256((base/'LIVE_MANIFEST.json').read_bytes()).hexdigest(),
                                      files=manifest['files'],opencv=cv2.__version__,profile=profile))
    queue=Queue(maxsize=1);results=Queue(maxsize=1);stop=Event();faults=[]
    camera=None;reader=None;detector=None;rows=[];snapshots={};events=[];error=None
    started=None;last_received=None;previous_state=None
    def read_loop():
        try:
            while not stop.is_set():
                ok,frame=camera.read()
                if not ok:
                    if not stop.is_set():faults.append('CAMERA_READ_FAILED')
                    break
                item=(camera.last_read_completed_time,frame)
                try:queue.put_nowait(item)
                except Full:
                    try:queue.get_nowait()
                    except Empty:pass
                    try:queue.put_nowait(item)
                    except Full:pass
        except Exception as exc:
            faults.append(type(exc).__name__+': '+str(exc))
    def detect_loop():
        try:
            while not stop.is_set():
                try:stamp,raw=queue.get(timeout=.05)
                except Empty:continue
                result=detect_candidate(raw,module,config)
                item=(stamp,raw,result,time.monotonic())
                try:results.put_nowait(item)
                except Full:
                    try:results.get_nowait()
                    except Empty:pass
                    try:results.put_nowait(item)
                    except Full:pass
        except Exception as exc:
            faults.append(type(exc).__name__+': '+str(exc))
    try:
        camera=LatestProjectCamera()
        reader=Thread(target=read_loop,daemon=True);reader.start()
        detector=Thread(target=detect_loop,daemon=True);detector.start()
        cv2.namedWindow('Partial ring live audit',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Partial ring live audit',1280,720)
        last_received=time.monotonic();last_frame=None
        while True:
            now=time.monotonic()
            if started is not None and now-started>=args.seconds:break
            if faults or now-last_received>.5:
                error=faults[0] if faults else 'NO_PROCESSED_RESULT_500MS'
                gate.fault(error)
                if last_frame is not None:cv2.imshow('Partial ring live audit',draw(last_frame,gate.current(now)))
                break
            try:stamp,raw,result,finished=results.get_nowait()
            except Empty:
                status=gate.current(now)
                if last_frame is not None and status['state']!=previous_state:
                    cv2.imshow('Partial ring live audit',draw(last_frame,status))
                    events.append(dict(time=now,event='DISPLAY_STATE_CHANGE',status=status))
                    previous_state=status['state']
                if cv2.waitKey(1)&0xff in (ord('q'),ord('Q'),27):break
                time.sleep(.002)
                continue
            if started is None:started=time.monotonic()
            last_received=time.monotonic();last_frame=raw
            status=gate.observe(result,raw.shape,stamp,time.monotonic())
            # 每次由原图重绘，过期时不会残留上一次绿色十字。
            canvas=draw(raw,status)
            display_time=time.monotonic()
            latest=gate.current(display_time)
            if latest['state']!=status['state']:
                canvas=draw(raw,latest)
            cv2.imshow('Partial ring live audit',canvas)
            shown=time.monotonic()
            presented=gate.current(shown)
            # imshow若耗时跨门，立即重新提交失效画面并记录；不沿用过期stable。
            if presented['state']!=latest['state']:
                cv2.imshow('Partial ring live audit',draw(raw,presented))
            status=presented
            row=dict(frame_index=len(rows)+1,host_read_time=stamp,detector_finished=finished,
                     detector_result_age_ms=(finished-stamp)*1000,display_submission_age_ms=(shown-stamp)*1000,
                     candidate=result,status=status)
            rows.append(row)
            names=['first-frame'] if len(rows)==1 else []
            if status['candidate_centers_px']:names.append('first-candidate')
            if status['pixel_ready']:names.append('first-window-stable')
            if status['state']=='STALE':names.append('first-stale')
            if status['state']=='LOST':names.append('first-lost')
            for name in names:
                if name not in snapshots:snapshots[name]=(raw,row)
            previous_state=status['state']
            print('Frame=%d State=%s Method=%s Age=%.1fms Centers=%s'%(len(rows),status['state'],result['method'],status['result_host_age_ms'] or 0,status['stable_centers_px']),flush=True)
            if cv2.waitKey(1)&0xff in (ord('q'),ord('Q'),27):break
    except Exception as exc:
        error=type(exc).__name__+': '+str(exc)
    finally:
        sampling_finished=time.monotonic()
        stop.set()
        if reader is not None:reader.join(.7)
        if detector is not None:detector.join(.7)
        if camera is not None:camera.release()
        cv2.destroyAllWindows()
        # PNG编码和磁盘输出移到采样结束后，避免在计时窗口内同步写图。
        for name,(raw,row) in snapshots.items():
            cv2.imencode('.png',raw)[1].tofile(folder/(name+'-raw.png'))
            cv2.imencode('.png',draw(raw,row['status']))[1].tofile(folder/(name+'-overlay.png'))
            write_json(folder/(name+'.json'),row)
        (folder/'observations.jsonl').write_text(''.join(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n' for row in rows),encoding='utf8')
        write_json(folder/'display-events.json',events)
        write_json(folder/'camera-events.json',getattr(camera,'events',[]) if camera else [])
        summary=dict(frames=len(rows),states=dict(Counter(row['status']['state'] for row in rows)),
                     completed_duration=started is not None and sampling_finished-started>=args.seconds,
                     sampling_span_s=sampling_finished-started if started is not None else None,
                     requested_seconds=args.seconds,error=error,metric_valid=False,motion_allowed=False,serial='DISABLED',
                     scope='Host-read age only; GUI submission is not measured monitor presentation. Short pixel window is not physical stationary or action acceptance.')
        write_json(folder/'run.json',summary)
        archive=folder.with_suffix('.zip')
        with ZipFile(archive,'x',ZIP_DEFLATED) as z:
            for path in folder.iterdir():z.write(path,path.name)
        print(json.dumps(summary,ensure_ascii=False))
        print('Evidence ZIP:',archive)
    if error:raise SystemExit(1)


if __name__=='__main__':
    main()
