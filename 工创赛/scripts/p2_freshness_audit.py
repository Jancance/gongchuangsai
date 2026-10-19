"""Desktop camera diagnostic: one latest-frame shared slot, bounded watchdog.

Timestamps are HOST READ COMPLETION times, not exposure/sensor timestamps.
This cannot establish USB/driver buffering, sensor frame loss or end-to-end age.
Does not connect to the robot, reopen a failed camera or issue motion commands.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import queue
import subprocess
import time
import cv2
import numpy as np

class NoFreshFrameError(RuntimeError):
    pass

def require_recent_frame(now,last_read,stall_ms):
    if age_ms(now,last_read)>stall_ms:
        raise NoFreshFrameError('No new host-read frame within stall limit')

class LatestSlot:
    def __init__(self, context, shape=(960,1280,3)):
        self.shape=shape
        self.buffer=context.RawArray('B',int(np.prod(shape)))
        self.sequence=context.RawValue('q',0)
        self.read_time=context.RawValue('d',0.0)
        self.lock=context.Lock()

    def publish(self, frame, read_completed):
        if frame.shape!=self.shape or frame.dtype!=np.uint8:
            raise ValueError('Unexpected decoded format/size')
        with self.lock:
            np.copyto(np.frombuffer(self.buffer,dtype=np.uint8).reshape(self.shape),frame)
            self.read_time.value=read_completed
            self.sequence.value+=1

    def take_after(self, previous):
        if not self.lock.acquire(timeout=.1):return None
        try:
            if self.sequence.value<=previous:return None
            return (self.sequence.value,self.read_time.value,
                    np.frombuffer(self.buffer,dtype=np.uint8).reshape(self.shape).copy())
        finally:self.lock.release()

def age_ms(now,read_completed):
    return max(0.0,(now-read_completed)*1000)

def result_fresh(now,read_completed,limit_ms):
    return age_ms(now,read_completed)<=limit_ms

def capture_worker(device,slot,ready,stop,status,buffers=1,simulate_stall_after=None):
    cap=None
    read_ms=[];publish_ms=[];stamps=[]
    try:
        cap=cv2.VideoCapture(device,cv2.CAP_V4L2)
        if not cap.isOpened():raise RuntimeError('Camera could not open; close previews first')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,960)
        cap.set(cv2.CAP_PROP_FPS,30)
        buffer_setting_accepted=bool(cap.set(cv2.CAP_PROP_BUFFERSIZE,buffers))
        ok,frame=cap.read()
        if not ok:raise RuntimeError('Initial read failed')
        for setting in ('exposure_auto=1','gain=30','white_balance_temperature_auto=0',
                        'white_balance_temperature=4600','exposure_absolute=50'):
            subprocess.run(['v4l2-ctl','-d',device,'--set-ctrl='+setting],check=True,timeout=5)
        for _ in range(90):
            if stop.is_set():return
            ok,frame=cap.read()
            if not ok:raise RuntimeError('Warm-up read failed')
        controls=subprocess.run(['v4l2-ctl','-d',device,
            '--get-ctrl=exposure_auto,exposure_absolute,gain,white_balance_temperature_auto,white_balance_temperature'],
            check=True,capture_output=True,text=True,timeout=5).stdout
        mode=subprocess.run(['v4l2-ctl','-d',device,'--get-fmt-video','--get-parm'],
            check=True,capture_output=True,text=True,timeout=5).stdout
        fourcc=int(cap.get(cv2.CAP_PROP_FOURCC))
        status.put({'event':'ready','controls':controls,
                    'requested_buffers':buffers,'v4l2_mode_readback':mode,
                    'opencv_mode_readback':{'width':cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                        'height':cap.get(cv2.CAP_PROP_FRAME_HEIGHT),'fps':cap.get(cv2.CAP_PROP_FPS),
                        'fourcc':''.join(chr((fourcc>>(8*i))&255) for i in range(4)),
                        'buffers':cap.get(cv2.CAP_PROP_BUFFERSIZE)},
                    'buffersize_set_returned_true':buffer_setting_accepted,
                    'buffersize_note':'Return value is not proof of zero driver backlog.'})
        capture_start=time.monotonic()
        ready.set()
        while not stop.is_set():
            if simulate_stall_after is not None and time.monotonic()-capture_start>=simulate_stall_after:
                status.put({'event':'simulated_capture_stall','monotonic_time':time.monotonic(),
                            'published_before_pause':len(stamps)})
                # No further reads or publications. Parent watchdog should stop
                # the run; this is software injection, not a physical USB fault.
                stop.wait()
                break
            before=time.monotonic();ok,frame=cap.read();stamp=time.monotonic()
            if not ok or frame is None:raise RuntimeError('Camera read failed')
            slot.publish(frame,stamp)
            read_ms.append((stamp-before)*1000)
            publish_ms.append((time.monotonic()-stamp)*1000)
            stamps.append(stamp)
    except BaseException as exc:
        status.put({'event':'error','message':type(exc).__name__+': '+str(exc)})
    finally:
        if cap is not None:cap.release()
        if stamps:
            intervals=np.diff(stamps)*1000
            def stats(values):
                if not len(values):return None
                return dict(zip(('min','p50','p95','max'),
                    [round(float(x),3) for x in np.percentile(values,[0,50,95,100])]))
            status.put({'event':'capture_timing','host_reads_including_shutdown':len(stamps),
                        'read_call_ms':stats(read_ms),'publish_lock_and_copy_ms':stats(publish_ms),
                        'read_completion_interval_ms':stats(intervals),
                        'note':'Includes any last capture during shutdown; read includes waiting and decoding.'})

def stop_worker(worker, stop):
    stop.set();worker.join(timeout=2)
    forced=worker.is_alive()
    if forced:worker.terminate();worker.join(timeout=2)
    if worker.is_alive():
        worker.kill();worker.join(timeout=2)
    return forced

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',default='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0')
    p.add_argument('--seconds',type=float,default=10)
    p.add_argument('--buffers',type=int,choices=(1,2,4),default=1,
                   help='Requested OpenCV/V4L2 buffers; single application latest slot is unchanged')
    p.add_argument('--max-host-age-ms',type=float,default=150)
    p.add_argument('--stall-ms',type=float,default=500)
    p.add_argument('--simulate-stall-after',type=float,
                   help='After this many seconds of capture, deliberately stop new frame publication')
    p.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/v0-main/freshness-audits'))
    args=p.parse_args()
    if not 1<=args.seconds<=60:p.error('--seconds must be 1..60')
    if not 0<args.max_host_age_ms<args.stall_ms:p.error('Require 0 < age limit < stall limit')
    if args.simulate_stall_after is not None and not 1<=args.simulate_stall_after<args.seconds-args.stall_ms/1000-1:
        p.error('Stall injection requires >=1 second and room for watchdog before run end')
    from p2_color_probe_v1 import analyze, VERSION
    folder=args.output_dir/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True,exist_ok=False)
    context=mp.get_context('spawn')
    slot=LatestSlot(context);ready=context.Event();stop=context.Event();status=context.Queue()
    worker=context.Process(target=capture_worker,args=(args.device,slot,ready,stop,status,args.buffers,args.simulate_stall_after),daemon=True)
    worker.start()
    samples=[];events=[];error=None;first=None;last=None;start=None;elapsed=0;previous=0
    skipped=0;stale_inputs=0;last_read=0;start_seq=0;end_seq=0;forced=False
    no_fresh_frame_detected=False;fault_time=None
    try:
        print('Starting capture and warm-up; keep camera and objects still.',flush=True)
        deadline=time.monotonic()+25
        while not ready.is_set():
            if not worker.is_alive():raise RuntimeError('Capture process stopped during startup')
            if time.monotonic()>deadline:raise RuntimeError('Capture startup timed out')
            ready.wait(.1)
        initial=slot.take_after(0)
        if initial is not None:previous=initial[0]
        start_seq=previous
        start=time.monotonic();last_read=start
        print('Recording latest-frame diagnostic for',args.seconds,'seconds.',flush=True)
        with (folder/'frames.jsonl').open('w',encoding='utf-8') as stream:
            while time.monotonic()-start<args.seconds:
                while True:
                    try:event=status.get_nowait()
                    except queue.Empty:break
                    events.append(event)
                    if event['event']=='error':raise RuntimeError(event['message'])
                if not worker.is_alive():raise RuntimeError('Capture process stopped')
                item=slot.take_after(previous)
                if item is None:
                    require_recent_frame(time.monotonic(),last_read,args.stall_ms)
                    time.sleep(.002);continue
                sequence,read_stamp,frame=item
                skipped+=max(0,sequence-previous-1)
                previous=sequence;last_read=read_stamp
                begin=time.monotonic();input_age=age_ms(begin,read_stamp)
                row={'sequence':sequence,'host_read_completed_s':read_stamp-start,'input_host_age_ms':input_age}
                if input_age>args.max_host_age_ms:
                    stale_inputs+=1;row['status']='STALE_INPUT_REJECTED';row['result_usable']=False
                    require_recent_frame(begin,read_stamp,args.stall_ms)
                else:
                    result,_=analyze(frame);end=time.monotonic()
                    fresh=result_fresh(end,read_stamp,args.max_host_age_ms)
                    row.update(analysis_ms=(end-begin)*1000,result_host_age_ms=age_ms(end,read_stamp),
                               status='HOST_AGE_OK' if fresh else 'STALE_RESULT_REJECTED',
                               result_usable=fresh,
                               diagnostic_counts=dict(Counter(r['label'] for r in result['regions'])))
                    # Counts are saved for diagnostics only even if result is stale.
                    if first is None:first=frame.copy()
                    last=frame
                samples.append(row);stream.write(json.dumps(row)+'\n')
        elapsed=time.monotonic()-start
        ending=slot.take_after(previous);end_seq=ending[0] if ending else previous
    except (Exception,KeyboardInterrupt) as exc:
        no_fresh_frame_detected=isinstance(exc,NoFreshFrameError)
        fault_time=time.monotonic()
        error=type(exc).__name__+': '+str(exc)
        if start is not None:elapsed=time.monotonic()-start
        end_seq=previous
    finally:
        forced=stop_worker(worker,stop)
        while True:
            try:events.append(status.get(timeout=.1))
            except queue.Empty:break
        worker.close();status.close()
    worker_errors=[e['message'] for e in events if e['event']=='error']
    if worker_errors and error is None:error='; '.join(worker_errors)
    if forced and error is None:error='Capture process required forced shutdown'
    for name,frame in [('first',first),('last',last)]:
        if frame is not None and not cv2.imwrite(str(folder/(name+'.png')),frame):
            error=(error or '')+' Save failed: '+name
    analyzed=[r for r in samples if 'analysis_ms' in r]
    injections=[e for e in events if e['event']=='simulated_capture_stall']
    expected_fault_observed=bool(injections and no_fresh_frame_detected and analyzed
                                and not forced and not worker_errors
                                and error=='NoFreshFrameError: No new host-read frame within stall limit')
    def percentile(key,pct):
        return round(float(np.percentile([r[key] for r in analyzed],pct)),2) if analyzed else None
    report=dict(version=VERSION,diagnostic_version='host-freshness-v3-stall-injection',
        simulated_stall_after_seconds=args.simulate_stall_after,
        expected_fault_observed=expected_fault_observed,
        no_fresh_frame_detected=no_fresh_frame_detected,
        fault_detection_after_injection_ms=round((fault_time-injections[0]['monotonic_time'])*1000,2)
            if injections and fault_time is not None else None,
        requested_buffers=args.buffers,requested_seconds=args.seconds,elapsed_seconds=round(elapsed,3),
        max_host_age_ms=args.max_host_age_ms,stall_ms=args.stall_ms,
        host_read_frames=max(0,end_seq-start_seq),processed_frames=len(analyzed),
        host_read_fps=round(max(0,end_seq-start_seq)/elapsed,2) if elapsed else None,
        processing_fps=round(len(analyzed)/elapsed,2) if elapsed else None,
        intentionally_skipped_before_processing=skipped,
        unconsumed_at_end=max(0,end_seq-previous),stale_input_frames=stale_inputs,
        stale_result_frames=sum(r['status']=='STALE_RESULT_REJECTED' for r in samples),
        p95_input_host_age_ms=percentile('input_host_age_ms',95),
        max_input_host_age_ms=percentile('input_host_age_ms',100),
        p95_analysis_ms=percentile('analysis_ms',95),
        p95_result_host_age_ms=percentile('result_host_age_ms',95),
        max_result_host_age_ms=percentile('result_host_age_ms',100),
        completed=error is None and elapsed>=args.seconds,error=error,
        capture_forced_shutdown=forced,capture_events=events,
        probe_sha256=hashlib.sha256(Path(__file__).with_name('p2_color_probe_v1.py').read_bytes()).hexdigest(),
        diagnostic_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        caveat='HOST READ COMPLETION AGE ONLY. Cannot prove exposure age, camera/USB buffering, sensor unique frames or frame loss. Skips are application overwrite, not sensor drops. No robot actions.')
    (folder/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    for key in ('completed','expected_fault_observed','fault_detection_after_injection_ms',
                'requested_buffers','host_read_frames','processed_frames','host_read_fps','processing_fps',
                'intentionally_skipped_before_processing','stale_input_frames','stale_result_frames',
                'p95_input_host_age_ms','p95_result_host_age_ms','max_result_host_age_ms','capture_forced_shutdown'):
        print(key+':',report[key])
    print('Saved:',folder,flush=True)
    if error:raise SystemExit(error)

if __name__=='__main__':main()
