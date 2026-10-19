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


