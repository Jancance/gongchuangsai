"""Static-scene color-count audit. Application frames only; no motion control.

Expected labels score results AFTER detection, never guide classification.
No claim of sensor frame uniqueness, tracking, exposure-to-result latency,
metric accuracy or robot acceptance is made.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import time
import cv2
import numpy as np
from p2_color_probe import analyze, VERSION

COLORS=('red','yellow','blue','green','black','lightblue')

def score_regions(regions, expected):
    actual=Counter(r['label'] for r in regions if r['label']!='UNKNOWN')
    unknown=[r for r in regions if r['label']=='UNKNOWN']
    return {'known_counts':dict(actual),'known_counts_match':actual==Counter(expected),
            'unknown_count':len(unknown),
            'unknown_reasons':dict(Counter(r['reason'] for r in unknown))}

def summarize(samples, expected, elapsed, requested_seconds, error):
    n=len(samples)
    matched=sum(s['known_counts_match'] for s in samples)
    unknown=sum(s['unknown_count']>0 for s in samples)
    gaps=np.diff([s['read_completed_s'] for s in samples])*1000
    return dict(version=VERSION,expected_counts=dict(Counter(expected)),
        requested_seconds=requested_seconds,elapsed_seconds=round(elapsed,3),
        processed_frames=n,matching_frames=matched,mismatch_frames=n-matched,
        matching_fraction=matched/n if n else None,
        frames_with_unknown=unknown,max_unknown_count=max((s['unknown_count'] for s in samples),default=0),
        observed_patterns=dict(Counter(json.dumps(s['known_counts'],sort_keys=True) for s in samples)),
        application_processing_fps=round(n/elapsed,2) if elapsed>0 else None,
        max_read_completion_gap_ms=round(float(gaps.max()),2) if len(gaps) else None,
        p95_analysis_ms=round(float(np.percentile([s['analysis_ms'] for s in samples],95)),2) if n else None,
        completed=not error and elapsed>=requested_seconds,error=error,
        scope='STATIC DEVELOPMENT OBSERVATION ONLY. UNKNOWN not counted as a known object. No hardware acceptance.',
        caveat='Application read completion timing; no sensor timestamps, frame uniqueness, latency or track IDs.')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expected',nargs='*',choices=COLORS,required=True)
    p.add_argument('--seconds',type=float,default=10)
    p.add_argument('--probe',choices=('v0','v1'),default='v0',help='v0 baseline or v1 development candidate')
    p.add_argument('--device',default='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0')
    p.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/v0-main/color-audits'))
    args=p.parse_args()
    if not 1<=args.seconds<=60: p.error('--seconds must be between 1 and 60')
    run_analyze=analyze
    probe_version=VERSION
    probe_file=Path(__file__).with_name('p2_color_probe.py')
    if args.probe=='v1':
        from p2_color_probe_v1 import analyze as run_analyze, VERSION as probe_version
        probe_file=Path(__file__).with_name('p2_color_probe_v1.py')
    run=args.output_dir/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    run.mkdir(parents=True,exist_ok=False)
    samples=[];error=None;start=None;elapsed=0;controls='';first=None;failure=None;last=None
    cap=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
    try:
        if not cap.isOpened(): raise RuntimeError('Camera could not open; close other camera programs')
        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);cap.set(cv2.CAP_PROP_FRAME_HEIGHT,960)
        cap.set(cv2.CAP_PROP_FPS,30)
        ok,frame=cap.read()
        if not ok: raise RuntimeError('Initial read failed')
        for value in ('exposure_auto=1','gain=30','white_balance_temperature_auto=0',
                      'white_balance_temperature=4600','exposure_absolute=50'):
            subprocess.run(['v4l2-ctl','-d',args.device,'--set-ctrl='+value],check=True)
        for _ in range(90):
            ok,frame=cap.read()
            if not ok: raise RuntimeError('Warm-up read failed')
        controls=subprocess.run(['v4l2-ctl','-d',args.device,
            '--get-ctrl=exposure_auto,exposure_absolute,gain,white_balance_temperature_auto,white_balance_temperature'],
            check=True,capture_output=True,text=True).stdout
        print('Probe:',probe_version,flush=True)
        print('Recording. Keep objects, camera and light still.',flush=True)
        start=time.monotonic()
        with (run/'frames.jsonl').open('w',encoding='utf-8') as stream:
            while time.monotonic()-start<args.seconds:
                ok,frame=cap.read();read_time=time.monotonic()
                if not ok: raise RuntimeError('Frame read failed')
                before=time.monotonic();result,_=run_analyze(frame)
                row=score_regions(result['regions'],args.expected)
                row.update(index=len(samples),read_completed_s=read_time-start,
                           analysis_ms=(time.monotonic()-before)*1000,regions=result['regions'])
                samples.append(row);stream.write(json.dumps(row)+'\n')
                if first is None: first=frame.copy()
                if not row['known_counts_match'] and failure is None: failure=frame.copy()
                last=frame
        elapsed=time.monotonic()-start
    except (Exception,KeyboardInterrupt) as exc:
        error=type(exc).__name__+': '+str(exc)
        if start is not None: elapsed=time.monotonic()-start
    finally:
        cap.release()
    for name,im in (('first',first),('first-mismatch',failure),('last',last)):
        if im is not None and not cv2.imwrite(str(run/(name+'.png')),im):
            error=(error or '')+' Image save failed: '+name
    report=summarize(samples,args.expected,elapsed,args.seconds,error)
    report['version']=probe_version
    report['controls_readback']=controls
    report['probe_sha256']=hashlib.sha256(probe_file.read_bytes()).hexdigest()
    (run/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    for key in ('completed','processed_frames','matching_frames','mismatch_frames',
                'frames_with_unknown','max_unknown_count','application_processing_fps','max_read_completion_gap_ms'):
        print(f'{key}: {report[key]}')
    print('Saved:',run,flush=True)
    if error: raise SystemExit(error)

if __name__=='__main__':
    main()
