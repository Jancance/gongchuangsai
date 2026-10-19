"""Latest-frame tabletop identity preview. No robot control or metric pose."""
import argparse
from collections import Counter
from datetime import datetime
import json
import multiprocessing as mp
from pathlib import Path
import queue
import time
import cv2
from p2_color_probe_v1 import analyze, VERSION
from p2_freshness_audit import LatestSlot,capture_worker,stop_worker,age_ms
from p2_tabletop_tracker import TabletopTracker,red_transition

def save_transition(folder,name,frame,previous_result,result,detector_regions):
    if not cv2.imwrite(str(folder/(name+'-raw.png')),frame):
        raise RuntimeError('Cannot save transition image')
    (folder/(name+'.json')).write_text(json.dumps({
        'previous_result':previous_result,'current_result':result,
        'detector_regions':detector_regions,
        'note':'One image before/after comparison; no claim of physical identity or motion state.'
    },indent=2),encoding='utf-8')
    print('Captured:',folder/name,flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',default='/dev/v4l/by-id/usb-DECXIN_DECXIN_CAMERA_01.00.00-video-index0')
    p.add_argument('--output-dir',type=Path,default=Path('/home/orangepi/v0-main/tracking-runs'))
    args=p.parse_args()
    folder=args.output_dir/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    folder.mkdir(parents=True,exist_ok=False)
    ctx=mp.get_context('spawn');slot=LatestSlot(ctx);ready=ctx.Event();stop=ctx.Event();status=ctx.Queue()
    worker=ctx.Process(target=capture_worker,args=(args.device,slot,ready,stop,status,4),daemon=True)
    tracker=TabletopTracker();events=[];error=None;previous=0;frame=None;read_time=None
    state='WARMING UP';unknown=0;processed=0;stale=0;last_result={};retired=[]
    saved_transitions=set()
    worker.start()
    try:
        deadline=time.monotonic()+25
        print('Warming up. One object per color. S save; Q quit.',flush=True)
        while not ready.is_set():
            if not worker.is_alive():raise RuntimeError('Capture stopped during startup')
            if time.monotonic()>deadline:raise RuntimeError('Capture startup timeout')
            ready.wait(.1)
        last_read=time.monotonic()
        with (folder/'observations.jsonl').open('w',encoding='utf-8') as log:
            while True:
                while True:
                    try:e=status.get_nowait()
                    except queue.Empty:break
                    events.append(e)
                    if e['event']=='error':raise RuntimeError(e['message'])
                if not worker.is_alive():raise RuntimeError('Capture stopped')
                item=slot.take_after(previous)
                if item is not None:
                    previous,read_time,frame=item;last_read=read_time
                    previous_result=last_result
                    detector_regions=[]
                    if age_ms(time.monotonic(),read_time)>150:
                        retired=tracker.clear();state='STALE INPUT';stale+=1
                        last_result={'sequence':previous,'reason':state,'retired_ids':retired,'observed':[]}
                    else:
                        detections,_=analyze(frame);now=time.monotonic();processed+=1
                        detector_regions=detections['regions']
                        unknown=sum(r['label']=='UNKNOWN' for r in detections['regions'])
                        if age_ms(now,read_time)>150:
                            retired=tracker.clear();state='STALE RESULT';stale+=1
                            last_result={'reason':state,'retired_ids':retired,'observed':[]}
                        else:
                            last_result=tracker.update(detections['regions'],read_time)
                            retired=last_result['retired_ids'];state=last_result['reason']
                        last_result.update(sequence=previous,host_result_age_ms=age_ms(now,read_time),
                                           read_completed_monotonic=read_time,unknown_candidates=unknown,
                                           detector_regions=detector_regions)
                    transition=red_transition(previous_result,last_result)
                    if transition is not None and transition not in saved_transitions:
                        save_transition(folder,transition,frame,previous_result,last_result,detector_regions)
                        saved_transitions.add(transition)
                    log.write(json.dumps(last_result)+'\n')
                now=time.monotonic()
                expired=tracker.expire(now)
                if expired:
                    log.write(json.dumps({'event':'EXPIRED','retired_ids':expired,'monotonic':now})+'\n')
                if age_ms(now,last_read)>150:state='WAITING FOR NEW FRAME'
                if age_ms(now,last_read)>500:raise RuntimeError('No new host-read frame within 500 ms')
                if frame is None:
                    time.sleep(.002);continue
                drawing=frame.copy();visible=tracker.visible(now)
                for t in visible:
                    x,y,w,h=t['bbox_px']
                    cv2.rectangle(drawing,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.putText(drawing,f"{t['id']} {t['color']} {t['state']}",
                        (x,max(105,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2)
                cv2.rectangle(drawing,(0,0),(drawing.shape[1]-1,88),(0,0,0),-1)
                cv2.putText(drawing,'TABLETOP IDs ONLY - NO MOTION OR GRASP',(12,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
                cv2.putText(drawing,f'{state} visible:{len(visible)} unknown:{unknown} stale:{stale}',
                    (12,57),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                cv2.imshow('P2 tracking - S save / Q quit',cv2.resize(drawing,(960,720)))
                key=cv2.waitKey(1)&0xff
                if key in (ord('q'),ord('Q'),27):break
                if key in (ord('s'),ord('S')):
                    stem=datetime.now().strftime('snapshot-%H%M%S-%f')
                    for suffix,im in [('raw',frame),('overlay',drawing)]:
                        if not cv2.imwrite(str(folder/f'{stem}-{suffix}.png'),im):raise RuntimeError('Save failed')
                    (folder/f'{stem}.json').write_text(json.dumps({'visible':visible,
                        'snapshot_host_age_ms':age_ms(time.monotonic(),read_time),
                        'state':state,'last_diagnostic':last_result},indent=2),encoding='utf-8')
                    print('Saved:',folder/stem,flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        error=type(exc).__name__+': '+str(exc);tracker.clear()
    finally:
        forced=stop_worker(worker,stop);cv2.destroyAllWindows()
        while True:
            try:events.append(status.get(timeout=.1))
            except queue.Empty:break
        worker.close();status.close()
        (folder/'run.json').write_text(json.dumps({'session_id':tracker.session_id,'probe':VERSION,
            'processed_frames':processed,'stale_rejections':stale,'error':error,
            'forced_shutdown':forced,'capture_events':events,
            'scope':'Tabletop identity demo, one item per color. No motion state or metric pose. Reappearance creates a new ID.'},indent=2),encoding='utf-8')
        print('Run saved:',folder,flush=True)
    if error:raise SystemExit(error)

if __name__=='__main__':main()
