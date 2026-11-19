"""Small offline planner or RealVNC zone preview; never opens an actuator port."""
from __future__ import annotations
import argparse,json,sys,time,hashlib
from dataclasses import asdict
from pathlib import Path

from ..task_materials import resolve_preliminary_task
from .parameters import load_parameters
from .task_plan import batch_jobs
from .types import Zone


def jsonable(value):
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [jsonable(v) for v in value]
    if hasattr(value,'tolist'):return jsonable(value.tolist())
    if hasattr(value,'value'):return value.value
    if isinstance(value,float) and not __import__('math').isfinite(value):return None
    return value


def preview(args, parameters):
    import cv2
    code=args.app_dir/'code'
    if not code.is_dir():raise FileNotFoundError(code)
    sys.path.insert(0,str(code))
    import main as app
    from project_camera import DisabledSerialLink
    app.TurntableSerialLink=DisabledSerialLink;app.DisabledSerialLink=DisabledSerialLink
    from .ring import RingAligner
    original_argv=sys.argv[:]
    sys.argv=[str(code/'main.py'),'--material-color-config',str(code/'config/material_color_thresholds.json')]
    app_args=app.parse_args();app.initialize(app_args)
    assert type(app.serial_link).__name__=='DisabledSerialLink'
    sys.argv=original_argv
    zone=Zone(args.zone);aligner=RingAligner(app.ring_station_detector,parameters.zone(zone),parameters.tuning)
    out=args.output_dir/time.strftime('zone-'+args.zone+'-%Y%m%d-%H%M%S');out.mkdir(parents=True,exist_ok=False)
    window='Vision module - '+args.zone+' - OBSERVATION';created=False;last=None;count=0
    begin=time.monotonic();report=dict(zone=args.zone,task_code=args.task_code,task_source='user_specified',
        serial='DISABLED',actuator_backend_registered=False,frames=0,error=None,
        hsv_path=str(app_args.material_color_config.resolve()),
        hsv_sha256=hashlib.sha256(app_args.material_color_config.read_bytes()).hexdigest())
    try:
        with (out/'observations.jsonl').open('w',encoding='utf-8') as log:
            while time.monotonic()-begin<args.duration:
                frame=app.read_camera()
                if frame is None:time.sleep(.03);continue
                last=frame.copy();overlay=frame.copy()
                if zone==Zone.SOURCE:
                    detection=app.material_pickup_detector.detect(frame,timestamp=time.monotonic())
                    app.draw_material_pickup_detection(overlay,detection,app.material_pickup_detector.config)
                    info=asdict(detection)
                    for r in info['regions']:r.pop('contour',None)
                    label='SOURCE: '+detection.state+' count='+str(detection.material_count)
                else:
                    view=aligner.process(frame);info=asdict(view);label='Waiting for current 3 rings'
                    if view.center_px is not None:
                        xy=tuple(round(x) for x in view.center_px);cv2.circle(overlay,xy,8,(0,255,0),2)
                        label=f'Middle ring error: {view.error_px[0]:.1f}, {view.error_px[1]:.1f}px; yaw {view.yaw_error_deg:.2f}deg'
                count+=1;report['frames']=count
                log.write(json.dumps(jsonable(dict(frame=count,zone=args.zone,result=info)),ensure_ascii=False,allow_nan=False)+'\n')
                h,w=frame.shape[:2];cv2.drawMarker(overlay,(w//2,h//2),(255,255,0),cv2.MARKER_CROSS,24,2)
                cv2.rectangle(overlay,(0,h-105),(w-1,h-1),(0,0,0),-1)
                for i,text in enumerate([f'Task {args.task_code} | MANUAL INPUT | zone {args.zone}',label,
                                          'VIEWER SENDS NO MOTOR COMMANDS | Q: save/exit']):
                    cv2.putText(overlay,text,(16,h-78+i*30),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                if not created:
                    cv2.namedWindow(window,cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO);cv2.resizeWindow(window,960,540);created=True
                cv2.imshow(window,overlay)
                if (cv2.waitKey(1)&255) in (ord('q'),ord('Q'),27):break
                if args.frames and count>=args.frames:break
            if not count:report['error']='NO_FRAME_RECEIVED'
    except KeyboardInterrupt:report['stopped_by_user']=True
    except Exception as e:report['error']=repr(e);raise
    finally:
        app.close_camera();app.serial_link.close()
        if created:cv2.destroyWindow(window)
        if last is not None:
            (out/'last-raw.png').write_bytes(cv2.imencode('.png',last)[1].tobytes())
            (out/'last-overlay.png').write_bytes(cv2.imencode('.png',overlay)[1].tobytes())
        (out/'session.json').write_text(json.dumps(jsonable(report),ensure_ascii=False,indent=2),encoding='utf-8')
        print('Saved:',out,flush=True)
    return 1 if report['error'] else 0


def main(argv=None):
    root=Path(__file__).resolve().parents[3]
    p=argparse.ArgumentParser();p.add_argument('--parameters',type=Path,default=root/'config/vision_manipulation_20261112.json')
    p.add_argument('--task-profile',type=Path,default=root/'config/task_materials_preliminary_2027.json')
    p.add_argument('--task-code',default='215+213+512+321');p.add_argument('--batch',type=int,choices=[1,2],default=1)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--preview',action='store_true');mode.add_argument('--simulate',action='store_true')
    p.add_argument('--zone',choices=['source','rough','storage'],default='source')
    p.add_argument('--app-dir',type=Path,default=Path('/home/orangepi/p2-source-observer-20261106/vision_app'))
    p.add_argument('--duration',type=float,default=120);p.add_argument('--frames',type=int,default=0)
    p.add_argument('--output-dir',type=Path,default=Path.home()/'vision_tools'/'diagnostics')
    args=p.parse_args(argv)
    if not 0<args.duration<=120 or args.frames<0:p.error('观察时长需在0～120秒、帧数非负')
    parameters=load_parameters(args.parameters)
    task=resolve_preliminary_task(args.task_code,1,args.task_profile)
    if args.preview:return preview(args,parameters)
    if args.simulate:
        from .simulation import simulate
        result=simulate(task,parameters)
        print(json.dumps({k:v for k,v in result.items() if k!='trace'},ensure_ascii=False,indent=2))
        return 0
    jobs=batch_jobs(task,args.batch,'offline-plan',supervised=True,manual_load_before_rough=True)
    print(json.dumps(jsonable(dict(task_source='user_specified',plan_only=True,actuator_backend_registered=False,
                                  jobs=[asdict(j) for j in jobs])),ensure_ascii=False,indent=2))
    return 0
