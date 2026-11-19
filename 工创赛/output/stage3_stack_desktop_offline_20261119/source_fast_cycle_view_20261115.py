"""Two camera views driven by MCU status; UART TX contains observations only."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import signal

CODE=Path('/home/orangepi/p2-source-observer-20261106/vision_app/code')
STATES={0:'Ready',1:'Preparing',2:'Aligning',3:'Moving chassis',4:'Finishing alignment',
        5:'Alignment complete',6:'Finding next material',7:'Picking and placing',8:'Batch complete',9:'Stopped'}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage3-config',type=Path,default=Path(__file__).resolve().parent/'stage3_candidate/config/stage3.json')
    parser.add_argument('--body-profile',type=Path,required=True)
    parser.add_argument('--material-pickup-config',type=Path,required=True)
    parser.add_argument('--session-dir',type=Path,required=True)
    parser.add_argument('--observe-motion',action='store_true',help='Collect moving-disc evidence; no pickup observation packets')
    parser.add_argument('--study-stopped-pose',action='store_true',help='Camera study only at the operator-verified stopped observation pose')
    parser.add_argument('--continuous-pick',action='store_true')
    parser.add_argument('--prediction-config',type=Path)
    parser.add_argument('--preview-only',action='store_true',help='No observation transmission; leaves MCU waiting without fault or reset')
    parser.add_argument('--rim-detector-config',type=Path,required=True)
    parser.add_argument('--orbit-config',type=Path,required=True)
    args=parser.parse_args()
    if args.study_stopped_pose and not args.observe_motion:parser.error('--study-stopped-pose requires --observe-motion')
    if args.continuous_pick and (args.prediction_config is None or args.observe_motion):parser.error('continuous pickup requires its validated timing config and excludes observation-only mode')
    def interrupted(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    sys.path.insert(0,str(CODE))
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    import cv2
    import serial
    import main as app
    from project_camera import DisabledSerialLink
    from source_body_registration import load_profile,estimate
    from latched_material_stream_20261115 import RemainingStopped
    from mcu_cycle_protocol import decode_status,observation,continuous_observation
    from continuous_turntable import rim_observation_valid
    from direct_frame_trigger_20261115 import DirectFrameTrigger
    from each_observation_rotation import EachObservationRotation
    from observation_frame_epoch import ObservationFrameEpoch,camera_metadata
    from source_cycle_dashboard_20261115 import compose
    app.TurntableSerialLink=app.DisabledSerialLink=DisabledSerialLink
    import commit_observe_unbounded_20261114 as capture
    profile=load_profile(args.body_profile);gate=RemainingStopped()
    rim=None
    if args.rim_detector_config is not None:
        from adaptive_source_rim_20261115 import AdaptiveSourceRim
        rim=AdaptiveSourceRim(profile,args.rim_detector_config)
    prediction_config=json.loads(args.prediction_config.read_text()) if args.prediction_config else None
    pickup_config=json.loads(args.material_pickup_config.read_text())
    orbit_config=json.loads(args.orbit_config.read_text())
    predictor=None
    direct=DirectFrameTrigger(pickup_config, required_frames=5)
    initial_rotation=EachObservationRotation(orbit_config,pickup_config)
    frame_epoch=ObservationFrameEpoch();epoch_revision=frame_epoch.revision
    from region_desktop import RegionDesktop
    region_ui=RegionDesktop(args.stage3_config)
    current_run=None
    link=serial.Serial();link.port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
    link.baudrate=115200;link.timeout=0;link.write_timeout=.2;link.dtr=link.rts=False;link.open()
    status=None;received=0.;buffer=b'';context=None;frame_count=0;sequence=0;last_sent=0.;packets=0
    window='原料区视觉 | 左：纠偏  右：抓取';window_ready=False
    cv2.namedWindow(window,cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)

    def process(frame,mode,calibration=None,context_unused=None):
        nonlocal status,received,buffer,context,frame_count,sequence,last_sent,packets,window_ready,current_run,epoch_revision
        now=time.monotonic();data=link.read(link.in_waiting)
        if data:
            buffer+=data
            if len(buffer)>65536:buffer=b''
            while b'\n' in buffer:
                line,buffer=buffer.split(b'\n',1)
                if region_ui.receive(line,time.monotonic()):
                    status=None;context=None;frame_epoch.invalidate()
                    gate.reset();direct.reset();initial_rotation.reset();current_run=None
                    continue
                try:latest=decode_status(line)
                except (ValueError,UnicodeError):continue
                received_at=time.monotonic()
                if not frame_epoch.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60):continue
                if status!=latest:print('MCU_STATUS '+json.dumps(latest),flush=True)
                region_ui.source_received()
                status=latest;received=received_at
        now=time.monotonic()
        metadata= camera_metadata(app.camera)
        region_ui.capture_boundary(metadata)
        if region_ui.region=='stage3':
            record,views=region_ui.process(frame,metadata,preview_only=args.preview_only)
            wire=record.pop('_wire',None)
            if record.pop('_snapshot',False):cv2.imwrite(str(args.session_dir/'views-current.jpg'),views)
            if wire is not None:
                if link.write(wire)!=len(wire):raise OSError('Observation UART short write')
                packets+=1
            cv2.setWindowTitle(window,'第三段视觉 | 左：纠偏  右：物料放置')
            cv2.imshow(window,views)
            if not window_ready:
                cv2.resizeWindow(window,960,400);cv2.moveWindow(window,20,70);window_ready=True
            if cv2.waitKey(1)&255 in (27,ord('q'),ord('Q')):raise KeyboardInterrupt
            return record,views
        cv2.setWindowTitle(window,'原料区视觉 | 左：纠偏  右：抓取')
        active=status is not None and now-received<=.60
        if not active:frame_epoch.invalidate()
        metadata=camera_metadata(app.camera)
        ready_frame=active and status['state']==6 and frame_epoch.frame(metadata,now)
        if epoch_revision!=frame_epoch.revision:
            direct.reset();initial_rotation.pause();epoch_revision=frame_epoch.revision
        if active and current_run!=status['run']:
            initial_rotation.reset();current_run=status['run']
            if rim is not None:rim.reset()
        key=(status['run'],status['state'],status['pose']) if active else None
        if key!=context:
            context=key;frame_count=0;sequence=0;gate.reset();direct.reset();initial_rotation.pause()
            if predictor is not None:predictor.pause()
        frame_count+=1;alignment=frame.copy();pickup=frame.copy();body=None;detected=None
        colors=[0,0,0];stopped=False;valid=False;prediction=None
        stamp=getattr(app.camera,'last_read_completed_time',None)
        observed=stamp if isinstance(stamp,(int,float)) and stamp<=now else None
        if active and status['state']==2:
            try:
                body=rim.estimate(frame) if rim is not None else estimate(frame,profile)
                body['observation_valid']=rim_observation_valid(body,3.0)
                valid=frame_count>=2 and body['observation_valid']
                if valid:initial_rotation.register_rim(body['center_px'])
                center=tuple(round(x-1) for x in body['center_px'])
                if body['observation_valid']:cv2.circle(alignment,center,round(body['radius_px']),(0,255,0),2)
                ref=tuple(round(x-1) for x in profile['reference_center_px'])
                cv2.drawMarker(alignment,ref,(0,255,255),cv2.MARKER_CROSS,30,3)
                text=f"Position error {body['error_norm_mm']:.1f}mm / 5mm"
                cv2.putText(alignment,text,(20,95),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
            except (ValueError,ArithmeticError):pass
        study=active and args.study_stopped_pose and status['state']==9
        if active and (ready_frame or study):
            if ready_frame and initial_rotation.previous is None and initial_rotation.branch is None:
                try:
                    ready_body=rim.estimate(frame) if rim is not None else estimate(frame,profile)
                    if rim_observation_valid(ready_body,3.0):initial_rotation.register_rim(ready_body['center_px'])
                except (ValueError,ArithmeticError):pass
            detected=app.material_pickup_detector.detect(frame,timestamp=now)
            value=capture.json_value(detected);colors=value['slot_colors']
            if not args.continuous_pick:
                stopped=gate.update(value,now,remaining=tuple(status['colors'][status['item']:]),
                    pose_id='mcu-observe',pose_sequence=status['pose'])
            valid=frame_count>=2
            if args.continuous_pick and status['state']==6 and valid:
                # Both gates observe every frame; do not short-circuit phase/stop history.
                target=status['colors'][status['item']]
                remaining=status['colors'][status['item']:]
                was_rotation_open=initial_rotation.rotation.opened
                rotation_ready=initial_rotation.update(value,observed,target,remaining)
                if initial_rotation.branch=='initial_stopped' and not was_rotation_open and initial_rotation.rotation.opened:direct.reset()
                stopped_prediction=direct.update(value,observed,status['colors'][status['item']],status['colors'][status['item']:])
                prediction=stopped_prediction if rotation_ready else None
                if prediction is not None:colors=prediction['colors']
            app.draw_material_pickup_detection(pickup,detected,app.material_pickup_detector.config)
        else:
            gate.reset()
            if predictor is not None:predictor.pause()
        title=STATES.get(status['state'],'Ready') if active else 'Waiting for robot'
        for image in (alignment,pickup):
            cv2.putText(image,title,(20,45),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
            if active:cv2.putText(image,f"Progress {status['item']}/3",(20,145),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        if active and status['state']==7:
            cv2.putText(pickup,'Recognition paused during motion',(20,95),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        send_phase=active and not args.preview_only and (status['state']==2 or (status['state']==6 and not args.observe_motion))
        if send_phase and observed is not None and time.monotonic()-last_sent>=.10:
            age=int((time.monotonic()-observed)*1000)
            if age<=200:
                right,forward=body['right_forward_mm'] if body is not None else (0,0)
                error=body['error_norm_mm'] if body is not None else 0
                wire=None
                if status['state']==2:
                    sequence+=1;wire=observation(status['run'],status['pose'],sequence,valid,right,forward,error,False,colors,age)
                elif args.continuous_pick:
                    if prediction is not None:
                        sequence+=1
                        delays=prediction['start_delay_ms']
                        wire=continuous_observation(status['run'],status['pose'],sequence,True,colors,age,prediction['target_color'],delays)
                        delays=prediction['start_delay_ms']
                        wire=continuous_observation(status['run'],status['pose'],sequence,True,colors,age,prediction['target_color'],delays)
                else:
                    sequence+=1;wire=observation(status['run'],status['pose'],sequence,valid,right,forward,error,stopped,colors,age)
                if wire is not None:
                    if link.write(wire)!=len(wire):raise OSError('Observation UART short write')
                    last_sent=time.monotonic();packets+=1
        if not active or status['state']!=6:initial_rotation.pause()
        learning=False  # Observe120 degrees once; no full-turn learning.
        views=compose(alignment,pickup,status,active=active,body=body,observe_motion=args.observe_motion,learning=learning,preview_only=args.preview_only,initial_rotation=initial_rotation.diagnostic())
        cv2.imshow(window,views)
        if not window_ready:
            cv2.resizeWindow(window,960,400);cv2.moveWindow(window,20,70);window_ready=True
        if frame_count==2:
            cv2.imwrite(str(args.session_dir/'views-current.jpg'),views)
        if cv2.waitKey(1)&255 in (27,ord('q'),ord('Q')):raise KeyboardInterrupt
        return dict(mode='M',mcu_status=status,body=body,detection=capture.json_value(detected),
            stop_confirmation=direct.diagnostic(),direct_frame_trigger=prediction,prediction_used=False,continuous_pick=args.continuous_pick,
            initial_rotation=initial_rotation.diagnostic(),
            frame_epoch=frame_epoch.diagnostic(),prearrival_motion='UNKNOWN',premotion_exception_enabled=False,
            observation_host_timestamp=observed,exposure_timestamp_verified=False,
            observe_motion_only=args.observe_motion,recognition_enabled=detected is not None,
            stationary=stopped,measurement_packets_sent=packets,motor_commands_sent=False),pickup

    capture.process=process
    print('CONFIG '+json.dumps({'body_profile_sha256':hashlib.sha256(args.body_profile.read_bytes()).hexdigest(),
        'material_config_sha256':hashlib.sha256(args.material_pickup_config.read_bytes()).hexdigest(),
        'motor_commands_sent':False,'mcu_status_rx_enabled':True,
        'observation_tx_enabled':not args.preview_only,'app_serial_link':'DISABLED_separate_from_status_rx'}),flush=True)
    sys.argv=[str(CODE/'source_observe.py'),'--mode','M','--duration','0','--session-dir',str(args.session_dir),
        '--material-pickup-config',str(args.material_pickup_config),
        '--material-color-config',str(CODE/'config/material_color_thresholds.json')]
    try:return capture.main()
    finally:link.close();cv2.destroyWindow(window)

if __name__=='__main__':raise SystemExit(main())
