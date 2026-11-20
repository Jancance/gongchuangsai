"""Two camera views driven by MCU status; UART TX contains observations only."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import signal

CODE=Path(os.environ.get('VISION_APP_CODE','/home/orangepi/p2-source-observer-20261106/vision_app/code'))
STATES={0:'Ready',1:'Preparing',2:'Aligning',3:'Moving chassis',4:'Finishing alignment',
        5:'Alignment complete',6:'Finding next material',7:'Picking and placing',8:'Batch complete',9:'Stopped'}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage3-only',action='store_true')
    parser.add_argument('--full-map',action='store_true')
    parser.add_argument('--storage-config',type=Path)
    parser.add_argument('--storage-observation-config',type=Path)
    parser.add_argument('--material-color-config',type=Path,default=CODE/'config/material_color_thresholds.json')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--camera-size',nargs=2,type=int,metavar=('WIDTH','HEIGHT'))
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
    if args.full_map and (args.stage3_only or args.storage_config is None or
                          args.storage_observation_config is None or args.observe_motion):
        parser.error('full map requires both storage profiles and competition observation mode')
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
    if args.full_map:
        from protocol import parse_status
        from vision_station import Station, StationGate
        from storage_observation import StorageObservation
        from 颜色标定 import load_color_config
        station_gate=StationGate()
        storage_ui=RegionDesktop(args.storage_config)
        storage=StorageObservation(args.storage_observation_config,
                                   load_color_config(args.material_color_config))
        for ui in (region_ui,storage_ui):
            doc=json.loads(Path(ui.config).read_text())
            ui.warm((doc['camera']['width'],doc['camera']['height']))
    if args.stage3_only:region_ui.region='stage3'
    current_run=None
    link=serial.Serial();link.port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
    link.baudrate=115200;link.timeout=0;link.write_timeout=.2;link.dtr=link.rts=False;link.open()
    status=None;received=0.;buffer=b'';context=None;frame_count=0;sequence=0;last_sent=0.;packets=0
    source_wire_context=None
    last_callback=None
    window='原料区视觉 | 左：纠偏  右：抓取';window_ready=False
    if not args.headless:cv2.namedWindow(window,cv2.WINDOW_NORMAL|cv2.WINDOW_KEEPRATIO)

    def show(title,views):
        nonlocal window_ready
        if args.headless:return
        cv2.setWindowTitle(window,title)
        cv2.imshow(window,views)
        if not window_ready:
            cv2.resizeWindow(window,960,400);cv2.moveWindow(window,20,70);window_ready=True
        if cv2.waitKey(1)&255 in (27,ord('q'),ord('Q')):raise KeyboardInterrupt

    def first_valid(at):
        if station_gate.first_valid:return
        station_gate.first_valid=True
        station=station_gate.station
        print('STATION_FIRST_VALID '+json.dumps(dict(run=station.run,request=station.request,
              batch=station.batch,mode=station.mode,pose=station.pose,
              host_frame_delay_ms=round((at-station_gate.activated_at)*1000))),flush=True)

    def process(frame,mode,calibration=None,context_unused=None):
        nonlocal status,received,buffer,context,frame_count,sequence,last_sent,packets,current_run,epoch_revision,source_wire_context,last_callback
        now=time.monotonic();data=link.read(link.in_waiting)
        if args.full_map and (last_callback is None or now-last_callback>.2):
            # Serial has no sender timestamp. Wait for the next status pair
            # after draining bytes accumulated while the camera was unavailable.
            buffer=b'';data=b''
        last_callback=now
        if data:
            buffer+=data
            if len(buffer)>65536:buffer=b''
            while b'\n' in buffer:
                line,buffer=buffer.split(b'\n',1)
                received_at=time.monotonic()
                if args.full_map:
                    if line.startswith(b'VS1,'):
                        try:station=Station.parse(line)
                        except ValueError:
                            station_gate.paired=False
                            continue
                        if station_gate.announce(station,received_at):
                            status=None;context=None;frame_epoch.invalidate();frame_epoch.latest=None
                            frame_epoch.pending_reboot=False
                            gate.reset();direct.reset();initial_rotation.reset();current_run=None
                            region_ui.reset();storage_ui.reset();storage.reset()
                            if rim is not None:rim.reset()
                        continue
                    if line.startswith(b'S31,'):
                        try:s31=parse_status(line+b'\n')
                        except (ValueError,UnicodeError):continue
                        if station_gate.accept_status(False,s31.run,s31.pose,s31.state,received_at):
                            ui=region_ui if station_gate.station.mode==2 else storage_ui
                            ui.receive(line,received_at)
                        continue
                elif region_ui.receive(line,received_at):
                    status=None;context=None;frame_epoch.invalidate()
                    gate.reset();direct.reset();initial_rotation.reset();current_run=None
                    continue
                if args.stage3_only:continue
                try:latest=decode_status(line)
                except (ValueError,UnicodeError):continue
                if args.full_map and not station_gate.accept_status(
                        True,latest['run'],latest['pose'],latest['state'],received_at):continue
                if not frame_epoch.receive(latest,received_at,link_was_expired=status is not None and received_at-received>.60):continue
                if status!=latest:print('MCU_STATUS '+json.dumps(latest),flush=True)
                region_ui.source_received()
                status=latest;received=received_at
        now=time.monotonic()
        metadata=camera_metadata(app.camera)
        acquired_after=station_gate.acquisition(metadata,now) if args.full_map else None
        station=station_gate.station if args.full_map else None
        if args.full_map and not station_gate.active(now):
            show('比赛视觉 | 待机',frame)
            return dict(region='transit',recognition_enabled=False,motor_commands_sent=False),frame
        if not args.full_map:region_ui.capture_boundary(metadata)
        if (args.full_map and station.mode!=1) or (not args.full_map and region_ui.region=='stage3'):
            ui=storage_ui if args.full_map and station.mode in (3,4) else region_ui
            if args.full_map and acquired_after is None:
                ui.pipe.detector.reset()
                storage.reset_history()
                show('比赛视觉 | 等待工位新帧',frame)
                return dict(region=station.mode,recognition_enabled=False,motor_commands_sent=False),frame
            record,views=ui.process(frame,metadata,preview_only=args.preview_only,
                                    acquired_after=acquired_after,full_map=args.full_map,
                                    storage=bool(args.full_map and station.ring and station.color),
                                    headless=args.headless)
            wire=record.pop('_wire',None)
            if record.pop('_snapshot',False):cv2.imwrite(str(args.session_dir/'views-current.jpg'),views)
            if wire is not None:
                if link.write(wire)!=len(wire):raise OSError('Observation UART short write')
                packets+=1
            if args.full_map:
                if record['metric_candidate_valid']:first_valid(time.monotonic())
                if station.mode in (3,4):
                    record['region']='storage_flat' if station.mode==3 else 'storage_stack'
                response=storage.observe(station,ui.last_image,ui.last_result,
                                         ui.session.status.state,acquired_after,
                                         record['calibration_verified'] or
                                         record['alignment_response_ready'])
                if response is not None:
                    first_valid(time.monotonic())
                    record['ring_occupancy_wire']=response.decode('ascii').strip()
                    if not args.preview_only:
                        if link.write(response)!=len(response):raise OSError('Ring occupancy UART short write')
                        packets+=1
            title='第三段视觉 | 左：纠偏  右：物料放置'
            if args.full_map and station.mode in (3,4):
                title='暂存区视觉 | 平放' if station.mode==3 else '暂存区视觉 | 码垛'
            show(title,views)
            return record,views
        active=status is not None and now-received<=.60
        if not active:frame_epoch.invalidate()
        new_frame=not args.full_map or acquired_after is not None
        ready_frame=active and new_frame and status['state']==6 and frame_epoch.frame(metadata,now)
        if epoch_revision!=frame_epoch.revision:
            direct.reset();initial_rotation.pause();epoch_revision=frame_epoch.revision
        if active and current_run!=status['run']:
            initial_rotation.reset();current_run=status['run']
            if rim is not None:rim.reset()
        key=(status['run'],status['state'],status['pose']) if active else None
        if key!=context:
            context=key;frame_count=0;gate.reset();direct.reset();initial_rotation.pause()
            wire_context=(status['run'],status['pose']) if active else None
            if not args.full_map or (wire_context is not None and wire_context!=source_wire_context):
                source_wire_context=wire_context;sequence=0
            if predictor is not None:predictor.pause()
        frame_count+=1;alignment=frame.copy();pickup=frame.copy();body=None;detected=None
        colors=[0,0,0];stopped=False;valid=False;prediction=None
        stamp=getattr(app.camera,'last_read_completed_time',None)
        observed=acquired_after if args.full_map else (stamp if isinstance(stamp,(int,float)) and stamp<=now else None)
        if active and new_frame and status['state']==2:
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
            elapsed=(time.monotonic()-observed)*1000
            age=math.ceil(elapsed) if args.full_map else int(elapsed)
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
                else:
                    sequence+=1;wire=observation(status['run'],status['pose'],sequence,valid,right,forward,error,stopped,colors,age)
                if wire is not None:
                    if link.write(wire)!=len(wire):raise OSError('Observation UART short write')
                    last_sent=time.monotonic();packets+=1
                    if args.full_map and (valid if status['state']==2 else prediction is not None or stopped):
                        first_valid(last_sent)
        if not active or status['state']!=6:initial_rotation.pause()
        learning=False  # Observe120 degrees once; no full-turn learning.
        views=pickup if args.headless else compose(alignment,pickup,status,active=active,body=body,observe_motion=args.observe_motion,learning=learning,preview_only=args.preview_only,initial_rotation=initial_rotation.diagnostic())
        show('原料区视觉 | 左：纠偏  右：抓取',views)
        if frame_count==2:
            cv2.imwrite(str(args.session_dir/'views-current.jpg'),views)
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
        'hsv_config_path':str(args.material_color_config.resolve()),
        'hsv_config_sha256':hashlib.sha256(args.material_color_config.read_bytes()).hexdigest(),
        'full_map':args.full_map,
        'motor_commands_sent':False,'mcu_status_rx_enabled':True,
        'observation_tx_enabled':not args.preview_only,'app_serial_link':'DISABLED_separate_from_status_rx'}),flush=True)
    sys.argv=[str(CODE/'source_observe.py'),'--mode','M','--duration','0','--session-dir',str(args.session_dir),
        '--material-pickup-config',str(args.material_pickup_config),
        '--material-color-config',str(args.material_color_config)]
    if args.camera_size:
        sys.argv.extend(('--width',str(args.camera_size[0]),'--height',str(args.camera_size[1])))
    try:return capture.main()
    finally:
        link.close()
        if not args.headless:cv2.destroyWindow(window)

if __name__=='__main__':raise SystemExit(main())
