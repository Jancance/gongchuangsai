"""Preserve existing versions; prepare5mm corridor and true recognition pause."""
from pathlib import Path
import json
import shutil

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/integration18/window5-pause'
B=R/'tmp/source_top_extension150_20261115/project'
P=R/'tmp/source_top_window5_pause_20261115/project'


def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    assert json.loads((E.parent/'recognition-pause/matlab-result.json').read_text())['passed']
    assert not P.exists(),'Preserve existing candidates'
    shutil.copytree(B,P,ignore=shutil.ignore_patterns('Objects','Listings'))
    p=P/'USER/vehicle_parameters.h';data=p.read_bytes();lines=data.splitlines(keepends=True)
    found=0
    for i,line in enumerate(lines):
        if line.startswith(b'#define SOURCE_EXTENSION_CORRIDOR_DEG '):
            found+=1;ending=b'\r\n' if line.endswith(b'\r\n') else b'\n'
            lines[i]=b'#define SOURCE_EXTENSION_CORRIDOR_DEG (5.0f * VEHICLE_EXTENSION_PULSES_PER_MM * 360.0f / 3200.0f) /* User2026-11-15:5mm path window;arrival unchanged. */'+ending
    assert found==1;p.write_bytes(b''.join(lines))
    for name in ['body-registration-matlab.json','wheel-hold-rearm-matlab.json','parking-current-matlab.json','source_body_registration_field.json']:
        (E/name).write_bytes((E.parent/'ext150-clog3000'/name).read_bytes())
    s=(R/'output/simple_material_stream_20261114.py').read_text(encoding='utf-8')
    helper='''def material_observation(detector, frame, *, control, now, to_json):
    """Never run the material detector outside a fresh OBSERVE control."""
    if control is None or control.get('state') != 'OBSERVE':
        return None, dict(state='PAUSED', calibrated=False, slot_colors=[0, 0, 0],
                          material_count=0, regions=[])
    detection = detector.detect(frame, timestamp=now)
    return detection, to_json(detection)


'''
    assert s.count('class RemainingStopped:')==1
    s=s.replace('class RemainingStopped:',helper+'class RemainingStopped:',1)
    old="""        detection = app.material_pickup_detector.detect(frame, timestamp=now)
        value = observer.json_value(detection)"""
    new="""        detection, value = material_observation(app.material_pickup_detector, frame,
            control=control, now=now, to_json=observer.json_value)
        recognition_enabled = detection is not None
        transport['material_detection_calls'] = transport.get('material_detection_calls', 0) + int(recognition_enabled)
        transport['recognition_paused_frames'] = transport.get('recognition_paused_frames', 0) + int(not recognition_enabled)"""
    assert s.count(old)==1;s=s.replace(old,new,1)
    s=s.replace('        app.draw_material_pickup_detection(overlay, detection, app.material_pickup_detector.config)',
                '        if recognition_enabled:\n            app.draw_material_pickup_detection(overlay, detection, app.material_pickup_detector.config)',1)
    old='        label = f"COMMIT TOP | '
    assert s.count(old)==1
    s=s.replace(old,'        label = f"{\'RECOGNITION ON\' if recognition_enabled else \'RECOGNITION PAUSED\'} | ',1)
    s=s.replace("{'STOP' if stopped else 'WAIT'}", "{'TARGET READY' if stopped else ('SEARCHING' if recognition_enabled else 'PAUSED')}",1)
    s=s.replace("        result = dict(mode='M', detection=value, stationary=stopped, pose_active=active,",
                "        result = dict(mode='M', detection=value, stationary=stopped, pose_active=active,\n                    recognition_enabled=recognition_enabled, recognition_paused=not recognition_enabled,",1)
    s=s.replace("                material_packets_sent=packets, body=body_result))",
                "                material_packets_sent=packets, body=body_result,\n                recognition_enabled=recognition_enabled, recognition_paused=not recognition_enabled,\n                material_detection_calls=transport.get('material_detection_calls', 0)))",1)
    (R/'output/latched_material_stream_20261115.py').write_text(s,encoding='utf-8',newline='\n')
    s=(R/'scripts/source_commit_trial_20261114.py').read_text(encoding='utf-8')
    old="""    def inactive(self,sequence,remaining):
        # Best effort only. Camera loss does not cancel an accepted controller piece.
        self.exchange('MOVING','committed-motion',sequence,remaining)"""
    new="""    def inactive(self,sequence,remaining):
        # Send-only pause. No acknowledgement wait before the MCU command.
        control=dict(session_id=self.session,state='MOVING',pose_id='committed-motion',
          pose_sequence=sequence,remaining_colors=list(remaining),
          material_pickup_config=REMOTE+'material_pickup_top65_20261114.json',
          client_sent_s=time.monotonic())
        try:
            if self.peer is not None:
                self.peer.socket.sendall((json.dumps(control)+'\\n').encode())
            self.report.setdefault('recognition_pause_requests',[]).append(dict(sequence=sequence,send_only=True))
        except (OSError,ConnectionError) as error:
            self.report['camera_link_events'].append(str(error));self.close()"""
    assert s.count(old)==1;s=s.replace(old,new,1)
    old='                        try:sent=board.start_from_observation(current,value)'
    assert s.count(old)==1;s=s.replace(old,'                        vision.inactive(sequence,colors[item:])\n'+old,1)
    old='                        vision.inactive(sequence,colors[item:])\n                        # No camera reads/rechecks here.'
    assert s.count(old)==1;s=s.replace(old,'                        # No camera reads/rechecks here.',1)
    (R/'scripts/source_latched_trial_20261115.py').write_text(s,encoding='utf-8',newline='\n')
    print('Prepared independent5mm firmware and recognition-pause Python; no deployment or retry')


if __name__=='__main__':main()
