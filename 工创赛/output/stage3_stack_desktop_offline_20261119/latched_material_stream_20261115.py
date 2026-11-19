"""Pose-scoped material observations only; never sends an actuator command."""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import socket

CODE = Path('/home/orangepi/p2-source-observer-20261106/vision_app/code')


def material_observation(detector, frame, *, control, now, to_json):
    """Never run the material detector outside a fresh OBSERVE control."""
    if control is None or control.get('state') != 'OBSERVE':
        return None, dict(state='PAUSED', calibrated=False, slot_colors=[0, 0, 0],
                          material_count=0, regions=[])
    detection = detector.detect(frame, timestamp=now)
    return detection, to_json(detection)


class RemainingStopped:
    """Use actual visible objects, including a real two/one-object remainder."""

    def __init__(self):
        self.history = deque(maxlen=2)
        self.context = None

    def reset(self):
        self.history.clear()
        self.context = None

    def update(self, detection, timestamp, *, remaining, pose_id, pose_sequence):
        context = (tuple(remaining), pose_id, pose_sequence)
        if self.context != context:
            self.reset()
            self.context = context
        if (not math.isfinite(timestamp) or not pose_id or
                (not remaining and value['state'] != 'ALIGN') or len(set(remaining)) != len(remaining) or
                any(type(c) is not int or not 1 <= c <= 6 for c in remaining)):
            self.reset()
            return False
        if self.history and timestamp <= self.history[-1][0]:
            self.reset()
            return False
        slots = tuple(detection.get('slot_colors', ()))
        regions = detection.get('regions', ())
        colors = [r['color_id'] for r in regions]
        if (not detection.get('calibrated') or len(slots) != 3 or not colors or
                len(colors) > len(remaining) or len(set(colors)) != len(colors) or
                not set(colors).issubset(remaining) or
                sorted(c for c in slots if c) != sorted(colors)):
            self.history.clear()
            return False
        centers = {r['color_id']: (float(r['center_x_px']), float(r['center_y_px'])) for r in regions}
        if not all(math.isfinite(v) for xy in centers.values() for v in xy):
            self.history.clear()
            return False
        self.history.append((timestamp, slots, centers))
        span = timestamp - self.history[0][0]
        if len(self.history) < 2 or span <= 0 or any(row[1] != slots for row in self.history):
            return False
        # Existing red/yellow stop evidence remains preferred when visible.
        # If only black remains its actual motion must also meet the same limit.
        tracked = [c for c in colors if c != 5] or colors
        speed = max(math.hypot(row[2][c][0] - centers[c][0], row[2][c][1] - centers[c][1]) / span
                    for row in self.history for c in tracked)
        return speed <= 25.0


def validated_control(value, now):
    try:
        remaining = value['remaining_colors']
        stamp = float(value['written_monotonic_s'])
        if (value['state'] not in ('OBSERVE', 'ALIGN') or not value['session_id'] or
                not value['pose_id'] or type(value['pose_sequence']) is not int or value['pose_sequence'] < 0 or
                not math.isfinite(stamp) or not 0 <= now - stamp <= .60 or
                (not remaining and value['state'] != 'ALIGN') or len(set(remaining)) != len(remaining) or
                any(type(c) is not int or not 1 <= c <= 6 for c in remaining)):
            return None
        return value
    except (ValueError, KeyError, TypeError):
        return None


def read_control(path, now):
    if path is None:
        return None
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
        return validated_control(value, now)
    except (OSError, ValueError, KeyError, TypeError):
        return None


class ControlBridge:
    """Loopback only; PC access is through the existing authenticated SSH tunnel."""
    def __init__(self, port):
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', port))
        self.server.listen(1)
        self.server.setblocking(False)
        self.client = None
        self.buffer = b''
        self.value = None

    def drop(self):
        if self.client is not None:
            self.client.close()
        self.client = None
        self.value = None
        self.buffer = b''

    def poll(self):
        try:
            client, _ = self.server.accept()
            self.drop()
            self.client = client
            client.setblocking(False)
        except BlockingIOError:
            pass
        if self.client is not None:
            try:
                data = self.client.recv(8192)
                if not data:
                    self.drop()
                else:
                    self.buffer += data
                    if len(self.buffer) > 65536:
                        self.drop()
                    else:
                        while b'\n' in self.buffer:
                            line, self.buffer = self.buffer.split(b'\n', 1)
                            value = json.loads(line)
                            value['written_monotonic_s'] = time.monotonic()
                            self.value = value
            except BlockingIOError:
                pass
            except (OSError, ValueError, TypeError):
                self.drop()
        return validated_control(self.value, time.monotonic())

    def publish(self, value):
        if self.client is not None:
            try:
                self.client.sendall((json.dumps(value, allow_nan=False) + '\n').encode())
            except (OSError, ValueError):
                self.drop()

    def close(self):
        self.drop()
        self.server.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--material-pickup-config', type=Path, required=True)
    parser.add_argument('--control-file', type=Path)
    parser.add_argument('--control-port', type=int)
    parser.add_argument('--send-material', action='store_true')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--session-dir', type=Path, required=True)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--body-profile', type=Path, required=True)
    args = parser.parse_args()
    if args.replay is not None and args.send_material:
        parser.error('saved-image replay must never send material observations')
    if args.send_material and args.control_file is None and args.control_port is None:
        parser.error('--send-material requires a fresh pose/status control file')
    if args.control_port is not None and not 1024 <= args.control_port <= 65535:
        parser.error('control port must be1024..65535')
    sys.path.insert(0, str(CODE))
    import cv2
    from source_body_registration import load_profile, estimate
    body_profile = load_profile(args.body_profile)
    import main as app
    from project_camera import DisabledSerialLink
    from material_pickup_detector import load_material_pickup_config
    app.TurntableSerialLink = app.DisabledSerialLink = DisabledSerialLink
    import commit_observe_unbounded_20261114 as observer
    gate = RemainingStopped()
    original = observer.process
    serial_tx = None
    packets = 0
    last_send = 0.0
    current_profile = None
    profile_since = 0.0
    profile_frames = 0
    motion_seen = False
    stop_since = None
    stop_sequence = 0
    bridge = ControlBridge(args.control_port) if args.control_port is not None else None
    window = 'Committed source pickup - observations'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(window, 960, 540)
    transport = dict(motor_commands_sent=False, serial_measurements_enabled=args.send_material,
                     pose_heartbeat_required=True, material_packets_sent=0, error=None)
    if args.send_material:
        import serial
        serial_tx = serial.Serial()
        serial_tx.port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        serial_tx.baudrate = 115200
        serial_tx.timeout = 0
        serial_tx.write_timeout = .2
        serial_tx.dtr = serial_tx.rts = False
        serial_tx.open()

    def process(frame, mode, calibration=None, context=None):
        nonlocal packets, last_send, current_profile, profile_since, motion_seen, stop_since, stop_sequence, profile_frames
        if mode != 'M':
            return original(frame, mode, calibration, context)
        now = time.monotonic()
        control = bridge.poll() if bridge is not None else read_control(args.control_file, now)
        active = control is not None
        config = Path(control.get('material_pickup_config', args.material_pickup_config)) if active else args.material_pickup_config
        profile = (str(config.resolve()), control['session_id'], control['pose_id'], control['pose_sequence']) if active else ('preview',)
        if profile != current_profile:
            document = load_material_pickup_config(config)
            app.material_pickup_detector.config = document
            app.material_pickup_detector.reset()
            gate.reset()
            current_profile = profile
            profile_since = now
            profile_frames = 0
            motion_seen = False
            stop_since = None
            transport['active_config'] = str(config)
            transport['active_config_sha256'] = hashlib.sha256(config.read_bytes()).hexdigest()
            print('COMMIT_POSE_CONFIG ' + json.dumps({'profile': profile, **transport}), flush=True)
        profile_frames += 1
        detection, value = material_observation(app.material_pickup_detector, frame,
            control=control, now=now, to_json=observer.json_value)
        recognition_enabled = detection is not None
        transport['material_detection_calls'] = transport.get('material_detection_calls', 0) + int(recognition_enabled)
        transport['recognition_paused_frames'] = transport.get('recognition_paused_frames', 0) + int(not recognition_enabled)
        stopped = False
        if active and control['state'] == 'OBSERVE':
            stopped = gate.update(value, now, remaining=tuple(control['remaining_colors']),
                                  pose_id=control['pose_id'], pose_sequence=control['pose_sequence'])
        else:
            gate.reset()
        settled = active and profile_frames >= 2
        if settled and not stopped and value.get('state') == 'MOVING':
            motion_seen = True
            stop_since = None
        if settled and stopped and stop_since is None:
            stop_since = now
            stop_sequence += 1
        stopped = stopped and settled
        if (serial_tx is not None and active and stopped and time.monotonic() - now < .20
                and 0 <= time.monotonic() - float(control['written_monotonic_s']) <= .60
                and time.monotonic() - last_send >= .10):
            packet = bytes([0xCC, 2, *value['slot_colors'], 0xDD])
            assert serial_tx.write(packet) == len(packet)
            last_send = time.monotonic()
            packets += 1
            print('COMMIT_MATERIAL_SENT ' + json.dumps({'colors': value['slot_colors'],
                  'pose_sequence': control['pose_sequence'], 'packets': packets}), flush=True)
        overlay = frame.copy()
        if recognition_enabled:
            app.draw_material_pickup_detection(overlay, detection, app.material_pickup_detector.config)
        label = f"{'RECOGNITION ON' if recognition_enabled else 'RECOGNITION PAUSED'} | {'TARGET READY' if stopped else ('SEARCHING' if recognition_enabled else 'PAUSED')} | {'POSE ACTIVE' if active else 'NO FRESH POSE'} | SENT {packets}"
        cv2.putText(overlay, label, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
        cv2.putText(overlay, 'NO ACTUATOR COMMANDS | Q CLOSES VIEW ONLY', (16, 1060),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
        cv2.imshow(window, overlay)
        if cv2.waitKey(1) & 0xff in (ord('q'), ord('Q'), 27):
            raise KeyboardInterrupt
        result = dict(mode='M', detection=value, stationary=stopped, pose_active=active,
                    recognition_enabled=recognition_enabled, recognition_paused=not recognition_enabled,
                    pose_id=control['pose_id'] if active else None,
                    pose_sequence=control['pose_sequence'] if active else None,
                    remaining_colors=control['remaining_colors'] if active else None,
                    measurement_packets_sent=packets, motor_trigger_sent=False,
                    motion_allowed=False)
        body_result = None
        if active and control['state'] == 'ALIGN' and settled:
            try:
                body_result = dict(available=True, **estimate(frame, body_profile))
            except (ValueError, ArithmeticError) as error:
                body_result = dict(available=False, error=str(error), motion_allowed=False)
        if bridge is not None:
            bridge.publish(dict(pose_active=active, stationary=stopped,
                pose_id=result['pose_id'], pose_sequence=result['pose_sequence'],
                frame_after_pose_ack=settled, motion_seen=motion_seen,
                colors=value['slot_colors'], material_count=value['material_count'],
                stop_sequence=stop_sequence, pause_age_s=now-stop_since if stop_since is not None else None,
                processing_age_s=time.monotonic()-now,
                client_sent_s=control.get('client_sent_s') if active else None,
                material_packets_sent=packets, body=body_result,
                recognition_enabled=recognition_enabled, recognition_paused=not recognition_enabled,
                material_detection_calls=transport.get('material_detection_calls', 0)))
        return result, overlay

    observer.process = process
    sys.argv = [str(CODE / 'source_observe.py'), '--mode', 'M', '--duration', str(args.duration),
                '--session-dir', str(args.session_dir), '--material-pickup-config', str(args.material_pickup_config),
                '--material-color-config', str(CODE / 'config/material_color_thresholds.json')]
    if args.replay is not None:
        sys.argv += ['--replay', str(args.replay)]
    try:
        return observer.main()
    except BaseException as error:
        transport['error'] = repr(error)
        raise
    finally:
        if serial_tx is not None:
            serial_tx.close()
        cv2.destroyWindow(window)
        if bridge is not None:
            bridge.close()
        transport['material_packets_sent'] = packets
        if args.session_dir.is_dir():
            (args.session_dir / 'commit-transport.json').write_text(json.dumps(transport, indent=2), encoding='utf-8')


if __name__ == '__main__':
    raise SystemExit(main())
