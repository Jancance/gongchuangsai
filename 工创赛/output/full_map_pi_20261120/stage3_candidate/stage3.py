#!/usr/bin/env python3
"""Independent third-stage capture, calibration records and S31/O31 bridge."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'vendor'))
import cv2
import numpy as np
from ring_station_detector import RingStationConfig, RingStationDetector
from stacked_ring_detector import (StackedRingDetector, load_stacked_ring_config,
    find_calibration_outer_ring, detect_scaled_reference_lines, make_search_calibration,
    camera_geometry_signature, draw_search_calibration)
from protocol import Session, parse_status


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_image(path):
    return cv2.imdecode(np.frombuffer(Path(path).read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)


def write_image(path, image):
    ok, data = cv2.imencode(Path(path).suffix, image)
    if not ok:
        raise OSError('Image encoding failed')
    Path(path).write_bytes(data.tobytes())


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def matrix(value, shape):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all() or np.linalg.matrix_rank(array) != shape[0]:
        raise ValueError(f'Need finite nonsingular {shape} matrix')
    return array


def angle_error_in_plane(affine, actual, reference):
    def angle(degrees):
        vector = affine @ np.array((math.cos(math.radians(degrees)), math.sin(math.radians(degrees))))
        return math.degrees(math.atan2(vector[1], vector[0]))
    # A real line has no arrow: reversing its endpoints is the same orientation.
    return (angle(actual) - angle(reference) + 90) % 180 - 90


def measured_motion(profile, result):
    a = matrix(profile['image_to_station_affine'], (2,2))
    reference = profile['reference']
    delta = a @ (np.asarray(result.middle_center_px)-np.asarray(reference['centers_px'][1]))
    yaw = angle_error_in_plane(a,result.station_yaw_deg,reference['yaw_deg'])
    residual = np.array((delta[0],delta[1],yaw))
    motion = matrix(profile['chassis_image_to_motion'],(3,3)) @ residual
    return tuple(motion), (float(np.linalg.norm(delta)),float(yaw))


def ring_config(profile, size):
    g = profile['geometry']
    d, spacing = float(g['outer_diameter_mm']), float(g['center_spacing_mm'])
    if d <= 0 or spacing <= 0:
        raise ValueError('Missing official nominal geometry')
    w, h = size
    reference = profile.get('reference')
    centers = reference['centers_px'] if reference else ((w*.25, h*.5), (w*.5, h*.5), (w*.75, h*.5))
    affine = profile.get('image_to_station_affine') or [[1, 0], [0, 1]]
    body = profile.get('chassis_image_to_motion')
    return RingStationConfig(spacing, d, 'left',
        60, 160, 40, 25., max(w, h)*.75, .25, 22., 5., 5, 4, .14, .07, 100, 2,
        float(reference['yaw_deg']) if reference else 0., tuple(map(tuple, centers)),
        tuple(map(tuple, affine)), .9, 3, 1.5, .35, (.85, 1.15), 2., 1.,
        tuple(map(tuple, body)) if body else None)


def stacked_config(profile):
    # Load the original parameter structure without importing its physical examples.
    from stacked_ring_detector import StackedRingConfig
    raw = read_json(ROOT / 'config/stacked_template.json')
    c, l = raw['middle_circle'], raw['reference_lines']
    return StackedRingConfig(c['search_half_width_px'], c['search_half_height_px'],
        c['maximum_reference_distance_px'], c['hough_dp'], c['hough_min_distance_px'],
        c['canny_high'], c['accumulator_threshold'], c['edge_canny_low'], c['edge_canny_high'],
        c['minimum_radius_px'], c['maximum_radius_px'], c['preferred_radius_px'],
        c['minimum_edge_support'], c['refinement_distance_px'], c['refinement_minimum_points'],
        c['refinement_minimum_arc_deg'], float(profile['geometry']['line_offset_mm']),
        l['band_half_height_px'], l['minimum_half_width_px'], l['station_span_width_factor'],
        l['hough_theta_step_deg'], l['hough_vote_threshold'], l['maximum_hough_candidates'],
        l['maximum_angle_deg'], l['maximum_target_distance_px'], l['refinement_distance_px'],
        l['minimum_edge_points'], l['minimum_x_span_px'], l['maximum_line_disagreement_deg'],
        profile.get('search_calibration'))


class Pipeline:
    def __init__(self, config, size):
        self.path = Path(config).resolve()
        self.profile = read_json(self.path)
        self.digest = sha(self.path)
        self.size = size
        camera = self.profile['camera']
        self.calibration = None
        if camera.get('calibration_file'):
            self.calibration = (self.path.parent / camera['calibration_file']).resolve()
        self.signature = camera_geometry_signature(*size, bool(camera['undistort']), self.calibration)
        self.maps = None
        if camera['undistort']:
            fs = cv2.FileStorage(str(self.calibration), cv2.FILE_STORAGE_READ)
            try:
                if (int(fs.getNode('image_width').real()), int(fs.getNode('image_height').real())) != size:
                    raise ValueError('Camera calibration size mismatch')
                k = fs.getNode('camera_matrix').mat()
                dist = fs.getNode('distortion_coefficients').mat()
                new = fs.getNode('new_camera_matrix').mat()
                self.maps = cv2.initUndistortRectifyMap(k, dist, None, new, size, cv2.CV_16SC2)
            finally:
                fs.release()
        self.ring = ring_config(self.profile, size)
        self.stack = stacked_config(self.profile)
        self.detector = StackedRingDetector(self.stack, self.ring)
        self.detector.set_camera_geometry(self.signature)

    def corrected(self, raw):
        if tuple(raw.shape[1::-1]) != self.size:
            raise ValueError('Frame geometry changed')
        return cv2.remap(raw, *self.maps, cv2.INTER_LINEAR) if self.maps is not None else raw.copy()

    def verified(self):
        p = self.profile
        if sha(self.path) != self.digest or p.get('calibration_verified') is not True:
            return False
        if not p.get('reference') or not p.get('search_calibration'):
            return False
        if p['camera'].get('width') != self.size[0] or p['camera'].get('height') != self.size[1]:
            return False
        try:
            matrix(p['image_to_station_affine'], (2, 2))
            matrix(p['chassis_image_to_motion'], (3, 3))
            reference = np.asarray(p['reference']['centers_px'], dtype=float)
            if reference.shape != (3,2) or not np.isfinite(reference).all():
                return False
            return (math.isfinite(float(p['reference']['yaw_deg']))
                    and all(p['search_calibration'][key] == self.signature[key]
                            for key in ('image_size','undistort_enabled','camera_fingerprint')))
        except (ValueError, TypeError, KeyError, OSError):
            return False

    def detect(self, raw):
        corrected = self.corrected(raw)
        # Without a search record the original hard-coded circle ROI is only a preview hint.
        if self.profile.get('search_calibration') is None:
            return corrected, None
        return corrected, self.detector.detect(corrected)


def audit(config):
    config = Path(config).resolve()
    return {'config_path': str(config), 'config_sha256': sha(config),
            'time_unix': time.time(), 'capture_timestamp_kind': 'host_read_completed_not_sensor_exposure'}


def save_capture(folder, raw, config, extra=None):
    folder = Path(folder).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    name = f'{time.time_ns()}'
    image = folder / (name + '-raw.png')
    write_image(image, raw)
    record = {**audit(config), 'raw_path': str(image), 'raw_sha256': sha(image),
              'image_size': list(raw.shape[1::-1]), **(extra or {})}
    write_json(folder / (name + '.json'), record)
    return record


def open_camera(args):
    if args.width is None or args.height is None:
        raise ValueError('Live capture requires explicit --width and --height; use actual source settings')
    device = int(args.device) if str(args.device).isdigit() else args.device
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError('Camera unavailable; stop source preview before stage3')
    return cap


def confirm_calibration(args):
    if not args.matrices or not args.tcp_axes_confirmed:
        raise ValueError('Need measured --matrices and explicit --tcp-axes-confirmed after real verification')
    p = read_json(args.config)
    if not p.get('reference') or not p.get('search_calibration'):
        raise ValueError('First record the actual target reference image and both real lines')
    measured = read_json(args.matrices)
    p['image_to_station_affine'] = matrix(measured['image_to_station_affine'], (2,2)).tolist()
    p['chassis_image_to_motion'] = matrix(measured['chassis_image_to_motion'], (3,3)).tolist()
    p['calibration_verified'] = True
    # Evidence is an audit record, not a new independent gate.
    p['verification_evidence'] = {'matrices_path':str(args.matrices.resolve()),
        'matrices_sha256':sha(args.matrices), 'time_unix':time.time(),
        'tcp_axes_confirmed_by_operator':True, 'axes':['right_positive','forward_positive','yaw_ccw_positive']}
    write_json(args.config,p)
    print(json.dumps(audit(args.config),ensure_ascii=False))


def estimate_affine(centers, ellipse, spacing_mm, diameter_mm):
    """Local A2 audited against MATLAB; image-y-positive sign convention."""
    from stacked_ring_detector import ellipse_shape_matrix
    centers = np.asarray(centers, dtype=float)
    if centers.shape != (3, 2) or not np.isfinite(centers).all():
        raise ValueError('Need three finite detected centers')
    if not math.isfinite(spacing_mm) or not math.isfinite(diameter_mm) or min(spacing_mm, diameter_mm) <= 0:
        raise ValueError('Need positive finite nominal dimensions')
    vx = (centers[2] - centers[0]) / (2 * spacing_mm)
    q = ellipse_shape_matrix(ellipse) / (diameter_mm * .5) ** 2 - np.outer(vx, vx)
    values, vectors = np.linalg.eigh((q + q.T) * .5)
    if not np.isfinite(values).all() or values[-1] <= 0:
        raise ValueError('Ellipse/spacing cannot establish local affine geometry')
    vy = vectors[:, -1] * math.sqrt(values[-1])
    if vy[1] < 0:
        vy = -vy
    affine = np.linalg.inv(matrix(np.column_stack((vx, vy)), (2, 2)))
    return affine, float(abs(values[0]) / values[-1])


def fit_response(commands, changes):
    """D=C*R.T; correction=-inv(R), with mm/mm/degree units."""
    c, d = np.asarray(commands, dtype=float), np.asarray(changes, dtype=float)
    if (c.ndim != 2 or c.shape[1] != 3 or d.shape != c.shape
            or not np.isfinite(c).all() or not np.isfinite(d).all()
            or np.linalg.matrix_rank(c) != 3):
        raise ValueError('Need finite independent right/forward/CCW command pairs (rank3)')
    response = np.linalg.lstsq(c, d, rcond=None)[0].T
    return response, -np.linalg.inv(matrix(response, (3, 3)))


def fit_calibration(args):
    if not args.measurements:
        raise ValueError('fit-calibration requires --measurements pairs.json')
    data = read_json(args.measurements)
    base = args.measurements.resolve().parent
    reference_image = (base / data['reference_image']).resolve()
    raw = read_image(reference_image)
    if raw is None:
        raise ValueError('Reference image missing')
    pipe = Pipeline(args.config,tuple(raw.shape[1::-1]))
    corrected = pipe.corrected(raw)
    triple = RingStationDetector(pipe.ring).detect(corrected)
    if not triple.detected or len(triple.centers_px)!=3:
        raise ValueError('A2 estimate requires three actually detected rings in reference_image')
    selection = tuple(data['middle_selection_px']) if data.get('middle_selection_px') else triple.centers_px[1]
    ellipse,_ = find_calibration_outer_ring(corrected,pipe.ring,selection)
    affine, rank_residual = estimate_affine(triple.centers_px, ellipse,
        pipe.ring.center_spacing_mm, pipe.ring.outer_diameter_mm)
    observations={}

    def observe(name):
        path=(base/name).resolve()
        if str(path) not in observations:
            raw_pair = read_image(path)
            if raw_pair is None:
                raise ValueError('Calibration pair image missing: ' + str(path))
            frame=pipe.corrected(raw_pair)
            e,_=find_calibration_outer_ring(frame,pipe.ring,selection,expected_axes=ellipse[1])
            o=detect_scaled_reference_lines(frame,e,pipe.stack,
                pipe.ring.outer_diameter_mm,pipe.profile['geometry']['line_offset_mm'],require_both=True)
            observations[str(path)]={'center_px':list(o['middle']),'yaw_deg':o['yaw'],
                                     'raw_sha256':sha(path),'raw_path':str(path)}
        return observations[str(path)]

    commands=[]; changes=[]; validation=[]
    for pair in data['pairs']:
        before,after=observe(pair['before']),observe(pair['after'])
        xy=affine@(np.asarray(after['center_px'])-np.asarray(before['center_px']))
        yaw=angle_error_in_plane(affine,after['yaw_deg'],before['yaw_deg'])
        command=np.asarray(pair['command_right_forward_ccw'],dtype=float)
        if command.shape!=(3,) or not np.isfinite(command).all():
            raise ValueError('Need finite three-axis measured command')
        entry=(command,np.asarray((xy[0],xy[1],yaw)))
        if pair.get('validation_only',False):validation.append(entry)
        else:commands.append(command);changes.append(entry[1])
    response, correction = fit_response(commands, changes)
    def residuals(entries):
        return [{'plane_mm':float(np.linalg.norm((delta-response@command)[:2])),
                 'yaw_deg':float((delta-response@command)[2])} for command,delta in entries]
    report={'image_to_station_affine':affine.tolist(), 'chassis_image_to_motion':correction.tolist(),
        'response_error_per_command':response.tolist(),
        'affine_initial_estimate_only':True,'ellipse_rank_residual':rank_residual,
        'fit_residuals':residuals(list(zip(commands,changes))),
        'independent_validation_residuals':residuals(validation),
        'independent_validation_present':bool(validation),'observations':observations,
        'reference_image_sha256':sha(reference_image), 'geometry':pipe.profile['geometry'],
        'calibration_verified':False,
        'note':'A2 from nominal circle/spacing is an initial estimate; residuals and real TCP reference need field acceptance. No auto-confirm.'}
    Path(args.output).mkdir(parents=True,exist_ok=True)
    path=Path(args.output)/'measured.json'
    write_json(path,report)
    print(json.dumps({'matrices':str(path.resolve()),'fit_pairs':len(commands),'validation_pairs':len(validation),'calibration_verified':False}))


def record_calibration(args):
    raw = read_image(args.image)
    if raw is None:
        raise ValueError('Cannot read calibration raw image')
    pipe = Pipeline(args.config, tuple(raw.shape[1::-1]))
    image = pipe.corrected(raw)
    selection = tuple(args.select) if args.select else None
    ellipse, support = find_calibration_outer_ring(image, pipe.ring, selection)
    observation = detect_scaled_reference_lines(image, ellipse, pipe.stack,
        pipe.profile['geometry']['outer_diameter_mm'], pipe.profile['geometry']['line_offset_mm'], require_both=True)
    pipe.profile['search_calibration'] = make_search_calibration(observation, pipe.signature,
        pipe.profile['geometry']['outer_diameter_mm'], pipe.profile['geometry']['line_offset_mm'])
    pipe.profile['camera'].update(width=raw.shape[1], height=raw.shape[0])
    triple = RingStationDetector(pipe.ring).detect(image)
    # A single image records a candidate. It never certifies stable calibration or TCP.
    if args.reference_confirmed:
        if not triple.detected or len(triple.centers_px) != 3:
            raise ValueError('Reference record needs three actually detected rings; use search-only record otherwise')
        pipe.profile['reference'] = {'centers_px': triple.centers_px,
            'yaw_deg': observation['yaw'], 'raw_sha256': sha(args.image),
            'note': 'Operator-declared reference candidate; physical TCP/axes validation still required'}
    if args.matrices:
        matrices = read_json(args.matrices)
        pipe.profile['image_to_station_affine'] = matrix(matrices['image_to_station_affine'], (2, 2)).tolist()
        pipe.profile['chassis_image_to_motion'] = matrix(matrices['chassis_image_to_motion'], (3, 3)).tolist()
    pipe.profile['calibration_verified'] = False
    pipe.profile['verification_evidence'] = None
    before = audit(args.config)
    write_json(args.config, pipe.profile)
    draw_search_calibration(image, observation)
    Path(args.output).mkdir(parents=True, exist_ok=True)
    write_image(Path(args.output) / 'search-preview.png', image)
    saved = save_capture(args.output, raw, args.config,
        {'operation': 'record_calibration', 'previous_config': before,
         'camera_signature': pipe.signature, 'outer_support': support,
         'search_observation': observation, 'calibration_verified': False})
    print(json.dumps(saved, ensure_ascii=False))


def run(args):
    capture = None
    serial = None
    pipe = None
    session = Session()
    frame_id = 0
    if args.command == 'bridge':
        import serial as serial_module
        serial = serial_module.Serial(args.serial, args.baud, timeout=0, write_timeout=1)
    buffer = bytearray()

    def poll():
        if serial is None:
            return
        buffer.extend(serial.read(serial.in_waiting))
        if len(buffer) > 4096:
            buffer.clear()
        while b'\n' in buffer:
            line, _, remaining = buffer.partition(b'\n')
            buffer[:] = remaining
            try:
                changed = session.accept(parse_status(bytes(line) + b'\n'), time.monotonic())
                if changed and pipe is not None:
                    pipe.detector.reset()
            except (ValueError, UnicodeError):
                pass

    Path(args.output).mkdir(parents=True, exist_ok=True)
    log = open(Path(args.output) / 'observations.jsonl', 'a', encoding='utf-8')
    try:
        if args.image:
            still = read_image(args.image)
            if still is None:
                raise ValueError('Cannot read --image')
        else:
            capture = open_camera(args)
            still = None
        while True:
            poll()
            acquisition_start = time.monotonic()
            if still is not None:
                raw = still.copy()
            else:
                ok, raw = capture.read()
                if not ok or raw is None:
                    raise RuntimeError('No fresh camera frame; no observation reused')
            captured_at = time.monotonic()
            poll()
            frame_id += 1
            if args.command == 'capture':
                print(json.dumps(save_capture(args.output, raw, args.config), ensure_ascii=False))
                break
            if pipe is None:
                pipe = Pipeline(args.config, tuple(raw.shape[1::-1]))
            preview, result = pipe.detect(raw)
            valid = bool(result is not None and result.detected and result.stable and pipe.verified())
            motion = (0.,0.,0.)
            # Residuals are independent of B3 motor response/sign/scaling.
            errors = (0.,0.)
            if valid:
                motion,errors=measured_motion(pipe.profile,result)
            wire = None
            if serial and session.status is not None:
                # Poll before encoding so age includes detection/transforms and status handling.
                context = session.status.context
                poll()
                if session.status.context == context:
                    wire = session.encode(frame_id, acquisition_start, captured_at,
                                          time.monotonic(), valid, motion, errors)
                    if wire:
                        serial.write(wire)
            if result is not None:
                cv2.drawMarker(preview, tuple(round(v) for v in result.middle_center_px), (0, 255, 0), cv2.MARKER_CROSS, 20, 2) if result.middle_center_px else None
                for a, b in result.line_segments_px:
                    cv2.line(preview, a, b, (0, 255, 0), 2)
            label = 'METRIC VERIFIED' if valid else 'PREVIEW ONLY / INVALID METRIC'
            state = session.status.state if session.status else None
            cv2.putText(preview, f'{label}  S31 state={state}', (12, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 220, 255), 2)
            entry = {**audit(args.config), 'frame_id': frame_id,
                'host_read_completed_monotonic': captured_at, 'result_stable': bool(result and result.stable),
                'metric_verified': pipe.verified(), 'motion_right_forward_yaw': list(motion),
                'observation': None if result is None else {
                    'detected': result.detected, 'middle_center_px': result.middle_center_px,
                    'line_yaw_deg': result.station_yaw_deg,
                    'top_line_angle_deg': result.top_line_angle_deg,
                    'bottom_line_angle_deg': result.bottom_line_angle_deg,
                    'line_quality': result.line_quality, 'circle_support': result.circle_support,
                    'detail': result.detail},
                'status': asdict(session.status) if session.status else None,
                'wire': wire.decode('ascii').strip() if wire else None}
            log.write(json.dumps(entry, ensure_ascii=False) + '\n'); log.flush()
            if args.image:
                write_image(Path(args.output) / 'preview.png', preview)
            if not args.headless:
                if frame_id == 1:
                    cv2.namedWindow('Stage3 - C capture / Q quit',cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('Stage3 - C capture / Q quit',960,540)
                cv2.imshow('Stage3 - C capture / Q quit', preview)
                key = cv2.waitKey(1) & 255
                if key in (ord('q'), 27):
                    break
                if key == ord('c'):
                    save_capture(args.output, raw, args.config, {'status': entry['status']})
            if still is not None or (args.frames and frame_id >= args.frames):
                break
    finally:
        log.close()
        if capture is not None:
            capture.release()
        if serial is not None:
            serial.close()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('preview', 'capture', 'record-calibration', 'fit-calibration', 'confirm-calibration', 'bridge'))
    parser.add_argument('--config', type=Path, default=ROOT / 'config/stage3.json')
    parser.add_argument('--image', type=Path)
    parser.add_argument('--device', default='/dev/v4l/by-id/usb-Generic_USB_CAMERA_EP.20CC54K01-video-index0')
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--fps', type=float, default=30.)
    parser.add_argument('--frames', type=int, default=0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT / 'records')
    parser.add_argument('--select', type=float, nargs=2, metavar=('X', 'Y'))
    parser.add_argument('--reference-confirmed', action='store_true')
    parser.add_argument('--matrices', type=Path)
    parser.add_argument('--measurements',type=Path)
    parser.add_argument('--tcp-axes-confirmed', action='store_true')
    parser.add_argument('--serial')
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    if args.command == 'bridge' and (not args.serial or args.image):
        parser.error('bridge requires --serial and a live camera; still images cannot feed motion')
    if args.command == 'record-calibration' and not args.image:
        parser.error('record-calibration requires --image raw.png')
    cv2.ocl.setUseOpenCL(False)
    if args.command == 'fit-calibration':
        fit_calibration(args)
    elif args.command == 'confirm-calibration':
        confirm_calibration(args)
    elif args.command == 'record-calibration':
        record_calibration(args)
    else:
        run(args)


if __name__ == '__main__':
    main()
