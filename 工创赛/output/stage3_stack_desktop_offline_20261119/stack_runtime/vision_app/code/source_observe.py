"""Bounded A/M observation capture. Reuses the existing application and configs."""
from pathlib import Path
import argparse
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math
import sys
import time
import cv2
import numpy as np
import main as app
from source_metric_preview import preview_rim


def json_value(value):
    if is_dataclass(value):
        return json_value(asdict(value))
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return str(value)


def write_image(path, frame):
    ok, encoded = cv2.imencode('.png', frame)
    if not ok:
        raise RuntimeError('failed to encode captured image')
    encoded.tofile(path)


def process(frame, mode, calibration=None, context=None):
    """Same detector instances as the main app, with no reporting side effects."""
    if mode == 'A':
        detection = app.turntable_detector.detect(frame)
        points = np.empty((0, 2))
        if detection.detected and app.turntable_detector.seed is not None:
            # Read raw boundary evidence, never synthesize it from a fitted circle.
            points, _ = app.turntable_detector._sample_boundary_points(frame, app.turntable_detector.seed)
        metric = preview_rim(points, calibration, frame_context=context,
                             stable=bool(detection.stable), confidence=float(detection.confidence),
                             timing_verified=False)
        overlay = frame.copy()
        app.draw_turntable_detection(overlay, detection)
        return {'mode': 'A', 'detection': json_value(detection), 'boundary_points_px': points.tolist(),
                'metric_preview': metric, 'motion_allowed': False}, overlay
    detection = app.material_pickup_detector.detect(frame, timestamp=time.monotonic())
    return {'mode': 'M', 'detection': json_value(detection), 'motion_allowed': False,
            'note': 'observation only; no grasp, inventory, task remainder inference or success generated'}, frame.copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['A', 'M'], default='A')
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--frames', type=int, default=0, help='0: bounded by duration')
    parser.add_argument('--replay', type=Path, help='single saved frame; does not open camera')
    parser.add_argument('--metric-profile', type=Path)
    parser.add_argument('--context', type=Path, help='current observed/declared context; never copied from calibration implicitly')
    parser.add_argument('--session-dir', type=Path)
    parser.add_argument('--check', action='store_true')
    opts, rest = parser.parse_known_args()
    if not 0 < opts.duration <= 120 or opts.frames < 0:
        parser.error('duration must be0..120sec and frames nonnegative')
    sys.argv = [str(Path(app.__file__)), *rest]
    args = app.parse_args()
    app.initialize(args)
    if not type(app.serial_link).__name__ == 'DisabledSerialLink':
        raise RuntimeError('observation requires disabled serial')
    calibration = json.loads(opts.metric_profile.read_text(encoding='utf-8')) if opts.metric_profile else None
    context = json.loads(opts.context.read_text(encoding='utf-8')) if opts.context else None
    configs = {}
    for key in ['camera_processing_config', 'turntable_config', 'turntable_material_seed_config',
                'material_pickup_config', 'material_color_config']:
        p = getattr(args, key)
        configs[key] = {'path': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
    report = {'schema_version': 1, 'serial': 'DISABLED', 'motion_allowed': False,
              'camera_opened': False, 'timing_source': 'host_read_completion_only',
              'sensor_exposure_age_verified': False, 'configs': configs,
              'mode': opts.mode, 'frames': 0, 'error': None}
    if opts.check:
        print(json.dumps(report, ensure_ascii=False))
        return 0
    out = opts.session_dir or Path(__file__).parent / 'diagnostics' / time.strftime('source-observe-%Y%m%d-%H%M%S')
    out.mkdir(parents=True, exist_ok=False)
    source = None
    start = time.monotonic()
    last_print = 0
    last_raw = last_overlay = None
    try:
        if opts.replay:
            source = cv2.imdecode(np.fromfile(opts.replay, dtype=np.uint8), cv2.IMREAD_COLOR)
            if source is None or source.shape[:2] != (args.height, args.width):
                raise RuntimeError('replay image must match actual configured1920x1080')
        with (out / 'observations.jsonl').open('w', encoding='utf-8') as log:
            while time.monotonic()-start < opts.duration:
                if opts.frames and report['frames'] >= opts.frames:
                    break
                frame = source.copy() if source is not None else app.read_camera()
                read_at = time.monotonic()
                if source is None:
                    report['camera_opened'] = report['camera_opened'] or app.camera is not None
                if frame is None:
                    time.sleep(.03)
                    continue
                raw = frame
                processed = cv2.remap(raw, app.map1, app.map2, cv2.INTER_LINEAR) if app.undistort_enabled else raw
                result, overlay = process(processed, opts.mode, calibration, context)
                report['frames'] += 1
                result.update(frame_sequence=report['frames'], host_read_completed_monotonic_s=read_at,
                              processed_after_host_read_ms=(time.monotonic()-read_at)*1000,
                              sensor_exposure_age_verified=False, replay=source is not None)
                log.write(json.dumps(json_value(result), ensure_ascii=False, allow_nan=False)+'\n')
                log.flush()
                if report['frames'] == 1:
                    write_image(out / 'first-raw.png', raw)
                last_raw, last_overlay = raw.copy(), overlay
                report['last'] = result
                if time.monotonic()-last_print >= 1:
                    print(json.dumps({'mode': opts.mode, 'frames': report['frames'],
                                      'metric': result.get('metric_preview', {}).get('status'),
                                      'serial': 'DISABLED', 'motion_allowed': False}), flush=True)
                    last_print = time.monotonic()
                if source is not None and not opts.frames:
                    break
        if not report['frames']:
            report['error'] = 'NO_FRAME_RECEIVED'
    except KeyboardInterrupt:
        report['stopped_by_user'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        app.serial_link.close()
        app.close_camera()
        if last_raw is not None:
            write_image(out / 'last-raw.png', last_raw)
            write_image(out / 'last-overlay.png', last_overlay)
        (out / 'session.json').write_text(json.dumps(json_value(report), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print('Saved:', out)
    return 1 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
