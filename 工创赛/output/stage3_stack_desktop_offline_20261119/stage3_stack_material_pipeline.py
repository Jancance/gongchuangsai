"""Offline three-base-material observation -> existing S31/O31; never open devices.

Desktop adapter contract: detect(raw), detector.reset(), alignment_ready(),
measured_motion(result), verified(). Select this adapter explicitly for stack mode;
do not replace the ordinary ring observation pipeline globally.
"""
from collections import deque
from dataclasses import dataclass
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if (Path(__file__).resolve().parent / 'stack_runtime').is_dir():
    ROOT = Path(__file__).resolve().parent / 'stack_runtime'
sys.path.insert(0, str(ROOT / 'vision_app/code'))
sys.path.insert(0, str(ROOT / 'src'))
import cv2
import numpy as np
from project_detector import analyze
from 颜色标定 import load_color_config
from smart_carrier.task_materials import resolve_preliminary_task


def protocol_module():
    path = ROOT / 'output/stage3_ring_pi_20261116/protocol.py'
    name = 'stage3_stack_existing_protocol'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


PROTOCOL = protocol_module()
STABLE_FRAMES = 5  # Existing user observation contract.
MAXIMUM_CENTER_JITTER_PX = 1.5  # Existing Stage3 detector value.
MAXIMUM_ANGLE_ERROR_DEG = 1.0  # Existing Stage3 yaw-before-translation split.
COLORS = {'red': 1, 'yellow': 2, 'blue': 3, 'green': 4, 'black': 5, 'lightblue': 6}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def matrix(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all() or np.linalg.matrix_rank(result) != shape[0]:
        raise ValueError('Need finite nonsingular material-plane matrix ' + str(shape))
    return result


@dataclass(frozen=True)
class MaterialStackView:
    detected: bool = False
    stable: bool = False
    centers_px: tuple = ()
    middle_center_px: tuple | None = None
    station_yaw_deg: float | None = None
    detail: str = 'UNREADY'
    line_segments_px: tuple = ()  # Materials provide the measured row; no empty-ring lines.


class StackMaterialPipeline:
    def __init__(self, config, size):
        self.path = Path(config).resolve()
        self.profile = json.loads(self.path.read_text(encoding='utf-8-sig'))
        if self.profile.get('mode') != 'storage-stack-materials' or self.profile.get('schema_version') != 1:
            raise ValueError('Explicit independent stack-material profile required')
        self.size = tuple(size)
        camera = self.profile['camera']
        if self.size != (camera['width'], camera['height']):
            raise ValueError('Camera dimensions do not match independent stack profile')
        self.digest = digest(self.path)
        self.color_path = (self.path.parent / self.profile['color_config']).resolve()
        self.color_config = load_color_config(self.color_path)
        self.color_digest = digest(self.color_path)
        self.maps = None
        self.camera_calibration = None
        if camera['undistort']:
            self.camera_calibration = (self.path.parent / camera['calibration_file']).resolve()
            self.camera_calibration_digest = digest(self.camera_calibration)
            fs = cv2.FileStorage(str(self.camera_calibration), cv2.FILE_STORAGE_READ)
            try:
                if (int(fs.getNode('image_width').real()), int(fs.getNode('image_height').real())) != self.size:
                    raise ValueError('Camera calibration dimensions changed')
                self.maps = cv2.initUndistortRectifyMap(fs.getNode('camera_matrix').mat(),
                    fs.getNode('distortion_coefficients').mat(), None,
                    fs.getNode('new_camera_matrix').mat(), self.size, cv2.CV_16SC2)
            finally:
                fs.release()
        self.expected = None
        order = self.profile.get('physical_image_order')
        self.image_order = None
        if order is not None:
            if (order.get('axis') not in ('x', 'y') or
                    sorted(order.get('physical_ring_order', [])) != [1, 2, 3]):
                raise ValueError('Need measured image axis and physical ring permutation')
            self.image_order = order
        if self.profile.get('task_code') is not None:
            mission = resolve_preliminary_task(self.profile['task_code'], self.profile['start_id'],
                ROOT / 'config/task_materials_preliminary_2027.json')
            colors = [0] * 3
            for target in mission.batch_1:
                colors[target.temporary_ring - 1] = target.color_code
            self.expected = tuple(colors)
        self.detector = self  # Same reset contract as RingRowPipeline.
        self.samples = deque(maxlen=STABLE_FRAMES)
        self.last_regions = []

    def reset(self):
        self.samples.clear()

    def verified(self):
        return False  # Offline code/command-response evidence is not physical acceptance.

    def alignment_ready(self):
        p = self.profile['material_plane']
        if (p.get('response_measured_on_material_plane') is not True or
                p.get('target_confirmed_by_user') is not True or self.expected is None or self.image_order is None):
            return False
        try:
            if digest(self.path) != self.digest or digest(self.color_path) != self.color_digest:
                return False
            if self.camera_calibration and digest(self.camera_calibration) != self.camera_calibration_digest:
                return False
            matrix(p['response_pixel_per_right_forward_ccw'], (3, 3))
            matrix(np.asarray(p['response_pixel_per_right_forward_ccw'])[:2, :2], (2, 2))
            matrix(p['nominal_image_to_station_affine'], (2, 2))
            target = np.asarray(p['target_center_px'], dtype=float)
            evidence = self.path.parent / p['calibration_evidence']
            return (target.shape == (2,) and np.isfinite(target).all() and
                    0 <= target[0] < self.size[0] and 0 <= target[1] < self.size[1] and
                    math.isfinite(float(p['target_row_yaw_deg'])) and
                    p['response_pixel_per_right_forward_ccw'][2][2] != 0 and evidence.is_file())
        except (OSError, ValueError, TypeError, KeyError, np.linalg.LinAlgError):
            return False

    def from_regions(self, regions):
        """Adapt only current observed silhouettes; never reconstruct missing bases."""
        self.last_regions = regions
        accepted = [r for r in regions if r.get('label') in COLORS]
        if self.expected is None:
            self.reset()
            return MaterialStackView(detail='UNREADY_TASK_CODE')
        if self.image_order is None:
            self.reset()
            return MaterialStackView(detail='UNREADY_PHYSICAL_RING_IMAGE_ORDER')
        actual = [COLORS[r['label']] for r in accepted]
        if len(regions) != 3 or len(actual) != 3 or sorted(actual) != sorted(self.expected):
            self.reset()
            return MaterialStackView(detail='WAIT_THREE_OBSERVED_BASE_MATERIALS')
        try:
            axis = 0 if self.image_order['axis'] == 'x' else 1
            spatial = sorted(accepted, key=lambda r: r['silhouette_center_px'][axis])
            positions = [r['silhouette_center_px'][axis] for r in spatial]
            if len(set(positions)) != 3:
                raise ValueError('Ambiguous physical ring order')
            for region, ring in zip(spatial, self.image_order['physical_ring_order']):
                if COLORS[region['label']] != self.expected[ring - 1]:
                    self.reset()
                    return MaterialStackView(detail='BASE_COLOR_AT_WRONG_PHYSICAL_RING')
        except (ValueError, KeyError, TypeError, IndexError):
            self.reset()
            return MaterialStackView(detail='INVALID_PHYSICAL_RING_IMAGE_ORDER')
        ordered = [accepted[actual.index(color)] for color in self.expected]
        try:
            centers = np.asarray([r['silhouette_center_px'] for r in ordered], dtype=float)
            if centers.shape != (3, 2) or not np.isfinite(centers).all():
                raise ValueError('centers')
            if not ((centers >= 0).all() and (centers < self.size).all()):
                raise ValueError('bounds')
            vector = centers[2] - centers[0]
            if np.linalg.norm(vector) == 0:
                raise ValueError('coincident centers')
        except (ValueError, KeyError, TypeError):
            self.reset()
            return MaterialStackView(detail='INVALID_CURRENT_MATERIAL_CENTERS')
        self.samples.append(centers)
        stable = (len(self.samples) == STABLE_FRAMES and
                  np.max(np.ptp(np.asarray(self.samples), axis=0)) <= MAXIMUM_CENTER_JITTER_PX)
        yaw = math.degrees(math.atan2(vector[1], vector[0]))
        return MaterialStackView(True, bool(stable), tuple(map(tuple, centers)), tuple(centers[1]),
            yaw, 'OBSERVED_MATERIAL_SILHOUETTES_STABLE' if stable else 'WAIT_FIVE_STABLE_FRAMES')

    def detect(self, raw):
        if tuple(raw.shape[1::-1]) != self.size:
            self.reset()
            raise ValueError('Camera geometry changed')
        corrected = cv2.remap(raw, *self.maps, cv2.INTER_LINEAR) if self.maps is not None else raw.copy()
        detection, _ = analyze(corrected, self.color_config, draw_overlay=False)
        return corrected, self.from_regions(detection['regions'])

    def measured_motion(self, result):
        if not self.alignment_ready() or not result.detected or not result.stable:
            raise ValueError('Independent material-plane response and stable three-base observation required')
        p = self.profile['material_plane']
        response = matrix(p['response_pixel_per_right_forward_ccw'], (3, 3))
        affine = matrix(p['nominal_image_to_station_affine'], (2, 2))
        pixel = np.asarray(result.middle_center_px) - p['target_center_px']
        row = (result.station_yaw_deg - p['target_row_yaw_deg'] + 90) % 180 - 90
        def angle(yaw):
            v = affine @ [math.cos(math.radians(yaw)), math.sin(math.radians(yaw))]
            return math.degrees(math.atan2(v[1], v[0]))
        yaw = (angle(result.station_yaw_deg) - angle(p['target_row_yaw_deg']) + 90) % 180 - 90
        if abs(yaw) > MAXIMUM_ANGLE_ERROR_DEG:
            motion = (0., 0., -row / response[2, 2])
        else:
            planar = -np.linalg.solve(response[:2, :2], pixel)
            motion = (float(planar[0]), float(planar[1]), 0.)
        return motion, (float(np.linalg.norm(affine @ pixel)), float(yaw))


class OfflineStackBridge:
    """One owner can call this API; it returns bytes and performs no serial I/O."""
    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.session = PROTOCOL.Session()

    def accept_status(self, wire, now):
        changed = self.session.accept(PROTOCOL.parse_status(wire), now)
        if changed:
            self.pipeline.reset()
        return changed

    def process(self, raw, frame_id, acquired_after, completed_at, now):
        # Reuse Session's existing timing contract for the whole stability window.
        # Duplicate/before-epoch/stale frames cannot contribute stable samples.
        s = self.session
        fresh = (s.status is not None and s.status.state == 2 and
                 frame_id > s.last_frame and acquired_after > s.fence and
                 0 <= now - s.last_status_at <= .2 and
                 0 <= completed_at - acquired_after <= now - acquired_after and
                 math.ceil((now - acquired_after) * 1000 - 1e-9) <= 200)
        if not fresh:
            self.pipeline.reset()
        _, result = self.pipeline.detect(raw)
        if not fresh:
            self.pipeline.reset()
        valid = fresh and result.detected and result.stable and self.pipeline.alignment_ready()
        motion, errors = self.pipeline.measured_motion(result) if valid else ((0., 0., 0.), (0., 0.))
        wire = self.session.encode(frame_id, acquired_after, completed_at, now, valid, motion, errors)
        return {'detected': result.detected, 'stable': result.stable,
                'centers_px': result.centers_px, 'reason': result.detail,
                'alignment_response_ready': bool(self.pipeline.alignment_ready()),
                'physical_calibration_verified': False, 'motion_allowed': False,
                'wire': None if wire is None else wire.decode('ascii'),
                'observation_valid': bool(wire and PROTOCOL.decode(wire)[4] == '1')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/stage3_stack_materials_offline_20261119.json')
    parser.add_argument('--sequence', type=Path, help='JSONL: image, frame_id, acquired_after, completed_at, now; optional status/status_at')
    parser.add_argument('--output', type=Path, default=ROOT / 'output/stage3_stack_vision_offline_20261119/report.json')
    args = parser.parse_args()
    profile = json.loads(args.config.read_text(encoding='utf-8-sig'))
    pipe = StackMaterialPipeline(args.config, (profile['camera']['width'], profile['camera']['height']))
    bridge = OfflineStackBridge(pipe)
    rows = []
    if args.sequence:
        for line in args.sequence.read_text(encoding='utf-8-sig').splitlines():
            record = json.loads(line)
            if 'status' in record:
                bridge.accept_status(record['status'].encode('ascii'), record['status_at'])
            image = (args.sequence.parent / record['image']).resolve()
            raw = cv2.imdecode(np.frombuffer(image.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            if raw is None:
                raise ValueError('Unreadable local image: ' + str(image))
            rows.append({'image': str(image), 'image_sha256': digest(image),
                **bridge.process(raw, **{k: record[k] for k in ('frame_id', 'acquired_after', 'completed_at', 'now')})})
    report = {'offline_only': True, 'devices_opened': [], 'config_path': str(args.config.resolve()),
        'config_sha256': pipe.digest, 'color_config_path': str(pipe.color_path),
        'color_config_sha256': pipe.color_digest, 'alignment_response_ready': bool(pipe.alignment_ready()),
        'physical_calibration_verified': False, 'observations': rows,
        'missing_calibration_is_expected': not pipe.alignment_ready(),
        'stack_height_subtractions_in_host': 0, 'status': 'OFFLINE_CANDIDATE'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k != 'observations'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
