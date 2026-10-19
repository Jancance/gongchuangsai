"""记录集成F预览的失检与换号证据；不改变阈值、跟踪或执行锁。"""
import argparse
from collections import Counter
import copy
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED

import cv2
import numpy as np


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class FollowRecorder:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.log = (self.folder / 'observations.jsonl').open('w', encoding='utf-8')
        self.previous = None
        self.previous_frame = None
        self.last_red_ids = None
        self.lost = False
        self.saved = set()
        self.frames = 0
        self.states = Counter()
        self.transitions = Counter()
        self.closed = False
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='evidence-writer')
        self.pending = []
        self.loss_started = None
        self.longest_loss_s = 0.
        self.longest_evidence = None

    def write_json(self, path, value):
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                   default=json_value), encoding='utf-8')

    def write_image(self, path, frame):
        ok, encoded = cv2.imencode('.png', frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise RuntimeError('Diagnostic image encoding failed')
        encoded.tofile(path)

    def event(self, name, frame, row):
        self.transitions[name] += 1
        if name in self.saved:
            return
        previous = None if self.previous_frame is None else self.previous_frame.copy()
        current = frame.copy()
        metadata = copy.deepcopy({'event': name, 'previous': self.previous, 'current': row,
            'note': 'Raw frames before drawing. Event is evidence, not a root-cause diagnosis.'})
        self.pending.append(self.writer.submit(self.save_event,name,previous,current,metadata))
        self.saved.add(name)
        self.log.flush()

    def save_event(self,name,previous,current,metadata):
        if previous is not None:
            self.write_image(self.folder / (name + '-before-raw.png'), previous)
        self.write_image(self.folder / (name + '-raw.png'), current)
        self.write_json(self.folder / (name + '.json'), metadata)
        print('Captured:', self.folder / name, flush=True)

    def observe(self, frame, result, started, finished, tracker=None):
        for future in self.pending:
            if future.done():
                future.result()
        # 在主程序绘图前取证，不能把上一帧的检测或预测点合入原图。
        row = copy.deepcopy(result)
        self.frames += 1
        row.update(frame_index=self.frames, detector_call_ms=(finished-started)*1000,
                   result_host_age_ms=(finished-result['timestamp'])*1000,
                   recorded_monotonic=finished, last_red_ids_before_frame=self.last_red_ids)
        if tracker is not None:
            row['internal_tracks_not_actionable'] = copy.deepcopy(list(tracker.tracks.values()))
        self.states[row['state']] += 1
        ids = tuple(sorted(t['id'] for t in row['targets'] if t['color'] == 'red'))
        old = tuple(sorted(t['id'] for t in (self.previous or {}).get('targets', [])
                           if t['color'] == 'red'))
        if ids and self.last_red_ids is None:
            self.event('first-red-visible', frame, row)
        if old and not ids:
            self.event('first-red-loss', frame, row)
            self.lost = True
            self.loss_started = row['timestamp']
        if ids:
            if self.lost:
                self.event('first-red-reappearance', frame, row)
                duration = row['timestamp'] - self.loss_started
                if duration > self.longest_loss_s:
                    self.longest_loss_s = duration
                    metadata = copy.deepcopy({'event':'longest-red-loss',
                        'previous':self.previous,'current':row,'loss_duration_s':duration,
                        'note':'Previous raw is last missing frame; current raw is reappearance.'})
                    # 只保留最长段两张原图，退出采集后再写，避免增加实时写图负担。
                    self.longest_evidence = (self.previous_frame.copy(),frame.copy(),metadata)
                self.lost = False
            if self.last_red_ids is not None and ids != self.last_red_ids:
                self.event('first-red-id-change', frame, row)
            self.last_red_ids = ids
        row['diagnostic_io_ms'] = (time.monotonic()-finished)*1000
        self.log.write(json.dumps(row, ensure_ascii=False, default=json_value) + '\n')
        if self.frames % 12 == 0:
            self.log.flush()
        self.previous = row
        self.previous_frame = frame.copy()
        # 原检测返回值保持不变，不延长150ms门，不复用丢失目标。

    def close(self, exit_code=None, error=None):
        if self.closed:
            return
        self.closed = True
        self.log.close()
        self.writer.shutdown(wait=True)
        save_failure = None
        for future in self.pending:
            try:
                future.result()
            except Exception as exc:
                error = 'Evidence save failed: '+str(exc)
                exit_code = 1
                save_failure = error
        if self.longest_evidence is not None:
            try:
                self.save_event('longest-red-loss',*self.longest_evidence)
            except Exception as exc:
                error = 'Evidence save failed: '+str(exc)
                exit_code = 1
                save_failure = error
        self.write_json(self.folder / 'run.json', {
            'frames': self.frames, 'states': dict(self.states),
            'transition_counts': dict(self.transitions), 'saved_events': sorted(self.saved),
            'longest_red_loss_s':self.longest_loss_s,
            'exit_code': exit_code, 'error': error,
            'serial': 'DISABLED', 'motion_allowed': False,
            'scope': 'Desktop pixel evidence; images encoded in a bounded background writer.'})
        bundle = self.folder.with_suffix('.zip')
        with ZipFile(bundle, 'x', ZIP_DEFLATED) as archive:
            for path in sorted(self.folder.iterdir()):
                if path.is_file():
                    archive.write(path, path.name)
        print('Run saved:', self.folder, flush=True)
        print('Evidence ZIP:', bundle, flush=True)
        if save_failure:
            raise RuntimeError(save_failure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path,
                        default=Path('/home/orangepi/p2-integrated-20261003/vision_app'))
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    code = args.app.resolve() / 'code'
    if not (code / 'main.py').is_file():
        raise RuntimeError('Integrated app not found: ' + str(code))
    sys.path.insert(0, str(code))
    workbench = importlib.import_module('main')
    sys.argv = [str(code / 'main.py')] + (['--check'] if args.check else [])
    if args.check:
        return workbench.main()
    parent = args.output_dir or args.app.parent / 'follow-audits'
    recorder = FollowRecorder(parent / datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    module_hashes = {}
    for name in ('project_detector.py','project_follow.py','project_tracker.py','project_camera.py'):
        module_hashes[name] = hashlib.sha256((code/name).read_bytes()).hexdigest()
    module_hashes['diagnostic_entry'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    recorder.write_json(recorder.folder/'module_source_hashes.json',module_hashes)
    for name in ('material_color_thresholds.json', 'project_profile.json', 'camera_processing.json'):
        source = code / 'config' / name
        data = source.read_bytes()
        (recorder.folder / name).write_bytes(data)
        recorder.write_json(recorder.folder / (name + '.source.json'), {
            'file': str(source), 'sha256': hashlib.sha256(data).hexdigest()})
    follower = workbench.MaterialFollower
    original = follower.detect

    def audited_detect(instance, frame, timestamp=None):
        started = time.monotonic()
        result = original(instance, frame, timestamp=timestamp)
        finished = time.monotonic()
        recorder.observe(frame, result, started, finished, instance.tracker)
        return result

    follower.detect = audited_detect
    exit_code = None
    error = None
    print('Diagnostic folder:', recorder.folder, flush=True)
    print('Keep F mode. Rest red object for 5s, gently touch/push once, release; Q quit.', flush=True)
    try:
        exit_code = workbench.main()
        return exit_code
    except BaseException as exc:
        error = type(exc).__name__ + ': ' + str(exc)
        raise
    finally:
        follower.detect = original
        recorder.close(exit_code, error)


if __name__ == '__main__':
    raise SystemExit(main())
