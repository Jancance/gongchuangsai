"""离线回归blue2并打包；不操作相机、GUI、串口或执行器。"""
from collections import Counter
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time
from zipfile import ZipFile, ZIP_DEFLATED

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic135_perf1_20261026/vision_app'
APP = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app'
EVIDENCE = ROOT / 'evidence/p2-camera/2026-10-26/blue2'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    EVIDENCE.mkdir(exist_ok=True)
    cv2.setNumThreads(8)
    sys.path.insert(0, str(APP / 'code'))
    old = load('blue2_validation_old', BASE / 'code/project_detector.py')
    new = load('blue2_validation_new', APP / 'code/project_detector.py')
    read = lambda p: json.loads(p.read_text(encoding='utf-8'))
    cfg = read(APP / 'code/config/material_color_thresholds.json')
    original = read(BASE / 'code/config/material_color_thresholds.json')
    off = copy.deepcopy(cfg)
    off['blue_projected_silhouette_candidate']['solidity94_candidate'] = False
    config_only_additions = copy.deepcopy(cfg)
    for key in ('solidity94_candidate', 'solidity94_note'):
        config_only_additions['blue_projected_silhouette_candidate'].pop(key)
    assert config_only_additions == original
    files = []
    for folder in ('blue-pair-diagnostic', 'blue1-board-diagnostic', 'perf1-board-diagnostic', 'rejection-audit'):
        files.extend((folder + '/' + p.name, p.read_bytes()) for p in
                     (ROOT / 'evidence/p2-camera/2026-10-26' / folder).glob('*raw.png'))
    for archive in ('20261024-121400-134643', '20261024-120512-675071', '20261024-114659-072282',
                    '20261024-112215-162612', '20261024-111235-577533', '20261024-104324-197600',
                    '20261024-105943-475482', '20261007-000604-409723'):
        with ZipFile(ROOT / 'evidence' / (archive + '.zip')) as z:
            files.extend((archive + '/' + name, z.read(name)) for name in z.namelist() if name.endswith('-raw.png'))
    files.extend((p.name, p.read_bytes()) for p in
                 (ROOT / 'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
    hints = [{'color': 'blue', 'bbox_px': [348, 511, 265, 170], 'center_px': [480, 596],
              'proposal_source': 'projected_blue_component'},
             {'color': 'lightblue', 'bbox_px': [1274, 524, 237, 155], 'center_px': [1388, 604]}]
    report = {'motion_allowed': False, 'hardware_acceptance': False, 'changed_results': [], 'checks': {}}
    seen = set()
    for name, data in files:
        digest = hashlib.sha256(data).hexdigest()
        if digest in seen:
            continue
        seen.add(digest)
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), 1)
        for prior in (None, hints):
            a, _ = old.analyze(frame, original, prior_regions=prior, draw_overlay=False)
            disabled, _ = new.analyze(frame, off, prior_regions=prior, draw_overlay=False)
            assert a == disabled, ('disabled path changed', name)
            b, _ = new.analyze(frame, cfg, prior_regions=prior, draw_overlay=False)
            if a != b:
                before = Counter(r['label'] for r in a['regions'] if r['label'] != 'UNKNOWN')
                after = Counter(r['label'] for r in b['regions'] if r['label'] != 'UNKNOWN')
                assert all(after[k] >= v for k, v in before.items()), ('known lost', name)
                added = [r for r in b['regions'] if r not in a['regions']]
                assert added and all(r['label'] == 'blue' and r['proposal_source'] == 'projected_blue_component'
                                     and .94 <= r['solidity'] < .95 for r in added), (name, added)
                assert [r for r in b['regions'] if r not in added] == a['regions'], ('other regions changed', name)
                assert b['circle_diagnostics'] == a['circle_diagnostics'], ('diagnostics changed', name)
                report['changed_results'].append({'file': name, 'history': prior is not None,
                                                  'before': dict(before), 'after': dict(after), 'added': added})
        if name.endswith('first-nonstale-blue-missing-raw.png'):
            result, overlay = new.analyze(frame, cfg, draw_overlay=True)
            assert sorted(r['label'] for r in result['regions'] if r['label'] != 'UNKNOWN') == ['blue', 'lightblue']
            cv2.imencode('.png', overlay)[1].tofile(EVIDENCE / 'recovered-blue-overlay.png')
    assert len(seen) >= 188
    report['unique_real_images'] = len(seen)
    report['checks']['all_images_disabled_full_results_exact'] = True
    report['checks']['all_images_no_known_color_loss'] = True
    for color in ((0, 0, 230), (0, 220, 220), (220, 0, 0), (0, 180, 0), (15, 15, 15), (230, 190, 120)):
        frame = np.full((480, 640, 3), 240, np.uint8)
        cv2.circle(frame, (320, 240), 65, color, -1)
        for config in (cfg, None):
            a, oa = old.analyze(frame, config, draw_overlay=True)
            b, ob = new.analyze(frame, config, draw_overlay=True)
            assert a == b
            np.testing.assert_array_equal(oa, ob)
    report['checks']['six_color_synthetic_and_unconfigured_overlays_exact'] = True
    from project_follow import MaterialFollower
    stale = MaterialFollower(cfg).detect(frame, timestamp=time.monotonic() - .2)
    assert stale['state'] == 'STALE_FRAME' and not stale['targets'] and not stale['motion_allowed']
    report['checks']['150ms_input_gate'] = True
    calls = [('unit', [sys.executable, '-X', 'utf8', '-m', 'unittest', 'discover', '-s', str(ROOT / 'tests'), '-p', 'test_generic135_blue2.py']),
             ('check', [sys.executable, '-X', 'utf8', str(APP / 'run.py'), '--check'])]
    sample = ROOT / 'evidence/p2-camera/2026-10-26/rejection-audit/first-nonstale-blue-missing-raw.png'
    for mode in ('follow', 'turntable', 'material', 'ring', 'stack', 'measurement'):
        args = [sys.executable, '-X', 'utf8', str(APP / 'run.py'), '--image', str(sample), '--mode', mode,
                '--output-dir', str(EVIDENCE / mode)]
        if mode == 'measurement':
            args += ['--measure-points', '500', '620', '1400', '620']
        calls.append(('offline-' + mode, args))
    for feature in ('color', 'turntable-settings', 'pickup-regions', 'stack-lines'):
        calls.append(('help-' + feature, [sys.executable, '-X', 'utf8', str(APP / 'run.py'), '--feature', feature, '--help']))
    for label, args in calls:
        proc = subprocess.run(args, capture_output=True, text=True, encoding='utf-8', timeout=60)
        (EVIDENCE / (label + '.log')).write_text(proc.stdout + proc.stderr, encoding='utf-8')
        assert proc.returncode == 0, (label, proc.stdout, proc.stderr)
        report['checks'][label] = True
    manifest = read(APP / 'BLUE2_MANIFEST.json')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    for name, digest in manifest['source_files'].items():
        assert sha(BASE / name) == digest
        if name not in manifest['changed_files']:
            assert sha(APP / name) == digest
    for name, digest in manifest['candidate_files'].items():
        assert sha(APP / name) == digest
    report['checks']['only_detector_and_candidate_config_changed'] = True
    archive = APP.parent.with_suffix('.zip')
    with ZipFile(archive, 'x', ZIP_DEFLATED) as z:
        for p in sorted(APP.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc':
                z.write(p, p.relative_to(APP.parent).as_posix())
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            assert z.read(name) == (APP.parent / name).read_bytes()
    report['checks']['zip_crc_and_members_match'] = True
    report['zip_sha256'] = sha(archive)
    (EVIDENCE / 'validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'unique_real_images': len(seen), 'changed_cases': len(report['changed_results']),
                      'checks': report['checks'], 'zip': str(archive)}, indent=2), flush=True)


if __name__ == '__main__':
    main()
