"""只读复核 live2 原包，保存可复现的来源、时效与状态检查报告。"""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
from zipfile import ZipFile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'evidence/p2-camera/2026-10-27/partial-live2-board'
PACKAGE = ROOT / 'output/p2_partial_ring_live2_20261027'


def read_json(path):
    return json.loads(path.read_text(encoding='utf8'))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def stats(values):
    return dict(zip(('min', 'p50', 'p95', 'max'), np.percentile(values, [0, 50, 95, 100]).tolist()))


def check_disabled(status):
    assert status['metric_valid'] is False
    assert status['motion_allowed'] is False
    assert status['serial'] == 'DISABLED'


def main():
    archive = ROOT / '20261027-220732-139248.zip'
    BASE.mkdir(parents=True, exist_ok=True)
    with ZipFile(archive) as z:
        assert z.testzip() is None
        assert len(z.namelist()) == len(set(z.namelist()))
        for name in z.namelist():
            assert Path(name).name == name and name not in ('.', '..')
            path = BASE / name
            data = z.read(name)
            if path.exists():
                assert path.read_bytes() == data, name
            else:
                path.write_bytes(data)
        member_hashes = {name: digest(z.read(name)) for name in sorted(z.namelist())}

    manifest = read_json(PACKAGE / 'LIVE_MANIFEST.json')
    source = read_json(BASE / 'source.json')
    assert source['manifest_sha256'] == digest((PACKAGE / 'LIVE_MANIFEST.json').read_bytes())
    assert source['files'] == manifest['files']
    for name, sha in source['files'].items():
        path = (PACKAGE / name).resolve()
        assert path.is_relative_to(PACKAGE.resolve())
        assert digest(path.read_bytes()) == sha, name
    assert source['profile'] == read_json(PACKAGE / 'vision_app/code/config/project_profile.json')
    run = read_json(BASE / 'run.json')
    rows = [json.loads(line) for line in (BASE / 'observations.jsonl').read_text(encoding='utf8').splitlines()]
    events = read_json(BASE / 'display-events.json')
    inputs = read_json(BASE / 'input-events.json')
    assert len(rows) == run['frames']
    assert dict(Counter(r['status']['state'] for r in rows)) == run['states']
    assert dict(Counter(e['decision'] for e in inputs)) == run['input_decisions']
    assert run['completed_duration'] and run['error'] is None
    assert run['maximum_input_age_ms'] == 12 and run['maximum_output_age_ms'] == 150
    assert np.all(np.diff([r['host_read_time'] for r in rows]) > 0)
    assert np.all(np.diff([e['capture_sequence'] for e in inputs]) > 0)
    check_disabled(run)

    processed = {e['host_read_time']: e for e in inputs if e['decision'] == 'PROCESS'}
    assert len(processed) == run['input_decisions']['PROCESS']
    for e in inputs:
        assert np.isclose(e['input_age_ms'], (e['selection_time']-e['host_read_time'])*1000, atol=1e-7)
        assert (0 <= e['input_age_ms'] <= 12) == (e['decision'] == 'PROCESS')
    for index, row in enumerate(rows, 1):
        assert row['frame_index'] == index
        stamp = row['host_read_time']
        event = processed[stamp]
        assert event['selection_time'] == row['detector_started']
        assert event['detector_finished'] == row['detector_finished']
        assert event['processing_ms'] == row['candidate']['processing_ms']
        assert stamp <= row['detector_started'] <= row['detector_finished'] <= row['gui_received']
        for key, expected in (
            ('detector_start_age_ms', (row['detector_started']-stamp)*1000),
            ('detector_result_age_ms', (row['detector_finished']-stamp)*1000),
            ('result_queue_wait_ms', (row['gui_received']-row['detector_finished'])*1000),
        ):
            assert np.isclose(row[key], expected, atol=1e-7)
        state = row['status']
        check_disabled(state)
        check_disabled(row['candidate'])
        assert state['stationary_verified'] is False
        assert state['result_host_age_ms'] == row['display_submission_age_ms']
        assert row['candidate']['accepted'] and row['candidate']['method'] == 'partial_arc_template_v2'
        if state['state'] == 'STALE':
            assert row['display_submission_age_ms'] > 150
            assert state['candidate_centers_px'] == state['stable_centers_px'] == []
            assert not state['pixel_ready']
        else:
            assert 0 <= row['display_submission_age_ms'] <= 150
            assert state['candidate_centers_px'] == row['candidate']['centers_px']
        if state['pixel_ready']:
            assert state['state'] == 'WINDOW_STABLE' and state['window_frames'] == 3
            assert state['jitter_px'] <= 1.5 and len(state['stable_centers_px']) == 3
            assert max(np.diff(state['window_observation_stamps']))*1000 <= 150

    # 用冻结发布类按日志时间回放所有检测和显示撤销事件。
    # gui_received 是 observe 调用前时间，未记录调用内部精确时刻；
    # 逐字段一致只验证这些已记录检查点，不能复原未记录的屏幕呈现。
    spec = importlib.util.spec_from_file_location('frozen_live2_gate', PACKAGE / 'p2_partial_ring_live_gate.py')
    gate_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gate_module)
    gate = gate_module.LivePixelGate()
    timeline = []
    for row in rows:
        timeline.append((row['gui_received'], 0, row))
        timeline.append((row['host_read_time']+row['display_submission_age_ms']/1000, 1, row))
    for event in events:
        assert event['event'] == 'DISPLAY_STATE_CHANGE'
        timeline.append((event['time'], 2, event))
    for timestamp, kind, item in sorted(timeline, key=lambda t: (t[0], t[1])):
        if kind == 0:
            gate.observe(item['candidate'], (1080, 1920, 3), item['host_read_time'], timestamp)
            continue
        actual = gate.current(timestamp)
        expected = item['status']
        assert actual.keys() == expected.keys()
        for key in actual:
            if key == 'result_host_age_ms':
                assert np.isclose(actual[key], expected[key], atol=1e-7)
            else:
                assert actual[key] == expected[key], (kind, key, item.get('frame_index'))
        check_disabled(expected)
        if kind == 2:
            assert expected['state'] == 'STALE'
            assert expected['candidate_centers_px'] == expected['stable_centers_px'] == []

    snapshot_indices = {}
    for name in ('first-frame', 'first-candidate', 'first-window-stable', 'first-stale'):
        saved = read_json(BASE / (name+'.json'))
        assert saved == rows[saved['frame_index']-1]
        image = cv2.imdecode(np.fromfile(BASE / (name+'-raw.png'), np.uint8), 1)
        overlay = cv2.imdecode(np.fromfile(BASE / (name+'-overlay.png'), np.uint8), 1)
        assert image.shape == (1080, 1920, 3) and overlay.shape == (540, 960, 3)
        snapshot_indices[name] = saved['frame_index']
    unseen = [e for stamp, e in processed.items() if stamp not in {r['host_read_time'] for r in rows}]
    # 检测线程在结束时 join：保留末条完成但未进入 GUI 的记录，单列账目。
    assert len(unseen) == 1 and unseen[0]['host_read_time'] > rows[-1]['host_read_time']
    metrics = {key: stats([r[key] for r in rows]) for key in (
        'detector_start_age_ms', 'detector_result_age_ms', 'result_queue_wait_ms',
        'preview_render_ms', 'imshow_ms', 'display_submission_age_ms')}
    metrics['algorithm_ms'] = stats([r['candidate']['processing_ms'] for r in rows])
    metrics['observation_gap_ms'] = stats(np.diff([r['host_read_time'] for r in rows])*1000)
    metrics['fresh_output_remaining_budget_ms'] = stats([
        150-r['display_submission_age_ms'] for r in rows if r['display_submission_age_ms'] <= 150])
    report = dict(
        reviewed_on='2026-10-28', archive=str(archive.relative_to(ROOT)),
        archive_bytes=archive.stat().st_size, archive_sha256=digest(archive.read_bytes()),
        archive_crc_pass=True, member_sha256=member_hashes,
        source_manifest_match=True, source_files_verified=len(source['files']),
        frozen_gate_checkpoint_replay_pass=True, snapshot_frame_indices=snapshot_indices,
        frames=len(rows), states=run['states'], sampling_span_s=run['sampling_span_s'],
        quality_accepted=sum(r['candidate']['accepted'] for r in rows),
        fresh_at_detection=sum(r['detector_result_age_ms'] <= 150 for r in rows),
        fresh_at_display_submission=sum(r['display_submission_age_ms'] <= 150 for r in rows),
        display_expiry_events=len(events), input_decisions=run['input_decisions'],
        selected_input_skip_fraction=run['input_decisions']['SKIP_INPUT_AGE_OVER_12MS']/len(inputs),
        processed_not_displayed_at_shutdown=unseen,
        durations_ms=metrics,
        maximum_window_span_ms=max(r['status'].get('window_span_ms', 0) for r in rows),
        conclusion='Latency improved and short fresh pixel windows observed; continuous output remains unaccepted.',
        limits=[
            'Selected-input skips exclude frames never selected from the latest-frame slot; not a sensor frame-loss rate.',
            'Host read timestamps and imshow return do not measure exposure-to-result or monitor presentation.',
            'Quality acceptance and short-window repeatability do not establish independent center ground truth or physical stillness.',
            'Snapshots cover four event types, not all 76 input images.',
            'Many active outputs expire between observations; 52 WINDOW_STABLE records are not uninterrupted availability.',
        ], metric_valid=False, motion_allowed=False, continuous_output_accepted=False,
        hardware_acceptance=False,
    )
    (BASE/'verified-review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps({k: report[k] for k in ('frames', 'states', 'fresh_at_detection',
        'fresh_at_display_submission', 'display_expiry_events', 'input_decisions',
        'frozen_gate_checkpoint_replay_pass', 'durations_ms')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
