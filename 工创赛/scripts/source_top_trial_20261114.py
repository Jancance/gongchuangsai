"""One supervised task batch via the low owner and authenticated camera tunnel.

No attachment or motion occurs by default. No flash/reset is implemented here.
Each piece has one prepare and one fresh visual confirmation; F407 owns motion.
"""
import argparse
import hashlib
import json
import socket
import sys
import time
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from smart_carrier.vision_manipulation.low_pick_board import LowPickBoard, LowPickTarget
from smart_carrier.vision_manipulation.turntable_phase import IndexedDiscPhase, StopEvent

E = ROOT / 'evidence/integration/2026-11-14/top-pick-lead8'
REMOTE = '/home/orangepi/vision_tools/'


@contextmanager
def guarded_attach(report, active):
    from source_body_probe_20261111 import attach
    with attach() as session:
        try:
            yield session
        except BaseException:
            # STOP must run before leaving the live attach session.
            if active.get('active'):
                board = active['board']
                try:
                    state = board.snapshot()
                    unstarted = (state['g_low_state'] in (0, 2) and not state['g_low_request'] and
                                 state['g_low_accepted_sequence'] != active.get('sequence'))
                except BaseException:
                    unstarted = False
                if not unstarted:
                    try:
                        board.cancel()
                        report['stop_requested'] = True
                        for _ in range(12):
                            time.sleep(.15)
                            state = board.snapshot()
                            report['after_stop_request'] = state
                            if (state['g_it_state'] == 9 and not state['g_it_request'] and
                                    not any(state[k] for k in ('g_lift_ruler_stop_pending',
                                            'g_vehicle_arm_stop_pending', 'g_source_parallel_stop_mask'))):
                                report['stop_protocol_confirmed'] = True
                                break
                    except BaseException as stop_error:
                        report['stop_error'] = repr(stop_error)
            raise


def manifest():
    candidate = json.loads((E / 'candidate.json').read_text(encoding='utf-8'))
    assert candidate['build_passed'] and json.loads((E / 'checks.json').read_text(encoding='utf-8'))['passed']
    assert json.loads((E / 'matlab-result.json').read_text(encoding='utf-8'))['passed']
    for name, digest in candidate['changes'].items():
        assert hashlib.sha256((ROOT / candidate['project'] / name).read_bytes()).hexdigest() == digest
    image = candidate['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((ROOT / image['axf']).read_bytes()).hexdigest() == image['sha256']
    geometry_path = E / 'parking-current.json'
    if not geometry_path.exists():
        geometry_path = ROOT / 'evidence/integration/2026-11-14/source-top-repark-20261114-initial/matlab-result.json'
    geometry = json.loads(geometry_path.read_text(encoding='utf-8'))
    assert geometry['passed'] and geometry['current_near_error_norm_mm'] <= 2
    poses = list(zip(geometry['current_arm_angles_deg'], geometry['current_extensions_mm']))
    poses[2] = (180.0, 0.0)
    assert all(0 <= yaw <= 270 and 0 <= extension <= 170 for yaw, extension in poses)
    return image, poses


class CameraPeer:
    def __init__(self, port):
        self.socket = socket.create_connection(('127.0.0.1', port), timeout=3)
        self.socket.settimeout(.05)
        self.buffer = b''

    def exchange(self, control):
        control = dict(control, client_sent_s=time.monotonic())
        self.socket.sendall((json.dumps(control) + '\n').encode())
        try:
            data = self.socket.recv(65536)
        except socket.timeout:
            return None
        if not data:
            raise ConnectionError('camera/SSH tunnel closed')
        self.buffer += data
        if len(self.buffer) > 1048576:
            raise RuntimeError('camera observation backlog')
        latest = None
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            latest = json.loads(line)
        if latest is not None and latest.get('client_sent_s') is not None:
            latest['age_s'] = time.monotonic() - latest['client_sent_s']
        return latest

    def close(self):
        self.socket.close()


def run(args):
    # These are concrete field prerequisites, never inferred from a passing build.
    if not args.focus_profile_verified:
        raise RuntimeError('取料姿态局部确认框尚需新图核对；本入口不会替用户假定已验证')
    image, poses = manifest()
    import gripper210_attach_read_20261105 as live
    from source_body_probe_20261111 import attach
    from smart_carrier.task_materials import resolve_preliminary_task
    # Existing task parser supplies the first batch; no colour order in firmware.
    plan = resolve_preliminary_task(args.task_code, 1, ROOT / args.profile)
    colors = [row.color_code for row in plan.batch_1]
    assert len(colors) == 3 and len(set(colors)) == 3
    start_item = getattr(args, 'start_item', 0)
    if not start_item and getattr(args, 'completed_prefix', None) is not None:
        raise ValueError('已提供装载确认记录却要求从第一件重抓，请核对恢复起点')
    prefix = []
    if start_item:
        prefix_path = getattr(args, 'completed_prefix', None)
        if prefix_path is None:
            raise RuntimeError('恢复剩余物料需要本轮用户确认的已放稳槽位记录')
        verified = json.loads(Path(prefix_path).read_text(encoding='utf-8'))
        prefix = verified['slots']
        assert verified['source'] == 'user_confirmation' and verified['task_code'] == args.task_code
        assert len(prefix) == start_item
        assert all(row['slot'] == i + 1 and row['color'] == colors[i] and row['placed_stable'] is True
                   for i, row in enumerate(prefix))
    live.IMAGE = ROOT / image['axf']
    symbols, segments = live.load_image()
    path = E / ('trial-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    report = dict(complete=False, flashed=False, reset=False, task_code=args.task_code,
                  image_sha256=image['sha256'], physical_placement_pending=True,
                  formal_inventory_updated=False, observations=[], trace=[], pieces=[])
    report['accepted_prefix'] = prefix
    report['start_item'] = start_item

    def save():
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')

    save()
    peer = None
    active = {'active': False}
    try:
        with guarded_attach(report, active) as session:
            board = LowPickBoard(session.target, symbols, segments)
            active['board'] = board
            report['verified_flash_bytes'] = board.verified_flash_bytes
            peer = CameraPeer(args.port)
            model = IndexedDiscPhase('source-regions-current')
            observed_events = 0
            last_stop_key = None
            cached_period = cached_permutation = None
            current_goal = None
            stage = 'OBSERVE'
            item = start_item
            stage_begin = batch_begin = time.monotonic()
            last_state = None
            while time.monotonic() - batch_begin < 120:
                now = time.monotonic()
                state = board.snapshot()
                if not board._healthy(state) or state['g_low_state'] == 9:
                    raise RuntimeError('actual controller fault')
                pose_id = current_goal.view_pose_id if stage == 'CONFIRM' else 'observer-top65'
                view_active = stage in ('OBSERVE', 'CONFIRM')
                control = dict(session_id=path.stem, state=stage if view_active else 'MOVING',
                               pose_id=pose_id, pose_sequence=current_goal.sequence if current_goal else item,
                               remaining_colors=colors[item:],
                               material_pickup_config=REMOTE + ('material_pickup_top_focus_20261114.json'
                                                                if stage == 'CONFIRM' else 'material_pickup_top65_20261114.json'))
                observation = peer.exchange(control)
                if observation is not None:
                    report['observations'].append(observation)
                phase_key = (stage, state['g_low_state'], state['g_low_phase'])
                if phase_key != last_state:
                    print('trial', item + 1, phase_key, 'clear_ms', state['g_low_clear_ms'], flush=True)
                    last_state = phase_key
                    save()
                report['trace'].append(state)
                fresh = (observation is not None and observation.get('pose_active') and
                         observation.get('pose_id') == pose_id and observation.get('frame_after_pose_ack') and
                         0 <= observation.get('age_s', 999) <= .20)
                # Global presence only prepositions the raised arm; do not wait for
                # a stop here. The grasp trigger is one two-frame check at the pose.
                if stage == 'OBSERVE' and fresh:
                    slots = tuple(observation['colors'])
                    candidates = [i for i, color in enumerate(slots) if color == colors[item]]
                    if len(candidates) != 1:
                        continue
                    region = candidates[0]
                    sequence = (state['g_low_accepted_sequence'] + 1) & 0xffffffff or 1
                    yaw, extension = poses[region]
                    current_goal = LowPickTarget(sequence, item + 1, colors[item], region, yaw, extension,
                                                 1, 1, f'source-{region}-top')
                    active.update(active=True, sequence=sequence)
                    board.prepare(current_goal)
                    report['pieces'].append(dict(target=current_goal.__dict__,
                        selection_source='fresh_region_preposition',
                        grasp_trigger='two_stable_frames_at_ready_pose'))
                    stage = 'APPROACH'
                    stage_begin = now
                elif stage == 'APPROACH':
                    if state['g_low_accepted_sequence'] == current_goal.sequence and state['g_low_phase'] == 3:
                        stage = 'CONFIRM'
                        stage_begin = now
                    elif now - stage_begin > 16:
                        raise TimeoutError('actual approach acknowledgement/completion')
                elif stage == 'CONFIRM':
                    if (state['g_low_accepted_sequence'] == current_goal.sequence and
                            state['g_low_phase'] >= 4):
                        stage = 'EXECUTE'
                        stage_begin = now
                    elif (fresh and observation.get('stationary') and
                            current_goal.color in observation.get('colors', ())):
                        # A write is not acceptance. Keep observations flowing until
                        # the MCU actually enters descent; rejected fresh writes may retry.
                        board.confirm(sequence=current_goal.sequence, observation=observation)
                    if stage == 'CONFIRM' and now - stage_begin > 20:
                        raise TimeoutError('no new usable stopped target; no blind descent')
                elif stage == 'EXECUTE':
                    if state['g_low_state'] == 2 and state['g_low_done_sequence'] == current_goal.sequence:
                        assert (state['g_low_clear_valid'] == 1 and state['g_low_clear_ms'] > 0 and
                                state['g_low_actual_rise_mm'] > 65 and state['g_low_release_ms'] > 0)
                        report['pieces'][-1]['command_complete'] = state
                        item += 1
                        active['active'] = False
                        if item == 3:
                            report.update(complete=True, command_batch_elapsed_s=now-batch_begin)
                            break
                        current_goal = None
                        stage = 'OBSERVE'
                        stage_begin = now
                        last_stop_key = None
                    elif now - stage_begin > 55:
                        raise TimeoutError('one low grasp/transfer/return command chain')
                # Initial learning spans three real transitions; later source waits
                # keep the supervised23s ceiling. This is not competition acceptance.
                observation_budget = 30 if cached_period is None else 23
                if stage == 'OBSERVE' and now - stage_begin > observation_budget:
                    raise TimeoutError('no usable observation within supervised waiting budget')
                time.sleep(.04)
            else:
                raise TimeoutError('one supervised batch budget')
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        if peer is not None:
            peer.close()
        save()
        print(path, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once-batch', action='store_true')
    parser.add_argument('--focus-profile-verified', action='store_true')
    parser.add_argument('--task-code', default='215+213+512+321')
    parser.add_argument('--port', type=int, default=18887)
    parser.add_argument('--start-item', type=int, choices=(0, 1, 2), default=0)
    parser.add_argument('--completed-prefix', type=Path)
    parser.add_argument('--profile', default='config/task_materials_preliminary_2027.json')
    parser.add_argument('--candidate-evidence', type=Path, default=E)
    arguments = parser.parse_args()
    E = arguments.candidate_evidence if arguments.candidate_evidence.is_absolute() else ROOT / arguments.candidate_evidence
    if arguments.once_batch:
        run(arguments)
    else:
        manifest()
        print('CHECK only: no devices, flash/reset or actuator command. Field focus profile and once-batch required.')
