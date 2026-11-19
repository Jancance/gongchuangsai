import importlib.util
import json
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stream = load('trial_stream', 'output/low_material_stream_20261114.py')
trial = load('trial_entry', 'scripts/source_low_trial_20261114.py')


class TransportTests(unittest.TestCase):
    def test_offline_three_item_orchestration_leaves_physical_success_pending(self, start_item=0):
        from test_low_pick_board_20261114 import state
        clock = [100.0]
        goals = []

        class Session:
            target = None
            def __enter__(self): return self
            def __exit__(self, *args): pass

        class Board:
            _healthy = staticmethod(trial.LowPickBoard._healthy)
            def __init__(self, *args):
                self.value = state()
                self.goal = None
                self.verified_flash_bytes = 1
            def snapshot(self): return self.value.copy()
            def prepare(self, goal):
                self.goal = goal
                goals.append(goal)
                self.value.update(g_low_state=1, g_low_phase=3, g_low_accepted_sequence=goal.sequence)
            def confirm(self, **kwargs):
                self.value.update(g_low_state=2, g_low_phase=10, g_low_done_sequence=self.goal.sequence,
                                  g_low_clear_valid=1, g_low_clear_ms=700, g_low_actual_rise_mm=66,
                                  g_low_release_ms=1000)
                return True
            def cancel(self): raise AssertionError('successful offline batch must not cancel')

        class Peer:
            def __init__(self, *args):
                self.sequence = 0
                self.colors = (2, 1, 5)
            def exchange(self, control):
                if control['state'] == 'OBSERVE':
                    clock[0] += 5.7
                    self.sequence += 1
                    colors = [c if c in control['remaining_colors'] else 0 for c in self.colors]
                    self.colors = (self.colors[1], self.colors[2], self.colors[0])
                else:
                    colors = [0, control['remaining_colors'][0], 0]
                return dict(pose_active=control['state'] != 'MOVING', stationary=True,
                            pose_id=control['pose_id'], pose_sequence=control['pose_sequence'],
                            frame_after_pose_ack=True, age_s=.01, pause_age_s=0,
                            stop_sequence=self.sequence, colors=colors, motion_seen=True)
            def close(self): pass

        fake_probe = SimpleNamespace(attach=lambda: Session())
        fake_live = SimpleNamespace(load_image=lambda: ({}, [(0x08000000, b'fixture')]))
        args = SimpleNamespace(focus_profile_verified=True, task_code='215+213+512+321',
                               profile='config/task_materials_preliminary_2027.json', port=18887,
                               start_item=start_item, completed_prefix=(ROOT / 'evidence/integration/2026-11-14/accepted-carrier-prefix.json') if start_item else None)
        with tempfile.TemporaryDirectory() as temp, \
                patch.dict(sys.modules, {'source_body_probe_20261111': fake_probe,
                                         'gripper210_attach_read_20261105': fake_live}), \
                patch.object(trial, 'E', Path(temp)), \
                patch.object(trial, 'manifest', return_value=({'axf': 'fixture.axf', 'sha256': 'fixture'},
                                                             [(194, 163), (163, 159), (180, 0)])), \
                patch.object(trial, 'LowPickBoard', Board), patch.object(trial, 'CameraPeer', Peer), \
                patch.object(trial.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(trial.time, 'sleep', side_effect=lambda delay: clock.__setitem__(0, clock[0] + delay)):
            trial.run(args)
            report = json.loads(next(Path(temp).glob('trial-*.json')).read_text(encoding='utf-8'))
            self.assertTrue(report['complete'])
            self.assertTrue(report['physical_placement_pending'])
            self.assertFalse(report['formal_inventory_updated'])
            self.assertEqual([(g.color, g.slot) for g in goals], [(2, 1), (1, 2), (5, 3)][start_item:])

    def test_confirmed_yellow_prefix_resumes_only_red_and_black(self):
        self.test_offline_three_item_orchestration_leaves_physical_success_pending(start_item=1)

    def test_loopback_heartbeat_and_observation(self):
        bridge = stream.ControlBridge(0)
        client = socket.create_connection(bridge.server.getsockname(), timeout=1)
        client.settimeout(1)
        try:
            payload = dict(session_id='trial', pose_id='actual-low', pose_sequence=5,
                           state='CONFIRM', remaining_colors=[1, 5], client_sent_s=time.monotonic())
            client.sendall((json.dumps(payload) + '\n').encode())
            value = None
            for _ in range(10):
                value = bridge.poll()
                if value is not None:
                    break
                time.sleep(.01)
            self.assertEqual(value['pose_sequence'], 5)
            self.assertEqual(value['remaining_colors'], [1, 5])
            bridge.publish(dict(colors=[0, 5, 1], pose_sequence=5, stationary=True))
            self.assertEqual(json.loads(client.recv(4096))['colors'], [0, 5, 1])
            bridge.value['written_monotonic_s'] -= 1
            self.assertIsNone(bridge.poll())
            client.sendall(b'not-json\n')
            for _ in range(10):
                bridge.poll()
                if bridge.client is None:
                    break
                time.sleep(.01)
            self.assertIsNone(bridge.client)
            self.assertIsNone(bridge.value)
        finally:
            client.close()
            bridge.close()

    def test_stop_is_requested_before_live_target_disconnects(self):
        events = []

        class Session:
            def __enter__(self):
                events.append('enter')
                return self

            def __exit__(self, *args):
                events.append('disconnect')

        class Board:
            def snapshot(self):
                return dict(g_low_state=1, g_low_request=0, g_low_accepted_sequence=7,
                            g_it_state=9, g_it_request=0, g_lift_ruler_stop_pending=0,
                            g_vehicle_arm_stop_pending=0, g_source_parallel_stop_mask=0)

            def cancel(self):
                events.append('stop')

        report = {}
        fake = SimpleNamespace(attach=lambda: Session())
        with patch.dict(sys.modules, {'source_body_probe_20261111': fake}):
            with self.assertRaisesRegex(RuntimeError, 'first fault'):
                with trial.guarded_attach(report, dict(active=True, board=Board(), sequence=7)):
                    raise RuntimeError('first fault')
        self.assertEqual(events, ['enter', 'stop', 'disconnect'])
        self.assertTrue(report['stop_protocol_confirmed'])

    def test_confirmed_unstarted_rejection_does_not_fault_the_vehicle(self):
        class Session:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

        board = SimpleNamespace(snapshot=lambda: dict(g_low_state=0, g_low_request=0,
                                                       g_low_accepted_sequence=0),
                                cancel=lambda: self.fail('unstarted request must not stop'))
        with patch.dict(sys.modules, {'source_body_probe_20261111': SimpleNamespace(attach=lambda: Session())}):
            with self.assertRaises(ValueError):
                with trial.guarded_attach({}, dict(active=True, board=board, sequence=7)):
                    raise ValueError('rejected before motion')


if __name__ == '__main__':
    unittest.main()
