import unittest
from types import SimpleNamespace
from smart_carrier.vision_manipulation.low_pick_board import LowPickBoard, LowPickTarget


class Symbols(dict):
    def __missing__(self, key):
        self[key] = key
        return key


class Target:
    def __init__(self):
        self.writes = []
        self.fail_request = False
        self.flash = b'validated flash'

    def get_state(self):
        return SimpleNamespace(name='RUNNING')

    def read_memory_block8(self, address, length):
        return self.flash[address - 0x08000000:address - 0x08000000 + length]

    def write32(self, address, value):
        self.writes.append((address, value))
        if self.fail_request and address == 'g_low_request' and value == 1:
            raise TimeoutError('start write outcome unknown')


def state():
    return dict(g_low_request=0, g_low_state=0, g_low_phase=0, g_low_fault=0, g_low_rejected=0,
                g_low_accepted_sequence=0, g_low_done_sequence=0, g_low_elapsed_ms=0,
                g_low_clear_valid=0, g_low_clear_ms=0, g_low_release_ms=0,
                g_it_state=2, g_it_request=0, g_it_cause=0, g_it_rejected=0,
                g_safety_ready=1, g_safety_fault=0, g_rtos_fault=0,
                g_can_ctrl_fault_latched=0, g_chassis_auto_active=0,
                g_vis_material_valid=1, g_vis_mat_cycle=15, g_vis_mat_left=5,
                g_vis_mat_right=2, g_vis_mat_down=1, g_vis_mat_tick=1000, g_src_now_ms=1020)


class LowBoardTests(unittest.TestCase):
    def test_bulk_snapshot_keeps_integer_and_float_fields(self):
        import struct
        class Addresses(dict):
            def __missing__(self, key):
                self[key] = 0x20000000 + len(self) * 4
                return self[key]
        addresses = Addresses()
        values = dict(g_low_state=1, g_vis_mat_left=5,
                      g_low_actual_rise_mm=struct.unpack('<I', struct.pack('<f', 65.01))[0])
        calls = []
        def read(start, count):
            calls.append((start, count))
            reverse = {address: key for key, address in addresses.items()}
            return [values.get(reverse.get(start + i * 4), 0) for i in range(count)]
        board = object.__new__(LowPickBoard)
        board.target = SimpleNamespace(read_memory_block32=read)
        board.symbols = addresses
        board._snapshot_groups = None
        snapshot = board.snapshot()
        self.assertEqual(snapshot['g_low_state'], 1)
        self.assertEqual(snapshot['g_vis_mat_left'], 5)
        self.assertGreater(snapshot['g_low_actual_rise_mm'], 65)
        self.assertLessEqual(len(calls), 2)

    def setup_board(self):
        target = Target()
        board = LowPickBoard(target, Symbols(), [(0x08000000, target.flash)])
        value = state()
        board.snapshot = lambda: value.copy()
        goal = LowPickTarget(1, 1, 2, 1, 163.5, 159, 4, 7, 'verified-right-low')
        return target, board, value, goal

    def test_prepare_only_has_one_request_and_never_confirms_or_clears_fault(self):
        target, board, value, goal = self.setup_board()
        board.prepare(goal)
        self.assertEqual([v for k, v in target.writes if k == 'g_low_request'], [1])
        self.assertFalse(any(k.endswith('fault') or k.endswith('cause') for k, _ in target.writes))
        value['g_low_state'] = 1
        with self.assertRaises(RuntimeError):
            board.prepare(goal)
        self.assertEqual([v for k, v in target.writes if k == 'g_low_request'], [1])

    def test_uncertain_start_sends_stop_instead_of_retry(self):
        target, board, _, goal = self.setup_board()
        target.fail_request = True
        with self.assertRaises(TimeoutError):
            board.prepare(goal)
        self.assertEqual([v for k, v in target.writes if k == 'g_low_request'], [1, 9])
        self.assertIn(('g_it_request', 9), target.writes)

    def test_actual_matching_rx_and_pose_needed_once(self):
        target, board, value, goal = self.setup_board()
        observation = dict(pose_active=True, stationary=True, pose_id=goal.view_pose_id,
                           pose_sequence=1, frame_after_pose_ack=True, age_s=.05, colors=[5, 2, 1])
        self.assertFalse(board.confirm(sequence=1, observation=observation))
        board.prepare(goal)
        value.update(g_low_state=1, g_low_phase=3, g_low_accepted_sequence=1)
        for change in [dict(pose_id='old-camera-view'), dict(frame_after_pose_ack=False),
                       dict(colors=[2, 5, 1]), dict(age_s=.201), dict(stationary=False)]:
            self.assertFalse(board.confirm(sequence=1, observation=observation | change))
        self.assertTrue(board.confirm(sequence=1, observation=observation))
        self.assertFalse(board.confirm(sequence=1, observation=observation))
        self.assertEqual([v for k, v in target.writes if k == 'g_low_request'], [1, 2])

    def test_wrong_flash_cannot_write_and_old_receiver_frame_cannot_confirm(self):
        target = Target()
        with self.assertRaises(RuntimeError):
            LowPickBoard(target, Symbols(), [(0x08000000, b'wrong image')])
        self.assertEqual(target.writes, [])
        target, board, value, goal = self.setup_board()
        board.prepare(goal)
        value.update(g_low_state=1, g_low_phase=3, g_low_accepted_sequence=1, g_src_now_ms=1201)
        self.assertFalse(board.confirm(sequence=1, observation=dict(pose_active=True, stationary=True,
            pose_id=goal.view_pose_id, pose_sequence=1, frame_after_pose_ack=True, age_s=.05, colors=[5, 2, 1])))


if __name__ == '__main__':
    unittest.main()
