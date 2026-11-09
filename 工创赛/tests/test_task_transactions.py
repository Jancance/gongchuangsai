from dataclasses import replace
from pathlib import Path
import unittest

from smart_carrier.task_materials import SlotInventory, resolve_preliminary_task
from smart_carrier.task_transactions import TaskLedger, Transfer, TransferEvidence


ROOT = Path(__file__).resolve().parents[1]


class TransactionTests(unittest.TestCase):
    def setUp(self):
        self.task = resolve_preliminary_task('123+123+321+321', 1,
                                           ROOT/'config/task_materials_preliminary_2027.json')
        self.empty = tuple(SlotInventory('run-test', i, '', 0, False, True) for i in (1, 2, 3))
        self.ledger = TaskLedger(self.task, 'run-test', self.empty, maximum_age=.5)
        self.request = Transfer('run-test', 1, 1, 1, 'red-1', 'source:1', 'carrier:1')

    def evidence(self, request=None, at=10.1, **changes):
        request = request or self.request
        return TransferEvidence(request.session, request.sequence, request.material_id, at, **changes)

    def pick(self, request=None, at=10.1):
        r = request or self.request
        color = (self.task.batch_1 if r.batch == 1 else self.task.batch_2)[r.order-1].color_code
        return self.ledger.observe(self.evidence(r, at, source_clear=True, held=True, color_code=color),
                                   now=at, phase='PICK', actuator_done=True)

    def place(self, request=None, at=10.2):
        return self.ledger.observe(self.evidence(request, at, released=True, destination_present=True,
                                                stable=True, correct_target=True),
                                   now=at, phase='PLACE', actuator_done=True)

    def test_ack_and_done_without_physical_confirmation_do_not_count(self):
        self.ledger.begin(self.request, now=10.)
        self.ledger.accepted(1)
        for done in (False, True):
            with self.assertRaises(ValueError):
                self.ledger.observe(self.evidence(), now=10.1, phase='PICK', actuator_done=done)
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 0)
        self.assertFalse(self.ledger.slots[0].occupied)

    def test_pick_then_load_once_and_transport_gate(self):
        self.ledger.begin(self.request, now=10.)
        self.pick()
        self.assertFalse(self.ledger.can_transport())
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 1)
        self.place()
        self.assertTrue(self.ledger.can_transport())
        self.assertEqual(self.ledger.slots[0].material_id, 'red-1')
        self.assertEqual(self.ledger.snapshot()['correct_places'], 1)
        self.pick()
        self.place()
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 1)
        self.assertEqual(self.ledger.snapshot()['correct_places'], 1)

    def test_retry_same_sequence_returns_original_and_conflict_rejected(self):
        first = self.ledger.begin(self.request, now=10.)
        self.assertIs(self.ledger.begin(self.request, now=11.), first)
        with self.assertRaises(ValueError):
            self.ledger.begin(replace(self.request, destination='carrier:2'), now=11.)

    def test_stale_future_pre_request_cross_session_wrong_instance_rejected(self):
        self.ledger.begin(self.request, now=10.)
        good = self.evidence(source_clear=True, held=True)
        for bad in (replace(good, observed_at=9.9), replace(good, observed_at=11.),
                    replace(good, observed_at=float('nan')), replace(good, session='old'),
                    replace(good, material_id='other'), replace(good, sequence=2)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.ledger.observe(bad, now=10.8, phase='PICK', actuator_done=True)
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 0)

    def test_independent_place_after_pick_with_all_checks(self):
        self.ledger.begin(self.request, now=10.)
        with self.assertRaises(ValueError):
            self.place()
        self.pick()
        good = self.evidence(at=10.2, released=True, destination_present=True,
                             stable=True, correct_target=True)
        for bad in (replace(good, observed_at=10.1), replace(good, released=False),
                    replace(good, stable=False), replace(good, correct_target=False),
                    replace(good, destination_present=False)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.ledger.observe(bad, now=10.2, phase='PLACE', actuator_done=True)
        self.assertEqual(self.ledger.snapshot()['correct_places'], 0)

    def test_failure_invalidates_inventory_and_late_done_cannot_restore(self):
        self.ledger.begin(self.request, now=10.)
        self.pick()
        self.ledger.invalidate('drop reported')
        self.assertFalse(self.ledger.can_transport())
        self.assertTrue(all(not s.verified for s in self.ledger.slots))
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 1)
        self.assertEqual(self.ledger.snapshot()['correct_places'], 0)
        with self.assertRaises(ValueError):
            self.place()
        with self.assertRaises(ValueError):
            self.ledger.begin(replace(self.request, sequence=2), now=11.)

    def test_loaded_snapshot_does_not_retroactively_increment(self):
        slots = (SlotInventory('run-test', 1, 'observed-red', 1, True, True),)+self.empty[1:]
        ledger = TaskLedger(self.task, 'run-test', slots, maximum_age=.5)
        self.assertTrue(ledger.can_transport())
        self.assertEqual((ledger.snapshot()['correct_picks'], ledger.snapshot()['correct_places']), (0, 0))
        yellow = Transfer('run-test', 2, 1, 2, 'yellow', 'source:2', 'carrier:2')
        self.assertEqual(ledger.begin(yellow, now=10.).state, 'REQUESTED')

    def test_occupied_slot_wrong_color_sequence_skip_and_parallel_rejected(self):
        for request in (replace(self.request, order=2, source='source:2'),
                        replace(self.request, batch=2), replace(self.request, sequence=True),
                        replace(self.request, source='source:2')):
            with self.subTest(request=request), self.assertRaises(ValueError):
                self.ledger.begin(request, now=10.)
        self.ledger.begin(self.request, now=10.)
        with self.assertRaises(ValueError):
            self.ledger.begin(replace(self.request, sequence=2), now=10.)
        self.pick()
        self.place()
        with self.assertRaises(ValueError):
            self.ledger.begin(Transfer('run-test', 2, 1, 2, 'yellow', 'source:2', 'carrier:1'), now=11.)

    def test_cannot_skip_rough_stage_or_relabel_batch(self):
        self.ledger.begin(self.request, now=10.)
        self.pick()
        self.place()
        for request in (Transfer('run-test', 2, 1, 1, 'red-1', 'carrier:1', 'temporary:1'),
                        Transfer('run-test', 2, 2, 3, 'red-1', 'carrier:1', 'rough:1'),
                        Transfer('run-test', 2, 1, 1, 'red-1', 'carrier:1', 'rough:2')):
            with self.subTest(request=request), self.assertRaises(ValueError):
                self.ledger.begin(request, now=11.)

    def test_complete_two_batches_color_to_slot_to_rough_to_stack(self):
        seq = 0
        for batch, targets in ((1, self.task.batch_1), (2, self.task.batch_2)):
            for target in targets:
                material = f'b{batch}-c{target.color_code}'
                # 槽位故意不等于颜色号或任务序号。
                slot = 4-target.order
                route = (f'source:{target.order}', f'carrier:{slot}', f'rough:{target.rough_ring}',
                         f'carrier:{slot}', f'temporary:{target.temporary_ring}')
                for source, destination in zip(route, route[1:]):
                    seq += 1
                    request = Transfer('run-test', seq, batch, target.order, material, source, destination)
                    at = 10.+seq
                    self.ledger.begin(request, now=at)
                    self.ledger.accepted(seq)
                    self.pick(request, at+.1)
                    self.place(request, at+.2)
                    self.assertFalse(self.ledger.motion_allowed)
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 24)
        self.assertEqual(self.ledger.snapshot()['correct_places'], 24)
        self.assertTrue(all(not s.occupied for s in self.ledger.slots))

    def test_unknown_slots_and_duplicate_instances_rejected(self):
        for slots in (self.empty[:2], (replace(self.empty[0], verified=False),)+self.empty[1:],
                      (replace(self.empty[0], context_id='old'),)+self.empty[1:]):
            with self.subTest(slots=slots), self.assertRaises(ValueError):
                TaskLedger(self.task, 'run-test', slots, maximum_age=.5)

    def test_wrong_or_unknown_observed_color_does_not_count(self):
        self.ledger.begin(self.request, now=10.)
        for color in (None, 2, True):
            with self.subTest(color=color), self.assertRaises(ValueError):
                self.ledger.observe(self.evidence(source_clear=True, held=True, color_code=color),
                                    now=10.1, phase='PICK', actuator_done=True)
        self.assertEqual(self.ledger.snapshot()['correct_picks'], 0)

    def test_actual_user_intake_replay_keeps_scope_and_counters(self):
        with self.assertRaises(ValueError):
            self.evidence(held='false')
        import json
        from smart_carrier.task_transaction_replay import replay
        intake = json.loads((ROOT/'config/p2_task_intake_20261101.json').read_text(encoding='utf-8'))
        for start_id in (1, 2):
            result = replay(intake, ROOT/'config/task_materials_preliminary_2027.json', start_id, [])
            self.assertFalse(result['motion_allowed'])
            self.assertFalse(result['start_id_observed'])
            self.assertEqual(result['snapshot']['correct_picks'], 0)
            self.assertEqual(result['slots'][0]['color_code'], 1)
            self.assertEqual([s['occupied'] for s in result['slots']], [True, False, False])


if __name__ == '__main__':
    unittest.main()
