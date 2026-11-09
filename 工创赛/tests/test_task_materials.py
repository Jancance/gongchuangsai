from dataclasses import replace
from pathlib import Path
import unittest

from smart_carrier.task_materials import (
    ObjectEvidence, SlotInventory, commit_slot_pick, infer_three_identities,
    resolve_preliminary_task, select_fixed_pick,
)

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT/'config/task_materials_preliminary_2027.json'


class TaskMaterialTests(unittest.TestCase):
    def test_scan_resolves_colors_and_same_color_stacking_not_fourth_segment(self):
        task = resolve_preliminary_task('156+231+516+123', 1, PROFILE)
        self.assertEqual([t.color_name for t in task.batch_1], ['red', 'black', 'lightblue'])
        self.assertEqual([t.color_name for t in task.batch_2], ['black', 'red', 'lightblue'])
        self.assertEqual([t.rough_ring for t in task.batch_2], [1, 2, 3])
        self.assertEqual([t.temporary_ring for t in task.batch_2], [3, 2, 1])
        self.assertTrue(all(t.stack_on_first_batch for t in task.batch_2))
        self.assertFalse(task.motion_allowed)

    def test_preliminary_duplicate_or_different_batch_colors_rejected(self):
        for code in ('116+123+116+123', '156+123+234+123', '156+123+516'):
            with self.subTest(code=code), self.assertRaises(ValueError):
                resolve_preliminary_task(code, 1, PROFILE)

    def objects(self):
        return tuple(ObjectEvidence('run1/batch1/groupA', str(i), 10., True, True, c, c is not None)
                     for i, c in enumerate((1, 5, None), 1))

    def infer(self, objects=None, **kw):
        args = dict(context_id='run1/batch1/groupA', closed_group_confirmed=True,
                    now=10.1, maximum_age=.15)
        args.update(kw)
        return infer_three_identities((1, 5, 6), objects or self.objects(), **args)

    def test_two_known_colors_resolve_third_with_explicit_inference_source(self):
        result = self.infer()
        self.assertEqual([r.color_code for r in result], [1, 5, 6])
        self.assertEqual(result[-1].source, 'TASK_REMAINDER_INFERRED')

    def test_identity_does_not_invent_missing_position(self):
        objects = self.objects()
        result = self.infer(objects[:-1]+(replace(objects[-1], position_confirmed=False),))
        self.assertEqual(result[-1].color_code, 6)
        self.assertFalse(result[-1].position_confirmed)

    def test_full_color_observations_preserved(self):
        objects = self.objects()
        result = self.infer(objects[:-1]+(replace(objects[-1], color_code=6, color_confirmed=True),))
        self.assertTrue(all(r.source == 'VISION_CONFIRMED' for r in result))

    def test_one_color_insufficient(self):
        objects = self.objects()
        with self.assertRaises(ValueError):
            self.infer((objects[0], replace(objects[1], color_confirmed=False), objects[2]))

    def test_six_objects_or_unconfirmed_group_not_forced_to_remaining_color(self):
        with self.assertRaises(ValueError):
            self.infer(self.objects()*2)
        with self.assertRaises(ValueError):
            self.infer(closed_group_confirmed=False)

    def test_conflicting_duplicate_color_and_duplicate_id_rejected(self):
        objects = self.objects()
        for changed in (replace(objects[1], color_code=1), replace(objects[1], color_code=2),
                        replace(objects[1], object_id=objects[0].object_id)):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.infer((objects[0], changed, objects[2]))

    def test_stale_cross_batch_absent_and_future_evidence_rejected(self):
        objects = self.objects()
        for changed in (replace(objects[2], observed_at=9.), replace(objects[2], observed_at=11.),
                        replace(objects[2], context_id='run1/batch2/groupA'),
                        replace(objects[2], present=False)):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                self.infer(objects[:-1]+(changed,))


class FixedCarrierTests(unittest.TestCase):
    def setUp(self):
        # 故意令颜色2位于槽3，防止把颜色号、抓取序号当槽号。
        self.slots = (SlotInventory('run1/batch1', 1, 'redA', 1, True, True),
                      SlotInventory('run1/batch1', 2, 'blueA', 3, True, True),
                      SlotInventory('run1/batch1', 3, 'yellowA', 2, True, True))

    def select(self, slots=None, **kw):
        args = dict(context_id='run1/batch1', arm_clear=True, base_stopped=True,
                    hand_empty=True, active_slot=1, position_confirmed=True, table_stopped=True)
        args.update(kw)
        return select_fixed_pick(2, self.slots if slots is None else slots, **args)

    def test_color_two_requests_slot_three(self):
        result = self.select()
        self.assertEqual((result.step, result.target_slot), ('INDEX_TARGET_SLOT', 3))
        self.assertFalse(result.motion_allowed)

    def test_fixed_station_pick_uses_verified_inventory_without_color_rescan(self):
        self.assertEqual(self.select(active_slot=3).step, 'PICK_FROM_FIXED_STATION')

    def test_preindex_allowed_after_arm_clear_while_carrying_but_next_pick_waits(self):
        self.assertEqual(self.select(hand_empty=False).step, 'INDEX_TARGET_SLOT')
        self.assertEqual(self.select(hand_empty=False, active_slot=3).step, 'WAIT_CURRENT_PLACEMENT')
        self.assertEqual(self.select(hand_empty=False, arm_clear=False).step, 'WAIT_ARM_CLEAR')

    def test_rotation_or_unknown_pose_or_base_motion_block_pick(self):
        for args, expected in ((dict(table_stopped=False), 'WAIT_TABLE_STOP'),
                               (dict(position_confirmed=False), 'RELOCALIZE_TABLE'),
                               (dict(active_slot=None), 'RELOCALIZE_TABLE'),
                               (dict(base_stopped=False), 'WAIT_BASE_STOP')):
            with self.subTest(args=args):
                self.assertEqual(self.select(**args).step, expected)

    def test_unknown_inventory_cross_batch_duplicate_instance_rejected(self):
        for changed in (replace(self.slots[0], verified=False),
                        replace(self.slots[0], context_id='run2/batch1'),
                        replace(self.slots[0], material_id='yellowA')):
            with self.subTest(changed=changed):
                self.assertEqual(self.select((changed,)+self.slots[1:]).step, 'RECONCILE_INVENTORY')

    def test_ack_or_empty_slot_alone_cannot_commit_pick(self):
        for held, empty in ((False, True), (True, False), (False, False)):
            with self.subTest(held=held, empty=empty), self.assertRaises(ValueError):
                commit_slot_pick(self.slots[2], context_id='run1/batch1', material_id='yellowA',
                                 held_confirmed=held, slot_empty_confirmed=empty)

    def test_pick_commit_matches_instance_and_cannot_repeat_empty_slot(self):
        result = commit_slot_pick(self.slots[2], context_id='run1/batch1', material_id='yellowA',
                                  held_confirmed=True, slot_empty_confirmed=True)
        self.assertFalse(result.occupied)
        for slot, context, mid in ((result, 'run1/batch1', 'yellowA'),
                                   (self.slots[2], 'run2/batch1', 'yellowA'),
                                   (self.slots[2], 'run1/batch1', 'wrong')):
            with self.subTest(slot=slot, context=context), self.assertRaises(ValueError):
                commit_slot_pick(slot, context_id=context, material_id=mid,
                                 held_confirmed=True, slot_empty_confirmed=True)
        self.assertEqual(self.select(self.slots[:2]+(result,)).step, 'RECONCILE_INVENTORY')


if __name__ == '__main__':
    unittest.main()
