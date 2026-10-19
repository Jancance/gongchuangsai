import unittest

from smart_carrier.task_code import DualChannelConsensus, parse_task_code


class TaskCodeParserTests(unittest.TestCase):
    def setUp(self):
        self.parser_kwargs = {"official_mapping_confirmed": False}

    def test_parse_four_segments(self):
        task = parse_task_code("156+123+516+231", start_id=2, **self.parser_kwargs)

        self.assertEqual(task.raw_text, "156+123+516+231")
        self.assertEqual(task.start_id, 2)
        self.assertEqual(task.batch_1.pickup_order, (1, 5, 6))
        self.assertEqual(task.batch_1.rough_positions, (1, 2, 3))
        self.assertEqual(task.batch_2.pickup_order, (5, 1, 6))
        self.assertEqual(task.batch_2.rough_positions, (2, 3, 1))
        self.assertFalse(task.executable)
        self.assertEqual(task.rejection_reason, "QR-04_MAPPING_UNCONFIRMED")

    def test_invalid_codes_are_rejected(self):
        bad_codes = (
            "156+123+516",
            "156-123-516-231",
            "156+120+516+231",
            "156+123+516+223",
            "156+123+51A+231",
        )
        for raw in bad_codes:
            with self.subTest(raw=raw):
                with self.assertRaises(ValueError):
                    parse_task_code(raw, start_id=1, **self.parser_kwargs)

    def test_invalid_start_id_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_task_code("156+123+516+231", start_id=3, **self.parser_kwargs)

    def test_consensus_rejects_only_after_second_channel_is_stably_conflicting(self):
        consensus = DualChannelConsensus(window=3, minimum_votes=2)
        self.assertFalse(consensus.add("scanner", "156+123+516+231").accepted)
        self.assertTrue(consensus.add("scanner", "156+123+516+231").accepted)
        # One provisional frame from the backup channel must not invalidate an
        # already stable scanner result.  A stable conflicting winner must.
        self.assertTrue(consensus.add("opencv", "156+123+516+132").accepted)
        decision = consensus.add("opencv", "156+123+516+132")
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "CHANNEL_CONFLICT")


if __name__ == "__main__":
    unittest.main()
