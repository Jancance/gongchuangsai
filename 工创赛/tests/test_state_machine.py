import unittest

from smart_carrier.models import RobotMode
from smart_carrier.state_machine import InvalidTransition, SafetyStateMachine


class StateMachineTests(unittest.TestCase):
    def test_normal_lifecycle(self):
        machine = SafetyStateMachine()
        machine.transition(RobotMode.SELF_TEST, "config")
        machine.transition(RobotMode.READY, "healthy")
        machine.transition(RobotMode.RUNNING, "one-key")
        machine.transition(RobotMode.DONE, "complete")
        self.assertIs(machine.mode, RobotMode.DONE)

    def test_motion_cannot_start_directly_from_boot(self):
        machine = SafetyStateMachine()
        with self.assertRaises(InvalidTransition):
            machine.transition(RobotMode.RUNNING, "unsafe")

    def test_safe_stop_is_idempotent(self):
        machine = SafetyStateMachine()
        machine.transition(RobotMode.SELF_TEST, "config")
        machine.safe_stop("camera missing")
        machine.safe_stop("still missing")
        self.assertIs(machine.mode, RobotMode.SAFE_STOP)
        self.assertEqual(machine.reason, "still missing")


if __name__ == "__main__":
    unittest.main()
