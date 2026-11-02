import unittest

from p1_serial_motion import Controller
from test_p1_serial_motion import record
from p1_route_preview import validate


class TurnTrialTests(unittest.TestCase):
    def ready(self, **changes):
        r=record(motion_mode=3,debug_plan_id=0,sequence_count=1,
                 base_rpm=27,feedback_limit_rpm=40,action_kind=3)
        r.update(changes)
        return r

    def test_explicit_opt_in_required(self):
        r=self.ready()
        self.assertFalse(Controller('run-once',3,0)._healthy(r))
        self.assertTrue(Controller('run-once',3,0,turn_trial=True)._healthy(r))
        self.assertFalse(Controller('run-once',3,0,route_trial=True)._healthy(r))

    def test_opt_in_rejects_wrong_mode_action_plan_or_other_trial(self):
        for args,kwargs in [
            (('monitor',3,0),{}), (('run-once',1,0),{}),
            (('run-once',3,None),{}), (('run-once',3,7),{}),
            (('run-once',3,0),{'route_trial':True}),
        ]:
            with self.assertRaises(ValueError): Controller(*args,turn_trial=True,**kwargs)

    def test_shape_mismatch_never_arms(self):
        for change in [dict(base_rpm=13),dict(feedback_limit_rpm=80),
                       dict(sequence_count=2),dict(debug_plan_id=1),dict(distance_mode=1)]:
            c=Controller('run-once',3,0,turn_trial=True)
            for n in range(1,6):
                self.assertEqual(c.consume(self.ready(seq=n,tick_ms=n*100,**change),n*.1),[])
            self.assertFalse(c.start_sent)

    def test_profile_change_after_start_stops_without_retry(self):
        for change in [dict(base_rpm=13),dict(feedback_limit_rpm=80),
                       dict(sequence_count=2),dict(debug_plan_id=1),
                       dict(distance_mode=1),dict(action_kind=4),dict(correction_rpm=28)]:
            c=Controller('run-once',3,0,turn_trial=True)
            c.start_sent=True;c.stage='running'
            messages=c.consume(self.ready(**change),1)
            self.assertEqual(c.status,'turn_trial_profile_changed')
            self.assertEqual(len(messages),1)
            self.assertEqual(c.tick(2),[])

    def test_complete_requires_four_wheel_stop_and_vm_off(self):
        for mask,vm,complete in [(7,0,False),(15,1,False),(15,0,True)]:
            c=Controller('run-once',3,0,turn_trial=True)
            c.start_sent=True;c.stage='running'
            c.consume(self.ready(phase=3,backend_result=1,stop_mask=mask,vm_request=vm,
                                 sequence_state=3,sequence_completed=1),1)
            self.assertEqual(c.status=='software_complete',complete)

    def test_preview_turn30_is_separate_from_turn15(self):
        rows=[('TURN',90.,30.)]
        self.assertFalse(validate(rows,(77,212,206))['ok'])
        self.assertFalse(validate(rows,(77,212,206),turn_trial=True)['ok'])
        r=validate(rows,(77,212,206),turn_trial=True,turn_dps=30)
        self.assertTrue(r['ok']);self.assertEqual(r['rows'][0]['rpm'],27)
        self.assertFalse(validate(rows*2,(77,212,206),turn_trial=True,turn_dps=30)['ok'])


if __name__=='__main__':unittest.main()
