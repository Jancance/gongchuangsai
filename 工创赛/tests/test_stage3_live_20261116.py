"""Regression tests for the live tool's real outcome and STOP evaluators.

Synthetic MCU samples only: no probe import, connection or actuator writes.
"""
import importlib.util
from pathlib import Path
import unittest
from functools import partial
from unittest.mock import patch
import sys
from datetime import datetime

spec=importlib.util.spec_from_file_location('stage3_live',Path(__file__).resolve().parents[1]/'scripts/stage3_live_20261116.py')
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)

def idle(**values):
    sample={k:0 for k in (*live.ERROR_FIELDS,*live.EXTRA_FIELDS)}
    sample.update(g_stage3_request=0,g_stage3_state=7,g_it_state=2,g_it_request=0,
                  g_it_action=3,g_it_sequence=21,g_it_elapsed_ms=850,
                  g_source_body_step_request=0,g_source_body_step_state=0,
                  g_chassis_auto_active=0,g_startup_axes_state=4,
                  g_source_align_command_seq=0,
                  g_vehicle_arm_stop_pending=0,g_lift_ruler_stop_pending=0,
                  g_source_parallel_stop_mask=0,body_stop_masks=[0,0,0])
    sample.update(values);return sample

it_done=partial(live.it_completed,request=3)

class LiveOutcomeTest(unittest.TestCase):
    def test_touchdown_rejects_stale_done_even_after_busy_and_checks_action(self):
        previous=idle(g_it_action=18,g_vehicle_lift_feedback_valid=1,lift_depth_pulses=60000)
        out=live.ActionOutcome('touchdown_z',previous,live.touchdown_completed)
        for sample in (previous,idle(g_it_action=18,g_it_state=1),
                       idle(g_it_action=18,g_it_elapsed_ms=100),idle(g_it_action=3,g_it_sequence=22),
                       idle(g_it_action=18,g_it_request=18,g_it_sequence=22)):
            self.assertFalse(out.observe(sample))
        self.assertFalse(out.observe(dict(previous,g_it_sequence=22,g_vehicle_lift_feedback_valid=0)))
        self.assertFalse(out.observe(dict(previous,g_it_sequence=22,lift_depth_pulses=float('nan'))))
        self.assertTrue(out.observe(dict(previous,g_it_sequence=22,lift_depth_pulses=60206)))

    def test_touchdown_rejection_not_reported_as_complete(self):
        previous=idle(g_it_action=18)
        out=live.ActionOutcome('touchdown_z',previous,partial(live.it_completed,request=18))
        self.assertFalse(out.observe(idle(g_it_action=18,g_it_sequence=22,g_it_rejected=5)))
        self.assertEqual(out.first_error['values'],{'g_it_rejected':5})

    def test_rejected_same_old_action_is_failure_and_retains_first_error(self):
        previous=idle();out=live.ActionOutcome('it_axis',previous,it_done)
        self.assertFalse(out.observe(idle(g_it_rejected=6)))
        self.assertFalse(out.observe(idle(g_it_sequence=22,g_it_cause=8)))
        self.assertEqual(out.first_error['values'],{'g_it_rejected':6})

    def test_old_done_cannot_complete_axis_or_return(self):
        for action,req in [('it_axis',3),('it_return',4)]:
            previous=idle(g_it_action=req)
            out=live.ActionOutcome(action,previous,partial(live.it_completed,request=req))
            self.assertFalse(out.observe(previous.copy()))

    def test_accepted_axis_sequence_completes_even_if_busy_sample_missed(self):
        out=live.ActionOutcome('it_axis',idle(),it_done)
        self.assertTrue(out.observe(idle(g_it_sequence=22)))

    def test_admission_during_snapshot_does_not_mix_old_done_with_new_sequence(self):
        previous=idle();mixed=idle(g_it_sequence=22)
        current=idle(g_it_sequence=22,g_it_state=1)
        reads=[]
        def read(name):
            reads.append(name);return current[name]
        sample=live.refresh_it_evidence(mixed,read)
        out=live.ActionOutcome('it_axis',previous,it_done)
        self.assertFalse(out.observe(sample))
        self.assertLess(reads.index('g_it_sequence'),reads.index('g_it_state'))

    def test_return_needs_current_progress(self):
        previous=idle(g_it_action=4)
        out=live.ActionOutcome('it_return',previous,partial(live.it_completed,request=4))
        self.assertFalse(out.observe(idle(g_it_action=4,g_it_state=1,g_it_elapsed_ms=0)))
        self.assertTrue(out.observe(idle(g_it_action=4,g_it_elapsed_ms=850)))

    def test_fault_overrides_otherwise_completed_action(self):
        out=live.ActionOutcome('center_only',idle(),lambda s:True)
        self.assertFalse(out.observe(idle(g_stage3_fault=12)))

    def test_body_requires_consumed_request_and_new_command_sequence(self):
        out=live.ActionOutcome('body_axis',idle(),lambda s:True)
        self.assertFalse(out.observe(idle()))
        self.assertFalse(out.observe(idle(g_source_align_command_seq=1,g_stage3_request=20)))
        self.assertTrue(out.observe(idle(g_source_align_command_seq=1)))

    def test_stop_waits_for_pending_and_wheel_ack(self):
        before=idle(g_it_state=1,g_chassis_auto_active=1,g_source_body_step_submitted=1)
        stopped=idle(g_stage3_state=9,g_stage3_fault=2,g_it_state=9,g_it_cause=2,
                     g_source_body_step_state=9,g_source_body_step_reason=9,
                     g_dm_jog_state=8,g_dm_jog_stop_result=1,body_stop_masks=[0,0,15])
        out=live.ActionOutcome('stop',before,lambda s:live.stop_acknowledged(before,s))
        for update in ({'g_vehicle_arm_stop_pending':2},{'g_lift_ruler_stop_pending':4},
                       {'g_source_body_step_state':8},{'body_stop_masks':[0,0,7]},
                       {'g_dm_posvel_holding':1},{'g_dm_jog_stop_result':2},
                       {'g_lift_ruler_stop_unconfirmed':1}):
            self.assertFalse(out.observe(dict(stopped,**update)),update)
        self.assertTrue(out.observe(stopped))
        self.assertIsNone(out.first_error)

    def test_idle_stop_does_not_claim_holding_was_disabled(self):
        before=idle(g_dm_posvel_holding=1,g_dm_can2_motor_state=1)
        after=dict(before,g_stage3_state=9,g_stage3_fault=2)
        self.assertTrue(live.stop_acknowledged(before,after))
        self.assertEqual(after['g_dm_posvel_holding'],1)

class ManifestMetadataTests(unittest.TestCase):
    def test_utf8_manifest_with_real_chinese_paths(self):
        import tempfile,json,hashlib
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'中文候选';root.mkdir()
            axf=root/'固件.axf';axf.write_bytes(b'offline-only')
            manifest=root/'清单.json'
            manifest.write_text(json.dumps(dict(build_passed=True,changes={},unchanged_sha256={},
                project=str(root),images={'Vehicle_LiftRuler':dict(axf=str(axf),sha256=hashlib.sha256(axf.read_bytes()).hexdigest())},
                stage3_recover_placed_request=32),ensure_ascii=False),encoding='utf-8')
            image=live.manifest(manifest)
            self.assertEqual(image['axf'],str(axf))
            self.assertEqual(image['stage3_recover_placed_request'],32)

    def test_ring2_replay_latch_only_permitted_at_idle_bench(self):
        image=dict(capabilities=dict(stage3_ring2_replay=True))
        before=idle(g_stage3_state=0,g_stage3_replay_enable=0,g_stage3_replay_state=0,
                    g_region_dispatch_area=3,g_region_dispatch_test_enable=1)
        live.replay_arm_admission(image,before)
        live.replay_arm_admission(image,dict(before,g_stage3_state=11))
        with self.assertRaises(ValueError):live.replay_arm_admission({},before)
        for change in (dict(g_stage3_state=7),dict(g_stage3_replay_enable=1),
                       dict(g_stage3_replay_state=3),dict(g_stage3_request=1),
                       dict(g_region_dispatch_test_enable=0),dict(g_chassis_auto_active=1),
                       dict(g_it_request=18)):
            with self.assertRaises(ValueError):live.replay_arm_admission(image,dict(before,**change))

    def test_ring2_replay_status_requires_terminal_step_and_relocked_wheels(self):
        final=idle(g_stage3_replay_enable=0,g_stage3_replay_state=5,g_stage3_replay_step=9,
                   g_stage3_align_done=1,g_low_wheel_hold_ready=1,wheel_hold_results=[1]*4)
        self.assertTrue(live.replay_status(final)['compensation_ready'])
        self.assertEqual(live.replay_status(final)['compensation_mode'],'nine_step_replay')
        self.assertNotIn(live.INTEGRATED_COMPENSATION_FIELD,live.replay_status(final))
        for change in (dict(g_stage3_replay_state=4),dict(g_stage3_replay_step=8),
                       dict(g_stage3_state=6),dict(wheel_hold_results=[1,1,1,0]),
                       dict(g_stage3_fault=3)):
            self.assertFalse(live.replay_status(dict(final,**change))['compensation_ready'])

    def test_integrated_visual_status_requires_real_marker_and_zero_replay_steps(self):
        aligned=idle(g_stage3_replay_enable=0,g_stage3_replay_state=5,g_stage3_replay_step=0,
                     g_stage3_align_done=1,g_low_wheel_hold_ready=1,wheel_hold_results=[1]*4)
        self.assertFalse(live.replay_status(aligned)['compensation_ready'])
        integrated=dict(aligned,g_stage3_integrated_compensation=1)
        status=live.replay_status(integrated)
        self.assertTrue(status['compensation_ready'])
        self.assertEqual(status['compensation_mode'],'integrated_visual')
        self.assertEqual(status[live.INTEGRATED_COMPENSATION_FIELD],1)
        for change in (dict(g_stage3_replay_step=9),dict(g_stage3_integrated_compensation=2),
                       dict(g_stage3_integrated_compensation=True),dict(g_low_wheel_hold_ready=0)):
            self.assertFalse(live.replay_status(dict(integrated,**change))['compensation_ready'])
        invalid=dict(aligned,g_stage3_integrated_compensation=2,g_stage3_replay_step=9)
        self.assertFalse(live.replay_status(invalid)['compensation_ready'])

    def test_ring2_replay_start_requires_armed_latch_and_new_run(self):
        image=dict(capabilities=dict(stage3_ring2_replay=True))
        before=idle(g_stage3_state=0,g_stage3_run=2,g_stage3_replay_enable=1,g_stage3_replay_state=0,
                    g_region_dispatch_area=3,g_region_dispatch_test_enable=1)
        live.replay_arm_admission(image,before,starting=True)
        for change in (dict(g_stage3_replay_enable=0),dict(g_stage3_state=7),dict(g_stage3_request=1)):
            with self.assertRaises(ValueError):
                live.replay_arm_admission(image,dict(before,**change),starting=True)
        admitted=dict(before,g_stage3_run=3,g_stage3_replay_run=3,
                      g_stage3_replay_enable=0,g_stage3_replay_state=1)
        self.assertTrue(live.replay_started(before,admitted))
        for change in (dict(g_stage3_run=2),dict(g_stage3_request=1),dict(g_stage3_replay_enable=1),
                       dict(g_stage3_replay_state=0),dict(g_stage3_replay_state=9),
                       dict(g_stage3_replay_run=2)):
            self.assertFalse(live.replay_started(before,dict(admitted,**change)))

    def test_touchdown_requires_manifest_before_hardware_import(self):
        import builtins
        original=builtins.__import__
        def guard(name,*args,**kwargs):
            if name in ('gripper210_attach_read_20261105','source_body_probe_20261111'):
                self.fail('Unsupported touchdown imported hardware: '+name)
            return original(name,*args,**kwargs)
        with patch.object(sys,'argv',['stage3_live','--touchdown-z','60000']),\
             patch.object(live,'manifest',return_value={'stage3_lift_max':82000}),\
             patch('builtins.__import__',side_effect=guard),patch.object(sys,'stderr'):
            with self.assertRaises(SystemExit):live.main()
        image={'stage3_lift_max':82000,'capabilities':{'stage3_ring2_touchdown':True}}
        self.assertEqual(live.touchdown_target(60206,image),60206)
        for bad in (3199,82001,float('inf'),float('nan')):
            with self.assertRaises(ValueError):live.touchdown_target(bad,image)

    def test_touchdown_checks_downward_delta_and_referenced_held_bench(self):
        sample=idle(g_region_dispatch_test_enable=1,g_region_dispatch_area=3,
                    g_stage3_align_done=1,g_stage3_manual_state=0,g_low_wheel_hold_ready=1,
                    wheel_hold_results=[1]*4,g_source_align_chain_active=0,
                    g_lift_compare_top_valid=1,g_lift_compare_backoff_valid=0,
                    g_lift_ruler_state=4,g_extension_home_reference_valid=1,
                    g_extension_hold_confirmed=1,g_vm_test_request=1,
                    g_vehicle_lift_feedback_valid=1,lift_depth_pulses=60000)
        for target in (60001,61200):live.touchdown_admission(target,sample)
        for target in (59999,60000,61201):
            with self.assertRaises(ValueError):live.touchdown_admission(target,sample)
        for update in (dict(g_stage3_state=5),dict(g_stage3_align_done=0),
                       dict(wheel_hold_results=[1,1,1,0]),dict(g_lift_compare_top_valid=0),
                       dict(g_lift_ruler_state=2),dict(g_extension_home_reference_valid=0),
                       dict(g_extension_hold_confirmed=0),dict(g_vm_test_request=0),
                       dict(g_vehicle_lift_feedback_valid=0),dict(g_source_body_step_request=1),
                       dict(g_region_dispatch_test_enable=0),dict(g_it_cause=5)):
            with self.assertRaises(ValueError):live.touchdown_admission(60001,dict(sample,**update))

    def test_teach_sequence_publication_ignores_unrelated_axis_cycles(self):
        before=idle(g_stage3_teach_sequence=8,g_stage3_teach_saved_sequence=8,
                    g_stage3_teach_request=0,g_stage3_teach_result=1)
        out=live.ActionOutcome('teach_kind',before,live.teach_completed)
        self.assertFalse(out.observe(before))
        pending=dict(before,g_stage3_teach_sequence=9,g_stage3_teach_result=2)
        self.assertFalse(out.observe(pending))
        self.assertFalse(out.observe(dict(pending,g_stage3_teach_result=1)))
        self.assertFalse(out.observe(dict(pending,g_stage3_teach_saved_sequence=9)))
        saved=dict(pending,g_stage3_teach_saved_sequence=9,g_stage3_teach_result=1)
        self.assertTrue(out.observe(saved))
        # No change to lift/recovery/DM read cycles is needed for an axis-independent save.
        self.assertEqual(saved['g_lift_diag_cycle'],before['g_lift_diag_cycle'])

    def test_measured_side_only_and_manifest_gate(self):
        image=dict(teach_feedback_mode='measured-axis-v1',teach_measured_x=True,teach_geometry_only_kinds=[6,7],
                   dm_manual_teach_existing_requests=True)
        for kind in (2,4,6,7):
            for measured in (0,94.5,170):live.action_metadata(image,kind,measured)
            for measured in (None,-1,171,float('nan')):
                with self.assertRaises(AssertionError):live.action_metadata(image,kind,measured)
        for kind in (1,3,5):
            live.action_metadata(image,kind)
            with self.assertRaises(AssertionError):live.action_metadata(image,kind,12)
        for kwargs in (dict(teach_kind=1),dict(dm_action=True)):
            with self.assertRaises(AssertionError):live.action_metadata({},**kwargs)
        with self.assertRaises(AssertionError):live.action_metadata(image,measured_x=10)

    def test_release_reuses_actual_top_and_existing_home_tolerances(self):
        sample=idle(g_vehicle_lift_feedback_valid=1,g_vehicle_lift_current_deg=5.,
                    g_lift_ruler_state=4,g_startup_axes_state=4,
                    g_extension_home_reference_valid=1,g_extension_hold_confirmed=1,
                    g_startup_extension_feedback_valid=1,g_vm_test_request=1,
                    g_startup_extension_current_deg=0.,g_extension_home_reference_deg=0.,
                    lift_controller=dict(top_valid=1,backoff_valid=1,top_scale=100,safe_position_raw=500))
        limits=dict(top_deg=22.5,x_zero_deg=17.75)
        live.dm_release_pose(sample,limits)
        for update in ({'g_vehicle_lift_current_deg':28.}, {'g_startup_extension_current_deg':18.},
                       {'g_vehicle_lift_feedback_valid':0},{'g_extension_home_reference_valid':0},
                       {'g_source_body_step_state':2},{'g_chassis_auto_active':1},
                       {'lift_controller':dict(top_valid=1,backoff_valid=0,top_scale=100,safe_position_raw=500)}):
            with self.assertRaises(AssertionError):live.dm_release_pose(dict(sample,**update),limits)

    def test_saved_profile_is_directly_restorable_without_runtime_fields(self):
        sample=dict(g_stage3_carrier_yaw=180,g_stage3_pick_depth=8003.,
                    g_stage3_target_yaw=[-1,180,-1],g_stage3_target_x=[-1,0,-1],
                    g_stage3_target_depth=[-1,-1,-1],g_stage3_teach_result=1,evidence='not-config')
        profile=live.calibration_profile(sample)
        self.assertEqual(live.calibration_values(profile,dict(stage3_lift_max=82000)),profile)
        self.assertNotIn('evidence',profile)

    def test_dm_read_only_can_request_never_jog_or_motion(self):
        before=idle(g_dm_can2_read_request=0,g_dm_can2_state=4,g_dm_can2_result=1,
                    g_dm_jog_state=8,g_dm_can2_feedback_valid=1,g_dm_can2_feedback_tick=10)
        self.assertEqual(live.dm_request('dm_read',before),('g_dm_can2_read_request',2))
        for update in ({'g_dm_posvel_holding':1},{'g_dm_can2_motor_state':1},
                       {'g_dm_jog_state':4},{'g_dm_can2_state':2}):
            with self.assertRaises(AssertionError):live.dm_request('dm_read',dict(before,**update))
        out=live.ActionOutcome('dm_read',before,live.dm_read_completed)
        self.assertFalse(out.observe(before))
        self.assertFalse(out.observe(dict(before,g_dm_can2_read_cycle=1)))
        self.assertTrue(out.observe(dict(before,g_dm_can2_read_cycle=1,g_dm_can2_feedback_tick=11)))

    def test_dm_release_requires_new_real_disable_ack(self):
        before=idle(g_dm_can2_read_request=0,g_dm_can2_feedback_tick=20,
                    g_dm_jog_tx_count=30,g_dm_jog_rx_count=29)
        self.assertEqual(live.dm_request('dm_release',before),('g_dm_jog_request',3))
        stopped=dict(before,g_dm_jog_state=8,g_dm_jog_result=2,g_dm_jog_stop_result=1)
        out=live.ActionOutcome('dm_release',before,live.dm_released)
        self.assertFalse(out.observe(stopped))
        fresh=dict(stopped,g_dm_can2_feedback_tick=21,g_dm_jog_tx_count=31,g_dm_jog_rx_count=30)
        for update in ({'g_dm_jog_request':3},{'g_dm_posvel_holding':1},
                       {'g_dm_can2_motor_state':1},{'g_dm_jog_stop_result':2},
                       {'g_dm_jog_state':6},{'g_dm_jog_result':0}):
            self.assertFalse(out.observe(dict(fresh,**update)))
        self.assertTrue(out.observe(fresh))

    def test_default_main_does_not_import_attach_or_hardware(self):
        import builtins
        original=builtins.__import__
        def guard(name,*args,**kwargs):
            if name in ('gripper210_attach_read_20261105','source_body_probe_20261111'):
                self.fail('Default path imported hardware: '+name)
            return original(name,*args,**kwargs)
        with patch.object(sys,'argv',['stage3_live']),patch.object(live,'manifest',return_value={}),patch('builtins.__import__',side_effect=guard):
            live.main()

    def test_calibration_explicit_subset_and_draft_constraints(self):
        image={'stage3_lift_max':82000}
        measured={'g_stage3_pick_depth':8003.467}
        self.assertEqual(live.calibration_values(measured,image),measured)
        draft={'g_stage3_target_yaw':[-1,180,-1],'g_stage3_target_x':[-1,0,-1],
               'g_stage3_target_depth':[-1,-1,-1]}
        self.assertEqual(live.calibration_values(draft,image),draft)
        for bad in ({'g_stage3_teach_result':1},{'g_stage3_pick_depth':float('nan')},
                    {'g_stage3_pick_depth':82001},{'g_stage3_pick_depth':3199},
                    {'g_stage3_target_yaw':[-1,179,-1]},
                    {'g_stage3_target_x':[-1,1,-1]}):
            with self.assertRaises(AssertionError):live.calibration_values(bad,image)

    def test_preclose_requires_manifest17_and_uses_same_freshness(self):
        for image in ({},{'preclose_request':16}):
            with self.assertRaises(AssertionError):live.grip_request('preclose',image)
        self.assertEqual(live.grip_request('preclose',{'preclose_request':17}),17)
        previous=idle(g_it_action=17)
        out=live.ActionOutcome('grip',previous,partial(live.it_completed,request=17),17)
        self.assertFalse(out.observe(previous))
        self.assertFalse(out.observe(dict(previous,g_it_state=1)))
        self.assertTrue(out.observe(previous))
        failed=live.ActionOutcome('grip',previous,partial(live.it_completed,request=17),17)
        self.assertFalse(failed.observe(dict(previous,g_it_rejected=7,g_it_elapsed_ms=0)))

    def test_slot_and_grip_reject_stale_done_and_accept_fresh_action(self):
        for action,req in (('carrier_slot',10),('grip',5),('grip',6)):
            previous=idle(g_it_action=req)
            outcome=live.ActionOutcome(action,previous,partial(live.it_completed,request=req),req)
            self.assertFalse(outcome.observe(previous))
            self.assertFalse(outcome.observe(dict(previous,g_it_request=req)))
            self.assertFalse(outcome.observe(dict(previous,g_it_state=1)))
            self.assertTrue(outcome.observe(previous))
            fresh=live.ActionOutcome(action,idle(g_it_action=3),partial(live.it_completed,request=req),req)
            self.assertTrue(fresh.observe(previous))
            failed=live.ActionOutcome(action,idle(g_it_action=3),partial(live.it_completed,request=req),req)
            self.assertFalse(failed.observe(dict(previous,g_it_rejected=7)))

    def test_enter_teach_requires_consumed_request_and_real_hold(self):
        sample=dict(g_stage3_request=0,g_stage3_state=7,g_stage3_align_done=0,
                    g_low_wheel_hold_ready=1,g_low_wheel_hold_stage=13)
        self.assertTrue(live.enter_teach_completed(sample))
        for change in ({'g_stage3_request':30},{'g_low_wheel_hold_ready':0},
                       {'g_low_wheel_hold_stage':12},{'g_stage3_align_done':1}):
            self.assertFalse(live.enter_teach_completed(dict(sample,**change)))

    def test_continue_distinguishes_observation_from_placement(self):
        self.assertTrue(live.continue_completed({'g_stage3_align_done':0},{'g_stage3_state':1}))
        self.assertFalse(live.continue_completed({'g_stage3_align_done':0},{'g_stage3_state':5}))
        self.assertTrue(live.continue_completed({'g_stage3_align_done':1},{'g_stage3_state':5}))
        self.assertFalse(live.continue_completed({'g_stage3_align_done':1},{'g_stage3_state':1}))

    def test_stage3_limit_uses_selected_manifest_and_legacy_default(self):
        import hashlib,json,tempfile
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);image=folder/'fixture.axf';image.write_bytes(b'local-only-fixture')
            candidate=dict(build_passed=True,project=str(folder),changes={},unchanged_sha256={},
                images={'Vehicle_LiftRuler':dict(axf=str(image),sha256=hashlib.sha256(image.read_bytes()).hexdigest())})
            path=folder/'candidate.json';path.write_text(json.dumps(candidate))
            self.assertEqual(live.manifest(path)['stage3_lift_max'],29700)
            candidate['stage3_lift_max']=82000;path.write_text(json.dumps(candidate))
            self.assertEqual(live.manifest(path)['stage3_lift_max'],82000)

class RecoverPlacedTests(unittest.TestCase):
    def profile(self):
        return dict(expected_run=4,expected_pose=8,recovery_depth_by_ring={'1':59400,'2':61400,'3':60600})

    def sample(self):
        return idle(g_stage3_state=6,g_stage3_placed_count=3,g_stage3_recovered_count=0,
                    g_stage3_run=4,g_stage3_locked_run=4,g_stage3_pose=8,
                    g_region_dispatch_area=3,g_region_dispatch_test_enable=1,
                    g_low_wheel_hold_ready=1,wheel_hold_results=[1]*4,g_stage3_manual_state=0)

    def test_explicit_depths_and_same_completed_locked_run_required(self):
        image=dict(stage3_lift_max=82000,stage3_recover_placed_request=32,
                   capabilities=dict(stage3_recover_placed=True))
        self.assertEqual(live.recover_placed_depths(self.profile(),image),[59400,61400,60600])
        live.recover_placed_admission(self.profile(),self.sample())
        for change in (dict(g_stage3_locked_run=0),dict(g_stage3_locked_run=3),dict(g_stage3_run=5),
                       dict(g_stage3_pose=9),dict(g_stage3_recovered_count=3),dict(g_stage3_recovered_count=1),
                       dict(g_stage3_placed_count=2),dict(g_it_request=3)):
            with self.assertRaises(ValueError):live.recover_placed_admission(self.profile(),dict(self.sample(),**change))
        with self.assertRaises(ValueError):live.recover_placed_depths(self.profile(),dict(image,capabilities={}))
        p=self.profile();p['recovery_depth_by_ring']['2']=-1
        with self.assertRaises(ValueError):live.recover_placed_depths(p,image)

    def test_stale_done_wrong_run_or_rejection_cannot_complete(self):
        before=self.sample()
        done=partial(live.recover_placed_completed,before)
        out=live.ActionOutcome('recover_placed',before,done)
        self.assertFalse(out.observe(before))
        final=dict(before,g_stage3_recovered_count=3,g_stage3_pose=9)
        for change in (dict(g_stage3_run=5),dict(g_stage3_locked_run=0),dict(g_stage3_pose=8),
                       dict(g_stage3_request=32),dict(g_stage3_recovered_count=2)):
            self.assertFalse(out.observe(dict(final,**change)))
        self.assertTrue(out.observe(final))
        rejected=live.ActionOutcome('recover_placed',before,done)
        self.assertFalse(rejected.observe(dict(final,g_stage3_rejected=1)))
        self.assertFalse(rejected.observe(final))

    def test_check_compiles_recovery_without_hardware_import(self):
        import tempfile,json,builtins
        original=builtins.__import__
        def guard(name,*args,**kwargs):
            if name in ('gripper210_attach_read_20261105','source_body_probe_20261111'):
                self.fail('Recovery check imported hardware: '+name)
            return original(name,*args,**kwargs)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'recover.json';path.write_text(json.dumps(self.profile()),encoding='utf-8')
            image=dict(stage3_lift_max=82000,stage3_recover_placed_request=32,
                       capabilities=dict(stage3_recover_placed=True))
            with patch.object(sys,'argv',['stage3_live','--recover-placed',str(path),'--check']),\
                 patch.object(live,'manifest',return_value=image),patch('builtins.__import__',side_effect=guard):
                live.main()


class StageProfileTests(unittest.TestCase):
    def cycle_image(self):
        return dict(stage3_round_trip=True,stage3_lift_max=82000,stage3_center_measured_xy=True,
                    capabilities=dict(stage3_cycle_modes=True,stage3_three_place=True))

    def test_cycle_archive_defaults_to_place_only_and_recovery_requires_opt_in(self):
        args=('215+123+521+132',1,[2,1,5],1,self.cycle_image())
        z,task=live.round_trip_profile(*args)
        self.assertEqual(task['g_stage3_cycle_mode'],0)
        self.assertEqual(task['g_stage3_first_mapping_valid'],0)
        self.assertNotIn('g_stage3_recovery_depth',z)
        recovery,task=live.round_trip_profile(*args,cycle_mode='recover')
        self.assertEqual(task['g_stage3_cycle_mode'],1)
        self.assertEqual(recovery['g_stage3_recovery_depth'],[59400,61400,60600])
        self.assertEqual(z['g_stage3_target_depth'],recovery['g_stage3_target_depth'])
        with self.assertRaisesRegex(AssertionError,'cycle-mode firmware'):
            live.round_trip_profile(*args[:-1],self.image(),cycle_mode='recover')

    def test_cycle_profiles_do_not_require_unused_clearance_capability(self):
        for mode,expected in (('three-place',0),('rough-round-trip',1),('storage-stack',2)):
            z,task=live.stage_profile(self.profile(mode),self.cycle_image())
            self.assertEqual(task['g_stage3_cycle_mode'],expected)
            self.assertNotIn(live.PLACE_CLEARANCE,z)
            self.assertNotIn('g_stage3_storage_only',task)
            self.assertEqual(z['g_stage3_target_depth'],[60207,60208,60209])
            if expected==1:self.assertEqual(z['g_stage3_recovery_depth'],[59400,61400,60600])
            if expected==2:
                self.assertEqual(task['g_stage3_first_colors'],[2,1,5])
                self.assertEqual(task['g_stage3_first_positions'],[1,2,3])
                self.assertEqual(task['g_stage3_first_mapping_valid'],1)
                self.assertEqual(task['g_stage3_positions'],[3,1,2])
            else:self.assertEqual(task['g_stage3_first_mapping_valid'],0)

    def test_cycle_stack_still_validates_effective_height_and_mapping(self):
        p=self.profile('storage-stack')
        p['rings']['2']['ground_release_depth_pulses']=27199
        with self.assertRaisesRegex(ValueError,'computed release depth'):
            live.stage_profile(p,self.cycle_image())
        p=self.profile('storage-stack');p['verified_stack_base_colors']=[1,2,5]
        with self.assertRaisesRegex(ValueError,'first-batch colors'):
            live.stage_profile(p,self.cycle_image())
        p=self.profile('three-place');p['rings']['2']['recovery_depth_pulses']=61400
        with self.assertRaisesRegex(ValueError,'Place-only'):
            live.stage_profile(p,self.cycle_image())

    def test_three_place_and_integrated_publish_requested_carrier_yaw(self):
        image=self.image()
        image['capabilities']['stage3_three_place']=True
        calibration,_=live.round_trip_profile('215+123+521+132',1,[2,1,5],1,image)
        self.assertEqual(calibration['g_stage3_carrier_yaw'],36.5)
        image['capabilities']['stage3_integrated_visual_compensation']=True
        candidate=self.profile('single-red')
        with self.assertRaisesRegex(ValueError,'36.5'):live.stage_profile(candidate,image)
        candidate['carrier_pick']['yaw_deg']=36.5
        calibration,_=live.stage_profile(candidate,image)
        self.assertEqual(calibration['g_stage3_carrier_yaw'],36.5)

    def image(self, legacy=False):
        return dict(stage3_round_trip=True, stage3_lift_max=82000,stage3_center_measured_xy=True,
                    capabilities={} if legacy else dict.fromkeys(live.PROFILE_CAPS,True))

    def profile(self, mode):
        storage=mode=='storage-stack'
        data=dict(schema_version=1,mode=mode,slot_colors=[2,1,5],
                  carrier_pick=dict(yaw_deg=37,depth_pulses=8000),
                  rings={str(n):dict(yaw_deg=180+n,extension_mm=n,
                         **{('ground_release_depth_pulses' if storage else 'release_depth_pulses'):60206+n})
                         for n in (1,2,3)})
        if mode=='single-red':
            data.update(slot_colors=[0,1,0],rings={'2':data['rings']['2']})
        else:
            data.update(task_code='215+123+521+132',start_id=1,batch=2 if storage else 1)
        if storage:data['verified_stack_base_colors']=[2,1,5]
        if mode=='rough-round-trip':
            for n,z in enumerate((59400,61400,60600),1):
                data['rings'][str(n)]['recovery_depth_pulses']=z
        return data

    def test_single_red_selects_slot_2_ring_2_only_and_legacy_can_compile(self):
        p=self.profile('single-red')
        for legacy,board_slots in ((False,[0,1,0]),(True,[2,1,5])):
            z,task=live.stage_profile(p,self.image(legacy))
            i=task['g_stage3_test_first_item']
            self.assertEqual((i,task['g_stage3_test_count']), (1,1))
            self.assertEqual(task['g_stage3_positions'][i],2)
            self.assertEqual(task['g_stage3_slot_colors'].index(task['g_stage3_colors'][i])+1,2)
            self.assertEqual(task['g_stage3_slot_colors'],board_slots)
            self.assertEqual(z['g_stage3_recovery_depth'],[-1]*3)
            self.assertEqual(z['g_stage3_target_depth'],[-1,60208,-1])
            self.assertEqual(task['g_stage3_task_batch'],0)
            if not legacy:self.assertEqual(task['g_stage3_storage_only'],0)
        self.assertEqual(p['slot_colors'],[0,1,0])

    def test_storage_uses_first_batch_color_with_distinct_rough_order(self):
        z,task=live.stage_profile(self.profile('storage-stack'),self.image())
        self.assertEqual(task['g_stage3_positions'],[3,1,2])
        self.assertEqual(task['g_stage3_storage_only'],1)
        self.assertEqual(z['g_stage3_target_depth'],[36207,36208,36209])
        self.assertEqual(z[live.PLACE_CLEARANCE],[-1]*3)
        self.assertEqual(z[live.RECOVERY_CLEARANCE],[-1]*3)
        self.assertEqual(z['g_stage3_recovery_depth'],[-1]*3)

    def test_rough_release_and_recovery_are_independent(self):
        p=self.profile('rough-round-trip')
        p['rings']['2']['place_clearance_depth_pulses']=40000
        p['rings']['1']['recovery_clearance_depth_pulses']=50000
        z,task=live.stage_profile(p,self.image())
        self.assertEqual(task['g_stage3_storage_only'],0)
        self.assertEqual(z['g_stage3_target_depth'],[60207,60208,60209])
        self.assertEqual(z['g_stage3_recovery_depth'],[59400,61400,60600])
        self.assertEqual(z[live.PLACE_CLEARANCE],[-1,40000,-1])
        self.assertEqual(z[live.RECOVERY_CLEARANCE],[50000,-1,-1])

    def test_height_boundaries_and_rejected_fields(self):
        for ground,valid in ((24000,False),(27199,False),(27200,True),(82000,True),(82001,False)):
            p=self.profile('storage-stack')
            p['rings']['2']['ground_release_depth_pulses']=ground
            if valid:self.assertEqual(live.stage_profile(p,self.image())[0]['g_stage3_target_depth'][1],ground-24000)
            else:
                with self.assertRaises(ValueError):live.stage_profile(p,self.image())
        for update in (dict(slot_colors=[2,1,1]),dict(rings={}),dict(verified_stack_base_colors=[2,5,1]),
                       dict(batch=1),dict(task_code='215+122+521+132')):
            with self.assertRaises(ValueError):live.stage_profile(dict(self.profile('storage-stack'),**update),self.image())
        p=self.profile('rough-round-trip')
        p['rings']['3']['recovery_clearance_depth_pulses']=60600
        with self.assertRaises(ValueError):live.stage_profile(p,self.image())
        with self.assertRaises(ValueError):live.stage_profile(self.profile('rough-round-trip'),self.image(legacy=True))

    def test_stale_run_pose_hold_observation_and_boot_are_rejected(self):
        old=dict(zip(live.PROFILE_CONTEXT,(3,5,2,180.55)),g_src_now_ms=11000)
        doc={'after':old,'when':'2026-11-19T00:00:00'}
        now=datetime.fromisoformat('2026-11-19T00:00:10')
        current=dict(old,g_src_now_ms=21000)
        live.require_profile_pose(doc,current,now)
        for key in (*live.PROFILE_CONTEXT,'g_src_now_ms'):
            with self.assertRaises(ValueError):live.require_profile_pose(doc,dict(current,**{key:1}),now)
        with self.assertRaises(ValueError):live.require_profile_pose(doc,current,datetime.fromisoformat('2026-11-19T00:01:00'))

    def test_check_path_validates_evidence_without_importing_hardware(self):
        import builtins,json,tempfile
        original=builtins.__import__
        def guard(name,*args,**kwargs):
            if name in ('gripper210_attach_read_20261105','source_body_probe_20261111'):
                self.fail('Check imported hardware: '+name)
            return original(name,*args,**kwargs)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            path=root/'profile.json'; observation=root/'stage3-live.json'
            sample=dict(zip(live.PROFILE_CONTEXT,(3,5,2,180.55)),g_src_now_ms=11000,
                        g_stage3_align_done=1,g_low_wheel_hold_ready=1,g_stage3_state=7,wheel_hold_results=[1]*4)
            observation.write_text(json.dumps(dict(after=sample,when='2026-11-19T00:00:00',image=dict(sha256='test'))))
            p=self.profile('single-red')
            p['evidence']=dict(same_fixed_pose=True,stage3_live=str(observation),pi=str(observation))
            p['carrier_pick']['evidence']=p['rings']['2']['evidence']=str(observation)
            path.write_text(json.dumps(p))
            with patch.object(sys,'argv',['stage3_live','--configure-profile',str(path),'--check']),\
                 patch.object(live,'manifest',return_value=dict(self.image(),sha256='test')),\
                 patch('builtins.__import__',side_effect=guard):
                live.main()
            path.write_text('{"rings":{"2":{},"2":{}}}')
            with self.assertRaisesRegex(ValueError,'Duplicate profile field'):live.profile_json(path)

if __name__=='__main__':unittest.main()
