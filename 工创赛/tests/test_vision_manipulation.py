from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
import unittest
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT / 'src'))
raises=unittest.TestCase().assertRaises

from smart_carrier.task_materials import SlotInventory, resolve_preliminary_task
from smart_carrier.vision_manipulation import ArmVisionModule, Feedback, Job, Operation, Parameters, Phase, Zone, ZoneCoordinator, load_parameters
from smart_carrier.vision_manipulation.types import Arrival, Target
from smart_carrier.vision_manipulation.source import PhysicalObject, resolve_source_objects
from smart_carrier.vision_manipulation.ring import RingAligner
from smart_carrier.vision_manipulation.alignment import AlignmentPlanner
from smart_carrier.vision_manipulation.ports import ActuatorPort, ChassisPort, split_axis_correction
from smart_carrier.vision_manipulation.task_plan import batch_jobs, recovery_from_placements

ROOT=Path(__file__).resolve().parents[1]


def params(taught=True):
    p=load_parameters(ROOT/'config/vision_manipulation_20261112.json')
    if taught:
        v=p.values;v['physical_limits']['lift_descent_max_mm']=100
        v['carrier']['pick_pose']['pick_depth_mm']=55
        v['carrier']['place_pose']['place_depth_mm']=10
        for zone in ('rough','storage'):
            v['zones'][zone]['positions']={str(i):dict(yaw_deg=yaw,extension_mm=ext,place_depth_mm=70,recover_depth_mm=68)
                for i,yaw,ext in [(1,195,160),(2,180,0),(3,165,160)]}
    return p


def seed(arm, colors=(2,1,5), context='run'):
    arm.ledger.slots={i:SlotInventory(context,i,f'object-{i}',c,True,True) for i,c in enumerate(colors,1)}


def finish(arm, *, include_evidence=True, start_ms=0, stop_at=None):
    commands=[];now=start_ms
    for _ in range(100):
        cmd=arm.tick(now)
        if cmd is None:break
        commands.append(cmd)
        if stop_at and cmd.kind==stop_at:break
        evidence={k:True for k in cmd.evidence_required} if include_evidence else {}
        arm.feedback(Feedback(cmd.context_id,cmd.sequence,'DONE',evidence),now+1)
        now+=2
    return commands,now


def test_task_jobs_preserve_task_positions_manual_load_and_same_colour_stack():
    task=resolve_preliminary_task('215+213+512+321',1,ROOT/'config/task_materials_preliminary_2027.json')
    jobs=batch_jobs(task,1,'run',supervised=True,manual_load_before_rough=True)
    assert [j.operation for j in jobs]==[Operation.LOAD_SOURCE,Operation.MANUAL_LOAD,Operation.PLACE,Operation.RECOVER,Operation.PLACE]
    assert jobs[2].colors==(2,1,5) and jobs[2].positions==(2,1,3)
    second=batch_jobs(task,2,'run2')
    assert second[-1].operation==Operation.STACK and second[-1].positions==(3,1,2)
    with raises(ValueError):batch_jobs(task,1,'run',manual_load_before_rough=True)


def test_missing_teaching_waits_without_fault_or_motion_then_resumes():
    p=params(False);arm=ArmVisionModule(p);seed(arm)
    arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,),(2,)),0)
    finish(arm)
    assert arm.phase==Phase.WAIT_PARAMETER and arm.pending is None
    assert arm.ledger.correct_picks==0 and arm.ledger.slots[1].occupied
    arm.update_parameters(params(),10);commands,_=finish(arm,start_ms=10)
    assert any(c.kind=='move_pose' for c in commands) and arm.phase==Phase.DONE


def test_ring_three_place_recover_order_inventory_and_verified_counts():
    arm=ArmVisionModule(params());seed(arm)
    arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,1,5),(2,1,3)),0)
    commands,now=finish(arm)
    assert arm.phase==Phase.DONE and [p.position for p in arm.ledger.placements]==[2,1,3]
    assert arm.ledger.correct_picks==3 and arm.ledger.correct_places==3 and not arm.ledger.held
    assert all(not s.occupied for s in arm.ledger.slots.values())
    # Open -> measured half-ascent preclose -> verified placement.
    preclose=[c for c in commands if 'preclose_at_ascent_fraction' in c.goal]
    assert len(preclose)==3 and all(c.goal['preclose_at_ascent_fraction']==.5 and c.goal['preclose_angle_deg']==160 for c in preclose)
    recover=recovery_from_placements(arm,'run',Zone.ROUGH)
    assert recover.colors==(2,1,5) and recover.positions==(2,1,3)
    arm.begin(recover,now+10);finish(arm,start_ms=now+10)
    assert arm.phase==Phase.DONE and all(s.occupied for s in arm.ledger.slots.values())
    assert all(p.recovered for p in arm.ledger.placements)
    assert arm.ledger.correct_picks==6 and arm.ledger.correct_places==6


def test_backend_done_without_physical_evidence_does_not_deduct_inventory():
    arm=ArmVisionModule(params());seed(arm)
    arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,),(2,)),0)
    commands,now=finish(arm,stop_at='verify_held')
    c=commands[-1]
    arm.feedback(Feedback(c.context_id,c.sequence,'DONE'),now)
    assert arm.phase==Phase.WAIT_EVIDENCE and arm.ledger.correct_picks==0 and arm.ledger.slots[1].occupied
    assert not arm.feedback(Feedback('old-run',c.sequence,'DONE',{'held':True,'source_clear':True}),now)
    arm.feedback(Feedback(c.context_id,c.sequence,'DONE',{'held':True,'source_clear':True}),now+1)
    assert not arm.ledger.slots[1].occupied and arm.ledger.correct_picks==1


def test_new_engineering_speeds_and_reachable_extension_not_equality_locked():
    p=params();p.values['tuning'].update(extension_rpm=200,lift_rpm=2500,stepper_accel_rpm_s=600)
    p.values['zones']['rough']['positions']['2']['extension_mm']=165
    arm=ArmVisionModule(p);seed(arm);arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,),(2,)),0)
    commands,_=finish(arm)
    assert arm.phase==Phase.DONE
    assert any(c.goal.get('speeds',{}).get('extension_rpm')==200 for c in commands)
    assert any(c.goal.get('pose',{}).get('extension_mm')==165 for c in commands)


def test_actual_overtravel_waits_before_pick_but_does_not_fault_lock():
    p=params();p.values['zones']['rough']['positions']['2']['extension_mm']=171
    arm=ArmVisionModule(p);seed(arm);arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,),(2,)),0)
    commands,_=finish(arm)
    assert arm.phase==Phase.WAIT_PARAMETER and not any(c.kind=='move_pose' for c in commands)
    assert arm.ledger.correct_picks==0


def test_stack_uses_height60_less_descent_and_requires_actual_same_colour_base():
    arm=ArmVisionModule(params());seed(arm)
    arm.begin(Job('run',Zone.STORAGE,Operation.STACK,(2,),(2,)),0)
    commands,now=finish(arm,stop_at='verify_base');c=commands[-1]
    arm.feedback(Feedback(c.context_id,c.sequence,'DONE',{'base_present':True}),now)
    assert arm.phase==Phase.WAIT_EVIDENCE and arm.ledger.correct_picks==0
    arm.feedback(Feedback(c.context_id,c.sequence,'DONE',{k:True for k in c.evidence_required}),now+1)
    commands,_=finish(arm,start_ms=now+2)
    assert arm.phase==Phase.DONE and any(c.goal.get('pose',{}).get('descent_mm')==10 for c in commands)


def test_manual_load_is_2second_three_angles_but_not_fake_occupancy():
    arm=ArmVisionModule(params());job=Job('run',Zone.ROUGH,Operation.MANUAL_LOAD,(2,1,5),(2,1,3),supervised=True)
    c=arm.begin(job,0)
    assert c.goal['angles_deg']==[0,120,240] and c.goal['interval_ms']==2000
    arm.feedback(Feedback('run',c.sequence,'DONE'),10000)
    assert arm.phase==Phase.WAIT_EVIDENCE and not arm.ledger.slots
    arm.feedback(Feedback('run',c.sequence,'DONE',{k:True for k in c.evidence_required}),11000)
    assert arm.phase==Phase.DONE and [s.color_code for s in arm.ledger.slots.values()]==[2,1,5]


def test_source_requires_fresh_true_target_then_preserves_native_pzq_action():
    arm=ArmVisionModule(params());arm.begin(Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,)),0)
    finish(arm)
    assert arm.phase==Phase.WAIT_VISION
    target=Target('run','real-yellow',2,{'yaw_deg':195,'extension_mm':166},10,True,True,True)
    arm.offer_targets((target,),2000)
    assert arm.pending is None
    arm.offer_targets((target,),20);c=arm.pending
    assert c.kind=='source_pick_load' and c.goal['source_pick_native_pulses']==118800
    assert c.goal['slot']==1 and c.goal['color']==2
    arm.feedback(Feedback('run',c.sequence,'DONE',{k:True for k in c.evidence_required}),21)
    finish(arm,start_ms=22)
    assert arm.phase==Phase.DONE and arm.ledger.slots[1].material_id=='real-yellow'


def test_two_colours_infer_one_present_positioned_object_without_using_shadow():
    def obj(i,colors,**kw):
        return PhysicalObject(f'id{i}',str(i),100,True,{'yaw_deg':180,'extension_mm':0},True,True,colors,**kw)
    objects=(obj(1,(2,)),obj(2,(1,)),obj(3,()))
    targets=resolve_source_objects((2,1,5),objects,context_id='run',now_ms=110,closed_group_confirmed=True)
    assert [x.color_code for x in targets]==[2,1,5] and targets[-1].identity_source=='TASK_REMAINDER_INFERRED'
    with raises(ValueError):resolve_source_objects((2,1,5),objects[:2],context_id='run',now_ms=110,closed_group_confirmed=True)
    with raises(ValueError):resolve_source_objects((2,1,5),(obj(1,(2,)),obj(2,()),obj(3,())),context_id='run',now_ms=110,closed_group_confirmed=True)


def test_ring_uses_current_frame_centre_and_never_activates_source_h():
    p=params();data=SimpleNamespace(detected=True,stable=True,centers_px=((400,540),(960,540),(1500,540)),station_yaw_deg=0)
    detector=SimpleNamespace(detect=lambda f:data)
    aligner=RingAligner(detector,p.zone(Zone.ROUGH),p.tuning)
    result=aligner.process(np.zeros((1080,1920,3),dtype=np.uint8))
    assert result.error_px==(0,0) and result.within_tolerance and result.body_delta_mm is None
    data.stable=False
    assert not aligner.process(np.zeros((1080,1920,3),dtype=np.uint8)).within_tolerance


def test_large_correction_splits_and_each_request_needs_new_view():
    assert split_axis_correction(65,20)==(16.25,)*4
    p=AlignmentPlanner();c=p.next('frame1',65,5,0)
    assert c.amount==20 and c.direction==2
    assert p.next('frame1',45,5,0) is None
    turn=p.next('frame2',0,0,-4.54)
    assert turn.amount==-5 and turn.kind=='turn'
    parameters=params();parameters.values['tuning']['recommended_body_step_mm']=65
    planner=AlignmentPlanner.from_parameters(parameters,dict(max_step_mm=20,min_step_mm=1,min_turn_deg=5,max_turn_deg=10))
    assert planner.next('new',65,0,0).amount==20


def test_matches_preceding_matlab_task_height_and_correction_fixtures():
    p=ROOT/'evidence/integration/2026-11-12/vision-modules/matlab-result.json'
    if not p.exists():raise unittest.SkipTest('MATLAB fixture not included')
    r=json.loads(p.read_text(encoding='utf-8'));assert r['passed'];f=r['fixtures']
    task=resolve_preliminary_task(f['task']['raw'],1,ROOT/'config/task_materials_preliminary_2027.json')
    assert [x.color_code for x in task.batch_1]==f['task']['first_colors']
    assert [x.temporary_ring for x in task.batch_2]==f['task']['second_storage']
    assert f['stack_depth']['base']-f['stack_depth']['height']==f['stack_depth']['result']
    for amount,expected in zip(f['correction_requests'],f['correction_segments']):
        if isinstance(expected,(int,float)):expected=[expected]
        assert list(split_axis_correction(amount,f['correction_cap']))==expected


def test_travel_and_arm_ownership_do_not_overlap_and_stop_needs_two_acks():
    arm=ArmVisionModule(params());calls=[];arrival=[None];failure=[False];acks=[False,False]
    def execute(c):calls.append(('arm',c.kind));return True
    def poll(c):
        return Feedback(c.context_id,c.sequence,'FAILED',reason='real actuator fault') if failure[0] else None
    chassis=ChassisPort(lambda *x:True,lambda c:arrival[0],lambda c:True,lambda c:None,
                        lambda c:True,lambda c:calls.append(('body','stop')),lambda c:acks[0])
    actuator=ActuatorPort(execute,poll,lambda c:calls.append(('arm','stop')),lambda c:acks[1])
    flow=ZoneCoordinator(arm,chassis,actuator);job=Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,))
    flow.request_job(job,{'destination':'source'},0);flow.tick(1)
    assert flow.phase==Phase.TRAVEL and not calls
    arrival[0]=Arrival('run',Zone.SOURCE,True,False,sequence=1);flow.tick(2)
    assert flow.phase==Phase.TRAVEL and not calls
    arrival[0]=Arrival('run',Zone.SOURCE,True,True,sequence=1);flow.tick(3);flow.tick(4)
    assert calls==[('arm','observe')]
    flow.tick(5);assert len(calls)==1
    failure[0]=True;flow.tick(6)
    assert flow.phase==Phase.STOPPING and ('body','stop') in calls and ('arm','stop') in calls
    flow.tick(7);assert flow.phase==Phase.STOPPING
    acks[0]=True;flow.tick(8);assert flow.phase==Phase.STOPPING
    acks[1]=True;flow.tick(9);assert flow.phase==Phase.STOPPED


def test_transient_feedback_does_not_repeat_motor_or_clear_health_fault():
    arm=ArmVisionModule(params());seed(arm);arm.begin(Job('run',Zone.ROUGH,Operation.PLACE,(2,),(2,)),0)
    commands,now=finish(arm,stop_at='move_pose');c=commands[-1]
    arm.feedback(Feedback('run',c.sequence,'TRANSIENT'),now+100)
    assert arm.tick(now+101).sequence==c.sequence and arm.phase==Phase.WORK
    arm.tick(31000)
    assert arm.phase==Phase.STOPPING and arm.pending.kind=='stop'
    with raises(ValueError):arm.update_parameters(params(),32000)


def test_synthetic_all_zone_jobs_and_stack_finish_with_no_devices():
    from smart_carrier.vision_manipulation.simulation import simulate
    task=resolve_preliminary_task('215+213+512+321',1,ROOT/'config/task_materials_preliminary_2027.json')
    p=params(False);original=deepcopy(p.values);r=simulate(task,p)
    assert len(r['jobs'])==10 and all(x['phase']=='DONE' for x in r['jobs'])
    assert r['hardware_access'] is False and r['actual_field_acceptance'] is False
    assert p.values==original


def test_python_geometry_matches_actual_local_matlab_output():
    from smart_carrier.vision_manipulation.geometry import arm_from_source_xy
    path=ROOT/'evidence/integration/2026-11-11/source-view-entry/pose-current-20261112/matlab-result.json'
    if not path.exists():raise unittest.SkipTest('field MATLAB fixture not included')
    r=json.loads(path.read_text(encoding='utf-8'));assert r['passed']
    for i,p in enumerate(r['current_target_source_mm']):
        actual=arm_from_source_xy(p,r['retained_retracted_pivot_TCP_mm'])
        assert abs(actual['yaw_deg']-r['current_arm_angles_deg'][i])<1e-6
        assert abs(actual['extension_mm']-r['current_extensions_mm'][i])<1e-6


def test_current_parameters_preserve_failed_depth_no_negative_stack_motion():
    p=params();p.values['zones']['storage']['positions']['2']['place_depth_mm']=40
    arm=ArmVisionModule(p);seed(arm);arm.begin(Job('run',Zone.STORAGE,Operation.STACK,(2,),(2,)),0)
    commands,_=finish(arm)
    assert arm.phase==Phase.WAIT_PARAMETER and not any(c.kind=='move_pose' for c in commands)


def test_source_actual_overtravel_never_enters_atomic_motion():
    arm=ArmVisionModule(params());arm.begin(Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,)),0);finish(arm)
    arm.offer_targets((Target('run','bad',2,dict(yaw_deg=180,extension_mm=171),10,True,True,True),),11)
    assert arm.phase==Phase.WAIT_PARAMETER and arm.pending is None


def test_coordinator_action_timeout_dispatches_both_stops_even_if_one_throws():
    arm=ArmVisionModule(params());calls=[]
    def body_stop(ctx):calls.append('body-stop');raise OSError('body stop unconfirmed')
    c=ChassisPort(lambda *x:True,lambda ctx:Arrival(ctx,Zone.SOURCE,True,True,sequence=1),lambda x:True,lambda x:None,
                  lambda x:True,body_stop,lambda x:False)
    a=ActuatorPort(lambda x:True,lambda x:None,lambda ctx:calls.append('arm-stop'),lambda x:False)
    flow=ZoneCoordinator(arm,c,a);job=Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,))
    flow.request_job(job,{},0);flow.tick(1);flow.tick(2)
    # Observation waits rather than faulting. Move to actual alignment with a
    # genuine sequence-matched completion before exercising action timeout.
    command=arm.pending;arm.feedback(Feedback('run',command.sequence,'DONE'),3)
    flow.tick(4);flow.tick(31005)
    assert flow.phase==Phase.STOPPING and calls==['body-stop','arm-stop']
    flow.tick(32000);assert flow.phase==Phase.STOPPING and calls==['body-stop','arm-stop']


def test_old_same_zone_arrival_cannot_complete_a_new_travel_request():
    arm=ArmVisionModule(params());calls=[];reply=[Arrival('run',Zone.SOURCE,True,True,sequence=1)]
    body=ChassisPort(lambda *x:True,lambda ctx:reply[0],lambda x:True,lambda x:None,
                     lambda x:True,lambda x:None,lambda x:True)
    actuator=ActuatorPort(lambda c:(calls.append(c.kind) or True),lambda x:None,lambda x:None,lambda x:True)
    flow=ZoneCoordinator(arm,body,actuator);job=Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,))
    flow.request_job(job,{},0)
    # End an earlier mocked leg, then request the same area again.
    flow.phase=Phase.DONE;arm.phase=Phase.DONE
    flow.request_job(job,{},1);flow.tick(2)
    assert flow.phase==Phase.TRAVEL and not calls
    reply[0]=Arrival('run',Zone.SOURCE,True,True,sequence=2);flow.tick(3);flow.tick(4)
    assert calls==['observe']


def test_normal_ambiguous_or_stale_source_data_is_soft_wait():
    from smart_carrier.vision_manipulation.source import observe_source_objects
    objects=tuple(PhysicalObject(str(i),'left',0,True,None,False,True,()) for i in range(6))
    r=observe_source_objects((2,1,5),objects,context_id='run',now_ms=100,closed_group_confirmed=False)
    assert not r.observation_usable and not r.targets and r.reason


def test_rejected_source_motion_requires_a_fresh_position_after_parameter_edit():
    arm=ArmVisionModule(params());arm.begin(Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,)),0);finish(arm)
    t=Target('run','real',2,dict(yaw_deg=195,extension_mm=160),10,True,True,True)
    arm.offer_targets((t,),20);c=arm.pending
    arm.feedback(Feedback('run',c.sequence,'PARAMETER_REJECTED',reason='unsupported trial speed'),21)
    assert arm.phase==Phase.WAIT_PARAMETER
    arm.update_parameters(params(),5000)
    assert arm.phase==Phase.WAIT_VISION and arm.pending is None


def test_dynamic_deadline_needs_measured_budget_and_never_replays_old_window():
    arm=ArmVisionModule(params());arm.begin(Job('run',Zone.SOURCE,Operation.LOAD_SOURCE,(2,),(2,)),0);finish(arm)
    t=Target('run','real',2,dict(yaw_deg=195,extension_mm=160),10,True,True,True,usable_until_ms=100)
    arm.offer_targets((t,),20)
    assert arm.phase==Phase.WAIT_PARAMETER and arm.pending is None
    p=params();p.values['tuning']['source_pick_exit_budget_ms']=200
    arm.update_parameters(p,30)
    assert arm.phase==Phase.WAIT_VISION and arm.pending is None


def load_tests(loader, tests, pattern):
    return unittest.TestSuite(unittest.FunctionTestCase(f) for name,f in globals().items() if name.startswith('test_') and callable(f))

if __name__ == '__main__':
    unittest.main()
