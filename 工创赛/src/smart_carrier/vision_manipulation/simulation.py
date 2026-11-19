"""Explicit synthetic integration replay. Never constructs a hardware port."""
from dataclasses import asdict
from copy import deepcopy
from .arm import ArmVisionModule
from .coordinator import ZoneCoordinator
from .parameters import Parameters
from .ports import ActuatorPort, ChassisPort
from .task_plan import batch_jobs
from .types import Arrival, Feedback, Phase, Target


def simulate(task, parameters):
    v=deepcopy(parameters.values)
    # Synthetic fixture values are never written into the user's profile.
    v['physical_limits']['lift_descent_max_mm']=100
    v['carrier']['pick_pose']['pick_depth_mm']=55;v['carrier']['place_pose']['place_depth_mm']=10
    for zone in ('rough','storage'):
        v['zones'][zone]['positions']={str(i):dict(yaw_deg=yaw,extension_mm=extension,place_depth_mm=70,recover_depth_mm=68)
            for i,yaw,extension in [(1,195,160),(2,180,0),(3,165,160)]}
    arm=ArmVisionModule(Parameters(v));trace=[];destination=[None];stops=[]
    def travel_begin(context,zone,goal):destination[0]=(context,zone,goal['command_sequence']);return True
    def arrival(context):return Arrival(context,destination[0][1],True,True,sequence=destination[0][2])
    def begin(c):trace.append(dict(sequence=c.sequence,kind=c.kind,goal=c.goal));return True
    def done(c):return Feedback(c.context_id,c.sequence,'DONE',{k:True for k in c.evidence_required})
    chassis=ChassisPort(travel_begin,arrival,begin,done,lambda ctx:True,lambda ctx:stops.append('body'),lambda ctx:True)
    actuator=ActuatorPort(begin,done,lambda ctx:stops.append('arm'),lambda ctx:True)
    flow=ZoneCoordinator(arm,chassis,actuator);now=0;results=[]
    for batch in (1,2):
        context=f'synthetic-batch-{batch}'
        for job in batch_jobs(task,batch,context,supervised=True,manual_load_before_rough=True):
            flow.request_job(job,{'synthetic_destination':job.zone.value},now)
            for _ in range(150):
                flow.tick(now)
                if arm.phase==Phase.WAIT_VISION and arm.pending is None:
                    color=job.colors[arm.item]
                    arm.offer_targets((Target(context,f'{context}-item-{arm.item}',color,
                        dict(yaw_deg=180,extension_mm=0),now,True,True,True),),now)
                now+=10
                if flow.phase in (Phase.DONE,Phase.STOPPING,Phase.STOPPED):break
            if flow.phase!=Phase.DONE:raise AssertionError((job,flow.status()))
            results.append(dict(batch=batch,zone=job.zone.value,operation=job.operation.value,phase=flow.phase.value))
    assert not stops
    return dict(synthetic=True,hardware_access=False,synthetic_teaching_used=True,
                actual_field_acceptance=False,task_code=task.raw_code,jobs=results,
                action_count=len(trace),correct_picks_in_fixture=arm.ledger.correct_picks,
                correct_places_in_fixture=arm.ledger.correct_places,trace=trace)
