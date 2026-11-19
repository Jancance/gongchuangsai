"""Area-level ownership switch; main only calls request_job/tick.

Travel belongs to the chassis module. Arrived-and-stopped transfers control to
the vision manipulation module. Alignment is delegated back to the chassis
without enabling general travel or arm XY motion at the same time.
"""
from copy import deepcopy
from .arm import ArmVisionModule
from .ports import ActuatorPort, ChassisPort
from .types import Arrival, Feedback, Job, Phase


class ZoneCoordinator:
    def __init__(self, arm: ArmVisionModule, chassis: ChassisPort, actuator: ActuatorPort):
        self.arm = arm;self.chassis = chassis;self.actuator = actuator
        self.phase = Phase.IDLE;self.reason = '';self.job: Job | None = None
        self._submitted = None;self._navigation_goal = None;self.last_arrival: Arrival | None = None
        self._now_ms = 0
        self._travel_sequence = 0

    def request_job(self, job: Job, navigation_goal: dict, now_ms: int):
        if self.phase not in (Phase.IDLE, Phase.DONE):
            raise ValueError('当前区域任务尚未结束')
        if self.arm.ledger.held is not None:
            self.phase = Phase.WAIT_EVIDENCE;self.reason = '手中仍有物料，先放载台再跨区';return
        self.job = job;self._navigation_goal = deepcopy(navigation_goal);self.last_arrival = None;self._submitted = None
        self._travel_sequence += 1
        self._navigation_goal['command_sequence'] = self._travel_sequence
        try:accepted = self.chassis.travel_begin(job.context_id, job.zone, deepcopy(self._navigation_goal))
        except Exception as e:
            self.stop('行驶请求结果未知：'+str(e));return
        if accepted:
            self.phase = Phase.TRAVEL;self.reason = ''
        else:
            self.phase = Phase.WAIT_PARAMETER;self.reason = '底盘未接纳行驶请求，未启动机械臂'

    def begin_work_here(self, job: Job, arrival: Arrival, now_ms: int):
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.TRAVEL):
            raise ValueError('当前模块正在执行其他任务')
        if not (arrival.context_id == job.context_id and arrival.zone == job.zone and arrival.arrived and
                arrival.stopped and arrival.status == 'DONE'):
            return False
        if arrival.source != 'chassis' and not job.supervised:
            return False
        if self.phase == Phase.TRAVEL and (job != self.job or arrival.source != 'chassis' or arrival.sequence != self._travel_sequence):
            return False
        if not self.chassis.hold(job.context_id):
            self.reason = '等待底盘实际保持';return False
        self.job = job;self.last_arrival = arrival;self.phase = Phase.WORK;self._submitted = None
        self.arm.begin(job, now_ms)
        return True

    def tick(self, now_ms: int):
        self._now_ms = now_ms
        if self.phase == Phase.STOPPING:
            try:
                body_done = self.chassis.stop_poll(self.job.context_id)
                arm_done = self.actuator.stop_poll(self.job.context_id)
            except Exception:
                return  # Remain pending; never invent a stopped acknowledgement.
            if body_done and arm_done:
                if self.arm.pending and self.arm.pending.kind == 'stop':
                    c = self.arm.pending
                    self.arm.feedback(Feedback(c.context_id, c.sequence, 'DONE', {'stopped': True}), now_ms)
                self.phase = Phase.STOPPED
            return
        if self.phase == Phase.TRAVEL:
            try:arrival = self.chassis.travel_poll(self.job.context_id)
            except Exception as e:
                self.stop('底盘执行反馈接口失败：'+str(e));return
            if arrival is not None:
                if arrival.status == 'FAILED':
                    self.stop('底盘行驶失败');return
                self.begin_work_here(self.job, arrival, now_ms)
            return
        if self.phase not in (Phase.WORK, Phase.WAIT_PARAMETER, Phase.WAIT_VISION, Phase.WAIT_EVIDENCE):
            return
        # A rejected travel request cannot accidentally run the previous arm job.
        if self.last_arrival is None:return
        command = self.arm.tick(now_ms)
        self.phase = self.arm.phase;self.reason = self.arm.reason
        if self.phase == Phase.DONE:
            self._submitted = None;return
        if command is None:return
        key = command.context_id, command.sequence
        if command.kind == 'stop':
            if self._submitted != key:
                self._request_stop_ports(command.context_id)
                self._submitted=key
            self.phase=Phase.STOPPING;return
        port = self.chassis if command.kind == 'align' else self.actuator
        if self._submitted != key:
            try:
                accepted = port.align_begin(command) if command.kind == 'align' else port.begin(command)
            except Exception as e:
                self.stop('执行请求结果未知：'+str(e));return
            if not accepted:
                self.arm.feedback(Feedback(command.context_id, command.sequence, 'PARAMETER_REJECTED', reason='执行模块未接纳参数/当前状态'), now_ms)
                self.phase = self.arm.phase;self.reason = self.arm.reason;return
            self._submitted = key
        try:
            feedback = port.align_poll(command) if command.kind == 'align' else port.poll(command)
        except Exception as e:
            self.stop('执行反馈持续接口失败：'+str(e));return
        if feedback is not None:
            self.arm.feedback(feedback, now_ms)
            self.phase = self.arm.phase;self.reason = self.arm.reason
            if self.phase == Phase.STOPPING:
                # Dispatch both STOPs in this tick, rather than waiting on an
                # arm phase that the normal WORK branch no longer services.
                self._request_stop_ports(command.context_id)
                self._submitted=(self.arm.pending.context_id,self.arm.pending.sequence)
                self.phase = Phase.STOPPING

    def _request_stop_ports(self, context_id):
        errors=[]
        for port in (self.chassis,self.actuator):
            try:port.stop(context_id)
            except Exception as e:errors.append(str(e))
        if errors:self.reason += '；停止回执待确认：'+'；'.join(errors)

    def stop(self, reason: str):
        if self.phase in (Phase.STOPPING, Phase.STOPPED):return
        self.phase = Phase.STOPPING;self.reason = reason
        if self.job is not None:
            self._request_stop_ports(self.job.context_id)

    def status(self):
        return dict(phase=self.phase.value, zone=self.job.zone.value if self.job else None,
                    owner='chassis' if self.phase == Phase.TRAVEL else 'vision_arm' if self.last_arrival else 'none',
                    operation=self.job.operation.value if self.job else None,
                    item=self.arm.item, reason=self.reason, correct_picks=self.arm.ledger.correct_picks,
                    correct_places=self.arm.ledger.correct_places)
