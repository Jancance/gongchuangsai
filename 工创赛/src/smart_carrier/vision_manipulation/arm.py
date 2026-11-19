"""Reusable source pickup / ring placement / recovery / stacking module.

No UART/CAN/PWM access. A registered actuator port executes actions and returns
actual sequence-matched feedback. Ordinary missing data waits, never locks a
robot fault. An actual actuator failure/timeout still requests STOP.
"""
from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass, field

from ..task_materials import SlotInventory, commit_slot_pick
from .parameters import Parameters, TeachingRequired
from .types import Action, Feedback, Job, Operation, Phase, Target, Zone


@dataclass
class Placement:
    context_id: str
    color_code: int
    position: int
    instance_id: str
    carrier_slot: int
    zone: Zone
    recovered: bool = False


@dataclass
class Ledger:
    slots: dict[int, SlotInventory] = field(default_factory=dict)
    placements: list[Placement] = field(default_factory=list)
    held: str | None = None
    correct_picks: int = 0
    correct_places: int = 0


class ArmVisionModule:
    def __init__(self, parameters: Parameters):
        self.parameters = parameters
        self.ledger = Ledger()
        self.phase = Phase.IDLE
        self.reason = ''
        self.job: Job | None = None
        self.pending: Action | None = None
        self.targets: tuple[Target, ...] = ()
        self.sequence = 0
        self.item = 0
        self._queue: list[tuple] = []
        self._stage = ''
        self._issued_ms = 0
        self._stop_pending = False
        self._target: Target | None = None

    def begin(self, job: Job, now_ms: int):
        if self.phase not in (Phase.IDLE, Phase.DONE):
            raise ValueError('当前作业尚未结束；不得覆盖或自动清故障')
        if (not job.context_id or not 1 <= len(job.colors) <= 3 or
            len(job.colors) != len(job.positions) or len(job.carrier_slots) < len(job.colors) or
            len(set(job.carrier_slots[:len(job.colors)])) != len(job.colors) or
            len(set(job.colors)) != len(job.colors) or any(c not in range(1, 7) for c in job.colors) or
            any(s not in (1, 2, 3) for s in job.carrier_slots[:len(job.colors)])):
            raise ValueError('作业颜色/位置/槽位不完整')
        if job.operation == Operation.LOAD_SOURCE and job.zone != Zone.SOURCE:
            raise ValueError('原料抓取只用于原料区')
        if job.operation == Operation.MANUAL_LOAD and not job.supervised:
            raise ValueError('人工装盘只用于明确看护调试')
        self.job = job;self.item = 0;self.reason = '';self.pending = None;self.targets = ()
        self._target = None;self._stage = 'initial';self._queue = []
        self._stop_pending = False;self.phase = Phase.WORK
        if job.operation == Operation.MANUAL_LOAD:
            self._queue = [('manual_load_cycle', {
                'colors': job.colors, 'slots': job.carrier_slots[:len(job.colors)],
                'angles_deg': [self.parameters.tuning['carrier_angles_deg'][str(s)] for s in job.carrier_slots[:len(job.colors)]],
                'interval_ms': self.parameters.tuning['manual_index_interval_ms'],
                'reconcile_previous_load': True,
            }, ('manual_loading_complete', 'previous_load_reconciled', 'operator_hands_clear'))]
            self._stage = 'manual'
        else:
            try:
                pose = self.parameters.pose(self.parameters.zone(job.zone)['observe_pose'], '观察姿态')
                self._queue = [self._spec('observe', pose=pose, gripper_deg=self.parameters.tuning['gripper_open_deg']),
                               self._spec('align', required=('aligned', 'base_stopped'), zone=job.zone.value,
                                          reference='source_triangle' if job.zone == Zone.SOURCE else 'middle_ring_to_image_center')]
            except TeachingRequired as e:
                self.phase = Phase.WAIT_PARAMETER;self.reason = str(e)
        return self.tick(now_ms)

    @staticmethod
    def _spec(kind, required=(), **goal):
        return kind, goal, tuple(required)

    def update_parameters(self, parameters: Parameters, now_ms: int):
        """Reload before generating an item, never hot-edit cached motion."""
        if self.pending is not None or self.phase in (Phase.STOPPING, Phase.STOPPED):
            raise ValueError('只在无执行中动作时更新参数，不用调参清故障')
        if self.phase == Phase.WAIT_PARAMETER and self._queue:
            if self.job.operation == Operation.LOAD_SOURCE and self._queue[0][0] == 'source_pick_load':
                self._queue=[];self.targets=();self._target=None;self._stage='items'
            else:
                raise ValueError('本件已有动作序列，不热改位姿；先结束/停止本段，再按实际库存重建')
        self.parameters = parameters
        if self.phase == Phase.WAIT_PARAMETER:
            self.phase = Phase.WORK;self.reason = ''
            # Initial observation pose may itself have been missing.
            if self._stage == 'initial' and not self._queue:
                job = self.job;self.phase = Phase.DONE
                return self.begin(job, now_ms)
        return self.tick(now_ms)

    def offer_targets(self, targets: tuple[Target, ...], now_ms: int):
        self.targets = tuple(targets)
        if self.phase == Phase.WAIT_VISION and self.pending is None:
            self.phase = Phase.WORK
        return self.tick(now_ms)

    def _source_item(self, now_ms):
        job = self.job;color = job.colors[self.item]
        possible = [t for t in self.targets if t.context_id == job.context_id and t.color_code == color and
                    t.present and t.position_confirmed and t.stationary and t.instance_id and
                    0 <= now_ms-t.observed_ms <= self.parameters.tuning['observation_max_age_ms'] and
                    (t.usable_until_ms is None or now_ms < t.usable_until_ms)]
        if len(possible) != 1:
            self.phase = Phase.WAIT_VISION;self.reason = '等待本件唯一、当前、静止的真实位置';return []
        target = possible[0];self._target = target
        if target.usable_until_ms is not None:
            budget = self.parameters.tuning.get('source_pick_exit_budget_ms')
            if budget is None:
                raise TeachingRequired('动态停盘抓起/退出预算待实测；静止调试不冒充动态通过')
            if target.usable_until_ms-now_ms < budget:
                self.phase=Phase.WAIT_VISION;self.reason='本次停盘余量不足，等下一窗口，不清故障';return []
        pose = self.parameters.pose(target.pose, '原料目标')
        if any(s.occupied and s.material_id == target.instance_id for s in self.ledger.slots.values()):
            self.phase = Phase.WAIT_VISION;self.reason = '该实例已装车，等待新目标';return []
        slot = job.carrier_slots[self.item]
        old = self.ledger.slots.get(slot)
        if old and old.occupied:
            raise TeachingRequired('目标车载槽已有物料，先核对库存')
        zone = self.parameters.zone(job.zone)
        # This adapter action preserves the validated pzq single-item mechanics;
        # no attempt to reimplement its parallel timing inside a new main loop.
        return [self._spec('source_pick_load', required=('held', 'source_clear', 'released', 'carrier_occupied'),
                           target=pose, instance_id=target.instance_id, color=color, slot=slot,
                           observed_ms=target.observed_ms, usable_until_ms=target.usable_until_ms,
                           source_pick_native_pulses=zone['native_source_pick_pulses'],
                           carrier_place_native_pulses=zone['native_carrier_place_pulses'],
                           speeds=self.parameters.speeds(), gripper_open_deg=self.parameters.tuning['gripper_open_deg'],
                           gripper_close_deg=self.parameters.tuning['gripper_close_deg'],
                           release_wait_ms=self.parameters.tuning['release_wait_ms']),
                self._spec('observe', pose=self.parameters.zone(job.zone)['observe_pose'],
                           gripper_deg=self.parameters.tuning['gripper_open_deg'])]

    def _ring_item(self):
        job = self.job;i = self.item;color = job.colors[i];position = job.positions[i];slot = job.carrier_slots[i]
        if position not in (1, 2, 3):
            raise TeachingRequired('目标位置号需为1/2/3')
        zone = self.parameters.zone(job.zone);raw = zone['positions'][str(position)]
        carrier = self.parameters.values['carrier'];recover = job.operation == Operation.RECOVER
        if recover:
            source = self.parameters.pose(raw, '圆环回收位', depth_key='recover_depth_mm')
            dest = self.parameters.pose(carrier['place_pose'], '车载槽放回位', depth_key='place_depth_mm')
            placed = [p for p in self.ledger.placements if p.context_id == job.context_id and p.zone == job.zone and
                      p.color_code == color and p.position == position and not p.recovered]
            if len(placed) != 1:
                raise TeachingRequired('无本轮已放稳的对应物料记录，不能按预测回收')
            old = self.ledger.slots.get(slot)
            if old and old.occupied:
                raise TeachingRequired('回收目标槽未空')
        else:
            source = self.parameters.pose(carrier['pick_pose'], '车载槽取料位', depth_key='pick_depth_mm')
            dest = self.parameters.pose(raw, '圆环放料位', depth_key='place_depth_mm', stack=job.operation == Operation.STACK)
            old = self.ledger.slots.get(slot)
            if not old or not old.verified or not old.occupied or old.color_code != color or old.context_id != job.context_id:
                raise TeachingRequired('车载槽颜色/占用与本轮不对应，先核对库存')
        t = self.parameters.tuning;speeds = self.parameters.speeds()
        above_source = dict(source, descent_mm=0.0);above_dest = dict(dest, descent_mm=0.0)
        result = []
        if job.operation == Operation.STACK:
            result.append(self._spec('verify_base', required=('base_present', 'base_upright', 'base_same_color', 'base_correct_position'), color=color, position=position))
        result += [self._spec('index_carrier', slot=slot, angle_deg=t['carrier_angles_deg'][str(slot)]),
                   self._spec('move_pose', pose=above_source, speeds=speeds),
                   self._spec('move_pose', pose=source, speeds=speeds),
                   self._spec('grip', angle_deg=t['gripper_close_deg']),
                   self._spec('verify_held', required=('held', 'source_clear'), color=color, position=position, slot=slot),
                   self._spec('move_pose', pose=above_source, speeds=speeds),
                   self._spec('move_pose', pose=above_dest, speeds=speeds),
                   self._spec('move_pose', pose=dest, speeds=speeds),
                   self._spec('grip', angle_deg=t['gripper_open_deg'], settle_ms=t['release_wait_ms']),
                   self._spec('move_pose', pose=above_dest, speeds=speeds,
                              preclose_at_ascent_fraction=t['preclose_ascent_fraction'],
                              preclose_angle_deg=t['gripper_preclose_deg'], progress_source='actual_lift_feedback'),
                   self._spec('verify_placed', required=('released', 'target_present', 'upright', 'correct_color', 'correct_position'),
                              color=color, position=position, slot=slot, destination='carrier' if recover else job.zone.value)]
        return result

    def tick(self, now_ms: int):
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.STOPPING, Phase.STOPPED):
            return deepcopy(self.pending)
        if self.pending is not None:
            if now_ms-self._issued_ms > self.parameters.tuning['action_timeout_ms']:
                if self.pending.kind.startswith(('observe', 'verify')):
                    self.phase = Phase.WAIT_VISION;self.reason = '观测/确认暂缺，保持等待，不重复机械动作'
                else:
                    self.stop('执行器动作超时，状态未知', now_ms)
            return deepcopy(self.pending)
        if self.phase == Phase.WAIT_PARAMETER:
            return None
        if not self._queue:
            try:
                if self._stage == 'manual':
                    self.phase = Phase.DONE;return None
                if self._stage == 'initial':
                    self._stage = 'items'
                if self._stage == 'items':
                    if self.item < len(self.job.colors):
                        self._queue = self._source_item(now_ms) if self.job.operation == Operation.LOAD_SOURCE else self._ring_item()
                        if not self._queue:return None
                        self._stage = 'item_running'
                    else:
                        self._queue = [self._spec('stow', required=('arm_stowed', 'hand_empty'))];self._stage = 'park'
                elif self._stage == 'item_running':
                    self.item += 1;self._stage = 'items';self.targets = ()
                    return self.tick(now_ms)
                elif self._stage == 'park':
                    self.phase = Phase.DONE;return None
            except TeachingRequired as e:
                self.phase = Phase.WAIT_PARAMETER;self.reason = str(e);return None
        kind, goal, required = self._queue.pop(0)
        self.sequence += 1;self.pending = Action(self.job.context_id, self.sequence, kind, deepcopy(goal), required)
        self._issued_ms = now_ms;self.phase = Phase.WORK;self.reason = ''
        return deepcopy(self.pending)

    def feedback(self, feedback: Feedback, now_ms: int):
        command = self.pending
        if command is None or (feedback.context_id, feedback.sequence) != (command.context_id, command.sequence):
            return False
        if self._stop_pending:
            if feedback.status == 'DONE' and feedback.evidence.get('stopped'):
                self.pending = None;self._stop_pending = False;self.phase = Phase.STOPPED
            return True
        if feedback.status in ('ACCEPTED', 'TRANSIENT'):
            # A late sample never repeats a motor command or clears a fault.
            return True
        if feedback.status == 'PARAMETER_REJECTED':
            self._queue.insert(0, (command.kind, command.goal, command.evidence_required))
            self.pending = None;self.phase = Phase.WAIT_PARAMETER;self.reason = feedback.reason
            return True
        if feedback.status == 'FAILED':
            self.stop(feedback.reason or '执行器报告真实失败', now_ms);return True
        if feedback.status != 'DONE':
            return False
        if not all(feedback.evidence.get(k) is True for k in command.evidence_required):
            self.phase = Phase.WAIT_EVIDENCE;self.reason = '命令结束，等待真实抓牢/放稳证据';return True
        self._commit(command, feedback)
        self.pending = None;self.phase = Phase.WORK;self.reason = ''
        self.tick(now_ms)
        return True

    def _commit(self, command: Action, feedback: Feedback):
        job = self.job
        if command.kind == 'manual_load_cycle':
            for color, slot in zip(job.colors, job.carrier_slots):
                self.ledger.slots[slot] = SlotInventory(job.context_id, slot, f'{job.context_id}:manual-slot-{slot}', color, True, True)
            return
        if self.item >= len(job.colors):return
        color = job.colors[self.item];slot = job.carrier_slots[self.item];position = job.positions[self.item]
        if command.kind == 'source_pick_load':
            self.ledger.slots[slot] = SlotInventory(job.context_id, slot, self._target.instance_id, color, True, True)
            self.ledger.correct_picks += 1;self.ledger.correct_places += 1
        elif command.kind == 'verify_held':
            if job.operation == Operation.RECOVER:
                p = next(p for p in self.ledger.placements if p.context_id == job.context_id and p.zone == job.zone and
                         p.color_code == color and p.position == position and not p.recovered)
                p.recovered = True;self.ledger.held = p.instance_id
            else:
                old = self.ledger.slots[slot];self.ledger.held = old.material_id
                self.ledger.slots[slot] = commit_slot_pick(old, context_id=job.context_id, material_id=old.material_id,
                                                        held_confirmed=True, slot_empty_confirmed=True)
            self.ledger.correct_picks += 1
        elif command.kind == 'verify_placed':
            if job.operation == Operation.RECOVER:
                self.ledger.slots[slot] = SlotInventory(job.context_id, slot, self.ledger.held, color, True, True)
            else:
                self.ledger.placements.append(Placement(job.context_id, color, position, self.ledger.held, slot, job.zone))
            self.ledger.held = None;self.ledger.correct_places += 1

    def stop(self, reason: str, now_ms: int):
        if self.phase in (Phase.STOPPING, Phase.STOPPED):return
        self.reason = reason;self.phase = Phase.STOPPING;self._queue = []
        self.sequence += 1;self.pending = Action(self.job.context_id, self.sequence, 'stop', {'reason': reason}, ('stopped',))
        self._issued_ms = now_ms;self._stop_pending = True
