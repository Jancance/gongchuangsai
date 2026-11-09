"""任务搬运账本：只消费显式证据，不发送硬件命令，不恢复旧会话运动。"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .task_materials import MaterialTask, SlotInventory


@dataclass(frozen=True)
class Transfer:
    session: str
    sequence: int
    batch: int
    order: int
    material_id: str
    source: str
    destination: str


@dataclass(frozen=True)
class TransferEvidence:
    session: str
    sequence: int
    material_id: str
    observed_at: float
    color_code: int | None = None
    source_clear: bool = False
    held: bool = False
    released: bool = False
    destination_present: bool = False
    stable: bool = False
    correct_target: bool = False

    def __post_init__(self):
        flags = ('source_clear', 'held', 'released', 'destination_present', 'stable', 'correct_target')
        if any(type(getattr(self, name)) is not bool for name in flags):
            raise ValueError('证据确认字段必须是布尔值，不能使用字符串或数字')


@dataclass(frozen=True)
class Transaction:
    request: Transfer
    started_at: float
    state: str = 'REQUESTED'
    picked_at: float | None = None
    committed_at: float | None = None


class TaskLedger:
    """单会话串行事务。重发只返回原事务；失败后须重新观测，不自动重试。

    时钟使用调用方同一单调时间域；maximum_age是观测时效，不是动作超时。
    手动调试的无限等待决定不在本模块更改。重启必须新建session并实测重建库存。
    """

    motion_allowed = False

    def __init__(self, task: MaterialTask, session: str, slots: tuple[SlotInventory, ...],
                 *, maximum_age: float, seed_batch: int = 1):
        if not session or not math.isfinite(maximum_age) or maximum_age <= 0:
            raise ValueError('会话或观测时效无效')
        if (len(slots) != 3 or {s.slot_id for s in slots} != {1, 2, 3}
                or any(s.context_id != session or not s.verified or type(s.slot_id) is not int
                       or type(s.occupied) is not bool or type(s.verified) is not bool for s in slots)):
            raise ValueError('需要本会话三个已实测槽位快照')
        occupied = [s for s in slots if s.occupied]
        if (any(not s.material_id or type(s.color_code) is not int
                or s.color_code not in range(1, 7) for s in occupied)
                or len({s.material_id for s in occupied}) != len(occupied)
                or any(s.material_id or s.color_code != 0 for s in slots if not s.occupied)):
            raise ValueError('槽位实例或颜色快照冲突')
        self.task, self.session, self.maximum_age = task, session, maximum_age
        self._slots = {s.slot_id: s for s in slots}
        self._transactions: dict[int, Transaction] = {}
        self._locations = {s.material_id: f'carrier:{s.slot_id}' for s in occupied}
        self._colors = {s.material_id: s.color_code for s in occupied}
        self._loaded: set[tuple[int, int]] = set()
        self._bindings: dict[str, tuple[int, int]] = {}
        self._stages: dict[str, str] = {}
        if type(seed_batch) is not int or seed_batch not in (1, 2):
            raise ValueError('库存批次无效')
        targets = task.batch_1 if seed_batch == 1 else task.batch_2
        if len({s.color_code for s in occupied}) != len(occupied):
            raise ValueError('同批车载库存颜色重复')
        for slot in occupied:
            matches = [t for t in targets if t.color_code == slot.color_code]
            if len(matches) != 1:
                raise ValueError('实测库存颜色不在本批任务中')
            binding = seed_batch, matches[0].order
            self._bindings[slot.material_id] = binding
            self._loaded.add(binding)
            self._stages[slot.material_id] = 'LOADED'
        self._temporary: dict[int, str] = {}
        self._active: int | None = None
        self._hand = 'EMPTY'
        self._healthy = True
        self._picks = self._places = 0

    @property
    def slots(self) -> tuple[SlotInventory, ...]:
        return tuple(self._slots[i] for i in (1, 2, 3))

    def snapshot(self) -> dict:
        return dict(session=self.session, task_code=self.task.raw_code,
                    correct_picks=self._picks, correct_places=self._places,
                    hand=self._hand, inventory_verified=self._healthy,
                    active_sequence=self._active, motion_allowed=False)

    @staticmethod
    def _location(value: str) -> tuple[str, int]:
        try:
            kind, number = value.split(':')
            index = int(number)
        except (ValueError, AttributeError):
            raise ValueError('位置需为source/carrier/rough/temporary:1..3') from None
        if kind not in ('source', 'carrier', 'rough', 'temporary') or index not in (1, 2, 3):
            raise ValueError('位置类型或编号无效')
        return kind, index

    def begin(self, request: Transfer, *, now: float) -> Transaction:
        if (request.session != self.session or type(request.sequence) is not int
                or not 0 <= request.sequence <= 65535 or not request.material_id
                or type(request.batch) is not int or request.batch not in (1, 2)
                or type(request.order) is not int or request.order not in (1, 2, 3)
                or not math.isfinite(now)):
            raise ValueError('事务身份或时钟无效')
        previous = self._transactions.get(request.sequence)
        if previous:
            if previous.request != request:
                raise ValueError('同一序号不能绑定不同请求')
            return previous
        if not self._healthy or self._active is not None or self._hand != 'EMPTY':
            raise ValueError('先处理在途事务或重新核对实物库存')
        source, source_index = self._location(request.source)
        destination, dest_index = self._location(request.destination)
        target = (self.task.batch_1 if request.batch == 1 else self.task.batch_2)[request.order-1]
        if (source, destination) not in {('source', 'carrier'), ('carrier', 'rough'),
                                         ('rough', 'carrier'), ('carrier', 'temporary')}:
            raise ValueError('搬运必须经过车载槽，场地接触后不推移校正')
        if source == 'source':
            if request.batch == 2 and len(self._temporary) != 3:
                raise ValueError('第一批暂存未闭环，不能进入第二批源抓取')
            if source_index != request.order or (request.batch, request.order) in self._loaded:
                raise ValueError('源抓取序号已完成或不匹配')
            if any((request.batch, i) not in self._loaded for i in range(1, request.order)):
                raise ValueError('不能跳过本批抓取顺序')
            if request.material_id in self._locations:
                raise ValueError('已知实例不能再次从原料区入账')
        elif (self._locations.get(request.material_id) != request.source
              or self._colors.get(request.material_id) != target.color_code
              or self._bindings.get(request.material_id) != (request.batch, request.order)):
            raise ValueError('物料实例、颜色与源位置不匹配')
        required_stage = {('carrier', 'rough'): 'LOADED', ('rough', 'carrier'): 'ROUGH',
                          ('carrier', 'temporary'): 'RETURNED'}.get((source, destination))
        if required_stage and self._stages.get(request.material_id) != required_stage:
            raise ValueError('不能跳过粗加工或重复已完成阶段')
        if source == 'rough' and source_index != target.rough_ring:
            raise ValueError('粗加工源位置不匹配')
        if destination == 'carrier' and self._slots[dest_index].occupied:
            raise ValueError('目标槽已占用')
        if destination == 'rough' and dest_index != target.rough_ring:
            raise ValueError('粗加工目标与任务不匹配')
        if destination == 'temporary':
            if dest_index != target.temporary_ring:
                raise ValueError('暂存目标必须按第一批同色位置匹配')
            support = self._temporary.get(target.color_code)
            if (request.batch == 1 and support is not None) or (request.batch == 2 and support is None):
                raise ValueError('第一批已放置或第二批缺少已确认同色底层')
        if destination != 'carrier' and any(
                location == request.destination for mid, location in self._locations.items()
                if mid != request.material_id and not (destination == 'temporary' and request.batch == 2
                                                      and mid == self._temporary.get(target.color_code))):
            raise ValueError('目标已有其他实例')
        transaction = Transaction(request, now)
        self._transactions[request.sequence] = transaction
        self._active = request.sequence
        return transaction

    def accepted(self, sequence: int) -> Transaction:
        """通信ACK只更新状态，不改库存或成功次数。"""
        transaction = self._transactions[sequence]
        if transaction.state == 'REQUESTED':
            transaction = replace(transaction, state='ACCEPTED')
            self._transactions[sequence] = transaction
        return transaction

    def observe(self, evidence: TransferEvidence, *, now: float, phase: str,
                actuator_done: bool) -> Transaction:
        transaction = self._transactions.get(evidence.sequence)
        if transaction is None:
            raise ValueError('未知序号')
        request = transaction.request
        if (evidence.session != self.session or evidence.material_id != request.material_id
                or phase not in ('PICK', 'PLACE')):
            raise ValueError('证据事务身份不匹配')
        if transaction.state == 'COMMITTED':
            return transaction
        if transaction.state == 'FAILED' or not self._healthy:
            raise ValueError('事务已失败，迟到结果不能恢复库存')
        if (not math.isfinite(now) or not math.isfinite(evidence.observed_at)
                or not 0 <= now-evidence.observed_at <= self.maximum_age
                or evidence.observed_at < transaction.started_at):
            raise ValueError('证据过期、来自未来或早于请求')
        if actuator_done is not True:
            raise ValueError('ACK或命令发出不能提交物理结果')
        if phase == 'PICK':
            if transaction.state == 'HELD':
                return transaction
            target = (self.task.batch_1 if request.batch == 1 else self.task.batch_2)[request.order-1]
            if type(evidence.color_code) is not int or evidence.color_code != target.color_code:
                raise ValueError('真实实例颜色或有效库存颜色必须与任务一致')
            if not evidence.source_clear or not evidence.held:
                raise ValueError('需源位已空和真实持料双确认')
            source, source_index = self._location(request.source)
            if source == 'carrier':
                self._slots[source_index] = SlotInventory(self.session, source_index, '', 0, False, True)
            self._locations[request.material_id] = 'HAND'
            self._colors[request.material_id] = target.color_code
            self._bindings[request.material_id] = request.batch, request.order
            self._hand = request.material_id
            self._picks += 1
            transaction = replace(transaction, state='HELD', picked_at=evidence.observed_at)
        else:
            if (transaction.state != 'HELD' or evidence.observed_at <= transaction.picked_at
                    or not evidence.released or not evidence.destination_present
                    or not evidence.stable or not evidence.correct_target):
                raise ValueError('需抓取后独立的新鲜释放、目标在位、落稳和目标正确证据')
            destination, dest_index = self._location(request.destination)
            color = self._colors[request.material_id]
            if destination == 'carrier':
                self._slots[dest_index] = SlotInventory(self.session, dest_index, request.material_id,
                                                       color, True, True)
            if destination == 'temporary' and request.batch == 1:
                self._temporary[color] = request.material_id
            if request.source.startswith('source:'):
                self._loaded.add((request.batch, request.order))
            self._locations[request.material_id] = request.destination
            self._stages[request.material_id] = {'carrier': 'LOADED' if request.source.startswith('source:')
                                               else 'RETURNED', 'rough': 'ROUGH',
                                               'temporary': 'TEMPORARY'}[destination]
            self._hand = 'EMPTY'
            self._places += 1
            self._active = None
            transaction = replace(transaction, state='COMMITTED', committed_at=evidence.observed_at)
        self._transactions[evidence.sequence] = transaction
        return transaction

    def invalidate(self, reason: str) -> None:
        """掉料、失败、复位或未受控移动：禁止沿用旧库存，保留已确认历史计数。"""
        if not reason:
            raise ValueError('需要记录失效原因')
        if self._active is not None:
            transaction = self._transactions[self._active]
            self._transactions[self._active] = replace(transaction, state='FAILED')
        self._slots = {i: replace(slot, verified=False) for i, slot in self._slots.items()}
        self._healthy = False
        self._hand = 'UNKNOWN'
        self._active = None
        self.failure_reason = reason

    def can_transport(self) -> bool:
        """仅为任务层必要条件；底盘仍须独立核对停靠、包络及全部运动准入。"""
        return (self._healthy and self._active is None and self._hand == 'EMPTY'
                and any(s.occupied for s in self._slots.values()))
