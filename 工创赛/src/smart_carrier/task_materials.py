"""任务颜色、同批余项推断与车载盘选择的纯逻辑；不访问硬件。

颜色编码、任务顺序、槽位号、目的圆环号独立。调用方必须提供经过
真实观测/动作确认的快照，不能把本模块输出当作到位或抓放反馈。
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .task_code import parse_task_code


@dataclass(frozen=True)
class MaterialTarget:
    order: int
    color_code: int
    color_name: str
    rough_ring: int
    temporary_ring: int
    stack_on_first_batch: bool


@dataclass(frozen=True)
class MaterialTask:
    raw_code: str
    profile_id: str
    batch_1: tuple[MaterialTarget, ...]
    batch_2: tuple[MaterialTarget, ...]
    motion_allowed: bool = False


def resolve_preliminary_task(raw_code: str, start_id: int, profile_path: str | Path) -> MaterialTask:
    """显式使用已核初赛profile；第二批暂存按第一批同色位置匹配。"""
    profile = json.loads(Path(profile_path).read_text(encoding='utf8'))
    if (profile.get('schema_version') != 1
            or profile.get('profile_id') != '2027_preliminary_attachment2_1'
            or profile.get('scope') != 'preliminary_only'):
        raise ValueError('需要明确的2027初赛profile，决赛不能沿用')
    names = profile['color_names']
    if set(names) != set('123456') or len(set(names.values())) != 6:
        raise ValueError('颜色编码映射不完整或不唯一')
    task = parse_task_code(raw_code, start_id=start_id, require_unique_pickup_symbols=True)
    first, second = task.batch_1, task.batch_2
    if Counter(first.pickup_order) != Counter(second.pickup_order):
        raise ValueError('两批颜色与本次初赛三色库存不一致')
    first_ring = dict(zip(first.pickup_order, first.rough_positions))

    def targets(batch, stacking):
        return tuple(MaterialTarget(i, code, names[str(code)], rough,
                                    first_ring[code], stacking)
                     for i, (code, rough) in enumerate(zip(batch.pickup_order, batch.rough_positions), 1))
    return MaterialTask(task.raw_text, profile['profile_id'], targets(first, False), targets(second, True))


@dataclass(frozen=True)
class ObjectEvidence:
    context_id: str  # 本轮/本批/本场景的联合版本，由上层显式产生。
    object_id: str
    observed_at: float
    present: bool
    position_confirmed: bool
    color_code: int | None = None
    color_confirmed: bool = False


@dataclass(frozen=True)
class ResolvedIdentity:
    object_id: str
    color_code: int
    source: str  # VISION_CONFIRMED 或 TASK_REMAINDER_INFERRED。
    position_confirmed: bool


def infer_three_identities(expected: tuple[int, ...], objects: tuple[ObjectEvidence, ...],
                           *, context_id: str, closed_group_confirmed: bool,
                           now: float, maximum_age: float) -> tuple[ResolvedIdentity, ...]:
    """只在已确认的三物料封闭组里做唯一余项推断；不填造坐标。

初赛原料盘可能同时有每色两件：看到两种颜色并不足以把其余物体
全归成第三色。closed_group_confirmed要求上层先确定这三个实例的组。
"""
    if (len(expected) != 3 or len(set(expected)) != 3
            or any(type(c) is not int or c not in range(1, 7) for c in expected)):
        raise ValueError('需要初赛本批三个不同颜色编码')
    if (not context_id or not closed_group_confirmed or len(objects) != 3
            or len({o.object_id for o in objects}) != 3 or any(not o.object_id for o in objects)):
        raise ValueError('三物料组或实例对应尚未确认')
    if not math.isfinite(now) or not math.isfinite(maximum_age) or maximum_age <= 0:
        raise ValueError('时效参数无效')
    for o in objects:
        if (o.context_id != context_id or not o.present or not math.isfinite(o.observed_at)
                or not 0 <= now-o.observed_at <= maximum_age):
            raise ValueError('物料证据过期、缺失或跨批次')
        if o.color_confirmed and (type(o.color_code) is not int or o.color_code not in range(1, 7)):
            raise ValueError('已确认颜色编码无效')
    known = [o.color_code for o in objects if o.color_confirmed]
    remainder = Counter(expected)
    remainder.subtract(known)
    if any(count < 0 for count in remainder.values()):
        raise ValueError('已识别颜色与任务冲突或重复实例')
    unknown = [o for o in objects if not o.color_confirmed]
    remaining = list(remainder.elements())
    if len(unknown) > 1 or len(unknown) != len(remaining):
        raise ValueError('剩余颜色不能唯一绑定到一个物料')
    return tuple(ResolvedIdentity(o.object_id, o.color_code if o.color_confirmed else remaining[0],
                                  'VISION_CONFIRMED' if o.color_confirmed else 'TASK_REMAINDER_INFERRED',
                                  o.position_confirmed) for o in objects)


@dataclass(frozen=True)
class SlotInventory:
    context_id: str
    slot_id: int
    material_id: str
    color_code: int
    occupied: bool
    verified: bool  # 已确认放入，且此后没有掉料/复位/未受控移动等失效事件。


@dataclass(frozen=True)
class CarrierDecision:
    step: str
    reason: str
    target_slot: int | None = None
    material_id: str | None = None
    motion_allowed: bool = False  # 本模块只给调度建议，不授权或发送运动。


def select_fixed_pick(target_color: int, slots: tuple[SlotInventory, ...], *, context_id: str,
                      arm_clear: bool, base_stopped: bool, hand_empty: bool,
                      active_slot: int | None, position_confirmed: bool,
                      table_stopped: bool) -> CarrierDecision:
    """已验库存可替代重复识色；下一槽可在夹爪持料且离盘后预定位。"""
    if type(target_color) is not int or target_color not in range(1, 7):
        raise ValueError('目标颜色编码无效')
    if (not context_id or len(slots) != 3 or {s.slot_id for s in slots} != {1, 2, 3}
            or any(s.context_id != context_id or not s.verified for s in slots)):
        return CarrierDecision('RECONCILE_INVENTORY', '槽位/批次或库存未确认')
    occupied = [s for s in slots if s.occupied]
    if (any(not s.material_id or s.color_code not in range(1, 7) for s in occupied)
            or len({s.material_id for s in occupied}) != len(occupied)):
        return CarrierDecision('RECONCILE_INVENTORY', '实例或颜色库存冲突')
    matches = [s for s in occupied if s.color_code == target_color]
    if len(matches) != 1:
        return CarrierDecision('RECONCILE_INVENTORY', '目标物料不唯一或不在盘上')
    target = matches[0]

    def decision(step, reason):
        return CarrierDecision(step, reason, target.slot_id, target.material_id)
    if not base_stopped:
        return decision('WAIT_BASE_STOP', '底盘尚未确认停稳')
    if not arm_clear:
        return decision('WAIT_ARM_CLEAR', '机械臂或所持物料尚未离开盘扫掠区')
    if not table_stopped:
        return decision('WAIT_TABLE_STOP', '盘仍在运动或状态未知')
    if not position_confirmed or active_slot not in (1, 2, 3):
        return decision('RELOCALIZE_TABLE', '当前盘角度/槽位未知，先建立位置基准')
    if active_slot != target.slot_id:
        return decision('INDEX_TARGET_SLOT', '将目标槽预定位到固定取料位')
    if not hand_empty:
        return decision('WAIT_CURRENT_PLACEMENT', '下一槽已到位，先完成手中物料放置')
    return decision('PICK_FROM_FIXED_STATION', '已验槽位库存和到位条件满足，无需重复识色')


def commit_slot_pick(slot: SlotInventory, *, context_id: str, material_id: str,
                     held_confirmed: bool, slot_empty_confirmed: bool) -> SlotInventory:
    """抓起与原槽已空均确认后才扣库存；ACK、命令发出或等待结束不调用此提交。"""
    if (slot.context_id != context_id or slot.material_id != material_id
            or not slot.occupied or not slot.verified or not held_confirmed or not slot_empty_confirmed):
        raise ValueError('取料完成证据不匹配或不足')
    return SlotInventory(slot.context_id, slot.slot_id, '', 0, False, True)
