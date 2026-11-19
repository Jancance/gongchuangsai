"""Bind observed physical instances to a task, reusing the existing inference.

An observed object is different from a colour contour or a predicted slot.
The camera/geometry provider supplies the physical-instance association; this
module never fills in a missing third object or invents a base-plane position.
"""
from dataclasses import dataclass
from ..task_materials import ObjectEvidence, infer_three_identities
from .types import Target


@dataclass(frozen=True)
class PhysicalObject:
    instance_id: str
    region: str
    observed_ms: int
    present: bool
    pose: dict | None
    position_confirmed: bool
    stationary: bool
    candidate_colors: tuple[int, ...]


@dataclass(frozen=True)
class SourceResolution:
    targets: tuple[Target, ...]
    reason: str
    observation_usable: bool


def resolve_source_objects(expected_colors, objects, *, context_id, now_ms,
                           closed_group_confirmed, maximum_age_ms=1000):
    evidence = []
    for obj in objects:
        colors = set(obj.candidate_colors)
        known = next(iter(colors)) if len(colors) == 1 else None
        evidence.append(ObjectEvidence(context_id, obj.instance_id, obj.observed_ms/1000,
                                       obj.present, obj.position_confirmed, known, known is not None))
    resolved = infer_three_identities(tuple(expected_colors), tuple(evidence), context_id=context_id,
                                     closed_group_confirmed=closed_group_confirmed, now=now_ms/1000,
                                     maximum_age=maximum_age_ms/1000)
    result = []
    for obj, identity in zip(objects, resolved):
        if obj.candidate_colors and identity.color_code not in obj.candidate_colors:
            raise ValueError('任务余项与本实体的颜色证据冲突')
        if obj.pose is None or not obj.position_confirmed:
            continue  # An inferred colour still does not create a grasp point.
        result.append(Target(context_id, identity.object_id, identity.color_code, dict(obj.pose),
                             obj.observed_ms, obj.present, obj.position_confirmed, obj.stationary, identity.source))
    return tuple(result)


def observe_source_objects(expected_colors, objects, **context):
    """Normal ambiguity/stale observations are a wait, never a motor fault."""
    try:
        targets=resolve_source_objects(expected_colors,objects,**context)
        return SourceResolution(targets,'' if targets else '等待实际接触平面位置',bool(targets))
    except ValueError as e:
        return SourceResolution((),str(e),False)
