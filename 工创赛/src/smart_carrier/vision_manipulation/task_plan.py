"""Task-code semantics stay in the existing parser; no fixed colour order."""
from ..task_materials import MaterialTask
from .types import Job, Operation, Zone


def batch_jobs(task: MaterialTask, batch: int, context_id: str, *, supervised=False,
               manual_load_before_rough=False):
    if batch not in (1, 2):raise ValueError('批次需为1或2')
    if manual_load_before_rough and not supervised:
        raise ValueError('正式自主流程不能插入人工装盘')
    rows = task.batch_1 if batch == 1 else task.batch_2
    colors = tuple(x.color_code for x in rows);rough = tuple(x.rough_ring for x in rows)
    temporary = tuple(x.temporary_ring for x in rows)
    source = 'user_specified' if supervised else 'scanner'
    def job(zone, op, positions):
        return Job(context_id, zone, op, colors, positions, task_source=source, supervised=supervised)
    result = [job(Zone.SOURCE, Operation.LOAD_SOURCE, rough)]
    if manual_load_before_rough:
        result.append(job(Zone.ROUGH, Operation.MANUAL_LOAD, rough))
    result += [job(Zone.ROUGH, Operation.PLACE, rough), job(Zone.ROUGH, Operation.RECOVER, rough),
               job(Zone.STORAGE, Operation.PLACE if batch == 1 else Operation.STACK, temporary)]
    return tuple(result)


def recovery_from_placements(arm, context_id, zone, *, supervised=False):
    """Use actual placement order, including a centre-first commissioning run."""
    placed = [x for x in arm.ledger.placements if x.context_id == context_id and x.zone == zone and not x.recovered]
    if not placed:raise ValueError('没有本轮实际放置记录')
    return Job(context_id, zone, Operation.RECOVER, tuple(x.color_code for x in placed),
               tuple(x.position for x in placed), tuple(x.carrier_slot for x in placed), supervised=supervised)
