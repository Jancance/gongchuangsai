"""离线任务事务回放入口；不导入串口、相机或运动控制器。"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .task_materials import SlotInventory, resolve_preliminary_task
from .task_transactions import TaskLedger, Transfer, TransferEvidence


def replay(intake: dict, profile: Path, start_id: int, events: list[dict]) -> dict:
    if intake.get('schema_version') != 1 or intake.get('motion_allowed') is not False:
        raise ValueError('仅接受显式禁止运动的离线快照')
    if intake.get('start_id') not in (None, start_id):
        raise ValueError('回放起点与已登记起点冲突')
    task = resolve_preliminary_task(intake['task_code'], start_id, profile)
    slots = tuple(SlotInventory(context_id=intake['session'], **s) for s in intake['slots'])
    # .5秒只用于合成离线事件的时效门禁，不作为实机时效验收值。
    ledger = TaskLedger(task, intake['session'], slots, maximum_age=.5, seed_batch=intake['batch'])
    journal = []
    for event in events:
        kind = event['kind']
        if kind == 'BEGIN':
            transaction = ledger.begin(Transfer(**event['request']), now=event['now'])
        elif kind == 'ACCEPTED':
            transaction = ledger.accepted(event['sequence'])
        elif kind in ('PICK', 'PLACE'):
            transaction = ledger.observe(TransferEvidence(**event['evidence']), now=event['now'],
                                         phase=kind, actuator_done=event['actuator_done'])
        elif kind == 'INVALIDATE':
            ledger.invalidate(event['reason'])
            transaction = None
        else:
            raise ValueError(f'未知事件: {kind}')
        journal.append(dict(event=event, transaction=asdict(transaction) if transaction else None,
                            snapshot=ledger.snapshot()))
    return dict(scope='offline_only', start_id=start_id,
                start_id_observed=intake.get('start_id') is not None,
                evidence_source=intake['evidence_source'],
                task=asdict(task), snapshot=ledger.snapshot(),
                slots=[asdict(s) for s in ledger.slots], journal=journal,
                transport_task_prerequisites_met=ledger.can_transport(),
                motion_allowed=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--intake', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--start-id', type=int, choices=(1, 2), required=True,
                        help='离线回放选择；不替代现场抽签确认')
    parser.add_argument('--events', type=Path, help='JSON事件数组；省略只核对任务和库存')
    args = parser.parse_args()
    intake = json.loads(args.intake.read_text(encoding='utf-8'))
    events = json.loads(args.events.read_text(encoding='utf-8')) if args.events else []
    print(json.dumps(replay(intake, args.profile, args.start_id, events), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
