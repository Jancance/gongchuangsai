"""Read-only firmware match, snapshot and one PC13-cycle monitor."""
from pathlib import Path
from datetime import datetime
import hashlib
import json
import sys
import source_mcu_cycle_watch_20261115 as monitor
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/integration23-sync'

def manifest():
    c=json.loads((E/'candidate.json').read_text())
    assert c['build_passed'] and json.loads((E/'checks.json').read_text())['passed']
    for name,digest in c['changes'].items():
        assert hashlib.sha256((R/c['project']/name).read_bytes()).hexdigest()==digest
    image=c['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
    return image

def main():
    image=manifest()
    if '--watch' in sys.argv:
        sys.path.insert(0,str(R/'src'))
        from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
        original=CommittedPickBoard.snapshot
        def snapshot_with_sync(board):
            state=original(board)
            state['g_source_parallel_sync_tick']=board.target.read32(board.symbols['g_source_parallel_sync_tick'])
            for key in ('g_low_group_phase','g_low_group_axes','g_low_group_skew_ms'):
                state[key]=list(board.target.read_memory_block32(board.symbols[key],8))
            return state
        CommittedPickBoard.snapshot=snapshot_with_sync
        monitor.E=E;monitor.manifest=manifest;monitor.main();return
    sys.path.insert(0,str(R/'src'))
    from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
    live.IMAGE=R/image['axf'];symbols,segments=live.load_image()
    with attach() as session:
        board=CommittedPickBoard(session.target,symbols,segments)
        report=dict(when=datetime.now().isoformat(),image_sha256=image['sha256'],
            verified_flash_bytes=board.verified_flash_bytes,state=board.snapshot(),
            core_state=session.target.get_state().name,host_request_writes=0,reset=False)
        for name in ('g_source_cycle_state','g_source_cycle_fault','g_safety_fault','g_rtos_fault',
                     'g_source_parallel_sync_tick','g_startup_axes_vm_request',
                     'g_lift_ruler_state','g_lift_ruler_phase','g_lift_ruler_stop_pending',
                     'g_lift_ruler_stop_unconfirmed','g_lift_ruler_vm_request',
                     'g_lift_exchange_first_valid','g_lift_exchange_first_cmd',
                     'g_lift_exchange_first_result','g_lift_exchange_first_phase',
                     'g_lift_move_last_result','g_can_test_result','g_vm_test_request'):
            report['state'][name]=session.target.read32(symbols[name])
        report['state']['g_lift_exchange_first_raw']=list(session.target.read_memory_block32(symbols['g_lift_exchange_first_raw'],8))
        report['state']['g_lift_move_last_reply']=list(session.target.read_memory_block32(symbols['g_lift_move_last_reply'],8))
        report['state']['GPIOE_ODR']=session.target.read32(0x40021014)
        report['can1_registers']={name:session.target.read32(address) for name,address in {
            'MCR':0x40006400,'MSR':0x40006404,'TSR':0x40006408,'RF0R':0x4000640c,
            'ESR':0x40006418,'BTR':0x4000641c,'GPIOB_MODER':0x40020400,
            'GPIOB_IDR':0x40020410,'GPIOB_AFRH':0x40020424,'GPIOE_IDR':0x40021010,
            'RCC_APB1ENR':0x40023840}.items()}
    out=E/('read-only-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    print(out)

if __name__=='__main__':main()
