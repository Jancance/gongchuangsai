"""Read-only PC13-cycle monitor. Never writes a request, reset or fault field."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-mcu-cycle'
sys.path.insert(0,str(R/'src'))

def manifest():
    c=json.loads((E/'candidate.json').read_text())
    assert c['build_passed'] and json.loads((E/'checks.json').read_text())['passed']
    assert json.loads((E/'python-and-buzzer-checks.json').read_text())['passed']
    for name,digest in c['changes'].items():
        assert hashlib.sha256((R/c['project']/name).read_bytes()).hexdigest()==digest,name
    image=c['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
    return image

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--watch',action='store_true');p.add_argument('--duration',type=float,default=180)
    args=p.parse_args();image=manifest()
    if not args.watch:
        print(json.dumps(dict(local_check=True,hardware_access=False,image_sha256=image['sha256'],start='PC13 only')));return
    import gripper210_attach_read_20261105 as live
    from source_body_probe_20261111 import attach
    from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
    live.IMAGE=R/image['axf'];symbols,segments=live.load_image()
    out=E/('pc13-watch-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(complete=False,start_source='PC13',host_request_writes=0,host_motion_commands_sent=False,
        image_sha256=image['sha256'],trace=[],physical_placement_pending=True)
    def save():out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    save();state=None
    try:
        with attach() as session:
            board=CommittedPickBoard(session.target,symbols,segments);report['verified_flash_bytes']=board.verified_flash_bytes
            keys=[k for k in symbols if k.startswith('g_source_cycle_') and k not in (
                'g_source_cycle_yaw','g_source_cycle_extension','g_source_cycle_colors','g_source_cycle_piece_ms',
                'g_source_cycle_start_delay_ms','g_source_cycle_grip_command_ms')]
            deadline=time.monotonic()+args.duration;last=None
            while time.monotonic()<deadline:
                state=board.snapshot();state.update({k:session.target.read32(symbols[k]) for k in keys})
                state['cycle_piece_ms']=list(session.target.read_memory_block32(symbols['g_source_cycle_piece_ms'],3))
                state['cycle_colors']=list(session.target.read_memory_block32(symbols['g_source_cycle_colors'],3))
                for name in ('g_source_cycle_start_delay_ms','g_source_cycle_grip_command_ms'):
                    if name in symbols:state[name]=list(session.target.read_memory_block32(symbols[name],3))
                if 'g_low_grip_command_ms' in symbols:state['g_low_grip_command_ms']=session.target.read32(symbols['g_low_grip_command_ms'])
                report['trace'].append(state)
                current=(state['g_source_cycle_state'],state['g_source_cycle_prep'],state['g_source_cycle_item'],state['g_low_phase'])
                if current!=last:
                    print('MCU',current,'fault',state['g_source_cycle_fault'],'body_error_mm',state['g_source_cycle_error_milli_mm']/1000,flush=True)
                    save();last=current
                if state['g_source_cycle_state'] in (8,9):
                    report['complete']=state['g_source_cycle_state']==8
                    report['final']=state
                    if not report['complete']:report['first_fault']=state
                    break
                time.sleep(.08)
            else:report['monitor_budget_finished']=True;report['final']=state
    except BaseException as error:
        report['error']=repr(error);report['controller_may_continue']=bool(state and state.get('g_source_cycle_state') not in (0,8,9));raise
    finally:save();print(out,flush=True)

if __name__=='__main__':main()
