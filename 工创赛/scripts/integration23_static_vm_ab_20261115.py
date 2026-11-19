"""Authorized static power comparison: full image match, one VM request, no motion requests."""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,sys,time
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'src'))
from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=['879','sync'],required=True)
    parser.add_argument('--enable-vm',action='store_true');args=parser.parse_args()
    folder='source_extension_lead_pick_20261115' if args.variant=='879' else 'source_integration23_sync_20261115'
    digest=('879f54772af63033d3c11296d877ba6dc834a33de0a6f808ff76b3f40ac385a6' if args.variant=='879'
            else '0ee3d48544a02433a217e8434619e3f0b97e0d75bb914ad7199d8abf07c95676')
    live.IMAGE=R/'tmp'/folder/'project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()==digest
    symbols,segments=live.load_image();e=R/'evidence/integration/2026-11-15/integration23-sync'
    report=dict(variant=args.variant,image_sha256=digest,motion_requests_written=0,
                fault_or_health_writes=0,reset_requested=False,samples=[],vm_request_writes=0)
    out=e/('static-vm-'+args.variant+'-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    def save():out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    save()
    try:
        with attach() as session:
            board=CommittedPickBoard(session.target,symbols,segments)
            report['verified_flash_bytes']=board.verified_flash_bytes
            def sample():
                t=session.target;s=board.snapshot()
                for n in ('g_source_cycle_state','g_source_cycle_fault','g_vm_test_request',
                          'g_lift_ruler_vm_request','g_startup_axes_vm_request',
                          'g_chassis_auto_request','g_source_cycle_request','g_dm_jog_request'):
                    s[n]=t.read32(symbols[n])
                s['GPIOE_MODER']=t.read32(0x40021000);s['GPIOE_ODR']=t.read32(0x40021014)
                s['GPIOE_IDR']=t.read32(0x40021010);s['CAN1_ESR']=t.read32(0x40006418)
                report['samples'].append(s);save();return s
            before=sample()
            assert before['g_source_cycle_state']==0 and before['g_it_state']==0
            assert not any(before[n] for n in ('g_safety_fault','g_rtos_fault','g_chassis_auto_active',
                'g_chassis_auto_request','g_it_request','g_low_request','g_source_cycle_request','g_dm_jog_request'))
            if args.enable_vm:
                # The normal idle chassis owner retracts a naked g_vm_test_request.
                # Existing startup VM request is independent of its state machine:
                # state0 returns before any command; leave state/request untouched.
                assert before['g_startup_axes_state']==0 and before['g_startup_axes_vm_request']==0
                session.target.write32(symbols['g_startup_axes_vm_request'],1)
                report['vm_request_field']='g_startup_axes_vm_request'
                report['vm_request_writes']=1;save()
            for _ in range(3):
                time.sleep(.4);s=sample()
                print(json.dumps({n:s[n] for n in ('g_vm_test_request','g_startup_axes_vm_request','GPIOE_ODR','GPIOE_IDR',
                    'g_source_cycle_state','g_it_state','g_safety_fault','g_rtos_fault')}),flush=True)
    except BaseException as exc:
        report['error']=repr(exc);raise
    finally:save();print(out,flush=True)

if __name__=='__main__':main()
