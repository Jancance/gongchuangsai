"""Frozen pzq with only release50ms: readonly or one complete three-item cycle."""
from datetime import datetime
from pathlib import Path
import argparse
import hashlib
import json
import struct
import time
import gripper210_attach_read_20261105 as live

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-11/pzq-release50'
P=R/'tmp/pzq_release50_20261111/project'


def run(readonly=False):
    m=json.loads((E/'candidate.json').read_text(encoding='utf-8'))
    assert json.loads((E/'checks.json').read_text(encoding='utf-8'))['passed']
    image=m['images']['Vehicle_LiftRuler'];live.IMAGE=R/image['axf']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()==image['sha256']
    for rel,h in m['changes'].items():assert hashlib.sha256((P/rel).read_bytes()).hexdigest()==h,rel
    symbols,segments=live.load_image()
    from elftools.elf.elffile import ELFFile
    with live.IMAGE.open('rb') as f:
        elf=ELFFile(f);axes=[(int(v['st_value']),int(v['st_size'])) for v in elf.get_section_by_name('.symtab').iter_symbols() if v.name=='vsp_axis']
    assert len(axes)==1 and axes[0][1]==144
    dest=E/(('pzq-readonly-' if readonly else 'pzq-three-once-')+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    assert not dest.exists()
    report=dict(sha256=image['sha256'],readonly=readonly,complete=False,reset=False,flashed=False,steps=[],mcu_trace=[],route3_enabled=False)
    def save():dest.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with live.attach() as session:
        t=session.target;assert t.get_state().name=='RUNNING'
        report['flash_bytes']=0
        for address,data in segments:
            for i in range(0,len(data),4096):
                chunk=data[i:i+4096];assert bytes(t.read_memory_block8(address+i,len(chunk)))==chunk,f'Flash mismatch {address+i:#x}'
                report['flash_bytes']+=len(chunk)
        def rd(k):return t.read32(symbols[k])
        def rf(k):return struct.unpack('<f',struct.pack('<I',rd(k)))[0]
        def wr(k,value):
            if isinstance(value,float):value=struct.unpack('<I',struct.pack('<f',value))[0]
            t.write32(symbols[k],value)
        def snap():
            keys='''g_src_now_ms g_it_request g_it_state g_it_action g_it_cause g_it_rejected g_it_sequence
g_it_auto_state g_it_auto_phase g_it_auto_item g_it_auto_fault g_it_auto_elapsed_ms g_it_auto_phase_ms
g_safety_ready g_safety_fault g_rtos_fault g_can_test_result g_can_ctrl_fault_latched
g_startup_axes_state g_startup_axes_cause g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_ruler_state g_lift_ruler_cause g_lift_ruler_samples g_lift_ruler_stop_pending
g_lift_compare_top_valid g_lift_compare_backoff_valid g_lift_compare_operation g_lift_compare_safe_raw
g_source_parallel_state g_source_parallel_cause g_source_parallel_stop_mask
g_source_parallel_started_mask g_source_parallel_done_mask g_source_parallel_start_skew_ms
g_vehicle_arm_state g_vehicle_arm_cause g_vehicle_arm_stop_pending g_dm_jog_request g_dm_jog_first_fault
g_chassis_auto_active g_it_lead5_grip_done g_extension_home_state g_extension_home_result'''.split()
            state={k:rd(k) for k in keys if k in symbols}
            for k in ['g_eh_first_reason','g_eh_first_command','g_eh_first_flags','g_eh_first_step']:
                if k in symbols:state[k]=rd(k)
            for k in ['g_dm_can2_logical_position_deg','g_startup_extension_current_deg','g_extension_home_reference_deg','g_vehicle_lift_current_deg']:
                state[k]=rf(k)
            state['release_command_times_ms']=list(t.read_memory_block32(symbols['g_it_auto_release_ms'],3))
            state['pe2']=(t.read32(0x40021010)>>2)&1
            state['gripper_pwm_us']=t.read32(0x40000034)
            state['carrier_pwm_us']=t.read32(0x40000440)
            block=bytes(t.read_memory_block8(axes[0][0],144));state['vsp_axes']=[]
            names=['phase','options','samples','flags','cycle','stall_active','stall_ms','position_tick','position_cycle','overshoot_active','overshoot_ms','before','goal','current']
            for i in range(2):
                a=dict(zip(names,struct.unpack_from('<11I3f',block,i*72)))
                a.update(node=6 if i==0 else 5,move_hex=block[i*72+56:i*72+71].hex(),position_minus_goal_deg=a['current']-a['goal'])
                state['vsp_axes'].append(a)
            return state
        def health(s):
            return s['g_safety_ready']==1 and s['g_can_test_result']==1 and not any(s[k] for k in ['g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_it_cause','g_startup_axes_cause','g_lift_ruler_cause','g_source_parallel_cause','g_vehicle_arm_cause','g_dm_jog_first_fault','g_chassis_auto_active'])
        def cmd(label,request,values=(),timeout=16,fresh_lift=False):
            before=snap();assert health(before) and before['g_it_state'] in (0,2) and not before['g_it_request'],before
            step=dict(label=label,request=request,before=before,samples=[]);report['steps'].append(step)
            for name,value in values:wr(name,value)
            begin=time.monotonic();wr('g_it_request',request);consistent=0
            while time.monotonic()-begin<timeout:
                s=snap();step['samples'].append(s)
                if not health(s) or s['g_it_rejected'] or s['g_it_state']==9:
                    report['first_fault']=s;raise RuntimeError(label+' rejected or faulted')
                done=s['g_it_state']==2 and s['g_it_action']==request and not s['g_it_request']
                if fresh_lift:done=done and s['g_lift_ruler_samples']!=before['g_lift_ruler_samples']
                consistent=consistent+1 if done else 0
                if consistent>=2:
                    step.update(after=s,elapsed_s=round(time.monotonic()-begin,3));save();print(label,step['elapsed_s'],'s',flush=True);return s
                time.sleep(.03)
            raise TimeoutError(label)
        writes_started=False
        try:
            s=snap();report['start']=s;save()
            if readonly:report['complete']=True;return
            assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_it_auto_state']==0 and s['g_startup_axes_state']==0 and s['g_lift_ruler_state']==0,s
            writes_started=True
            cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
            cmd('extension_home',2)
            cmd('read_arm',7,timeout=8)
            cmd('open',6)
            assert not rd('g_it_request') and rd('g_it_auto_state')==0
            start_sequence=rd('g_it_sequence');seen_running=False;before_phase=None
            begin=time.monotonic();wr('g_it_request',16)
            while time.monotonic()-begin<70:
                s=snap();report['mcu_trace'].append(s)
                if not health(s) or s['g_it_rejected'] or s['g_it_state']==9 or s['g_it_auto_state']==9:
                    report['first_fault']=s;raise RuntimeError('pzq continuous three-item cycle stopped')
                phase=(s['g_it_auto_item'],s['g_it_auto_phase'])
                if phase!=before_phase:print('MCU item/phase',phase,'elapsed_ms',s['g_it_auto_elapsed_ms'],flush=True);before_phase=phase;save()
                if s['g_it_auto_state']==1:seen_running=True
                if seen_running and s['g_it_auto_state']==2 and s['g_it_sequence']!=start_sequence:
                    times=s['release_command_times_ms'];assert 0<times[0]<times[1]<times[2]
                    report['three_to_release_command_s']=times[2]/1000
                    report['three_to_zero_s']=s['g_it_auto_elapsed_ms']/1000
                    report['physical_placement_confirmation_pending']=True;report['complete']=True;return
                time.sleep(.025)
            raise TimeoutError('pzq three-item cycle')
        except BaseException as exc:
            report['error']=repr(exc)
            if writes_started:
                try:wr('g_it_request',9);report['stop_request_sent']=True
                except BaseException as error:report['stop_request_error']=repr(error)
            if 'first_fault' not in report:
                try:report['first_fault']=snap()
                except BaseException as error:report['fault_snapshot_error']=repr(error)
            if report.get('stop_request_sent'):
                for _ in range(8):
                    time.sleep(.1)
                    try:s=snap()
                    except BaseException as error:report['stop_status_read_error']=repr(error);break
                    report['after_stop']=s
                    if not s['g_it_request'] and not s['g_source_parallel_stop_mask'] and not s['g_lift_ruler_stop_pending'] and not s['g_vehicle_arm_stop_pending']:break
            try:save()
            except BaseException as error:report['save_error']=repr(error)
            raise
        finally:
            try:report['last']=snap()
            except BaseException as error:report['last_read_error']=repr(error)
            try:save()
            except BaseException as error:print('Save failed',repr(error),flush=True)
            print(json.dumps({k:report.get(k) for k in ['complete','sha256','flash_bytes','three_to_release_command_s','three_to_zero_s']}),flush=True)
            print(dest,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Original pzq with release50ms only; one continuous three-item cycle.')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--check',action='store_true')
    modes.add_argument('--read-only',action='store_true')
    modes.add_argument('--three-once',action='store_true')
    args=parser.parse_args()
    if args.read_only or args.three_once:run(readonly=args.read_only)
    else:print('CHECK only; no hardware access. pzq1846.4/106RPM,37deg,release50ms; ONE three-item cycle.')
