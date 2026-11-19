"""One explicit supervised body step. Default check never opens devices.
Separate prepare/read/step modes; no flash/reset, no automatic retry.
"""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,struct,time
from contextlib import contextmanager
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach
from source_body_fault_snapshot_20261111 import capture, stop_acknowledged
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-11/source-body-step'
P=R/'tmp/source_body_step_20261111/project'
GRIPPER_PWM_REGISTER=0x40000034

@contextmanager
def attach_with_report(report,save):
    try:
        with attach() as session:yield session
    except BaseException as exc:
        report.setdefault('error',repr(exc));save();raise
def manifest():
    m=json.loads((E/'candidate.json').read_text());assert m['build_passed']
    assert json.loads((E/'checks.json').read_text())['passed']
    for rel,digest in m['changes'].items():assert hashlib.sha256((P/rel).read_bytes()).hexdigest()==digest,rel
    image=m['images']['Vehicle_LiftRuler'];assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
    return m,image
def run(mode):
    m,im=manifest();live.IMAGE=R/im['axf'];symbols,segments=live.load_image()
    dest=E/(mode+'-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json');assert not dest.exists()
    report=dict(mode=mode,image_sha256=im['sha256'],flashed=False,reset=False,complete=False,steps=[],trace=[])
    def save():dest.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    save()
    with attach_with_report(report,save) as session:
        t=session.target;report['core_state']=t.get_state().name
        # Inspection may diagnose a halted/reset core; motion modes still
        # require RUNNING and never resume it on the operator's behalf.
        assert mode=='read-only' or report['core_state']=='RUNNING',report['core_state']
        report['flash_bytes_verified']=0
        for address,data in segments:
            for i in range(0,len(data),4096):
                part=data[i:i+4096];assert bytes(t.read_memory_block8(address+i,len(part)))==part,'wrong board image'
                report['flash_bytes_verified']+=len(part)
        def rd(key):return t.read32(symbols[key])
        def wr(key,value):t.write32(symbols[key],struct.unpack('<I',struct.pack('<f',value))[0] if isinstance(value,float) else value)
        def snap():
            keys='''g_src_now_ms g_source_body_step_request g_source_body_step_state g_source_body_step_reason
g_source_body_step_session g_source_body_step_elapsed_ms g_source_body_step_submitted
g_it_request g_it_state g_it_action g_it_axes g_it_cause g_it_rejected g_it_sequence g_it_auto_state
g_safety_ready g_safety_fault g_rtos_fault g_can_test_result g_can_ctrl_fault_latched
g_can_guard_ready g_can_guard_cause g_can_guard_bad_nodes g_can_poll_valid_mask g_can_poll_protect_mask
g_chassis_auto_active g_chassis_auto_state g_chassis_auto_result g_chassis_auto_cause g_chassis_auto_session
g_motion_auto_stop_state g_motion_auto_stop_ack_mask g_motion_auto_stop_fail_mask g_motion_auto_stop_session g_motion_auto_stop_cause g_motion_auto_stop_cut_vm g_vm_test_request
g_startup_axes_state g_startup_axes_cause g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_ruler_state g_lift_ruler_cause g_lift_ruler_samples g_lift_ruler_stop_pending g_lift_compare_operation
g_lift_compare_top_valid g_lift_compare_backoff_valid g_source_parallel_cause g_source_parallel_stop_mask
g_vehicle_arm_cause g_vehicle_arm_stop_pending g_dm_jog_first_fault
g_hwt_valid_mask g_hwt_angle_age_ms g_hwt_rate_age_ms'''.split()
            if 'g_low_state' in symbols:
                keys += 'g_low_state g_low_phase g_low_request g_low_fault g_low_accepted_sequence g_low_done_sequence'.split()
            state={key:rd(key) for key in keys}
            for key in ['g_dm_can2_logical_position_deg','g_it_extension_mm','g_startup_extension_current_deg','g_extension_home_reference_deg','g_vehicle_lift_current_deg']:
                state[key]=struct.unpack('<f',struct.pack('<I',rd(key)))[0]
            for key in ['g_can_poll_result','g_can_poll_flags','g_can_poll_tick','g_chassis_auto_stop_mask']:
                state[key]=list(t.read_memory_block32(symbols[key],3 if key.endswith('stop_mask') else 4))
            state['pe2']=(t.read32(0x40021010)>>2)&1
            state['gripper_pwm_us']=t.read32(GRIPPER_PWM_REGISTER)
            state['carrier_pwm_us']=t.read32(0x40000440)
            return state
        def healthy(s):
            zero='''g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_it_cause g_it_rejected
g_startup_axes_cause g_lift_ruler_cause g_source_parallel_cause g_vehicle_arm_cause g_dm_jog_first_fault'''.split()
            return s['g_safety_ready']==1 and s['g_can_test_result']==1 and not any(s[k] for k in zero)
        writes=False
        def command(label,request,values=(),timeout=18,fresh_lift=False):
            nonlocal writes
            before=snap();assert healthy(before) and before['g_it_state'] in (0,2) and not before['g_it_request'],before
            step=dict(label=label,request=request,before=before,samples=[]);report['steps'].append(step)
            writes=True
            for key,value in values:wr(key,value)
            wr('g_it_request',request);start=time.monotonic();seen=0
            while time.monotonic()-start<timeout:
                s=snap();step['samples'].append(s)
                if not healthy(s) or s['g_it_state']==9:report['first_fault']=s;raise RuntimeError(label+' failed')
                done=s['g_it_state']==2 and s['g_it_action']==request and not s['g_it_request']
                if fresh_lift:done=done and s['g_lift_ruler_samples']!=before['g_lift_ruler_samples']
                seen=seen+1 if done else 0
                if seen>=2:
                    step.update(after=s,elapsed_s=round(time.monotonic()-start,3));save();print(label,step['elapsed_s'],flush=True);return
                time.sleep(.04)
            raise TimeoutError(label)
        try:
            s=snap();report['before']=s;save()
            if mode=='read-only':report['complete']=True;return
            assert healthy(s) and not s['g_chassis_auto_active'] and s['g_source_body_step_state']==0 and not s['g_source_body_step_request'],s
            if mode=='prepare-once':
                # Only a freshly flashed/reset candidate, no reused references.
                assert s['g_it_state']==0 and s['g_it_auto_state']==0 and s['g_startup_axes_state']==0 and s['g_lift_ruler_state']==0,s
                assert s['pe2']==0,s
                # No descent target override: initialIT1 establishes actual top.
                command('actual_top_reference',1,(('g_it_carrier_auto_deg',-1.0),),timeout=63,fresh_lift=True)
                command('actual_extension_home',2)
                command('read_current_arm',7)
                command('open_gripper',6)
                command('view_yaw180',3,(('g_it_axes',1),('g_it_dm_deg',180.0),('g_it_extension_mm',0.0)))
                command('view_extend65',3,(('g_it_axes',2),('g_it_dm_deg',180.0),('g_it_extension_mm',65.0)))
            else:
                assert mode=='step-once'
                assert s['g_it_state']==2 and s['g_lift_compare_top_valid']==1 and s['g_lift_compare_backoff_valid']==1
                assert s['g_extension_home_reference_valid']==1 and s['g_extension_hold_confirmed']==1 and s['gripper_pwm_us']==2056
                assert abs(s['g_dm_can2_logical_position_deg']-180)<1 and s['pe2']==1
                assert s['g_it_action']==3 and s['g_it_axes']==2 and s['g_it_extension_mm']==65
                # Original VSP nonzero extension completion window is45 virtual
                # motor degrees. The2deg window belongs only to zero-return.
                assert abs(s['g_startup_extension_current_deg']-(s['g_extension_home_reference_deg']-65*315.5*360/3200))<=45
                assert s['g_hwt_valid_mask']==3 and s['g_hwt_angle_age_ms']<100 and s['g_hwt_rate_age_ms']<100
                writes=True;wr('g_source_body_step_request',1);start=time.monotonic();seen_running=False
                while time.monotonic()-start<16:
                    s=snap();report['trace'].append(s)
                    if s['g_source_body_step_submitted']:seen_running=True
                    if not healthy(s) or s['g_source_body_step_state'] in (8,9) or s['g_source_body_step_reason']:
                        report['first_fault']=s;raise RuntimeError('body step fault or rejection')
                    if seen_running and s['g_source_body_step_state']==3 and not s['g_chassis_auto_active'] and s['g_chassis_auto_result']==1:
                        assert s['g_source_body_step_session']==s['g_chassis_auto_session'] and s['g_chassis_auto_stop_mask'][2]==15
                        assert s['pe2']==1 and s['g_vm_test_request']==1
                        report['software_complete']=True;report['physical_displacement_and_stop_pending_user']=True;break
                    time.sleep(.03)
                else:raise TimeoutError('body step')
            report['complete']=True
        except BaseException as exc:
            report['error']=repr(exc)
            if 'first_fault' not in report:
                try:report['first_fault']=snap()
                except BaseException as err:report['snapshot_error']=repr(err)
            if writes:
                try:
                    wr('g_source_body_step_request',9)
                    if mode=='prepare-once':wr('g_it_request',9)
                    report['stop_request_sent']=True
                    for _ in range(15):
                        time.sleep(.1);last=snap();report['after_stop']=last
                        if mode=='step-once' and last['g_source_body_step_state']==9 and not last['g_chassis_auto_active'] and last['g_chassis_auto_stop_mask'][2]==15:break
                except BaseException as err:report['stop_error']=repr(err)
            raise
        finally:
            try:report['last']=snap()
            except BaseException as exc:report['last_error']=repr(exc)
            # Capture the already-frozen first CAN fault after issuing STOP;
            # detailed diagnostics must never delay the stop request above.
            try:
                report['fault_detail']=capture(t,symbols)
                report['stop_protocol_acknowledged']=stop_acknowledged(report.get('last',{}))
            except BaseException as exc:report['fault_detail_error']=repr(exc)
            save();print(dest,flush=True)
    print(json.dumps(dict(complete=report['complete'],mode=mode,software_complete=report.get('software_complete')),ensure_ascii=False),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();group=p.add_mutually_exclusive_group()
    candidate=p.add_mutually_exclusive_group()
    candidate.add_argument('--stop-candidate',action='store_true',help='Select isolated STOP repair; never flash or reset automatically.')
    candidate.add_argument('--hold-candidate',action='store_true',help='Select held-body ownership fix; never flash or reset automatically.')
    candidate.add_argument('--alignment-candidate',action='store_true',help='Select bounded alignment entry; never flash or reset automatically.')
    candidate.add_argument('--near-candidate',action='store_true',help='Select fixed-near pzq/guard repair; never flash or reset automatically.')
    candidate.add_argument('--region-candidate',action='store_true',help='Select stopped-disc region batch; never flash or reset automatically.')
    candidate.add_argument('--low-candidate',action='store_true',help='Select low-height per-piece owner; no automatic flash/reset.')
    candidate.add_argument('--homeexit-candidate',action='store_true',help='Select bounded home-exit revision; no automatic flash/reset.')
    candidate.add_argument('--observe-settle-candidate',action='store_true',help='Select scoped observer-return settling revision.')
    candidate.add_argument('--readonly-diag-candidate',action='store_true',help='Select one-shot read-only extension configuration access.')
    group.add_argument('--read-only',action='store_true');group.add_argument('--prepare-once',action='store_true');group.add_argument('--step-once',action='store_true')
    a=p.parse_args()
    if a.stop_candidate:
        E=R/'evidence/integration/2026-11-11/source-body-stop';P=R/'tmp/source_body_stop_20261111/project'
    elif a.hold_candidate:
        E=R/'evidence/integration/2026-11-11/source-body-hold';P=R/'tmp/source_body_hold_20261111/project'
    elif a.alignment_candidate:
        E=R/'evidence/integration/2026-11-12/source-align';P=R/'tmp/source_alignment_20261112/project'
    elif a.near_candidate:
        E=R/'evidence/integration/2026-11-13/near-pzq';P=R/'tmp/source_near_pzq_20261113/project'
    elif a.region_candidate:
        E=R/'evidence/integration/2026-11-13/region-batch';P=R/'tmp/source_region_batch_20261113/project'
    elif a.low_candidate:
        E=R/'evidence/integration/2026-11-14/low-pick';P=R/'tmp/source_low_pick_20261114/project'
    elif a.homeexit_candidate:
        E=R/'evidence/integration/2026-11-14/low-pick-homeexit';P=R/'tmp/source_low_pick_homeexit_20261114/project'
    elif a.observe_settle_candidate:
        E=R/'evidence/integration/2026-11-14/low-pick-observe-settle';P=R/'tmp/source_low_pick_observe_settle_20261114/project'
    elif a.readonly_diag_candidate:
        E=R/'evidence/integration/2026-11-14/low-readonly-diag';P=R/'tmp/source_low_readonly_diag_20261114/project'
    if a.read_only:run('read-only')
    elif a.prepare_once:run('prepare-once')
    elif a.step_once:run('step-once')
    else:
        manifest();print('CHECK only: no devices; separate prepare/read/one-left10mm modes; no automatic flash/reset/retry.')
