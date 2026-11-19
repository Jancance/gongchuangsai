"""One explicit bounded alignment action using real owner/DONE/STOP state.
Default is local manifest check only. No flash/reset or automatic retry.
"""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,struct,time
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-12/source-align'
P=R/'tmp/source_alignment_20261112/project'
KEYS='''g_source_body_step_request g_source_body_step_state g_source_body_step_reason
g_source_body_step_session g_source_body_step_submitted g_source_body_step_elapsed_ms
g_source_align_command_seq g_source_align_rejected g_it_state g_it_cause g_it_rejected
g_safety_ready g_safety_fault g_rtos_fault g_can_test_result g_can_ctrl_fault_latched
g_chassis_auto_active g_chassis_auto_result g_chassis_auto_cause g_chassis_auto_session
g_motion_auto_stop_state g_motion_auto_stop_ack_mask g_motion_auto_stop_fail_mask
g_extension_hold_confirmed g_extension_home_reference_valid g_lift_compare_top_valid
g_lift_compare_backoff_valid g_vm_test_request g_heading_exec_phase g_heading_exec_state
g_hwt_angle_age_ms g_hwt_rate_age_ms g_hwt_valid_mask'''.split()
FLOATS=['g_dm_can2_logical_position_deg','g_startup_extension_current_deg','g_vehicle_lift_current_deg']

def manifest():
    m=json.loads((E/'candidate.json').read_text());assert m['build_passed']
    assert json.loads((E/'checks.json').read_text())['passed']
    for name,digest in m['changes'].items():assert hashlib.sha256((P/name).read_bytes()).hexdigest()==digest,name
    im=m['images']['Vehicle_LiftRuler'];assert hashlib.sha256((R/im['axf']).read_bytes()).hexdigest()==im['sha256']
    return m,im

def snapshot(t,symbols):
    # Coalesce adjacent RAM fields to reduce debug USB traffic. No peripheral
    # window is included; PE2 is a separate read. Fault/STOP rules are unchanged.
    pairs=sorted((symbols[k],k) for k in KEYS+FLOATS);groups=[]
    for address,name in pairs:
        if not groups or address-groups[-1][-1][0]>12 or address-groups[-1][0][0]>=128:groups.append([])
        groups[-1].append((address,name))
    result={}
    for group in groups:
        start=group[0][0];words=t.read_memory_block32(start,(group[-1][0]-start)//4+1)
        for address,name in group:
            value=words[(address-start)//4]
            result[name]=struct.unpack('<f',struct.pack('<I',value))[0] if name in FLOATS else value
    result['pe2']=(t.read32(0x40021010)>>2)&1
    # ChassisAuto_ReadStatus/MotionReadStatus use this backend mask.
    # g_motion_auto_stop_ack_mask belongs to a different stop owner.
    result['backend_stop_masks']=list(t.read_memory_block32(symbols['g_chassis_auto_stop_mask'],3))
    return result

def run(a):
    _,im=manifest();live.IMAGE=R/im['axf'];symbols,segments=live.load_image()
    out=E/('align-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(image_sha256=im['sha256'],task_code='215+213+512+321',task_source='user specified',
                complete=False,flashed=False,reset=False,trace=[])
    wrote=False
    def save():out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    save()
    try:
        with attach() as session:
            t=session.target;report['core_state']=t.get_state().name;assert report['core_state']=='RUNNING'
            count=0
            for address,data in segments:
                for i in range(0,len(data),4096):
                    chunk=data[i:i+4096];assert bytes(t.read_memory_block8(address+i,len(chunk)))==chunk,'wrong image'
                    count+=len(chunk)
            report['flash_bytes_verified']=count
            def wr(name,value):
                t.write32(symbols[name],struct.unpack('<I',struct.pack('<f',value))[0] if isinstance(value,float) else value)
            try:
                before=snapshot(t,symbols);report['before']=before;save()
                if a.read_only:report['complete']=True;return
                assert before['g_source_body_step_state'] in (0,3) and not before['g_source_body_step_request']
                assert not any(before[k] for k in ['g_source_body_step_reason','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_it_cause','g_it_rejected','g_chassis_auto_active'])
                assert before['g_safety_ready']==1 and before['g_can_test_result']==1 and before['g_it_state']==2
                assert all(before[k]==1 for k in ['g_extension_hold_confirmed','g_extension_home_reference_valid','g_lift_compare_top_valid','g_lift_compare_backoff_valid','pe2'])
                assert abs(before['g_dm_can2_logical_position_deg']-180)<1
                assert before['g_hwt_valid_mask']==3 and before['g_hwt_angle_age_ms']<100 and before['g_hwt_rate_age_ms']<100
                if a.turn_deg is not None:
                    assert 5<=abs(a.turn_deg)<=10
                    wr('g_source_align_turn_deg',float(a.turn_deg));request=3;report['requested_turn_deg']=a.turn_deg
                elif a.axis is not None:
                    assert a.mm is not None and 1<=a.mm<=20
                    wr('g_source_align_direction',a.axis);wr('g_source_align_distance_mm',float(a.mm));request=2
                    report.update(requested_axis=a.axis,requested_distance_mm=a.mm)
                else:
                    assert a.finish and before['g_source_body_step_state']==3;request=7
                wrote=True;wr('g_source_body_step_request',request);start=time.monotonic()
                while time.monotonic()-start<14:
                    s=snapshot(t,symbols);report['trace'].append(s)
                    if any(s[k] for k in ['g_source_body_step_reason','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_it_cause','g_it_rejected']):
                        report['first_fault']=s;raise RuntimeError('alignment fault/rejection')
                    fresh=request==7 or s['g_source_align_command_seq']!=before['g_source_align_command_seq']
                    if fresh and not s['g_source_body_step_request'] and s['g_source_body_step_state']==3 and not s['g_chassis_auto_active']:
                        assert s['g_chassis_auto_result']==1 and s['backend_stop_masks'][2]==15
                        report.update(complete=True,software_complete=True,physical_result_pending=True,after=s);return
                    time.sleep(.15)
                raise TimeoutError('alignment action')
            except BaseException as exc:
                report['error']=repr(exc)
                if wrote:
                    try:
                        wr('g_source_body_step_request',9);report['stop_request_sent']=True
                        for _ in range(15):
                            time.sleep(.15);s=snapshot(t,symbols);report['after_stop']=s
                            if not s['g_chassis_auto_active'] and s['backend_stop_masks'][2]==15:break
                    except BaseException as err:report['stop_error']=repr(err)
                raise
            finally:
                try:report['last']=snapshot(t,symbols)
                except BaseException as err:report['last_error']=repr(err)
    except BaseException as exc:
        report.setdefault('error',repr(exc));raise
    finally:save();print(out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group()
    g.add_argument('--read-only',action='store_true');g.add_argument('--turn-deg',type=float)
    g.add_argument('--axis',type=int,choices=[0,1,2,3]);g.add_argument('--finish',action='store_true');p.add_argument('--mm',type=float)
    p.add_argument('--near-candidate',action='store_true')
    p.add_argument('--region-candidate',action='store_true')
    a=p.parse_args()
    if a.near_candidate:
        E=R/'evidence/integration/2026-11-13/near-pzq';P=R/'tmp/source_near_pzq_20261113/project'
    if a.region_candidate:
        E=R/'evidence/integration/2026-11-13/region-batch';P=R/'tmp/source_region_batch_20261113/project'
    if a.mm is not None and a.axis is None:p.error('--mm requires --axis')
    if a.read_only or a.turn_deg is not None or a.axis is not None or a.finish:run(a)
    else:manifest();print('CHECK only; no devices, reset or motion. Separate explicit axis/turn actions.')
