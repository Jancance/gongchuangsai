"""Restore user-required180deg/65mm observation from zero, no yaw/lift/body/grasp."""
from pathlib import Path
from datetime import datetime
import argparse,hashlib,json,struct,time
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach
from source_alignment_live_20261112 import manifest
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-13/near-pzq'

def run():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    from source_alignment_live_20261112 import R as align_root
    m=json.loads((E/'candidate.json').read_text());assert m['build_passed']
    im=m['images']['Vehicle_LiftRuler'];live.IMAGE=R/im['axf']
    yaw=180.0
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()==im['sha256']
    symbols,segments=live.load_image()
    out=E/('near-observe65-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(image_sha256=im['sha256'],reset=False,flashed=False,body_command=False,lift_command=False,
                complete=False,steps=[])
    def save():out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with attach() as s:
        t=s.target;assert t.get_state().name=='RUNNING'
        count=0
        for address,data in segments:
            for i in range(0,len(data),4096):
                part=data[i:i+4096];assert bytes(t.read_memory_block8(address+i,len(part)))==part
                count+=len(part)
        report['flash_bytes_verified']=count
        def rd(k):return t.read32(symbols[k])
        def wr(k,v):t.write32(symbols[k],struct.unpack('<I',struct.pack('<f',v))[0] if isinstance(v,float) else v)
        def snap():
            keys='''g_it_request g_it_state g_it_action g_it_sequence g_it_cause g_it_rejected
g_it_auto_state g_it_auto_fault g_safety_ready g_safety_fault g_rtos_fault g_can_test_result
g_can_ctrl_fault_latched g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_compare_top_valid g_lift_compare_backoff_valid g_lift_ruler_state g_lift_ruler_cause
g_lift_ruler_stop_pending g_startup_axes_cause g_source_parallel_cause g_source_parallel_stop_mask
g_vehicle_arm_cause g_vehicle_arm_stop_pending g_dm_jog_first_fault g_chassis_auto_active'''.split()
            v={k:rd(k) for k in keys}
            for k in ['g_dm_can2_logical_position_deg','g_startup_extension_current_deg','g_extension_home_reference_deg','g_vehicle_lift_current_deg']:
                v[k]=struct.unpack('<f',struct.pack('<I',rd(k)))[0]
            v['pe2']=(t.read32(0x40021010)>>2)&1
            v['gripper_pwm_us']=t.read32(0x40000034)
            return v
        def healthy(v):
            zero='''g_it_cause g_it_rejected g_it_auto_fault g_safety_fault g_rtos_fault g_can_ctrl_fault_latched
g_lift_ruler_cause g_lift_ruler_stop_pending g_startup_axes_cause g_source_parallel_cause
g_source_parallel_stop_mask g_vehicle_arm_cause g_vehicle_arm_stop_pending g_dm_jog_first_fault g_chassis_auto_active'''.split()
            one='''g_safety_ready g_can_test_result g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_compare_top_valid g_lift_compare_backoff_valid pe2'''.split()
            return not any(v[k] for k in zero) and all(v[k]==1 for k in one) and v['g_it_auto_state']!=1 and v['g_it_state']!=9
        wrote=False
        try:
            first=snap();report['before']=first;save()
            assert healthy(first) and first['g_it_state']==2 and not first['g_it_request']
            assert first['gripper_pwm_us']==2056 and abs(first['g_dm_can2_logical_position_deg']-180)<1
            assert abs(first['g_startup_extension_current_deg']-first['g_extension_home_reference_deg'])<=2
            for name,axes,extension in [('near_observe65',2,65.0)]:
                before=snap();assert healthy(before) and before['g_it_state']==2 and not before['g_it_request']
                step=dict(name=name,before=before,samples=[]);report['steps'].append(step)
                wr('g_it_axes',axes);wr('g_it_dm_deg',yaw);wr('g_it_extension_mm',extension)
                wrote=True;wr('g_it_request',3);begin=time.monotonic()
                while time.monotonic()-begin<18:
                    v=snap();step['samples'].append(v)
                    if not healthy(v):raise RuntimeError('original backend fault/rejection')
                    if v['g_it_state']==2 and v['g_it_action']==3 and v['g_it_sequence']!=before['g_it_sequence'] and not v['g_it_request']:
                        step.update(after=v,elapsed_s=round(time.monotonic()-begin,3));save();print(name,step['elapsed_s'],flush=True);break
                    time.sleep(.12)
                else:raise TimeoutError(name)
            report['complete']=True
        except BaseException as exc:
            report['error']=repr(exc)
            if wrote:
                try:
                    wr('g_it_request',9);report['stop_requested']=True
                    for _ in range(12):
                        time.sleep(.1);v=snap();report['after_stop']=v
                        if not v['g_it_request'] and not v['g_source_parallel_stop_mask'] and not v['g_lift_ruler_stop_pending'] and not v['g_vehicle_arm_stop_pending']:break
                except BaseException as stop_error:report['stop_error']=repr(stop_error)
            raise
        finally:
            report['last']=snap();save();print(out,flush=True)
    print(json.dumps(dict(complete=report['complete'],yaw_deg=report['last']['g_dm_can2_logical_position_deg'],
                         extension_virtual_deg=report['last']['g_startup_extension_current_deg'])),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare-once',action='store_true')
    p.add_argument('--region-candidate',action='store_true');a=p.parse_args()
    if a.region_candidate:E=R/'evidence/integration/2026-11-13/region-batch'
    if a.prepare_once:run()
    else:print('CHECK only. --prepare-once extends0 to65 at180 safe top for observation; no yaw/lift/body/grasp.')
