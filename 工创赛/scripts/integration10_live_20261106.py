"""整合10现场核对；默认只读，显式 down 仅提交一次既有下降请求。"""
from pathlib import Path
import sys, json, hashlib, time
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

ROOT = Path(__file__).resolve().parents[1]
IMAGE = ROOT/'tmp/lift_retarget_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
OUT = ROOT/'evidence/integration/2026-11-06/integration10-single-teach'
NAMES = ['g_lift_ruler_state','g_lift_ruler_request','g_lift_ruler_cause',
 'g_lift_ruler_stop_pending','g_lift_ruler_stop_unconfirmed','g_lift_ruler_rejected',
 'g_lift_compare_operation','g_lift_compare_top_valid','g_lift_compare_top_raw',
 'g_lift_compare_target_pulses','g_lift_ruler_after_raw','g_lift_ruler_samples',
 'g_vm_test_request','g_safety_ready','g_can_test_result','g_can_guard_ready',
 'g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_chassis_auto_active',
 'g_vehicle_arm_state','g_startup_axes_state','g_dm_jog_request','g_dm_jog_state']

def main():
    action = sys.argv[1] if len(sys.argv)>1 else 'read'
    tag = sys.argv[2] if len(sys.argv)>2 else ''
    assert all(c.isalnum() or c in '-_' for c in tag)
    assert action in ('read','read-top','home','down','up')
    global IMAGE
    if action!='read':
        IMAGE=ROOT/'tmp/integration10_top_start_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
    extension_candidate=tag.startswith('ext-')
    if extension_candidate:
        IMAGE=ROOT/'tmp/integration10_extension_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
    OUT.mkdir(parents=True, exist_ok=True)
    report={'action':action,'request_writes':0,'reset':False,'halt':False,'flash':False}
    transport.live.IMAGE=IMAGE
    symbols, segments=transport.live.load_image()
    report['image_sha256']=hashlib.sha256(IMAGE.read_bytes()).hexdigest()
    expected='9da090ee48aa1bc446219b93e112ce26544bdaebe097b61f390439aa6cc515c7'
    if action!='read':
        expected=json.loads((OUT/'candidate-manifest.json').read_text())['online_sha256']
    if extension_candidate:
        expected=json.loads((OUT/'extension-ready.json').read_text())['builds']['Vehicle_LiftRuler']['sha256']
    assert report['image_sha256']==expected
    options={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,
      'auto_unlock':False,'resume_on_disconnect':False,'dap_protocol':'swd','no_config':True,
      'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=options)
    probes=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes)==1 and probes[0].unique_id=='6afaae24'
    transport.synchronous_interface(probes[0]._link._interface)
    session=Session(probes[0],options=options)
    def snapshot(target):
        assert target.get_state()==Target.State.RUNNING
        snap={n:target.read32(symbols[n]) for n in NAMES}
        snap['VM_PE2_high']=bool(target.read32(0x40021014)&4)
        snap['TIM2_CCR1']=target.read32(0x40000034)
        return snap
    try:
        session.open();target=session.target
        assert target.get_state()==Target.State.RUNNING
        for address,data in segments:
            for offset in range(0,len(data),4096):
                expected=data[offset:offset+4096]
                assert bytes(target.read_memory_block8(address+offset,len(expected)))==expected,'Flash mismatch'
        report['flash_matches']=True
        report['before']=snapshot(target)
        if action in ('home','down','up'):
            s=report['before']
            if action in ('down','up'):
                assert s['g_lift_ruler_state']==4 and s['g_lift_compare_operation'] in ((2,) if action=='up' else (1,3))
                assert s['g_lift_compare_top_valid']==1
                assert s['g_vm_test_request']==1 and s['VM_PE2_high'] and s['g_can_guard_ready']==1
            else:
                assert s['g_lift_ruler_state']==s['g_lift_compare_operation']==s['g_lift_compare_top_valid']==0
            assert s['g_lift_compare_target_pulses']==108400
            assert s['g_safety_ready']==s['g_can_test_result']==1
            for n in ['g_lift_ruler_request','g_lift_ruler_cause','g_lift_ruler_stop_pending',
              'g_lift_ruler_stop_unconfirmed','g_safety_fault','g_rtos_fault',
              'g_can_ctrl_fault_latched','g_chassis_auto_active','g_vehicle_arm_state',
              'g_startup_axes_state','g_dm_jog_request','g_dm_jog_state']:
                assert s[n]==0,n
            # Existing controller owns CAN, reference, target validation and STOP.
            with (OUT/(action+('-'+tag if tag else '')+'-samples.jsonl')).open('x',encoding='utf-8') as f:
                target.write32(symbols['g_lift_ruler_request'],1)
                report['request_writes']=1
                start=time.monotonic()
                while time.monotonic()-start<50:
                    s=snapshot(target);s['elapsed_s']=time.monotonic()-start
                    f.write(json.dumps(s)+'\n');f.flush();report['after']=s
                    if s['g_lift_ruler_cause'] or s['g_lift_ruler_state']>=5:break
                    if s['g_lift_compare_operation']==({'home':1,'down':2,'up':3}[action]) and s['g_lift_ruler_state']==4:break
                    time.sleep(.05)
    except Exception as e:
        report['error']=repr(e)
    finally:
        session.close()
        (OUT/(action+'-'+time.strftime('%H%M%S')+'.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report),flush=True)
    return int('error' in report)

if __name__=='__main__':raise SystemExit(main())
