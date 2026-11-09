"""一次170开爪→已验源下降→140闭爪→真实回顶；不横移、不计抓牢成功。"""
from pathlib import Path
import sys,json,time,struct,hashlib
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R=Path(__file__).resolve().parents[1];O=R/'evidence/integration/2026-11-07/single-cycle'
IMAGE=R/'tmp/integration11_two_stage_ready_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED='1651a68a84782aeab63c325114aac8817db3cf8e97b3d7ab5c2eda237b3f9343'
NAMES=['g_it_state','g_it_cause','g_it_request','g_it_lift_permit',
 'g_lift_ruler_state','g_lift_ruler_request','g_lift_ruler_rejected','g_lift_ruler_phase',
 'g_lift_ruler_cause','g_lift_ruler_samples','g_lift_ruler_stop_pending','g_lift_ruler_stop_unconfirmed',
 'g_lift_ruler_before_raw','g_lift_ruler_after_raw','g_lift_ruler_target_raw',
 'g_lift_compare_operation','g_lift_compare_top_valid','g_lift_compare_top_raw',
 'g_lift_compare_home_seen','g_lift_compare_home_reported','g_lift_compare_target_pulses',
 'g_startup_axes_state','g_startup_axes_cause','g_extension_home_reference_valid','g_extension_hold_confirmed',
 'g_vehicle_arm_state','g_vehicle_arm_stop_pending','g_source_parallel_stop_mask',
 'g_dm_can2_read_cycle','g_dm_can2_state','g_dm_can2_result','g_dm_can2_ctrl_mode',
 'g_dm_can2_feedback_valid','g_dm_can2_valid_mask','g_dm_can2_range_mask','g_dm_can2_motor_state',
 'g_dm_jog_state','g_dm_jog_request','g_dm_jog_first_fault','g_dm_jog_result','g_dm_posvel_holding','g_vm_test_request','g_can_guard_ready',
 'g_safety_ready','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_chassis_auto_active']
FLOATS=['g_dm_can2_logical_position_deg','g_dm_can2_velocity_rad_s',
 'g_extension_home_reference_deg','g_startup_extension_current_deg']

def main():
    tag=sys.argv[1] if len(sys.argv)>1 else 'pickup-to-top-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert json.loads((O/'pickup-to-top-matlab.json').read_text())['passed']
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==EXPECTED
    transport.live.IMAGE=IMAGE;n,segs=transport.live.load_image()
    opt={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,'auto_unlock':False,
      'resume_on_disconnect':False,'no_config':True,'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=opt);probes=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes)==1 and probes[0].unique_id=='6afaae24'
    transport.synchronous_interface(probes[0]._link._interface);session=Session(probes[0],options=opt)
    report={'image_sha256':EXPECTED,'reset':False,'halt':False,'flash':False,'writes':[],
      'horizontal_commands':0,'physical_grasp_confirmed':False,'loaded_carrier_lip_clearance_confirmed':False}
    target=None;pending=False
    def get(name):return target.read32(n[name])
    def put(name,value):target.write32(n[name],value);report['writes'].append([name,value])
    def snap():
        assert target.get_state()==Target.State.RUNNING
        d={k:get(k) for k in NAMES}
        for k in FLOATS:d[k]=struct.unpack('<f',struct.pack('<I',get(k)))[0]
        d['claw_pwm_us']=target.read32(0x40000034);d['VM_PE2_high']=bool(target.read32(0x40021014)&4)
        return d
    def healthy(d):
        for k in ['g_lift_ruler_cause','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
          'g_startup_axes_cause','g_dm_jog_first_fault','g_it_cause']:assert d[k]==0,k
        assert d['g_vm_test_request']==1 and d['VM_PE2_high'] and d['g_safety_ready']==1
    def operation(request,expected_op,timeout,phase,log):
        nonlocal pending
        old_samples=get('g_lift_ruler_samples');put('g_lift_ruler_request',request);pending=True
        begin=time.monotonic()
        while time.monotonic()-begin<timeout:
            d=snap();d['host_phase']=phase;d['elapsed_s']=round(time.monotonic()-begin,3)
            log.write(json.dumps(d)+'\n');log.flush();report['last']=d
            healthy(d);assert d['g_lift_ruler_state']<5 and not d['g_lift_ruler_rejected'],'lift failed or rejected'
            if d['g_lift_ruler_state']==4 and d['g_lift_compare_operation']==expected_op and d['g_lift_ruler_samples']!=old_samples:
                pending=False;report[phase]=d;print(phase+' SOFTWARE_DONE',flush=True);return
            time.sleep(.05)
        raise RuntimeError(phase+' timeout; not retried')
    with (O/(tag+'-samples.jsonl')).open('x',encoding='utf-8') as log:
        try:
            session.open();target=session.target;assert target.get_state()==Target.State.RUNNING
            for a,b in segs:
                for k in range(0,len(b),4096):assert bytes(target.read_memory_block8(a+k,len(b[k:k+4096])))==b[k:k+4096]
            report['before']=d=snap();healthy(d)
            assert d['g_it_state'] in (0,2) and d['g_it_request']==0
            assert d['g_lift_ruler_state']==4 and d['g_lift_compare_operation'] in (1,3) and d['g_lift_compare_top_valid']==1
            assert d['g_startup_axes_state']==4 and d['g_extension_home_reference_valid']==d['g_extension_hold_confirmed']==1
            for k in ['g_lift_ruler_request','g_lift_ruler_stop_pending','g_dm_jog_request',
              'g_vehicle_arm_stop_pending','g_source_parallel_stop_mask','g_chassis_auto_active']:assert d[k]==0,k
            assert d['g_vehicle_arm_state'] in (0,2)
            assert d['g_dm_jog_state']==0 or (d['g_dm_jog_state']==7 and d['g_dm_jog_result']==1 and d['g_dm_posvel_holding']==1)
            assert d['g_lift_compare_target_pulses']==108400
            assert abs(d['g_startup_extension_current_deg']-d['g_extension_home_reference_deg'])<=1
            # Direct mailbox activity from the prior operator did not update RAM angle/mode.
            # Ask the original sole CAN owner for actual current identity/ranges/status first.
            cycle=get('g_dm_can2_read_cycle');put('g_dm_can2_read_request',2);begin=time.monotonic()
            while time.monotonic()-begin<16:
                if get('g_dm_can2_read_cycle')!=cycle and get('g_dm_can2_state') in (4,5):break
                time.sleep(.05)
            report['fresh_source_status']=d=snap();healthy(d)
            assert d['g_dm_can2_state']==4 and d['g_dm_can2_result']==1 and d['g_dm_can2_feedback_valid']==1
            assert d['g_dm_can2_valid_mask']==d['g_dm_can2_range_mask']==7
            assert abs(d['g_dm_can2_logical_position_deg']-180)<=3 and abs(d['g_dm_can2_velocity_rad_s'])<=.035
            assert d['g_dm_can2_motor_state']==0 or (d['g_dm_can2_motor_state']==1 and d['g_dm_jog_state']==7 and d['g_dm_jog_result']==1 and d['g_dm_posvel_holding']==1)
            assert d['g_dm_can2_ctrl_mode'] in (1,2)
            assert target.read32(0x40000000)&1 and target.read32(0x40000028)==83 and target.read32(0x4000002c)==19999
            target.write32(0x40000034,1759);report['writes'].append(['TIM2_CCR1/open170',1759]);time.sleep(.6)
            assert target.read32(0x40000034)==1759
            put('g_it_lift_permit',1) # actual one-shot vertical-operation admission, no reference/fault writes
            operation(1,2,20,'down_to_pick',log)
            target.write32(0x40000034,1537);report['writes'].append(['TIM2_CCR1/close140',1537]);time.sleep(.6)
            operation(3,3,65,'return_to_top',log)
            assert get('g_lift_compare_top_valid')==get('g_lift_compare_home_seen')==get('g_lift_compare_home_reported')==1
            put('g_it_lift_permit',0)
            report['software_pickup_return_complete']=True
            report['holding_for_physical_grasp_and_clearance_observation']=True
        except Exception as e:
            report['error']=repr(e)
            if target is not None and pending:
                try:put('g_lift_ruler_request',2);report['stop_requested']=True
                except Exception as stop_error:report['stop_request_error']=repr(stop_error)
        finally:
            if target is not None:
                try:
                    if any(w[0]=='g_lift_ruler_request' and w[1]==1 for w in report['writes']) and not report.get('software_pickup_return_complete') and not any(w[0]=='TIM2_CCR1/close140' for w in report['writes']):
                        target.write32(n['g_lift_ruler_controller']+4,0)
                        target.write32(n['g_startup_axes_vm_request'],0)
                        put('g_it_lift_permit',0)
                        time.sleep(.5)
                        report['empty_failure_VM_off']=get('g_vm_test_request')==0 and not target.read32(0x40021014)&4
                    report['final']=snap()
                except Exception as e:report['final_read_error']=repr(e)
            session.close();(O/(tag+'-result.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps({k:v for k,v in report.items() if k not in ['before','fresh_source_status','down_to_pick','return_to_top','last']}),flush=True)
    return int('error' in report)

if __name__=='__main__':raise SystemExit(main())
