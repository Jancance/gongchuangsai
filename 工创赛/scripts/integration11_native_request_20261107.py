"""完整单件示教的只读监视/单次请求；不暂停、不复位、不写健康或完成字段。"""
from pathlib import Path
import argparse,hashlib,json,struct,time
import integration10_live_20261106 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'evidence/integration/2026-11-06/integration10-complete'
IMAGE=ROOT/'tmp/integration10_complete_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
FLOATS=['g_it_dm_deg','g_it_extension_mm','g_it_lift_pulses',
 'g_dm_can2_position_deg','g_dm_can2_logical_position_deg','g_dm_can2_velocity_rad_s',
 'g_vehicle_lift_current_deg','g_extension_home_reference_deg','g_startup_extension_current_deg']
NAMES=['g_it_request','g_it_state','g_it_cause','g_it_rejected','g_it_action','g_it_sequence',
 'g_it_axes','g_it_elapsed_ms','g_it_place_depth_limit_pulses','g_lift_ruler_state','g_lift_ruler_cause',
 'g_lift_ruler_request','g_lift_ruler_stop_pending','g_lift_compare_operation','g_lift_compare_top_valid',
 'g_startup_axes_state','g_startup_axes_cause','g_extension_home_state','g_extension_home_result',
 'g_extension_home_reported','g_extension_home_reference_valid','g_extension_hold_confirmed',
 'g_dm_can2_read_cycle','g_dm_can2_feedback_valid','g_dm_can2_feedback_tick','g_dm_can2_ctrl_mode',
 'g_dm_can2_motor_state','g_dm_can2_state','g_dm_can2_result','g_dm_posvel_holding',
 'g_dm_jog_state','g_dm_jog_result','g_dm_jog_first_fault','g_dm_jog_request',
 'g_source_parallel_state','g_source_parallel_cause','g_source_parallel_stop_mask',
 'g_vehicle_arm_state','g_vehicle_arm_cause','g_vm_test_request',
 'g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']+FLOATS

def main():
    global IMAGE
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--action',type=int,choices=range(1,10))
    p.add_argument('--axes',type=int,choices=range(1,8))
    p.add_argument('--angle',type=float)
    p.add_argument('--extension',type=float)
    p.add_argument('--depth',type=float)
    p.add_argument('--place-limit',type=int)
    p.add_argument('--tag',required=True)
    p.add_argument('--recovery',action='store_true',help='Use the deployed CAN2 recovery image and its symbols')
    p.add_argument('--schedule',action='store_true',help='Use the CAN2 scheduling isolation candidate')
    p.add_argument('--empty-power-off-on-failure',action='store_true',help='Only for supervised empty-gripper trials: revoke both VM requests on failure')
    a=p.parse_args();assert a.tag and all(c.isalnum() or c in '-_' for c in a.tag)
    assert not (a.recovery and a.schedule)
    if a.schedule:
        IMAGE=ROOT/'tmp/integration10_can2_schedule_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
        ready=json.loads((ROOT/'evidence/integration/2026-11-06/can2-recovery/schedule-ready.json').read_text(encoding='utf-8'))
    elif a.recovery:
        IMAGE=ROOT/'tmp/integration10_can2_recovery_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
        ready=json.loads((ROOT/'evidence/integration/2026-11-06/can2-recovery/ready.json').read_text(encoding='utf-8'))
    else:
        ready=json.loads((OUT/'ready.json').read_text(encoding='utf-8'))
    expected=ready['builds']['Vehicle_LiftRuler']['sha256']
    if a.schedule:
        # Current deployed image: complete Flash verified during Integration11 recovery.
        expected='7c210e8222d047fcf3809d0f307e752fec11483be887672536779f78b10edb0e'
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()==expected
    live.transport.live.IMAGE=IMAGE;symbols,segments=live.transport.live.load_image()
    assert all(n in symbols for n in NAMES)
    options={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,
      'auto_unlock':False,'resume_on_disconnect':False,'dap_protocol':'swd','no_config':True,
      'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=options);probes=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes)==1 and probes[0].unique_id=='6afaae24'
    live.transport.synchronous_interface(probes[0]._link._interface)
    session=Session(probes[0],options=options)
    result={'tag':a.tag,'requested_action':a.action,'image_sha256':expected,
      'reset':False,'halt':False,'flash':False,'writes':[]}
    target=None
    with (OUT/(a.tag+'-samples.jsonl')).open('x',encoding='utf-8') as f:
        try:
            session.open();target=session.target
            def read():
                assert target.get_state()==Target.State.RUNNING
                s={}
                for n in NAMES:
                    v=target.read32(symbols[n]);s[n]=struct.unpack('<f',struct.pack('<I',v))[0] if n in FLOATS else v
                s['VM_PE2_high']=bool(target.read32(0x40021014)&4)
                s['gripper_pwm_us']=target.read32(0x40000034)
                s['carrier_pwm_us']=target.read32(0x40000440)
                return s
            for address,data in segments:
                for k in range(0,len(data),4096):
                    expected_bytes=data[k:k+4096]
                    assert bytes(target.read_memory_block8(address+k,len(expected_bytes)))==expected_bytes
            result['flash_matches']=True;before=read();result['before']=before
            if a.action:
                assert before['g_it_state'] in (0,2) and not before['g_it_request'] and not before['g_it_cause']
                assert not any(before[n] for n in ['g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched'])
                values={}
                if a.action==3:
                    assert a.axes is not None and a.angle is not None and a.extension is not None and a.depth is not None
                    assert 0<=a.angle<=270 and 0<=a.extension<=170 and 0<=a.depth<=108400
                    values={'g_it_axes':a.axes,'g_it_dm_deg':a.angle,'g_it_extension_mm':a.extension,'g_it_lift_pulses':a.depth}
                if a.place_limit is not None:
                    assert a.action==3 and 3200<=a.place_limit<=108400
                    values['g_it_place_depth_limit_pulses']=a.place_limit
                for n,v in values.items():
                    raw=struct.unpack('<I',struct.pack('<f',v))[0] if n in FLOATS else v
                    target.write32(symbols[n],raw);assert target.read32(symbols[n])==raw
                    result['writes'].append([n,v])
                target.write32(symbols['g_it_request'],a.action);result['writes'].append(['g_it_request',a.action])
            start=time.monotonic();last=None
            while True:
                s=read();s['host_elapsed_s']=round(time.monotonic()-start,3)
                f.write(json.dumps(s)+'\n');f.flush();result['after']=s
                signature=(s['g_it_state'],s['g_it_action'],s['g_it_rejected'],s['g_it_cause'])
                if signature!=last:
                    print(json.dumps({'elapsed':s['host_elapsed_s'],'state':signature}),flush=True);last=signature
                if a.action is None:break
                if s['g_it_request']==0 and s['g_it_action']==a.action:
                    if s['g_it_rejected']:result['error']='request rejected';break
                    if s['g_it_state'] in (2,9):
                        result['software_done']=s['g_it_state']==2;break
                if time.monotonic()-start>70:result['error']='host wait timeout; inspect firmware first fault';break
                time.sleep(.05)
        except Exception as e:result['error']=repr(e)
        finally:
            if a.empty_power_off_on_failure and a.action is not None and not result.get('software_done') and target is not None and result.get('flash_matches'):
                try:
                    target.write32(symbols['g_lift_ruler_controller']+4,0)
                    target.write32(symbols['g_startup_axes_vm_request'],0)
                    end=time.monotonic()+3
                    while time.monotonic()<end:
                        if target.read32(symbols['g_vm_test_request'])==0 and not target.read32(0x40021014)&4:
                            result['empty_trial_VM_off_confirmed']=True;break
                        time.sleep(.05)
                except Exception as e:result['empty_trial_power_off_error']=repr(e)
            session.close();(OUT/(a.tag+'-result.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
            print(json.dumps(result),flush=True)
    return int('error' in result or (a.action is not None and not result.get('software_done')))

if __name__=='__main__':raise SystemExit(main())
