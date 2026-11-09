"""用户授权重启后仅恢复供电并读取DM，不发回零/使能/运动。"""
import hashlib,json,time,struct,sys
import integration10_complete_live_20261106 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

def main():
    recovery='--recovery' in sys.argv
    if recovery:
        live.IMAGE=live.ROOT/'tmp/integration10_can2_recovery_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
        expected=json.loads((live.ROOT/'evidence/integration/2026-11-06/can2-recovery/ready.json').read_text(encoding='utf-8'))['builds']['Vehicle_LiftRuler']['sha256']
    else:
        expected=json.loads((live.OUT/'ready.json').read_text(encoding='utf-8'))['builds']['Vehicle_LiftRuler']['sha256']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()==expected
    live.live.transport.live.IMAGE=live.IMAGE
    symbols,segments=live.live.transport.live.load_image()
    # First two members are the controller state and the actual VM request.
    header=(live.ROOT/'tmp/integration10_complete_20261106/project/CHASSIS/USER/lift_ruler_calibration.h').read_text(encoding='utf-8')
    assert 'volatile uint32_t state,vm_request,cause,' in header
    options={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,
      'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,
      'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=options);p=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(p)==1 and p[0].unique_id=='6afaae24'
    live.live.transport.synchronous_interface(p[0]._link._interface)
    s=Session(p[0],options=options)
    report={'reset':False,'halt':False,'flash':False,'motor_enable_or_motion_commands':0,
      'power_request_written':False,'readonly_DM_request_written':False,
      'empty_idle_diagnostic_only':True,'VM_off_after_test':False}
    target=None
    tag=sys.argv[1] if len(sys.argv)>1 else 'user-reboot'
    assert all(c.isalnum() or c in '-_' for c in tag)
    path=live.OUT/(tag+'-power-readonly.json')
    assert not path.exists()
    try:
        s.open();t=s.target;target=t;assert t.get_state()==Target.State.RUNNING
        for a,b in segments:
            for k in range(0,len(b),4096):
                assert bytes(t.read_memory_block8(a+k,len(b[k:k+4096])))==b[k:k+4096]
        for name in ['g_it_state','g_it_request','g_lift_ruler_state','g_lift_ruler_request',
          'g_lift_ruler_cause','g_startup_axes_state','g_dm_jog_state','g_dm_jog_request',
          'g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:
            assert t.read32(symbols[name])==0,name
        # Real power request only. No state, position, reference or completion writes.
        t.write32(symbols['g_lift_ruler_controller']+4,1)
        report['power_request_written']=True
        start=time.monotonic()
        while time.monotonic()-start<5:
            if t.read32(symbols['g_vm_test_request'])==1 and t.read32(0x40021014)&4 and t.read32(symbols['g_can_guard_ready'])==1:break
            time.sleep(.1)
        assert t.read32(0x40021014)&4 and t.read32(symbols['g_can_guard_ready'])==1
        time.sleep(1)
        previous_cycle=t.read32(symbols['g_dm_can2_read_cycle'])
        t.write32(symbols['g_dm_can2_read_request'],2);report['readonly_DM_request_written']=True
        start=time.monotonic();samples=[]
        while time.monotonic()-start<5:
            snap={n:t.read32(symbols[n]) for n in ['g_dm_can2_state','g_dm_can2_result',
             'g_dm_can2_read_cycle','g_dm_can2_valid_mask','g_dm_can2_range_mask',
             'g_dm_can2_feedback_valid','g_dm_can2_ctrl_mode','g_dm_can2_motor_state',
             'g_dm_can2_tx_count','g_dm_can2_rx_count','g_dm_can2_esr',
             'g_vm_test_request','g_lift_compare_top_valid','g_safety_fault','g_rtos_fault']}
            samples.append(snap)
            if snap['g_dm_can2_read_cycle']!=previous_cycle and snap['g_dm_can2_state'] in (4,5):break
            time.sleep(.05)
        report['samples']=samples;report['after']=snap
        report['angles']={n:struct.unpack('<f',struct.pack('<I',t.read32(symbols[n])))[0] for n in ['g_dm_can2_position_deg','g_dm_can2_logical_position_deg','g_dm_can2_velocity_rad_s']}
        report['CAN2_ESR_now']=hex(t.read32(0x40006818));report['VM_PE2_high']=bool(t.read32(0x40021014)&4)
        report['recovery_diagnostics']={n:t.read32(symbols[n]) for n in [
          'g_dm_can2_lec','g_dm_can2_tec','g_dm_can2_rec','g_dm_can2_bus_off',
          'g_dm_can2_first_error_valid','g_dm_can2_first_error_esr','g_dm_can2_first_error_tick',
          'g_dm_can2_filter_repairs','g_dm_can2_hardware_initializations'] if n in symbols}
        report['CAN2_MCR']=hex(t.read32(0x40006800))
    except Exception as e:report['error']=repr(e)
    finally:
        # 本入口只在重启后空载/空闲调用。只读诊断结束即撤销实际VM请求，
        # 包含通信失败/异常出口，避免诊断等待期间让其他轴长期通电发热。
        # 正式持料/运动入口不使用本脚本，不能在有载时据此取消保位。
        if report['power_request_written'] and target is not None:
            try:
                target.write32(symbols['g_lift_ruler_controller']+4,0)
                end=time.monotonic()+3
                while time.monotonic()<end:
                    request=target.read32(symbols['g_vm_test_request'])
                    pin=bool(target.read32(0x40021014)&4)
                    if request==0 and not pin:
                        report['VM_off_after_test']=True;break
                    time.sleep(.05)
                report['VM_off_is_pin_readback_not_voltage_measurement']=True
            except Exception as e:
                report['power_off_error']=repr(e)
        s.close();path.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))
    return int('error' in report)

if __name__=='__main__':raise SystemExit(main())
