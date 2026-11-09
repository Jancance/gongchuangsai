"""本次持料顶部单次伸缩回零；原CAN任务执行，脚本只提交一次请求并只读记录。"""
from pathlib import Path
import json, hashlib, time
import integration10_live_20261106 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

def main():
    image=live.ROOT/'tmp/integration10_extension_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
    assert hashlib.sha256(image.read_bytes()).hexdigest()==json.loads((live.OUT/'extension-ready.json').read_text())['builds']['Vehicle_LiftRuler']['sha256']
    live.transport.live.IMAGE=image
    symbols,segments=live.transport.live.load_image()
    names=['g_lift_ruler_state','g_lift_ruler_cause','g_lift_compare_operation','g_lift_compare_top_valid',
      'g_startup_axes_state','g_startup_axes_cause','g_integration10_extension_request',
      'g_integration10_extension_rejected','g_extension_home_state','g_extension_home_result',
      'g_extension_home_reported','g_extension_home_flags','g_extension_home_reference_valid',
      'g_extension_home_abort_result','g_extension_home_stop_result','g_extension_hold_confirmed',
      'g_vm_test_request','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']
    options={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,
      'auto_unlock':False,'resume_on_disconnect':False,'dap_protocol':'swd','no_config':True,
      'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=options)
    probes=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes)==1 and probes[0].unique_id=='6afaae24'
    live.transport.synchronous_interface(probes[0]._link._interface)
    session=Session(probes[0],options=options)
    report={'request_writes':0,'reset':False,'halt':False,'flash':False,'physical_home_verified':False}
    with (live.OUT/'extension-home-samples.jsonl').open('x',encoding='utf-8') as f:
        try:
            session.open();target=session.target
            def read():
                assert target.get_state()==Target.State.RUNNING
                s={n:target.read32(symbols[n]) for n in names}
                s['VM_PE2_high']=bool(target.read32(0x40021014)&4)
                s['claw_pwm_us']=target.read32(0x40000034)
                return s
            for a,b in segments:
                for k in range(0,len(b),4096):
                    assert bytes(target.read_memory_block8(a+k,len(b[k:k+4096])))==b[k:k+4096]
            s=read();report['before']=s
            assert s['g_lift_ruler_state']==4 and s['g_lift_compare_operation']==3 and s['g_lift_compare_top_valid']==1
            for n in ['g_lift_ruler_cause','g_startup_axes_state','g_integration10_extension_request',
                      'g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:assert not s[n],n
            assert s['claw_pwm_us']==1537 and s['g_vm_test_request']==1 and s['VM_PE2_high']
            target.write32(symbols['g_integration10_extension_request'],1);report['request_writes']=1
            start=time.monotonic();terminal=None
            while time.monotonic()-start<20:
                s=read();s['elapsed_s']=time.monotonic()-start
                f.write(json.dumps(s)+'\n');f.flush();report['after']=s
                if s['g_startup_axes_state'] in (4,5,6):
                    if terminal is None:terminal=time.monotonic()
                    if time.monotonic()-terminal>1:break
                time.sleep(.05)
            report['software_home_done']=s['g_startup_axes_state']==4 and s['g_extension_home_reported']==1 and s['g_extension_home_reference_valid']==1 and s['g_extension_hold_confirmed']==1
        except Exception as e:report['error']=repr(e)
        finally:
            session.close()
            (live.OUT/'extension-home-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps(report),flush=True)
    return int('error' in report or not report.get('software_home_done'))

if __name__=='__main__':raise SystemExit(main())
