"""Empty stationary car: reversible silent internal TX test, then original readonly query.
No reset/halt/flash/actuator command, no health or position writes.
"""
import time,json
import integration10_complete_live_20261106 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

def main():
    live.live.transport.live.IMAGE=live.IMAGE;n,segs=live.live.transport.live.load_image()
    options={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,
      'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,
      'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=options);p=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(p)==1 and p[0].unique_id=='6afaae24'
    live.live.transport.synchronous_interface(p[0]._link._interface)
    session=Session(p[0],options=options);report={'motor_commands':0,'silent_loopback':True,'reset':False,'halt':False}
    path=live.OUT/('can2-silent-loopback-'+time.strftime('%H%M%S')+'.json');assert not path.exists()
    saved=None;t=None
    try:
        session.open();t=session.target;assert t.get_state()==Target.State.RUNNING
        for a,b in segs:
            for k in range(0,len(b),4096):assert bytes(t.read_memory_block8(a+k,len(b[k:k+4096])))==b[k:k+4096]
        assert t.read32(n['g_dm_can2_state'])==5 and t.read32(n['g_dm_jog_state'])==0
        assert t.read32(n['g_it_state'])==0 and t.read32(n['g_lift_ruler_state'])==0
        base=0x40006800;saved=(t.read32(base),t.read32(base+0x1c))
        report['before']={'MCR':saved[0],'BTR':saved[1],'ESR':t.read32(base+0x18)}
        def init(on):
            t.write32(base,(saved[0]|1) if on else (saved[0]&~1))
            start=time.monotonic()
            while bool(t.read32(base+4)&1)!=on:
                assert time.monotonic()-start<1,'INAK timeout'
        init(True);t.write32(base+0x1c,saved[1]|0xc0000000);init(False)
        assert t.read32(base+0x1c)&0xc0000000==0xc0000000
        # Only a parameter-read frame, additionally isolated by SILM+LBKM.
        t.write32(base+8,1) # clear prior mailbox0 completion
        t.write32(base+0x184,8);t.write32(base+0x188,0x08330001);t.write32(base+0x18c,0)
        t.write32(base+0x180,(0x7ff<<21)|1)
        start=time.monotonic()
        while not t.read32(base+8)&1:
            assert time.monotonic()-start<1,'loopback TX timeout'
        report['loopback_TSR']=hex(t.read32(base+8));report['internal_TX_OK']=bool(t.read32(base+8)&2)
        report['loopback_ESR']=hex(t.read32(base+0x18))
        init(True);t.write32(base+0x1c,saved[1]);init(False)
        report['BTR_restored']=t.read32(base+0x1c)==saved[1]
        assert report['BTR_restored'];saved=None
        t.write32(n['g_dm_can2_read_request'],2)
        previous=t.read32(n['g_dm_can2_read_cycle']);start=time.monotonic()
        while time.monotonic()-start<3:
            if t.read32(n['g_dm_can2_read_cycle'])!=previous and t.read32(n['g_dm_can2_state']) in (4,5):break
            time.sleep(.05)
        report['external_query']={k:t.read32(n[k]) for k in ['g_dm_can2_read_cycle','g_dm_can2_state','g_dm_can2_result','g_dm_can2_rx_count','g_dm_can2_esr']}
    except Exception as e:report['error']=repr(e)
    finally:
        if saved is not None and t is not None:
            t.write32(0x40006800,saved[0]|1);time.sleep(.01)
            t.write32(0x4000681c,saved[1]);t.write32(0x40006800,saved[0])
        session.close();path.write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))

if __name__=='__main__':main()
