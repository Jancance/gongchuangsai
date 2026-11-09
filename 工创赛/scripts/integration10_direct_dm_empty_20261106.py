"""既有独立4310空爪点位入口：保留伸缩故障，真实新反馈准入，不伪造参考。
本次操作者已确认空爪顶部/伸缩机械停稳。一次向载盘候选点转向后停止并关VM。
"""
import json,time,struct,hashlib,sys
import integration10_complete_live_20261106 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from elftools.elf.elffile import ELFFile

def main():
    image=live.ROOT/'tmp/integration10_can2_schedule_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
    ready=json.loads((live.ROOT/'evidence/integration/2026-11-06/can2-recovery/schedule-ready.json').read_text(encoding='utf-8'))
    assert hashlib.sha256(image.read_bytes()).hexdigest()==ready['builds']['Vehicle_LiftRuler']['sha256']
    live.live.transport.live.IMAGE=image;n,segs=live.live.transport.live.load_image()
    with image.open('rb') as f:
        elf=ELFFile(f);private={x.name:int(x['st_value']) for x in elf.get_section_by_name('.symtab').iter_symbols() if x.name=='eh_stop_pending'}
    assert 'eh_stop_pending' in private
    opt={'target_override':'cortex_m','connect_mode':'attach','frequency':1000000,'auto_unlock':False,
      'resume_on_disconnect':False,'no_config':True,'cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True}
    Session(None,options=opt);p=ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(p)==1 and p[0].unique_id=='6afaae24'
    live.live.transport.synchronous_interface(p[0]._link._interface);session=Session(p[0],options=opt)
    report={'reset':False,'halt':False,'flash':False,'writes':[],
      'operator_confirmed_empty_top_and_extension_stopped':True,'separate_supervised_DM_entry':True,
      'target_candidate_deg':35.1079407,'physical_carrier_alignment_confirmed':False,'fault_or_reference_fields_written':False}
    tag=sys.argv[1] if len(sys.argv)>1 else 'direct-empty-carrier-01'
    assert all(c.isalnum() or c in '-_' for c in tag)
    path=live.OUT/(tag+'.json');assert not path.exists()
    target=None;power=False;motion=False
    names=['g_it_state','g_it_cause','g_startup_axes_state','g_startup_axes_cause','g_startup_axes_vm_request',
      'g_lift_ruler_state','g_lift_ruler_cause','g_lift_ruler_request','g_lift_ruler_stop_pending',
      'g_dm_can2_state','g_dm_can2_result','g_dm_can2_read_cycle','g_dm_can2_ctrl_mode',
      'g_dm_can2_feedback_valid','g_dm_can2_valid_mask','g_dm_can2_range_mask','g_dm_can2_feedback_tick',
      'g_dm_can2_motor_state','g_dm_can2_esr','g_dm_jog_request','g_dm_jog_state','g_dm_jog_result',
      'g_dm_jog_first_fault','g_dm_jog_stop_result','g_dm_posvel_holding','g_vm_test_request',
      'g_safety_ready','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_chassis_auto_active',
      'g_vehicle_arm_state','g_vehicle_arm_stop_pending','g_source_parallel_stop_mask']
    def get(name):return target.read32(n[name])
    def fl(name):return struct.unpack('<f',struct.pack('<I',get(name)))[0]
    def put(name,value,isfloat=False):
        target.write32(n[name],struct.unpack('<I',struct.pack('<f',value))[0] if isfloat else value)
        report['writes'].append([name,value])
    def snap():
        assert target.get_state()==Target.State.RUNNING
        d={x:get(x) for x in names};d['extension_stop_pending']=target.read32(private['eh_stop_pending'])
        for x in ['g_dm_can2_logical_position_deg','g_dm_can2_position_deg','g_dm_can2_velocity_rad_s']:d[x]=fl(x)
        d['PE2_high']=bool(target.read32(0x40021014)&4);return d
    try:
        session.open();target=session.target;assert target.get_state()==Target.State.RUNNING
        for a,b in segs:
            for k in range(0,len(b),4096):assert bytes(target.read_memory_block8(a+k,len(b[k:k+4096])))==b[k:k+4096]
        before=snap();report['before']=before
        assert before['g_lift_ruler_state']==4 and not before['g_lift_ruler_cause']
        assert before['g_dm_jog_state']==0 and not before['g_dm_jog_request']
        for x in ['g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_chassis_auto_active',
          'g_vehicle_arm_stop_pending','g_source_parallel_stop_mask','g_dm_jog_first_fault','g_lift_ruler_stop_pending']:
            assert before[x]==0,x
        assert before['g_vehicle_arm_state']!=1
        target.write32(n['g_lift_ruler_controller']+4,1);power=True
        report['writes'].append(['lift_controller.vm_request',1])
        start=time.monotonic()
        while time.monotonic()-start<5:
            if get('g_vm_test_request')==1 and target.read32(0x40021014)&4 and get('g_can_guard_ready')==1 and not target.read32(private['eh_stop_pending']):break
            time.sleep(.05)
        assert get('g_vm_test_request')==1 and target.read32(0x40021014)&4
        assert get('g_can_guard_ready')==1 and not target.read32(private['eh_stop_pending'])
        time.sleep(1)
        previous=get('g_dm_can2_read_cycle');put('g_dm_can2_read_request',2)
        start=time.monotonic()
        while time.monotonic()-start<3:
            if get('g_dm_can2_read_cycle')!=previous and get('g_dm_can2_state') in (4,5):break
            time.sleep(.05)
        report['fresh_read']=snap()
        assert get('g_dm_can2_state')==4 and get('g_dm_can2_result')==1,'DM fresh read failed; no motion'
        assert get('g_dm_can2_valid_mask')==get('g_dm_can2_range_mask')==7 and get('g_dm_can2_feedback_valid')==1
        assert get('g_dm_can2_motor_state')==0 and get('g_dm_can2_ctrl_mode') in (1,2)
        assert abs(fl('g_dm_can2_velocity_rad_s'))<=.035
        assert 150<=fl('g_dm_can2_logical_position_deg')<=182,'unexpected starting branch'
        # Existing mode switch/preload/enable/fresh endpoint checks remain in JogService.
        put('g_dm_integrated_owner',1);put('g_dm_point_enable',1)
        put('g_dm_posvel_speed_deg_s',300.0,True);put('g_dm_place_target_deg',35.1079407,True)
        put('g_dm_jog_request',5);motion=True
        start=time.monotonic();trace=[]
        while time.monotonic()-start<10:
            d=snap();d['elapsed_s']=time.monotonic()-start;trace.append(d)
            if d['g_dm_jog_first_fault'] or d['g_dm_jog_state']==8:break
            if d['g_dm_jog_state']==7 and d['g_dm_jog_result']==1 and d['g_dm_posvel_holding']==1:
                report['software_arrival']=d;break
            time.sleep(.05)
        report['motion_trace']=trace
        assert 'software_arrival' in report,'DM did not reach fresh held endpoint'
        assert abs(report['software_arrival']['g_dm_can2_logical_position_deg']-35.1079407)<=3
    except Exception as e:report['error']=repr(e)
    finally:
        if target is not None and power:
            if motion:
                try:
                    put('g_dm_jog_request',3);start=time.monotonic()
                    while time.monotonic()-start<2:
                        if get('g_dm_jog_stop_result')==1 and not get('g_dm_posvel_holding'):break
                        time.sleep(.05)
                    report['DM_stop_acknowledged']=get('g_dm_jog_stop_result')==1
                except Exception as e:report['stop_error']=repr(e)
            try:
                target.write32(n['g_lift_ruler_controller']+4,0);put('g_startup_axes_vm_request',0)
                end=time.monotonic()+3
                while time.monotonic()<end:
                    if not get('g_vm_test_request') and not target.read32(0x40021014)&4:break
                    time.sleep(.05)
                report['VM_off_confirmed']=not get('g_vm_test_request') and not target.read32(0x40021014)&4
                report['final']=snap()
            except Exception as e:report['power_off_error']=repr(e)
        session.close();path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='motion_trace'}),flush=True)
    return int('error' in report)

if __name__=='__main__':raise SystemExit(main())
