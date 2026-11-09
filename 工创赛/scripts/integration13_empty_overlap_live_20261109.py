"""Explicit one-shot empty commissioning. Default/--check never opens a probe."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import time
import gripper210_attach_read_20261105 as live

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-07/stage2-mainline'
PLAN = ['top_reference', 'extension_home', 'read_arm', 'open', 'yaw_195',
        'empty_extension_and_descent_once', 'return_up', 'retract', 'read_arm_again', 'yaw_0']


def run(readonly=False, loaded_up5=False, recover_held=False, loaded_overlap=False, both_overlap=False, three_direct=False, immediate=False):
    m = json.loads((E/'stage2-fast-candidate-20261109.json').read_text(encoding='utf-8'))
    image = m['images']['Vehicle_LiftRuler']
    live.IMAGE = R / image['axf']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest() == image['sha256']
    for name, digest in m['changed'].items():
        p = R / 'tmp/stage2_sub20_20261109/project/CHASSIS/USER' / name
        assert hashlib.sha256(p.read_bytes()).hexdigest() == digest
    symbols, segments = live.load_image()
    dest = E/(('integration13-three-immediate-once-' if immediate else
               'integration13-three-direct-once-' if three_direct else
               'integration13-left-both-overlap-once-' if both_overlap else
               'integration13-left-carry-overlap-once-' if loaded_overlap else
               'integration13-left-held-place-once-' if recover_held else
               'integration13-left-up5-once-' if loaded_up5 else
               'integration13-empty-'+('readonly-' if readonly else 'once-'))+
              datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    assert not dest.exists()
    report = {'sha256':image['sha256'], 'plan':PLAN, 'complete':False,
              'readonly':readonly, 'reset':False, 'flashed':False, 'steps':[]}
    def save():
        dest.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    with live.attach() as session:
        t = session.target
        assert t.get_state().name == 'RUNNING'
        report['flash_bytes'] = 0
        for addr, data in segments:
            for i in range(0,len(data),4096):
                expected = data[i:i+4096]
                assert bytes(t.read_memory_block8(addr+i,len(expected))) == expected, f'Flash mismatch {addr+i:#x}'
                report['flash_bytes'] += len(expected)
        def rd(k): return t.read32(symbols[k])
        def rf(k): return struct.unpack('<f',struct.pack('<I',rd(k)))[0]
        def wr(k,v):
            if isinstance(v,float):v=struct.unpack('<I',struct.pack('<f',v))[0]
            t.write32(symbols[k],v)
        def snap():
            keys='''g_src_now_ms g_it_state g_it_action g_it_request g_it_rejected g_it_cause
g_it_sequence g_it_elapsed_ms g_safety_ready g_safety_fault g_rtos_fault g_can_test_result
g_can_ctrl_fault_latched g_startup_axes_state g_startup_axes_cause
g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_ruler_state g_lift_ruler_cause g_lift_ruler_samples g_lift_compare_operation
g_lift_ruler_options g_lift_compare_top_raw g_lift_ruler_after_raw g_lift_ruler_target_raw
g_lift_compare_top_valid g_lift_compare_backoff_valid g_lift_ruler_stop_pending
g_source_parallel_state g_source_parallel_cause g_source_parallel_stop_mask
g_source_parallel_started_mask g_source_parallel_done_mask g_source_parallel_start_skew_ms
g_vehicle_arm_state g_vehicle_arm_cause g_vehicle_arm_stop_pending
g_chassis_auto_active g_dm_jog_first_fault g_dm_jog_request g_dm_posvel_holding'''.split()
            keys += [k for k in ('g_it_auto_state','g_it_auto_phase','g_it_auto_item','g_it_auto_fault',
                                'g_it_auto_elapsed_ms','g_it_auto_phase_ms','g_it_lead5_grip_done',
                                'g_vsp_prepare_miss','g_vsp_prepare_nonzero','g_vsp_prepare_speed_raw',
                                'g_vsp_prepare_wait_address') if k in symbols]
            s={k:rd(k) for k in keys}
            if 'g_it_auto_release_ms' in symbols:
                s['release_command_times_ms']=list(t.read_memory_block32(symbols['g_it_auto_release_ms'],3))
            for k in ('g_dm_can2_logical_position_deg','g_startup_extension_current_deg',
                      'g_extension_home_reference_deg','g_vehicle_lift_current_deg'):
                s[k]=rf(k)
            s['pe2']=(t.read32(0x40021010)>>2)&1
            s['gripper_pwm_us']=t.read32(0x40000034)
            s['carrier_pwm_us']=t.read32(0x40000440)
            return s
        def health(s):
            return (s['g_safety_ready']==1 and s['g_can_test_result']==1 and
                    not any(s[k] for k in ('g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
                        'g_it_cause','g_startup_axes_cause','g_lift_ruler_cause','g_source_parallel_cause',
                        'g_vehicle_arm_cause','g_dm_jog_first_fault','g_chassis_auto_active')))
        def cmd(label,request,values=(),timeout=15,fresh_lift=False,fresh_sequence=False):
            before=snap()
            assert health(before) and before['g_it_state'] in (0,2) and not before['g_it_request'], before
            step={'label':label,'request':request,'before':before,'samples':[]}
            report['steps'].append(step)
            for k,v in values:wr(k,v)
            start=time.monotonic();wr('g_it_request',request);consistent=0
            while time.monotonic()-start<timeout:
                s=snap();step['samples'].append(s)
                if not health(s) or s['g_it_rejected'] or s['g_it_state']==9:
                    raise RuntimeError(label+' first fault: '+json.dumps(s))
                done=(s['g_it_state']==2 and s['g_it_action']==request and s['g_it_request']==0)
                if fresh_lift:done=done and s['g_lift_ruler_samples']!=before['g_lift_ruler_samples']
                if fresh_sequence:done=done and s['g_it_sequence']!=before['g_it_sequence']
                consistent=consistent+1 if done else 0
                if consistent>=2:
                    step['after']=s;step['elapsed_s']=round(time.monotonic()-start,3);save()
                    print(label,step['elapsed_s'],'s',flush=True)
                    return s
                time.sleep(.03)
            raise TimeoutError(label)
        writes_started=False
        try:
            s=snap();report['start']=s;save()
            if readonly:
                report['complete']=True
                return
            if three_direct or immediate:
                assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_lift_ruler_state']==0 and s['g_startup_axes_state']==0
                assert json.loads((E/'integration13-three-direct-matlab-20261109.json').read_text())['passed']
                report['plan']=['references','down->slot1','left->slot2','right->slot3','yaw0']
                report['route3_enabled']=False
                writes_started=True
                cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
                cmd('extension_home',2)
                cmd('read_arm',7,timeout=6)
                cmd('open',6)
                sequence_start=time.monotonic();report['item_elapsed_s']={}
                if immediate:
                    assert rd('g_it_auto_state')==0 and not rd('g_it_request')
                    report['plan']=['references','MCU down->left->right immediate chain','yaw0']
                    report['mcu_trace']=[];wr('g_it_request',16)
                    prev=None
                    while time.monotonic()-sequence_start<65:
                        now=snap();report['mcu_trace'].append(now)
                        if not health(now) or now['g_it_rejected'] or now['g_it_state']==9 or now['g_it_auto_state']==9:
                            raise RuntimeError('immediate chain first fault: '+json.dumps(now))
                        phase=(now['g_it_auto_item'],now['g_it_auto_phase'])
                        if phase!=prev:
                            print('MCU item/phase',phase,'elapsed_ms',now['g_it_auto_elapsed_ms'],flush=True);prev=phase;save()
                        if now['g_it_auto_state']==2:
                            times=now['release_command_times_ms']
                            assert 0<times[0]<times[1]<times[2]
                            report['three_to_release_command_s']=times[2]/1000
                            report['three_to_zero_s']=now['g_it_auto_elapsed_ms']/1000
                            report['item_to_release_command_s']=[times[0]/1000,(times[1]-times[0])/1000,(times[2]-times[1])/1000]
                            report['physical_placement_confirmation_pending']=True
                            report['complete']=True
                            return
                        time.sleep(.025)
                    raise TimeoutError('immediate three chain')
                for index,(region,yaw,ext) in enumerate((('down',180.0,0.0),('left',195.0,156.0),('right',165.0,160.0))):
                    item_start=time.monotonic()
                    cmd(region+'_aim',3,(('g_it_axes',1),('g_it_dm_deg',yaw),('g_it_extension_mm',0.0)),fresh_sequence=True)
                    approach=cmd(region+'_approach',14,(('g_it_dm_deg',yaw),('g_it_extension_mm',ext),
                                    ('g_it_lift_pulses',118800.0),('g_it_carrier_auto_deg',float(index)*120)),fresh_sequence=True)
                    assert approach['g_source_parallel_done_mask']==(6 if ext else 4)
                    assert approach['g_lift_compare_operation']==2 and approach['gripper_pwm_us']!=2056
                    assert approach['carrier_pwm_us']==(500,1389,2278)[index]
                    cmd(region+'_grip',5)
                    carry=cmd(region+'_carry',15,(('g_it_dm_deg',yaw),('g_it_extension_mm',ext),
                                                  ('g_it_lift_pulses',7200.0)),fresh_sequence=True)
                    assert carry['g_source_parallel_done_mask']==(7 if ext else 5)
                    assert abs(carry['g_dm_can2_logical_position_deg']-38.380798)<=1
                    assert abs(carry['g_startup_extension_current_deg']-carry['g_extension_home_reference_deg'])<=1
                    cmd(region+'_release',6)
                    report['item_elapsed_s'][region]=round(time.monotonic()-item_start,3);save()
                report['three_to_release_s']=round(time.monotonic()-sequence_start,3)
                cmd('yaw0',3,(('g_it_axes',1),('g_it_dm_deg',0.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                report['three_to_zero_s']=round(time.monotonic()-sequence_start,3)
                report['complete']=True
                return
            if loaded_overlap or both_overlap:
                report['plan']=['top_reference','extension_home','read_arm','open','slot1','yaw195',
                                'extend156','down118800','grip140','carry_overlap_to_slot','release','yaw0']
                if both_overlap:
                    assert health(s) and s['pe2']==1 and s['g_it_state']==2 and s['g_lift_ruler_state']==4
                    assert s['g_lift_compare_top_valid'] and s['g_lift_compare_backoff_valid'] and s['g_extension_hold_confirmed']
                    assert abs(s['g_dm_can2_logical_position_deg'])<=1
                    assert abs(s['g_startup_extension_current_deg']-s['g_extension_home_reference_deg'])<=1
                    assert s['gripper_pwm_us']==2056
                    assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text())['gate_cases']==199
                    report['plan']=['read_arm','slot1','yaw195','approach_overlap','grip140',
                                    'carry_overlap_to_slot','release','yaw0']
                    writes_started=True
                else:
                    assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_lift_ruler_state']==0 and s['g_startup_axes_state']==0
                    writes_started=True
                    cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
                    cmd('extension_home',2)
                cmd('read_arm',7,timeout=6)
                if not both_overlap:cmd('open',6)
                cmd('slot1',10,(('g_it_carrier_deg',0.0),))
                item_start=time.monotonic()
                cmd('yaw195',3,(('g_it_axes',1),('g_it_dm_deg',195.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                if both_overlap:
                    approach=cmd('approach_overlap',14,(('g_it_extension_mm',156.0),('g_it_lift_pulses',118800.0)),fresh_sequence=True)
                    assert approach['g_source_parallel_done_mask']==6
                    assert abs(approach['g_startup_extension_current_deg']-approach['g_extension_home_reference_deg']+5537.025)<=45
                    assert abs(approach['g_vehicle_lift_current_deg']-13365)<=45
                    # New firmware publishes actual fully settled IT14 pose.
                    # No host writes to reference/state/sample flags.
                    assert approach['g_lift_compare_operation']==2
                else:
                    cmd('extend156',3,(('g_it_axes',2),('g_it_dm_deg',195.0),('g_it_extension_mm',156.0)),fresh_sequence=True)
                    cmd('down118800',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),fresh_lift=True)
                cmd('grip140',5)
                carry=cmd('carry_overlap_to_slot',15,(('g_it_lift_pulses',7200.0),),fresh_sequence=True)
                assert carry['g_source_parallel_done_mask']==7 and abs(carry['g_dm_can2_logical_position_deg']-38.380798)<=1
                assert abs(carry['g_startup_extension_current_deg']-carry['g_extension_home_reference_deg'])<=1
                cmd('release',6)
                report['item_to_release_s']=round(time.monotonic()-item_start,3)
                cmd('yaw0',3,(('g_it_axes',1),('g_it_dm_deg',0.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                report['complete']=True
                return
            if recover_held:
                assert health(s) and s['pe2']==1 and s['g_it_state']==2 and s['g_lift_ruler_state']==4
                assert abs(s['g_dm_can2_logical_position_deg']-195)<=1
                assert abs(s['g_startup_extension_current_deg']-s['g_extension_home_reference_deg']+5537.025)<=45
                assert s['gripper_pwm_us']!=2056 and s['g_lift_compare_top_valid']
                report['plan']=['return_up','retract','slot1','read_arm','to_slot','release','yaw0']
                writes_started=True
                cmd('return_up',4,timeout=63,fresh_lift=True)
                cmd('retract',3,(('g_it_axes',2),('g_it_dm_deg',195.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                cmd('slot1',10,(('g_it_carrier_deg',0.0),))
                cmd('read_arm',7,timeout=6)
                cmd('to_slot',3,(('g_it_axes',1),('g_it_dm_deg',38.380798),('g_it_extension_mm',0.0)),fresh_sequence=True)
                cmd('release',6)
                cmd('yaw0',3,(('g_it_axes',1),('g_it_dm_deg',0.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                report['complete']=True
                return
            if loaded_up5:
                assert health(s) and s['pe2']==1 and s['g_it_state']==2 and s['g_lift_ruler_state']==4, s
                assert s['g_lift_compare_top_valid'] and s['g_lift_compare_backoff_valid'] and s['g_extension_hold_confirmed']
                assert abs(s['g_dm_can2_logical_position_deg'])<=1 and abs(s['g_startup_extension_current_deg']-s['g_extension_home_reference_deg'])<=1
                assert s['gripper_pwm_us']==2056 and not s['g_chassis_auto_active']
                assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text())['gate_cases']==191
                report['plan']=['read_arm','yaw_195','extend156','down118800','grip140','lift5mm_then_hold']
                writes_started=True
                cmd('read_arm',7,timeout=6)
                cmd('yaw_195',3,(('g_it_axes',1),('g_it_dm_deg',195.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
                cmd('extend156',3,(('g_it_axes',2),('g_it_dm_deg',195.0),('g_it_extension_mm',156.0)),fresh_sequence=True)
                down=cmd('down118800',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),fresh_lift=True)
                cmd('grip140',5)
                up=cmd('lift5mm_then_hold',3,(('g_it_axes',4),('g_it_dm_deg',195.0),('g_it_extension_mm',156.0),
                                             ('g_it_lift_pulses',110800.0)),fresh_sequence=True)
                delta_deg=down['g_vehicle_lift_current_deg']-up['g_vehicle_lift_current_deg']
                report['observed_lift_change_motor_deg']=delta_deg
                report['derived_lift_mm']=delta_deg/(1600*360/3200)
                report['operator_clearance_observation_pending']=True
                assert abs(delta_deg-900)<=90
                assert abs(up['g_dm_can2_logical_position_deg']-195)<=1
                assert abs(up['g_startup_extension_current_deg']-down['g_startup_extension_current_deg'])<=45
                report['complete']=True
                return
            assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_lift_ruler_state']==0 and s['g_startup_axes_state']==0, s
            writes_started=True
            home_lift=cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
            scale=100.0 if home_lift['g_lift_ruler_options']&128 else 10.0
            top_raw=home_lift['g_lift_compare_top_raw']
            if top_raw>=0x80000000:top_raw-=0x100000000
            target_lift_deg=top_raw/scale+118800.0*360/3200
            cmd('extension_home',2)
            cmd('read_arm',7,timeout=6)
            cmd('open',6)
            cmd('yaw_195',3,(('g_it_axes',1),('g_it_dm_deg',195.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
            down=cmd('empty_extension_and_descent_once',14,
                     (('g_it_extension_mm',156.0),('g_it_lift_pulses',118800.0)),fresh_sequence=True)
            assert down['g_source_parallel_done_mask']==6 and down['g_source_parallel_started_mask']==6
            ext=down['g_startup_extension_current_deg']-down['g_extension_home_reference_deg']
            assert abs(ext+5537.025)<=45.0 and abs(down['g_vehicle_lift_current_deg']-target_lift_deg)<=45.0
            cmd('return_up',4,timeout=63,fresh_lift=True)
            cmd('retract',3,(('g_it_axes',2),('g_it_dm_deg',195.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
            cmd('read_arm_again',7,timeout=6)
            cmd('yaw_0',3,(('g_it_axes',1),('g_it_dm_deg',0.0),('g_it_extension_mm',0.0)),fresh_sequence=True)
            report['complete']=True
        except BaseException as exc:
            report['error']=repr(exc)
            report['first_fault']=snap()
            save()
            if writes_started:
                wr('g_it_request',9)  # actual board STOP request, including Ctrl-C
                report['stop_request_sent']=True
                for _ in range(8):
                    time.sleep(.1)
                    s=snap()
                    if s['g_it_request']==0 and not s['g_source_parallel_stop_mask'] and not s['g_lift_ruler_stop_pending'] and not s['g_vehicle_arm_stop_pending']:
                        break
                report['after_stop']=s
            raise
        finally:
            report['last']=snap();save()
            print(json.dumps({k:report[k] for k in ('complete','flash_bytes','sha256')},indent=2),flush=True)
            print(dest,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--run-empty-once',action='store_true')
    modes.add_argument('--read-only',action='store_true')
    modes.add_argument('--check',action='store_true')
    modes.add_argument('--left-grab-up5-once',action='store_true')
    modes.add_argument('--place-current-held-once',action='store_true')
    modes.add_argument('--left-carry-overlap-once',action='store_true')
    modes.add_argument('--left-both-overlap-once',action='store_true')
    modes.add_argument('--three-direct-once',action='store_true')
    modes.add_argument('--three-immediate-once',action='store_true')
    args=parser.parse_args()
    if args.run_empty_once or args.read_only or args.left_grab_up5_once or args.place_current_held_once or args.left_carry_overlap_once or args.left_both_overlap_once or args.three_direct_once or args.three_immediate_once:
        run(readonly=args.read_only,loaded_up5=args.left_grab_up5_once,recover_held=args.place_current_held_once,
            loaded_overlap=args.left_carry_overlap_once,both_overlap=args.left_both_overlap_once,three_direct=args.three_direct_once,immediate=args.three_immediate_once)
    else:print(' -> '.join(PLAN)+'; no device access')
