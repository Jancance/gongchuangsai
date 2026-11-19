"""One supervised three-piece run with read-only node6 diagnostics. Default never connects."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import time
import gripper210_attach_read_20261105 as live

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-11/driver-fifo-fix'
PLAN = ['top_reference', 'extension_home', 'read_arm', 'open', 'yaw_195',
        'empty_extension_and_descent_once', 'return_up', 'retract', 'read_arm_again', 'yaw_0']


def repeat_ready(s):
    zeros=('g_it_request','g_it_rejected','g_it_cause','g_it_auto_fault','g_safety_fault',
           'g_rtos_fault','g_can_ctrl_fault_latched','g_startup_axes_cause','g_lift_ruler_cause',
           'g_source_parallel_cause','g_source_parallel_stop_mask','g_vehicle_arm_cause',
           'g_vehicle_arm_stop_pending','g_lift_ruler_stop_pending','g_dm_jog_first_fault',
           'g_dm_jog_request','g_chassis_auto_active')
    if any(s[k] for k in zeros): return False
    ones=('g_safety_ready','g_can_test_result','g_extension_home_reference_valid',
          'g_extension_hold_confirmed','g_lift_compare_top_valid','g_lift_compare_backoff_valid','pe2')
    if not all(s[k]==1 for k in ones): return False
    if not (s['g_it_state']==2 and s['g_it_auto_state']==2 and s['g_it_auto_phase']==5 and
            s['g_it_auto_item']==2 and s['g_lift_ruler_state']==4 and s['g_startup_axes_state']==4 and
            s['g_lift_compare_operation']==3 and s['gripper_pwm_us']==2056): return False
    safe=s['g_lift_compare_safe_raw']
    if safe>=0x80000000: safe-=0x100000000
    scale=100 if s['g_lift_ruler_options']&128 else 10
    return (abs(s['g_dm_can2_logical_position_deg'])<=1 and
            abs(s['g_startup_extension_current_deg']-s['g_extension_home_reference_deg'])<=2 and
            abs(s['g_vehicle_lift_current_deg']-safe/scale)<=1)


def run(readonly=False, loaded_up5=False, recover_held=False, loaded_overlap=False, both_overlap=False, three_direct=False, immediate=False, repeat=False, driver_config_only=False, reference_config_only=False):
    assert not any((loaded_up5, recover_held, loaded_overlap, both_overlap, three_direct))
    assert readonly or driver_config_only or immediate or reference_config_only
    assert TRIAL_INDEX == 2 and S_CURVE == 0
    m = json.loads((E/'candidate.json').read_text(encoding='utf-8'))
    image = m['images']['Vehicle_LiftRuler']
    live.IMAGE = R / image['axf']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest() == image['sha256']
    for name, digest in m['changed'].items():
        p = R / 'tmp/pickup_driver_fifo_fix_20261111/project' / name
        assert hashlib.sha256(p.read_bytes()).hexdigest() == digest
    symbols, segments = live.load_image()
    # Read-only diagnostics use the exact verified image's local axis symbol.
    from elftools.elf.elffile import ELFFile
    with live.IMAGE.open('rb') as f:
        elf = ELFFile(f)
        axes_symbols = [(int(v['st_value']), int(v['st_size']))
                        for v in elf.get_section_by_name('.symtab').iter_symbols()
                        if v.name == 'vsp_axis']
    assert len(axes_symbols) == 1 and axes_symbols[0][1] == 144
    axes_address = axes_symbols[0][0]
    dest = E/(('driver-config-after-references-' if reference_config_only else
               'integration13-three-immediate-once-' if immediate else
               'integration13-three-direct-once-' if three_direct else
               'integration13-left-both-overlap-once-' if both_overlap else
               'integration13-left-carry-overlap-once-' if loaded_overlap else
               'integration13-left-held-place-once-' if recover_held else
               'integration13-left-up5-once-' if loaded_up5 else
               'integration13-empty-'+('readonly-' if readonly else 'once-'))+
              datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    assert not dest.exists()
    report = {'sha256':image['sha256'], 'plan':PLAN, 'complete':False,
              'readonly':readonly, 'reset':False, 'flashed':False, 'steps':[],
              'requested_trial':TRIAL_INDEX, 's_curve_requested':bool(S_CURVE),
              'repeat_from_completed_state':repeat}
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
g_lift_compare_top_valid g_lift_compare_backoff_valid g_lift_compare_safe_raw g_lift_ruler_stop_pending
g_source_parallel_state g_source_parallel_cause g_source_parallel_stop_mask
g_source_parallel_started_mask g_source_parallel_done_mask g_source_parallel_start_skew_ms
g_vehicle_arm_state g_vehicle_arm_cause g_vehicle_arm_stop_pending
g_chassis_auto_active g_dm_jog_first_fault g_dm_jog_request g_dm_posvel_holding'''.split()
            keys += [k for k in ('g_it_auto_state','g_it_auto_phase','g_it_auto_item','g_it_auto_fault',
                                'g_it_auto_elapsed_ms','g_it_auto_phase_ms','g_it_lead5_grip_done',
                                'g_vsp_prepare_miss','g_vsp_prepare_nonzero','g_vsp_prepare_speed_raw',
                                'g_vsp_prepare_wait_address','g_pick_trial_profile','g_pick_trial_scurve','g_pick_scurve_frames','g_pick_scurve_last_gap_ms','g_pick_scurve_max_gap_ms','g_pick_extension_segment','g_it_lead5_grip_tick','g_dm_jog_state','g_dm_can2_feedback_tick','g_ext_diag_state','g_ext_diag_cycle','g_ext_move_target_sequence','g_ext_move_target_result','g_ext_move_target_tick','g_ext_move_ack_sequence','g_ext_move_ack_result','g_ext_move_ack_code','g_ext_move_ack_tick','g_ext_trace_count','g_ext_trace_speed_miss_count','g_it_half_extension_released','g_it_half_extension_release_tick') if k in symbols]
            s={k:rd(k) for k in keys}
            if 'g_it_auto_release_ms' in symbols:
                s['release_command_times_ms']=list(t.read_memory_block32(symbols['g_it_auto_release_ms'],3))
            for k in ('g_dm_can2_logical_position_deg','g_startup_extension_current_deg',
                      'g_extension_home_reference_deg','g_vehicle_lift_current_deg'):
                s[k]=rf(k)
            for k in ('g_pick_extension_waypoint_deg','g_pick_extension_final_goal_deg','g_it_lead5_grip_depth','g_it_lead5_grip_x_error','g_dm_can2_velocity_rad_s','g_it_half_lift_before','g_it_half_lift_goal','g_it_half_lift_at_release'):
                if k in symbols:s[k]=rf(k)
            s['next_yaw_request_times_ms']=list(t.read_memory_block32(symbols['g_it_auto_next_request_ms'],3))
            s['pe2']=(t.read32(0x40021010)>>2)&1
            s['gripper_pwm_us']=t.read32(0x40000034)
            s['carrier_pwm_us']=t.read32(0x40000440)
            block = bytes(t.read_memory_block8(axes_address, 144))
            axis_keys = ('phase','options','samples','flags','cycle','stall_active',
                         'stall_ms','position_tick','position_cycle','overshoot_active',
                         'overshoot_ms','before','goal','current')
            s['vsp_axes'] = []
            for axis_index in range(2):
                axis = dict(zip(axis_keys, struct.unpack_from('<11I3f', block, axis_index*72)))
                axis['node'] = 6 if axis_index == 0 else 5
                axis['move_hex'] = block[axis_index*72+56:axis_index*72+71].hex()
                axis['position_minus_goal_deg'] = axis['current'] - axis['goal']
                s['vsp_axes'].append(axis)
            s['ext_motion_target_raw']=list(t.read_memory_block32(symbols['g_ext_move_target_raw'],8))
            s['ext_motion_frame_raw']=list(t.read_memory_block32(symbols['g_ext_move_frame'],15))
            return s
        def health(s):
            return (s['g_safety_ready']==1 and s['g_can_test_result']==1 and
                    not any(s[k] for k in ('g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
                        'g_it_cause','g_startup_axes_cause','g_lift_ruler_cause','g_source_parallel_cause',
                        'g_vehicle_arm_cause','g_dm_jog_first_fault','g_chassis_auto_active')))
        def ext_diag_snapshot():
            return {'state':rd('g_ext_diag_state'),'cycle':rd('g_ext_diag_cycle'),
                    'index':rd('g_ext_diag_index'),'rejected':rd('g_ext_diag_rejected'),'firmware_context':rd('g_ext_diag_context'),
                    'commands':list(t.read_memory_block32(symbols['g_ext_diag_cmd'],7)),
                    'results':list(t.read_memory_block32(symbols['g_ext_diag_result'],7)),
                    'ticks':list(t.read_memory_block32(symbols['g_ext_diag_tick'],7)),
                    'raw_words':list(t.read_memory_block32(symbols['g_ext_diag_raw'],56))}
        def request_ext_diag(label,strict=False):
            before=rd('g_ext_diag_cycle');wr('g_ext_diag_request',1)
            begin=time.monotonic()
            while time.monotonic()-begin<4.0:
                if rd('g_ext_diag_cycle')!=before and rd('g_ext_diag_state') in (2,3) and not rd('g_ext_diag_request'):
                    data=ext_diag_snapshot();data['context']=label;report[label]=data;save()
                    if strict:
                        assert data['state']==2 and data['commands']==[0x1a,0x33,0x34,0x35,0x36,0x37,0x3a],data
                        assert all(data['results'][i]==1 for i in (0,1,3,4,6)),data
                        assert data['firmware_context']==0,data
                    return data
                time.sleep(.025)
            data=ext_diag_snapshot();data['context']=label;data['host_wait_timed_out']=True;report[label]=data;save()
            if strict:raise TimeoutError(label)
            return data
        def request_ext_config(label):
            before=rd('g_ext_diag_cycle');wr('g_ext_diag_request',2)
            begin=time.monotonic()
            while time.monotonic()-begin<5.0:
                if rd('g_ext_diag_cycle')!=before and rd('g_ext_diag_state') in (2,3) and not rd('g_ext_diag_request'):
                    break
                time.sleep(.025)
            data=ext_diag_snapshot()
            data.update(kind=rd('g_ext_diag_kind'), decode_mask=rd('g_ext_cfg_decode_mask'),
                        lengths=list(t.read_memory_block32(symbols['g_ext_cfg_length'],7)),
                        raw_words=list(t.read_memory_block32(symbols['g_ext_cfg_raw'],7*64)),
                        pid=list(t.read_memory_block32(symbols['g_ext_cfg_pid'],4)))
            for name in ('firmware_version','hardware_word','bus_mv','bus_ma','phase_ma',
                         'ma_limit','vm_limit','prwindow_raw','scale','control_mode'):
                data[name]=rd('g_ext_cfg_'+name)
            data['host_wait_timed_out']=not (rd('g_ext_diag_cycle')!=before and data['state'] in (2,3))
            report[label]=data;save()
            return data

        def capture_ext_ring():
            before=rd('g_ext_trace_count')
            words=list(t.read_memory_block32(symbols['g_ext_trace_rows'],128*12))
            after=rd('g_ext_trace_count')
            return {'count_before':before,'count_after':after,'stable_read':before==after,
                    'row_fields':['tick','sequence','position_tick','position_sign','position_raw','speed_sign','speed_raw','flags','goal_float_bits','move_position_field','ack_result','ack_code'],
                    'raw_rows':[words[i*12:(i+1)*12] for i in range(128)],
                    'speed_miss_count':rd('g_ext_trace_speed_miss_count')}
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
                    report['first_fault']=s
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
            if reference_config_only:
                assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_it_auto_state']==0
                assert s['g_lift_ruler_state']==0 and s['g_startup_axes_state']==0
                report['plan']=['top_reference','extension_home','open_empty_gripper','read_only_node6_configuration']
                report['three_piece_cycle_started']=False
                report['route3_enabled']=False
                writes_started=True
                cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
                cmd('extension_home',2)
                cmd('open_empty_gripper',6)
                request_ext_diag('node6_basic_after_references',strict=True)
                data=request_ext_config('node6_configuration_after_references')
                report['configuration_capture_complete']=(data['state']==2 and data['kind']==2 and
                    data['commands']==[0x1f,0x21,0x24,0x26,0x27,0x42,0x43] and
                    all(x==1 for x in data['results']) and data['decode_mask']==127 and not data['host_wait_timed_out'])
                report['complete']=report['configuration_capture_complete']
                return
            if driver_config_only:
                assert s['pe2']==1 and s['g_it_state'] in (0,2,9) and s['g_it_auto_state']!=1
                assert not any(s[k] for k in ('g_it_request','g_source_parallel_stop_mask',
                    'g_lift_ruler_stop_pending','g_vehicle_arm_stop_pending','g_chassis_auto_active','g_dm_jog_request'))
                report['driver_configuration_only']=True
                report['no_actuator_command_sent']=True
                request_ext_config('node6_driver_configuration_only')
                return
            if readonly:
                report['complete']=True
                return
            if three_direct or immediate:
                if repeat:
                    assert immediate and repeat_ready(s), s
                else:
                    assert health(s) and s['pe2']==0 and s['g_it_state']==0 and s['g_lift_ruler_state']==0 and s['g_startup_axes_state']==0
                assert json.loads((E/'half-descent-checks.json').read_text(encoding='utf-8'))['passed']
                report['plan']=['references','down->slot1','left->slot2','right->slot3','yaw0']
                report['route3_enabled']=False
                writes_started=True
                if not repeat:
                    cmd('top_reference',1,(('g_it_carrier_auto_deg',-1.0),('g_lift_compare_target_pulses',118800)),timeout=63,fresh_lift=True)
                    cmd('extension_home',2)
                cmd('read_arm',7,timeout=6)
                if not repeat:cmd('open',6)
                request_ext_diag('node6_before_trial_idle',strict=True)
                request_ext_config('node6_configuration_before_trial_idle')
                sequence_start=time.monotonic();report['item_elapsed_s']={}
                if immediate:
                    cmd('select_trial',17,(('g_pick_trial_requested',TRIAL_INDEX),('g_pick_trial_scurve_requested',S_CURVE)))
                    assert rd('g_it_auto_state')==(2 if repeat else 0) and not rd('g_it_request')
                    report['plan']=['references','MCU down->left->right immediate chain','yaw0']
                    report['mcu_trace']=[];start_sequence=rd('g_it_sequence');seen_running=False
                    wr('g_it_request',16)
                    prev=None
                    while time.monotonic()-sequence_start<65:
                        now=snap();report['mcu_trace'].append(now)
                        if not health(now) or now['g_it_rejected'] or now['g_it_state']==9 or now['g_it_auto_state']==9:
                            report['first_fault']=now  # preserve first observed sample before STOP
                            raise RuntimeError('immediate chain first fault: '+json.dumps(now))
                        phase=(now['g_it_auto_item'],now['g_it_auto_phase'])
                        if phase!=prev:
                            print('MCU item/phase',phase,'elapsed_ms',now['g_it_auto_elapsed_ms'],flush=True);prev=phase;save()
                        if now['g_it_auto_state']==1:seen_running=True
                        if seen_running and now['g_it_auto_state']==2 and now['g_it_sequence']!=start_sequence:
                            times=now['release_command_times_ms']
                            assert 0<times[0]<times[1]<times[2]
                            report['release_to_next_request_ms']=[a-b for a,b in zip(now['next_yaw_request_times_ms'],times)]
                            assert all(x>=50 for x in report['release_to_next_request_ms'])
                            report['three_to_release_command_s']=times[2]/1000
                            report['three_to_zero_s']=now['g_it_auto_elapsed_ms']/1000
                            report['item_to_release_command_s']=[times[0]/1000,(times[1]-times[0])/1000,(times[2]-times[1])/1000]
                            report['extension_motion_ring']=capture_ext_ring()
                            request_ext_diag('node6_after_complete_idle')
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
                    assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text(encoding='utf-8'))['gate_cases']==199
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
                assert json.loads((E/'stage2-fast-gates-matlab-20261109.json').read_text(encoding='utf-8'))['gate_cases']==191
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
            # STOP must not depend on a further successful read or log write.
            if writes_started:
                try:
                    wr('g_it_request',9)
                    report['stop_request_sent']=True
                except BaseException as stop_exc:
                    report['stop_request_error']=repr(stop_exc)
            if 'first_fault' not in report:
                try:report['first_fault']=snap()
                except BaseException as read_exc:report['fault_snapshot_error']=repr(read_exc)
            if report.get('stop_request_sent'):
                for _ in range(8):
                    time.sleep(.1)
                    try:s=snap()
                    except BaseException as read_exc:
                        report['stop_status_read_error']=repr(read_exc)
                        break
                    report['after_stop']=s
                    if s['g_it_request']==0 and not s['g_source_parallel_stop_mask'] and not s['g_lift_ruler_stop_pending'] and not s['g_vehicle_arm_stop_pending']:
                        break
                try:
                    report['extension_motion_ring']=capture_ext_ring()
                    stopped=report.get('after_stop',{})
                    if stopped and not stopped['g_source_parallel_stop_mask'] and not stopped['g_lift_ruler_stop_pending'] and not stopped['g_vehicle_arm_stop_pending']:
                        request_ext_diag('node6_after_stop_only')
                except BaseException as diag_exc:report['post_stop_diagnostic_error']=repr(diag_exc)
            try:save()
            except BaseException as log_exc:report['save_error']=repr(log_exc)
            raise
        finally:
            try:report['last']=snap()
            except BaseException as read_exc:report['final_snapshot_error']=repr(read_exc)
            try:save()
            except BaseException as log_exc:print('Log save failed: '+repr(log_exc),flush=True)
            print(json.dumps({k:report.get(k) for k in ('complete','flash_bytes','sha256')},indent=2),flush=True)
            print(dest,flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='Establish real top/extension references, open empty gripper, capture node6 driver configuration. No pickup cycle.')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--check',action='store_true')
    modes.add_argument('--read-only',action='store_true')
    modes.add_argument('--driver-config-only',action='store_true')
    modes.add_argument('--references-and-config-once',action='store_true')
    args=parser.parse_args()
    TRIAL_INDEX=2;S_CURVE=0
    if args.read_only or args.driver_config_only or args.references_and_config_once:
        run(readonly=args.read_only,driver_config_only=args.driver_config_only,
            reference_config_only=args.references_and_config_once)
    else:
        print('CHECK only; no hardware access. One reference sequence and read-only driver configuration. No pickup/chassis cycle.')
