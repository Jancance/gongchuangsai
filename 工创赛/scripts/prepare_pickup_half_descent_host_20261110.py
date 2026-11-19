"""Prepare restricted second-profile host tools; never access devices."""
from pathlib import Path

R = Path(__file__).resolve().parents[1]
S = R / 'scripts'
E = R / 'evidence/integration/2026-11-10/half-descent'
E.mkdir(parents=True, exist_ok=True)


def main():
    source = (S / 'pickup_second_diag_live_20261110.py').read_text(encoding='utf-8')
    source = source.replace('2026-11-10/second-diag', '2026-11-10/half-descent')
    source = source.replace('tmp/pickup_second_diag_20261110/project', 'tmp/pickup_half_descent_20261110/project')
    source = source.replace('integration13-three-direct-matlab-20261109.json', 'half-descent-checks.json')
    source = source.replace('immediate=False, repeat=False):', 'immediate=False, repeat=False, driver_config_only=False):')
    source = source.replace("    m = json.loads((E/'candidate.json')", "    assert not any((loaded_up5, recover_held, loaded_overlap, both_overlap, three_direct))\n    assert readonly or driver_config_only or immediate\n    assert TRIAL_INDEX == 2 and S_CURVE == 0\n    m = json.loads((E/'candidate.json')", 1)
    source = source.replace("'g_ext_trace_speed_miss_count') if k in symbols]", "'g_ext_trace_speed_miss_count','g_it_half_extension_released','g_it_half_extension_release_tick') if k in symbols]", 1)
    source = source.replace("'g_dm_can2_velocity_rad_s'):", "'g_dm_can2_velocity_rad_s','g_it_half_lift_before','g_it_half_lift_goal','g_it_half_lift_at_release'):", 1)
    insert = '''        def request_ext_config(label):
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
'''
    source = source.replace('        def capture_ext_ring():', insert + '\n        def capture_ext_ring():', 1)
    branch = '''            if driver_config_only:
                assert s['pe2']==1 and s['g_it_state'] in (0,2,9) and s['g_it_auto_state']!=1
                assert not any(s[k] for k in ('g_it_request','g_source_parallel_stop_mask',
                    'g_lift_ruler_stop_pending','g_vehicle_arm_stop_pending','g_chassis_auto_active','g_dm_jog_request'))
                report['driver_configuration_only']=True
                report['no_actuator_command_sent']=True
                request_ext_config('node6_driver_configuration_only')
                return
'''
    source = source.replace('            if readonly:', branch + '            if readonly:', 1)
    needle = "request_ext_diag('node6_before_trial_idle',strict=True)"
    assert source.count(needle) == 1
    source = source.replace(needle, needle + "\n                request_ext_config('node6_configuration_before_trial_idle')", 1)
    source = source[:source.index("\nif __name__")]
    source += '''if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='Half-descent profile2: lift2500, extension200/600/600. One three-piece cycle only.')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--check',action='store_true')
    modes.add_argument('--read-only',action='store_true')
    modes.add_argument('--driver-config-only',action='store_true')
    modes.add_argument('--three-immediate-once',action='store_true')
    modes.add_argument('--repeat-ready-once',action='store_true')
    parser.add_argument('--trial',type=int,choices=(2,),default=2)
    args=parser.parse_args()
    TRIAL_INDEX=2;S_CURVE=0
    if args.read_only or args.driver_config_only or args.three_immediate_once or args.repeat_ready_once:
        run(readonly=args.read_only,driver_config_only=args.driver_config_only,
            immediate=args.three_immediate_once or args.repeat_ready_once,repeat=args.repeat_ready_once)
    else:
        print('CHECK only; no hardware access. Trial2 lift2500/45055, extension200/600/600; half-descent gate; one continuous three-piece cycle. No third trial.')
'''
    compile(source, 'half-descent-runner', 'exec')
    (S / 'pickup_half_descent_live_20261110.py').write_text(source, encoding='utf-8')
    stop = (S / 'check_second_diag_host_stop_20261110.py').read_text(encoding='utf-8')
    stop = stop.replace('pickup_second_diag_live_20261110.py', 'pickup_half_descent_live_20261110.py').replace('2026-11-10/second-diag', '2026-11-10/half-descent')
    (S / 'check_half_descent_host_stop_20261110.py').write_text(stop, encoding='utf-8')
    owner = (S / 'check_pickup_single_speed_owner_20261110.py').read_text(encoding='utf-8')
    owner = owner.replace('tmp/pickup_single_speed_20261110/project', 'tmp/pickup_half_descent_20261110/project').replace('2026-11-10/single-speed', '2026-11-10/half-descent')
    (S / 'check_half_descent_owner_20261110.py').write_text(owner, encoding='utf-8')
    print('Prepared restricted profile2 runner, STOP fault-injection and owner/DM checks; no device access.')


if __name__ == '__main__':
    main()
