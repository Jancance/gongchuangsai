"""Prepare a references-and-config capture entry; never connect to hardware."""
from pathlib import Path

R = Path(__file__).resolve().parents[1]


def main():
    source = (R / 'scripts/pickup_half_descent_live_20261110.py').read_text(encoding='utf-8')
    source = source.replace('2026-11-10/half-descent', '2026-11-11/driver-fifo-fix')
    source = source.replace('tmp/pickup_half_descent_20261110/project', 'tmp/pickup_driver_fifo_fix_20261111/project')
    source = source.replace('driver_config_only=False):', 'driver_config_only=False, reference_config_only=False):', 1)
    source = source.replace('assert readonly or driver_config_only or immediate', 'assert readonly or driver_config_only or immediate or reference_config_only', 1)
    source = source.replace("dest = E/(('integration13-three-immediate-once-' if immediate else", "dest = E/(('driver-config-after-references-' if reference_config_only else\n               'integration13-three-immediate-once-' if immediate else", 1)
    branch = '''            if reference_config_only:
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
'''
    marker = '            if driver_config_only:'
    assert source.count(marker) == 1
    source = source.replace(marker, branch + marker, 1)
    source = source[:source.index('\nif __name__')]
    source += '''
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
'''
    compile(source, 'fifo-diagnostic-host', 'exec')
    (R / 'scripts/pickup_fifo_diagnostic_live_20261111.py').write_text(source, encoding='utf-8')
    checks = (R / 'scripts/check_half_descent_host_stop_20261110.py').read_text(encoding='utf-8')
    checks = checks.replace('pickup_half_descent_live_20261110.py', 'pickup_fifo_diagnostic_live_20261111.py')
    checks = checks.replace('2026-11-10/half-descent', '2026-11-11/driver-fifo-fix')
    (R / 'scripts/check_fifo_diagnostic_host_stop_20261111.py').write_text(checks, encoding='utf-8')
    print('Prepared reference/config-only CLI; no hardware access.')


if __name__ == '__main__':
    main()
