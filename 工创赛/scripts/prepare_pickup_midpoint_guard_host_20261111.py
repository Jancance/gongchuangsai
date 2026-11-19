"""Prepare one-cycle profile2 midpoint guarded host; no devices."""
from pathlib import Path

R = Path(__file__).resolve().parents[1]


def main():
    source = (R/'scripts/pickup_half_descent_live_20261110.py').read_text(encoding='utf-8')
    source = source.replace('tmp/pickup_half_descent_20261110/project','tmp/pickup_midpoint_guard_20261111/project')
    source = source.replace('2026-11-10/half-descent','2026-11-11/midpoint-guard')
    source = source.replace('half-descent-checks.json','midpoint-guard-checks.json')
    source = source.replace("'g_it_half_extension_release_tick') if k in symbols]", "'g_it_half_extension_release_tick','g_pick_midpoint_stage','g_pick_midpoint_ready_tick','g_pick_midpoint_final_start_tick') if k in symbols]",1)
    source = source.replace("'g_it_half_lift_at_release'):", "'g_it_half_lift_at_release','g_pick_midpoint_before_deg','g_pick_midpoint_hold_deg','g_pick_midpoint_final_deg'):",1)
    source = source.replace('half-descent gate; one continuous three-piece cycle.','midpoint hold until extension done; one continuous three-piece cycle.')
    compile(source,'midpoint-host','exec')
    (R/'scripts/pickup_midpoint_guard_live_20261111.py').write_text(source,encoding='utf-8')
    for original,target in [('check_half_descent_host_stop_20261110.py','check_midpoint_guard_host_stop_20261111.py'),('check_half_descent_owner_20261110.py','check_midpoint_guard_owner_20261111.py')]:
        test=(R/'scripts'/original).read_text(encoding='utf-8')
        test=test.replace('pickup_half_descent_live_20261110.py','pickup_midpoint_guard_live_20261111.py')
        test=test.replace('tmp/pickup_half_descent_20261110/project','tmp/pickup_midpoint_guard_20261111/project')
        test=test.replace('2026-11-10/half-descent','2026-11-11/midpoint-guard')
        (R/'scripts'/target).write_text(test,encoding='utf-8')
    print('Prepared profile2 one-cycle runner and actual owner/STOP checks. No device access.')


if __name__=='__main__':
    main()
