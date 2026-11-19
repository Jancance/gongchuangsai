"""Prepare a single guarded low-speed extension-home recovery; no devices."""
from pathlib import Path
import ast
import copy
import json

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-11/driver-fault-read'


def main():
    source = (R / 'scripts/pickup_driver_fault_read_live_20261111.py').read_text(encoding='utf-8')
    source = source.replace('reference_config_only=False):', 'reference_config_only=False, home_extension_only=False):', 1)
    source = source.replace('assert readonly or driver_config_only or immediate or reference_config_only', 'assert readonly or home_extension_only', 1)
    source = source.replace("dest = E/(('driver-config-after-references-' if reference_config_only else", "dest = E/(('extension-home-once-' if home_extension_only else\n               'driver-config-after-references-' if reference_config_only else", 1)
    branch = '''            if home_extension_only:
                assert health(s) and s['pe2']==1 and s['g_it_state']==2 and s['g_it_auto_state']==0
                assert s['g_lift_ruler_state']==4 and s['g_lift_compare_top_valid']==1 and s['g_lift_compare_backoff_valid']==1
                assert s['g_startup_axes_state']==0 and s['g_extension_home_reference_valid']==0
                assert s['gripper_pwm_us']==2056
                safe=s['g_lift_compare_safe_raw']
                if safe>=0x80000000:safe-=0x100000000
                scale=100.0 if s['g_lift_ruler_options']&128 else 10.0
                assert abs(s['g_vehicle_lift_current_deg']-safe/scale)<=1.0
                report['plan']=['read_only_node6_preflight','one_existing_30RPM_extension_home','read_only_node6_result']
                report['three_piece_cycle_started']=False
                report['route3_enabled']=False
                before=request_ext_diag('node6_before_extension_home',strict=True)
                assert before['raw_words'][6*8+1]&0x0c==0, 'Driver still reports stall/protection; no home request'
                assert before['raw_words'][3*8+2]==0 and before['raw_words'][3*8+3]==0, 'Extension not stationary; no home request'
                writes_started=True
                home=cmd('extension_home',2)
                assert home['g_startup_axes_state']==4 and home['g_extension_home_reference_valid']==1 and home['g_extension_hold_confirmed']==1
                request_ext_diag('node6_after_extension_home',strict=True)
                report['extension_home_complete']=True
                report['physical_retraction_confirmation_pending']=True
                report['complete']=True
                return
'''
    marker = '            if reference_config_only:'
    assert source.count(marker) == 1
    source = source.replace(marker, branch + marker, 1)
    source = source[:source.index('\nif __name__')]
    source += '''
if __name__ == '__main__':
    parser=argparse.ArgumentParser(description='One low-speed extension-home recovery using the unchanged30RPM homing backend.')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--check',action='store_true')
    modes.add_argument('--read-only',action='store_true')
    modes.add_argument('--extension-home-once',action='store_true')
    args=parser.parse_args()
    TRIAL_INDEX=2;S_CURVE=0
    if args.read_only or args.extension_home_once:
        run(readonly=args.read_only,home_extension_only=args.extension_home_once)
    else:print('CHECK only; no hardware access. Requires fresh zero-speed/no-protection reply, empty gripper and real lift safe reference; one30RPM extension home. No pickup cycle.')
'''
    compile(source, 'extension-recovery-host', 'exec')
    (R / 'scripts/pickup_extension_recover_live_20261111.py').write_text(source, encoding='utf-8')
    tree = ast.parse(source)
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    node = next(n for n in ast.walk(fn) if isinstance(n, ast.If) and isinstance(n.test, ast.Name) and n.test.id == 'home_extension_only')
    unit = ast.FunctionDef(name='branch', args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]), body=copy.deepcopy(node.body), decorator_list=[])
    code = compile(ast.fix_missing_locations(ast.Module(body=[unit], type_ignores=[])), 'actual-recovery-branch', 'exec')
    checks = 0
    for state_change, flags, speed, allowed in [({},3,0,True), ({},15,0,False), ({},5,0,False), ({},3,1,False), ({'pe2':0},3,0,False), ({'g_startup_axes_state':5},3,0,False), ({'gripper_pwm_us':1685},3,0,False), ({'g_vehicle_lift_current_deg':1000},3,0,False), ({'g_extension_home_reference_valid':1},3,0,False)]:
        state=dict(pe2=1,g_it_state=2,g_it_auto_state=0,g_lift_ruler_state=4,g_lift_compare_top_valid=1,g_lift_compare_backoff_valid=1,g_startup_axes_state=0,g_extension_home_reference_valid=0,gripper_pwm_us=2056,g_lift_compare_safe_raw=27000,g_lift_ruler_options=132,g_vehicle_lift_current_deg=270.0)
        state.update(state_change)
        raw=[0]*56;raw[49]=flags;raw[27]=speed
        calls=[];report={}
        ns=dict(s=state,health=lambda _:True,report=report,
                request_ext_diag=lambda *a,**kw:dict(raw_words=raw),
                cmd=lambda *a,**kw:(calls.append(a) or dict(g_startup_axes_state=4,g_extension_home_reference_valid=1,g_extension_hold_confirmed=1)))
        exec(code,ns)
        rejected=False
        try:ns['branch']()
        except AssertionError:rejected=True
        assert rejected != allowed and len(calls)==int(allowed)
        if allowed:assert calls==[('extension_home',2)] and report['complete'] and not report['three_piece_cycle_started']
        checks+=1
    (E / 'extension-recovery-entry-checks.json').write_text(json.dumps(dict(passed=True,checks=checks,actual_branch_exercised=True,no_fault_or_reference_writes=True,hardware_access=False),indent=2)+'\n',encoding='utf-8')
    print('PASS', checks, 'actual low-speed recovery branch checks; no device access.')


if __name__ == '__main__':
    main()
