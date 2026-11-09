"""Check the next-stroke adapter and actual CAN exchange on simulated transport."""
from pathlib import Path
import shutil
import check_lift_ruler_dispatch_20261105 as check
from prepare_lift_next_stroke_20261106 import adapt_unit

ROOT = Path(__file__).resolve().parents[1]
check.STAGE = ROOT / 'tmp/lift_next_stroke_20261106/project'
check.OUT = ROOT / 'evidence/integration/2026-11-06/lift-next-stroke'
check.UNIT = adapt_unit(check.UNIT)
check.MAIN = check.MAIN.replace('j<180', 'j<320')
old = 'j=moves;g_vehicle_key=1;mission_key();dispatch();CK(moves==j);'
assert old in check.MAIN
# VM and guard are already ready after DONE; the same adapter service call
# advances POWER to PREP. No requirement to insert another power-off wait.
check.MAIN = check.MAIN.replace(old, old + 'if(!i)CK(g_lift_ruler_state==LRC_PREP && g_vm_test_request==1);')
cap = ''' reset_case("key_cap");g_lift_ruler_controller.state=LRC_DONE;
 g_lift_ruler_controller.samples=7;g_lift_ruler_controller.vm_request=1;
 LiftRuler_DebugKey();CK(g_lift_ruler_rejected==3 && g_lift_ruler_request==0);
 LiftRuler_DebugService();
#if APP_MOTOR_OFFLINE_ONLY
 CK(!g_vm_test_request && !moves);
#else
 CK(g_vm_test_request==1 && !moves && g_lift_ruler_state==LRC_DONE);
#endif
'''
check.MAIN = check.MAIN.replace(' printf("DISPATCH offline=', cap + ' printf("DISPATCH offline=').replace('scenarios=6', 'scenarios=7')

if __name__ == '__main__':
    previous = check.OUT / 'dispatch-host-0.log'
    archive = check.OUT / 'dispatch-host-0-fixture-phase-error.log'
    if previous.exists() and 'FAIL' in previous.read_text(encoding='utf-8') and not archive.exists():
        shutil.copy2(previous, archive)
    check.main()
