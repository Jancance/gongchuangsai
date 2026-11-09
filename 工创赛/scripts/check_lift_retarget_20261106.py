"""Verify corrected target and explicit runtime retuning on actual C paths."""
from pathlib import Path
import json
import check_lift_speed_compare_20261106 as core
import check_lift_speed_compare_dispatch_20261106 as dispatch

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_retarget_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-retarget'


def main():
    assert json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))['passed']
    fixture = core.FIXTURE
    marker = 'LiftRuler_Init(&c);'
    fixture = fixture.replace(marker, marker + '''
 CK(c.down_target_pulses==108400U);
 CK(!LiftRuler_SetDownTarget(&c,99999U));CK(!LiftRuler_SetDownTarget(&c,110801U));
 CK(LiftRuler_SetDownTarget(&c,108400U));''')
    fixture = fixture.replace('for(k=0;k<3;k++)', 'for(k=0;k<5;k++)')
    fixture = fixture.replace('c.operation==k+1', 'c.operation==(k==0?1U:(k%2?2U:3U))')
    fixture = fixture.replace('homes==(k==0?1:k==1?1:2)', 'homes==1+k/2')
    fixture = fixture.replace('if(k!=1)CK(', 'if(k%2==0)CK(')
    fixture = fixture.replace('CK(!LiftRuler_Start(&c,now,1));', '''CK(!LiftRuler_Start(&c,now,1));
  {uint32_t locked=c.down_target_pulses;CK(!LiftRuler_SetDownTarget(&c,100000U));CK(c.down_target_pulses==locked);}''')
    marker = 'origin=motor;goal=motor+(q[1]?-1:1)*(double)u/scale;'
    assert marker in fixture
    fixture = fixture.replace(marker, marker + '''
  if(c->operation==1)CK(fabs(goal-((double)c->top_position_raw/scale+c->down_target_pulses*360.0/3200))<.001);''')
    marker = 'now+=500;continue;'
    fixture = fixture.replace(marker, '''if(k==2){CK(LiftRuler_SetDownTarget(&c,108000U));CK(c.down_target_pulses==108000U);}
   ''' + marker)
    marker = 'CK(!LiftRuler_Start(&c,now+20,1) && c.vm_request==1);'
    assert marker in fixture
    fixture = fixture.replace(marker, 'CK(LiftRuler_Start(&c,now+20,1) && c.vm_request==1 && c.operation==1);')
    core.FIXTURE, core.STAGE, core.OUT = fixture, STAGE, OUT
    core.main()

    dispatch.FIXTURE, dispatch.STAGE, dispatch.OUT = fixture, STAGE, OUT
    main_c = dispatch.MAIN
    main_c = main_c.replace('g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;', 'g_lift_ruler_request=g_lift_ruler_rejected=g_vm_test_request=0;g_lift_compare_target_pulses=108400U;')
    main_c = main_c.replace('for(k=0;k<3;k++)', 'for(k=0;k<5;k++)')
    main_c = main_c.replace('g_lift_compare_operation==k+1', 'g_lift_compare_operation==(k==0?1U:(k%2?2U:3U))')
    main_c = main_c.replace('homes==(k==0?1:k==1?1:2)', 'homes==1+k/2')
    main_c = main_c.replace('if(k!=1)CK(', 'if(k%2==0)CK(')
    old = '''LiftRuler_DebugKey();CK(g_lift_ruler_rejected==3 && !g_lift_ruler_request);
  dispatch();CK(g_vm_test_request==1 && g_lift_compare_operation==3);'''
    new = '''g_lift_compare_target_pulses=110801U;
  LiftRuler_DebugKey();dispatch();CK(g_lift_ruler_rejected==4 && g_lift_compare_operation==3 && g_vm_test_request==1);
  g_lift_compare_target_pulses=108000U;LiftRuler_DebugKey();dispatch();
  CK(g_lift_ruler_controller.down_target_pulses==108000U && g_vm_test_request==1 && g_lift_compare_operation==1);'''
    assert old in main_c
    main_c = main_c.replace(old, new)
    dispatch.MAIN = main_c
    dispatch.main()


if __name__ == '__main__':
    main()
