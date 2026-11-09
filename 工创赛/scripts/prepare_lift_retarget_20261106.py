"""Prepare complete return/down test with bounded, latched runtime target."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/lift_speed_compare_20261106/project'
STAGE = ROOT / 'tmp/lift_retarget_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-retarget'


def main():
    model = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert model['passed'] and not model['failure_count']
    assert not STAGE.exists(), 'Preserve existing retarget candidate'
    shutil.copytree(BASE, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    changed = {}

    def edit(name, transform):
        p = STAGE / name
        raw = p.read_bytes()
        text = raw.decode('utf-8').replace('\r\n', '\n')
        p.write_bytes(transform(text).replace('\n', '\r\n').encode('utf-8'))
        changed[name] = {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(p.read_bytes()).hexdigest()}

    def controller(t):
        old = 'void LiftRuler_Init(LiftRulerController*c){if(c)memset(c,0,sizeof(*c));}'
        assert t.count(old) == 1
        t = t.replace(old, '''void LiftRuler_Init(LiftRulerController*c){if(c){memset(c,0,sizeof(*c));c->down_target_pulses=108400U;}}
uint32_t LiftRuler_SetDownTarget(LiftRulerController*c,uint32_t pulses)
{
 if(!c || c->cause || c->stop_pending || (c->state!=LRC_IDLE && c->state!=LRC_DONE) ||
    pulses<100000U || pulses>DOWN_TARGET_PULSES)return 0U;
 c->down_target_pulses=pulses;return 1U;
}''')
        t = t.replace('c->operation>=3U)return 0;', 'c->operation>3U)return 0;')
        marker = ' if(c->operation && !c->top_valid)return 0;'
        assert t.count(marker) == 1
        t = t.replace(marker, marker + '''
 if(!c->down_target_pulses)c->down_target_pulses=108400U;
 if(c->down_target_pulses<100000U || c->down_target_pulses>DOWN_TARGET_PULSES)return 0;
 /* A completed return can start the next explicitly requested down test.
  * No fault is cleared and no reference is synthesized. */
 if(c->operation==3U)c->operation=1U;''')
        old = 'units=c->operation==1U?DOWN_TARGET_PULSES:RETURN_TOP_MARGIN_PULSES;'
        assert t.count(old) == 1
        return t.replace(old, 'units=c->operation==1U?c->down_target_pulses:RETURN_TOP_MARGIN_PULSES;')

    edit('CHASSIS/USER/lift_ruler_calibration.c', controller)
    edit('CHASSIS/USER/lift_ruler_calibration.h', lambda t: t.replace(' uint8_t home_parameters[17];', ' uint32_t down_target_pulses;\n uint8_t home_parameters[17];').replace('void LiftRuler_Stop(', 'uint32_t LiftRuler_SetDownTarget(LiftRulerController*,uint32_t pulses);\nvoid LiftRuler_Stop('))

    def adapter(t):
        t = t.replace('volatile int32_t g_lift_compare_top_raw;', '''volatile int32_t g_lift_compare_top_raw;
volatile uint32_t g_lift_compare_target_pulses=108400U; /* configuration, not a motion request */''')
        old = ''' if(g_lift_ruler_controller.state==LRC_DONE && g_lift_ruler_controller.operation>=3U)
  g_lift_ruler_rejected=3U;
 else g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state==LRC_DONE) &&
  !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;'''
        new = ''' g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state==LRC_DONE) &&
  !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;'''
        assert t.count(old) == 1
        t = t.replace(old, new)
        old = ' if(request==LRC_REQUEST_START && !LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;'
        new = ''' if(request==LRC_REQUEST_START){
  if(!LiftRuler_SetDownTarget(&g_lift_ruler_controller,g_lift_compare_target_pulses))g_lift_ruler_rejected=4U;
  else if(!LiftRuler_Start(&g_lift_ruler_controller,now,ready))g_lift_ruler_rejected=1U;
 }'''
        assert t.count(old) == 1
        return t.replace(old, new)

    edit('CHASSIS/USER/lift_ruler_adapter.c', adapter)
    edit('CHASSIS/USER/lift_ruler_adapter.h', lambda t: t.replace('extern volatile int32_t g_lift_compare_top_raw;', 'extern volatile int32_t g_lift_compare_top_raw;\nextern volatile uint32_t g_lift_compare_target_pulses;'))
    report = {'stage': str(STAGE), 'changed_files': changed, 'matlab_checks': model['check_count'],
              'corrected_down_target_pulses': 108400, 'previous_target': 110800, 'correction_up_equivalent_pulses': 2400,
              'configurable_target_range': [100000, 110800], 'configuration_latched_on_explicit_request': True,
              'retune_after_return_requires_new_key': True, 'automatic_repeat': False,
              'VM_held_through_motion_and_DONE': True, 'ordinary_reference_speeds_preserved': True,
              'other_axis_service_enabled': False, 'firmware_downloaded': False, 'hardware_operated': False}
    (OUT / 'staged-manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'staged': True, 'changed_files': len(changed), 'target': 108400}))


if __name__ == '__main__':
    main()
