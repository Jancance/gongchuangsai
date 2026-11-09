"""Stage VM-holding comparison and all three reference-speed settings. No flash."""
from pathlib import Path
import hashlib
import json
import shutil
from finish_lift_ruler_candidate_20261105 import function_end

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'tmp/lift_final50_20261106/project'
STAGE = ROOT / 'tmp/lift_speed_compare_20261106/project'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-speed-compare'
MODULE = ROOT / 'software_interfaces/lift_speed_compare'


def main():
    model = json.loads((OUT / 'matlab-result.json').read_text(encoding='utf-8'))
    assert model['passed'] and model['failure_count'] == 0
    assert not STAGE.exists(), 'Preserve any existing staged result'
    shutil.copytree(BASE, STAGE, ignore=shutil.ignore_patterns('Objects', 'ObjectsOffline', 'Listings', '*.uvguix.*'))
    changed = {}

    def edit(name, transform):
        path = STAGE / name
        raw = path.read_bytes()
        try:
            text, encoding = raw.decode('utf-8'), 'utf-8'
        except UnicodeDecodeError:
            text, encoding = raw.decode('gbk'), 'gbk'
        result = transform(text.replace('\r\n', '\n'))
        path.write_bytes(result.replace('\n', '\r\n').encode(encoding))
        changed[name] = {'before': hashlib.sha256(raw).hexdigest(), 'after': hashlib.sha256(path.read_bytes()).hexdigest()}

    for name in ['lift_ruler_calibration.c', 'lift_ruler_calibration.h']:
        edit('CHASSIS/USER/' + name, lambda _, name=name: (MODULE / name).read_text(encoding='utf-8'))

    def adapter(t):
        t = t.replace('volatile uint32_t g_lift_ruler_request,g_lift_ruler_rejected;', '''volatile uint32_t g_lift_ruler_request,g_lift_ruler_rejected;
volatile uint32_t g_lift_compare_operation,g_lift_compare_top_valid,g_lift_compare_home_flags;
volatile uint32_t g_lift_compare_home_seen,g_lift_compare_home_reported,g_lift_compare_home_exit;
volatile uint32_t g_lift_compare_speed_raw,g_lift_compare_move_elapsed_ms;
volatile int32_t g_lift_compare_top_raw;''')
        old = ' if(n>15 || rn>8)return 0;'
        assert t.count(old) == 1
        t = t.replace(old, ''' if(n==2U && q[0]==0x22U && q[1]==0x6bU && rn==17U)
  return LiftRuler_ReadHomeParameters(r)==1U;
 if(n>15 || rn>8)return 0;''')
        old = ' g_lift_ruler_state=g_lift_ruler_controller.state;'
        t = t.replace(old, ''' g_lift_compare_operation=g_lift_ruler_controller.operation;
 g_lift_compare_top_valid=g_lift_ruler_controller.top_valid;
 g_lift_compare_top_raw=(int32_t)g_lift_ruler_controller.top_position_raw;
 g_lift_compare_home_flags=g_lift_ruler_controller.home_flags;
 g_lift_compare_home_seen=g_lift_ruler_controller.home_seen_running;
 g_lift_compare_home_reported=g_lift_ruler_controller.home_reported;
 g_lift_compare_home_exit=g_lift_ruler_controller.home_exit_confirmed;
 g_lift_compare_speed_raw=g_lift_ruler_controller.speed_raw;
 g_lift_compare_move_elapsed_ms=g_lift_ruler_controller.move_elapsed_ms;
''' + old)
        old = ' g_lift_ruler_request=(g_lift_ruler_controller.state==LRC_IDLE && !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;'
        assert t.count(old) == 1
        t = t.replace(old, ''' if(g_lift_ruler_controller.state==LRC_DONE && g_lift_ruler_controller.operation>=3U)
  g_lift_ruler_rejected=3U;
 else g_lift_ruler_request=((g_lift_ruler_controller.state==LRC_IDLE || g_lift_ruler_controller.state==LRC_DONE) &&
  !g_lift_ruler_request)?LRC_REQUEST_START:LRC_REQUEST_STOP;''')
        return t

    edit('CHASSIS/USER/lift_ruler_adapter.c', adapter)
    edit('CHASSIS/USER/lift_ruler_adapter.h', lambda t: t.replace('#endif', '''extern volatile uint32_t g_lift_compare_operation,g_lift_compare_top_valid,g_lift_compare_home_flags;
extern volatile uint32_t g_lift_compare_home_seen,g_lift_compare_home_reported,g_lift_compare_home_exit;
extern volatile uint32_t g_lift_compare_speed_raw,g_lift_compare_move_elapsed_ms;
extern volatile int32_t g_lift_compare_top_raw;
uint32_t LiftRuler_ReadHomeParameters(uint8_t out[17]);
#endif'''))

    def can(t):
        begin = t.index('uint32_t StartupAxis_ReadHomeParameters(')
        helper = t[begin:function_end(t, 'uint32_t StartupAxis_ReadHomeParameters(') + 1]
        helper = helper.replace('StartupAxis_ReadHomeParameters', 'LiftRuler_ReadHomeParameters')
        old = 'if(!g_startup_axes_vm_request || g_startup_axes_state!=2U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;'
        assert helper.count(old) == 1
        helper = helper.replace(old, 'if(!VEHICLE_LIFT_RULER_ONLY || !g_lift_ruler_controller.vm_request || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;')
        helper = helper.replace('0x0600U', '0x0500U').replace('(rx.ExtId >> 8) == 6U', '(rx.ExtId >> 8) == 5U')
        t += '\n#if !APP_MOTOR_OFFLINE_ONLY\n' + helper + '\n#else\nuint32_t LiftRuler_ReadHomeParameters(uint8_t out[17]){(void)out;return 9U;}\n#endif\n'
        marker = '    if(length==4U && q[0]==0xfe && q[1]==0x98 && q[2]==0 && q[3]==0x6b)'
        assert t.count(marker) == 1
        t = t.replace(marker, '''    if(VEHICLE_LIFT_RULER_ONLY && address==5U && reply_length==3U &&
       ((length==2U && q[0]==0x3bU && q[1]==0x6bU) ||
        (length==3U && q[0]==0x9cU && q[1]==0x48U && q[2]==0x6bU)))
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
''' + marker)
        marker = '    if(length==5U && q[0]==0xf3 && q[1]==0xab && q[2]==1 && q[3]==0 && q[4]==0x6b)'
        assert t.count(marker) == 1
        t = t.replace(marker, '''    if(VEHICLE_LIFT_RULER_ONLY && address==5U && length==4U && reply_length==3U &&
       q[0]==0x9aU && q[1]==2U && q[2]==0U && q[3]==0x6bU)
        return CAN1_ExchangeSingle(5U,q,length,3U,reply);
''' + marker)
        return t

    edit('CHASSIS/SYSTEM/CAN/can.c', can)

    def params(t):
        replacements = [
            ('#define VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S 150.0f', '#define VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S 300.0f'),
            ('#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 30.0f', '#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 106.0f'),
            ('#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 60U', '#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 45055U'),
            ('#define STARTUP_DM_SPEED_SCALE 0.5f', '#define STARTUP_DM_SPEED_SCALE 1.0f'),
        ]
        for old, new in replacements:
            assert t.count(old) == 1, old
            t = t.replace(old, new)
        t = t.replace('/* 300×0.5=150deg/s，按本次角度逆时针到0。 */', '/* Reference300deg/s; no startup half-speed override. */')
        return t

    edit('USER/vehicle_parameters.h', params)
    edit('CHASSIS/USER/source_prepare.inc', lambda t: t.replace('VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S<=150', 'VEHICLE_SOURCE_PREP_DM_SPEED_DEG_S<=300').replace('VEHICLE_SOURCE_PREP_EXTENSION_RPM<=30', 'VEHICLE_SOURCE_PREP_EXTENSION_RPM<=106').replace('VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S<=300', 'VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S<=45055'))
    # Keep LR1 as a per-operation trace. Pulse/speed fields are dynamic, so
    # the old single-step 12800/30RPM literals cannot mislabel fast runs.
    def telemetry(t):
        old = ',12800,300,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,'
        new = ',%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,'
        assert t.count(old) == 1
        t = t.replace(old, new)
        marker = '(unsigned long)g_lift_ruler_cause,(unsigned long)g_lift_ruler_samples,(unsigned long)g_lift_ruler_options,'
        assert t.count(marker) == 1
        return t.replace(marker, '''(unsigned long)g_lift_ruler_cause,(unsigned long)g_lift_ruler_samples,
        (unsigned long)(g_lift_ruler_controller.move_units*3200ULL/(360U*(g_lift_ruler_controller.scale?g_lift_ruler_controller.scale:100U))),
        (unsigned long)g_lift_ruler_controller.command_speed,(unsigned long)g_lift_ruler_options,''')
    edit('CHASSIS/USER/vehicle_mission.inc', telemetry)
    report = {'stage': str(STAGE), 'changed_files': changed, 'matlab_checks': model['check_count'],
              'reference_speeds': {'lift_rpm': 1846.4, 'extension_rpm': 106, 'DM_deg_s': 300, 'stepper_accel_rpm_s': 45055},
              'other_axis_service_enabled_in_compare': False, 'ordinary_vehicle_lift_limit_changed': False,
              'firmware_downloaded': False, 'hardware_operated': False,
              'deployment_requires_resolving_user_no_VM_interruption_constraint': True}
    (OUT / 'staged-manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    (OUT / 'dispatch-matlab-result.json').write_text(json.dumps({'passed': True, 'check_count': model['check_count']}), encoding='utf-8')
    print(json.dumps({'stage': str(STAGE), 'changed_count': len(changed), 'flash_performed': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
