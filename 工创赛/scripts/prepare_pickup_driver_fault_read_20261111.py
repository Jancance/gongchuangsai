"""Decouple strict read-only driver diagnosis from successful extension homing."""
from pathlib import Path
import hashlib
import json
import shutil

R = Path(__file__).resolve().parents[1]
B = R / 'tmp/pickup_driver_fifo_fix_20261111/project'
P = R / 'tmp/pickup_driver_fault_read_20261111/project'
E = R / 'evidence/integration/2026-11-11/driver-fault-read'


def replace_bytes(relative, old, new):
    p = P / relative
    data = p.read_bytes()
    old, new = old.encode('utf-8'), new.encode('utf-8')
    assert data.count(old) == 1, relative
    p.write_bytes(data.replace(old, new, 1))


def main():
    assert not P.exists(), 'Preserve existing candidate and inspect it'
    E.mkdir(parents=True, exist_ok=True)
    shutil.copytree(B, P, ignore=shutil.ignore_patterns('Objects', 'Listings'))
    header = P / 'CHASSIS/USER/startup_axes.h'
    data = header.read_bytes()
    marker = b'#endif'
    at = data.rindex(marker)
    header.write_bytes(data[:at] + b'/* Strict read-only diagnostics never grant homing or motion. */\r\nuint32_t StartupAxes_DiagnosticReadReady(void);\r\n' + data[at:])
    source = P / 'CHASSIS/USER/startup_axes.c'
    helper = r'''

/* Read-only access depends on idle ownership, not a valid mechanical origin. */
uint32_t StartupAxes_DiagnosticReadReady(void)
{
#if APP_MOTOR_OFFLINE_ONLY
 return 0U;
#elif VEHICLE_EXTENSION_HOME_ONLY && !VEHICLE_STARTUP_GROUP_ENABLE
 if(eh_stop_pending || eh_started)return 0U;
 return g_startup_axes_state==0U || g_startup_axes_state==4U ||
        g_startup_axes_state==5U || g_startup_axes_state==6U;
#else
 return g_startup_axes_state==0U || g_startup_axes_state==4U;
#endif
}
'''
    source.write_bytes(source.read_bytes() + helper.replace('\n', '\r\n').encode('utf-8'))
    replace_bytes('CHASSIS/USER/integration10_complete.inc',
                  '(g_startup_axes_state==0U || g_startup_axes_state==4U) &&',
                  'StartupAxes_DiagnosticReadReady() &&')
    replace_bytes('CHASSIS/SYSTEM/CAN/can.c',
                  '  if(!s_can1_normal_ready || !g_startup_axes_vm_request || g_startup_axes_state!=4U ||\r\n   g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;',
                  '  PowerCommandStatus diagnostic_power;Power_ReadCommandStatus(&diagnostic_power);\r\n  if(!s_can1_normal_ready || !diagnostic_power.active || !StartupAxes_DiagnosticReadReady() ||\r\n   g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)return 9U;')
    changed = []
    for file in P.rglob('*'):
        if file.is_file() and ((not (B / file.relative_to(P)).exists()) or file.read_bytes() != (B / file.relative_to(P)).read_bytes()):
            changed.append(file.relative_to(P).as_posix())
    (E / 'scope.json').write_text(json.dumps(dict(base=B.relative_to(R).as_posix(), project=P.relative_to(R).as_posix(),
        changed_files=changed, hardware_access=False, deployed=False, no_motion_or_reference_validity_change=True,
        no_fault_clear_or_power_enable=True, physical_stall_resolved=False), indent=2)+'\n', encoding='utf-8')
    build = (R / 'scripts/build_pickup_driver_fifo_fix_20261111.py').read_text(encoding='utf-8')
    build = build.replace('tmp/pickup_driver_fifo_fix_20261111/project', 'tmp/pickup_driver_fault_read_20261111/project')
    build = build.replace('2026-11-11/driver-fifo-fix', '2026-11-11/driver-fault-read')
    build = build.replace('driver_fifo_readonly_receive_fix', 'driver_fault_readonly_admission_fix')
    build = build.replace('tmp/pickup_half_descent_20261110/project', 'tmp/pickup_driver_fifo_fix_20261111/project')
    build = build.replace('455e5ea1b496a22ee96092765fa888e22f1060521b0d8e5f768b9dc27bd7d0a5', '9641f3d692f167da3396a7633a744b689536a856ab5467e274e823aa8225eab1')
    (R / 'scripts/build_pickup_driver_fault_read_20261111.py').write_text(build, encoding='utf-8')
    host = (R / 'scripts/pickup_fifo_diagnostic_live_20261111.py').read_text(encoding='utf-8')
    host = host.replace('tmp/pickup_driver_fifo_fix_20261111/project', 'tmp/pickup_driver_fault_read_20261111/project')
    host = host.replace('2026-11-11/driver-fifo-fix', '2026-11-11/driver-fault-read')
    host = host.replace("                cmd('extension_home',2)\n                cmd('open_empty_gripper',6)", "                # Extension homing is deliberately not requested for diagnosis.\n                cmd('open_empty_gripper',6)", 1)
    host = host.replace("report['plan']=['top_reference','extension_home','open_empty_gripper','read_only_node6_configuration']", "report['plan']=['top_reference','open_empty_gripper','read_only_node6_configuration_without_extension_homing']", 1)
    host = host.replace('--references-and-config-once', '--lift-reference-and-config-once')
    host = host.replace('args.references_and_config_once', 'args.lift_reference_and_config_once')
    host = host.replace('One reference sequence and read-only driver configuration.', 'One lift reference and read-only driver configuration; extension homing not requested.')
    compile(host, 'fault-read-host', 'exec')
    (R / 'scripts/pickup_driver_fault_read_live_20261111.py').write_text(host, encoding='utf-8')
    print('Prepared strict read-only admission candidate; no hardware access.')


if __name__ == '__main__':
    main()
