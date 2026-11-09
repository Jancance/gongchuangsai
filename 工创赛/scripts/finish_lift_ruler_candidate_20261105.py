"""Complete the isolated candidate only, after the MATLAB dispatch gate."""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'tmp/lift_ruler_20261105/project'
OUT = ROOT / 'evidence/integration/2026-11-05/lift-ruler-candidate'


def function_end(text, name):
    start = text.index('{', text.index(name))
    depth = 0
    for i in range(start, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if not depth:
                return i
    raise AssertionError(name)


def finish(stage=STAGE):
    proof = json.loads((OUT / 'dispatch-matlab-result.json').read_text(encoding='utf-8'))
    assert proof['engine'] == 'local MATLAB' and proof['passed'] and not proof['failure_count']
    path = stage / 'CHASSIS/USER/vehicle_mission.inc'
    raw = path.read_bytes()
    encoding = 'utf-8'
    try:
        text = raw.decode(encoding)
    except UnicodeDecodeError:
        encoding = 'gbk'
        text = raw.decode(encoding)
    text = text.replace('\r\n', '\n')
    # An explicit else retains the ordinary mission for other modes without
    # putting statements after a compile-time unconditional return.
    for name, marker in [
        ('static void VehicleMission_Service(void)',
         '    if(VEHICLE_LIFT_RULER_ONLY){if(key)LiftRuler_DebugKey();return;}'),
        ('static void VehicleMission_Telemetry(void)',
         '        if(App_DebugSendString(frame))ruler_last=now;return;\n    }'),
    ]:
        assert text.count(marker) == 1
        end = function_end(text, name)
        text = text[:end].replace(marker, marker + ' else {', 1) + '}\n' + text[end:]
    path.write_bytes(text.replace('\n', '\r\n').encode(encoding))
    path = stage / 'CHASSIS/SYSTEM/CAN/can.c'
    raw = path.read_bytes()
    encoding = 'utf-8'
    try:
        text = raw.decode(encoding)
    except UnicodeDecodeError:
        encoding = 'gbk'
        text = raw.decode(encoding)
    text = text.replace('\r\n', '\n')
    marker = '    if(!q || !out || (address!=5U && address!=6U))return 7U;'
    assert text.count(marker) == 1
    text = text.replace(marker, marker + '\n    if(VEHICLE_LIFT_RULER_ONLY && address!=5U)return 9U;', 1)
    old = '''       ((!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U) &&
        !LiftRuler_DebugCanAllowed(address,q,length)))return 9U;'''
    new = '''       (VEHICLE_LIFT_RULER_ONLY ? !LiftRuler_DebugCanAllowed(address,q,length) :
        (!g_startup_axes_vm_request || g_startup_axes_state<2U || g_startup_axes_state>4U)))return 9U;'''
    assert text.count(old) == 1
    text = text.replace(old, new, 1)
    path.write_bytes(text.replace('\n', '\r\n').encode(encoding))


if __name__ == '__main__':
    finish()
    print('Dedicated candidate mission branches and node5 admission completed.')
