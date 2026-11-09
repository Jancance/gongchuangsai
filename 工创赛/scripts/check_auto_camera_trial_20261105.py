"""Control-contract checks with simulated V4L2; no camera I/O."""
import contextlib
import io
import json
from pathlib import Path
from prepare_auto_camera_trial_20261105 import AUTO_FUNCTION

MENUS = ''' exposure_auto 0x009a0901 (menu) : value=1
                1: Manual Mode
                3: Aperture Priority Mode
 exposure_auto_priority 0x009a0903 (bool) : value=0
 white_balance_temperature_auto 0x0098090c (bool) : value=0
'''


def case(wb='auto', menus=MENUS, bad_readback=False, write_failure=False):
    writes = []
    def write(device, setting):
        if write_failure:
            raise RuntimeError('simulated control write failure')
        writes.append(setting)
    def command(args):
        if args[-1] == '--list-ctrls-menus':
            return menus
        return ('exposure_auto: ' + ('1' if bad_readback else '3') + '\n'
                'exposure_auto_priority: 0\nwhite_balance_temperature_auto: ' + str(int(wb == 'auto')) + '\n')
    space = dict(command=command, set_control=write, read_for=lambda *args: 'frame',
                 CONTROL_NAMES=('exposure_auto', 'exposure_absolute', 'gain',
                                'white_balance_temperature_auto', 'white_balance_temperature'))
    exec(AUTO_FUNCTION, space)
    with contextlib.redirect_stdout(io.StringIO()):
        result = space['initialize_auto_controls'](object(), dict(device='fake', height=1080, width=1920,
                                      white_balance_mode=wb, awb_adjust_seconds=7, locked_capture_seconds=2))
    assert result[0] == 'frame'
    assert writes[:3] == ['exposure_auto_priority=0', 'exposure_auto=3', 'white_balance_temperature_auto=1']
    assert not any(x.startswith(('exposure_absolute=', 'gain=', 'white_balance_temperature=')) for x in writes)
    assert ('white_balance_temperature_auto=0' in writes) == (wb == 'startup_lock')


case()
case('startup_lock')
checks = 2
for kwargs in (dict(menus=MENUS.replace('3: Aperture Priority Mode', '2: Shutter Priority Mode')),
               dict(menus=MENUS.replace('exposure_auto_priority', 'unsupported')),
               dict(bad_readback=True), dict(write_failure=True), dict(wb='invalid')):
    try:
        case(**kwargs)
    except RuntimeError:
        checks += 1
    else:
        raise AssertionError('Expected failed initialization: ' + repr(kwargs))
report = dict(checks=checks, failures=0, scope='simulated V4L2 control contracts; no hardware acceptance',
              cases=['continuous AE/AWB', 'AE with startup WB lock', 'unsupported mode rejection',
                     'missing control rejection', 'readback mismatch rejection', 'write error propagation', 'invalid WB rejection'])
out = Path(__file__).resolve().parents[1]/'evidence/p2-camera/2026-11-05/auto-camera'
out.mkdir(exist_ok=True)
(out/'control-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report))
