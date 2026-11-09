"""One authorised live servo adjustment; no halt/reset/flash/mission changes."""
from pathlib import Path
import argparse
import json
import time
import gripper210_attach_read_20261105 as live

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--execute', action='store_true')
args = parser.parse_args()
if not args.execute:
    raise SystemExit('Explicit --execute required; no attachment or write performed')
model = json.loads((live.OUT/'signal-model.json').read_text())
assert model['result'] == 'checks=55 failures=0 pulse=2056'
baseline = json.loads((live.OUT/'attach-read.json').read_text())
assert baseline.get('flash_matches') and baseline['flash_bytes_verified'] == 148728
symbols, segments = live.load_image()
CCR1 = 0x40000034  # The sole allowed actuator write: PA0/TIM2_CH1 compare.
EXPECTED = {'g_vehicle_stage':2, 'g_vehicle_state':4, 'g_vehicle_cause':0,
            'g_vehicle_run_id':1, 'g_vehicle_key':0, 'g_vehicle_alignment_state':1,
            'g_vehicle_arm_state':2, 'g_vehicle_arm_cause':0,
            'g_source_prepare_state':3, 'g_source_prepare_cause':0,
            'g_source_prepare_done_mask':7, 'g_startup_axes_state':4,
            'g_startup_axes_cause':0, 'g_vm_test_request':0,
            'g_chassis_auto_active':0, 'g_rtos_fault':0, 'g_safety_fault':0,
            'g_safety_ready':1, 'g_dm_jog_request':0, 'g_dm_jog_state':7,
            'core_state':'RUNNING', 'TIM2_CR1':129, 'TIM2_CCMR1':104,
            'TIM2_CCER':1, 'TIM2_PSC':83, 'TIM2_ARR':19999}


def guard(target, pulse):
    state = live.snapshot(target, symbols)
    for key, value in EXPECTED.items():
        if state.get(key) != value:
            raise RuntimeError(f'Live guard changed: {key}={state.get(key)}, expected {value}')
    if state['GPIOA_MODER'] & 3 != 2 or state['GPIOA_AFRL'] & 15 != 1:
        raise RuntimeError('PA0 is not TIM2_CH1')
    if state['TIM2_CCR1'] != pulse:
        raise RuntimeError(f'PWM changed unexpectedly: {state["TIM2_CCR1"]}, expected {pulse}')
    return state


report = {'requested_angle_deg':210, 'temporary_debug_adjustment':True,
          'write_address':hex(CCR1), 'reset_requested':False, 'halt_requested':False,
          'firmware_downloaded':False, 'mission_triggered':False,
          'physical_angle_feedback_available':False, 'writes':[]}
session = live.attach()
try:
    session.open()
    target = session.target
    report['before'] = guard(target, 1833)
    # Verify executable identity again immediately before the first actuator write.
    for address, data in segments:
        for offset in range(0, len(data), 4096):
            expected = data[offset:offset+4096]
            if bytes(target.read_memory_block8(address+offset, len(expected))) != expected:
                raise RuntimeError('Firmware changed; no actuator write')
    pulse = 1833
    for step in range(1, 21):
        guard(target, pulse)
        angle = 180.0 + 30.0 * step / 20.0
        next_pulse = int(500 + angle * 2000.0 / 270.0 + .5)
        assert pulse <= next_pulse <= 2056 and next_pulse-pulse <= 12
        target.write32(CCR1, next_pulse)
        target.flush()
        report['writes'].append({'step':step, 'commanded_deg':angle, 'pulse_us':next_pulse})
        pulse = next_pulse
        time.sleep(.04)
        guard(target, pulse)
    report['held_samples'] = []
    for _ in range(3):
        time.sleep(.4)
        report['held_samples'].append(guard(target, 2056))
    report['pwm_command_completed'] = True
    report['other_state_unchanged'] = True
except Exception as error:
    report['error'] = repr(error)
    raise
finally:
    session.close()
    (live.OUT/'execute-result.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report))
