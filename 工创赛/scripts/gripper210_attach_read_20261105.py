"""Read-only live attachment: no reset, halt, flash, resume, or actuator writes."""
from pathlib import Path
import hashlib
import json
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tmp/gripper_debug_deps'))
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
from elftools.elf.elffile import ELFFile

IMAGE = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程/Objects/Vehicle_Integrated/Vehicle_Integrated.axf')
OUT = ROOT / 'evidence/integration/2026-11-05/gripper210'
OUT.mkdir(parents=True, exist_ok=True)


def load_image():
    with IMAGE.open('rb') as f:
        elf = ELFFile(f)
        symbols = {s.name: int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols()
                   if s['st_info']['type'] == 'STT_OBJECT' and s.name.startswith('g_')}
        segments = [(int(s['p_paddr']), s.data()) for s in elf.iter_segments()
                    if s['p_type'] == 'PT_LOAD' and s['p_filesz'] and 0x08000000 <= s['p_paddr'] < 0x08100000]
    return symbols, segments


def attach():
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    if len(probes) != 1:
        raise RuntimeError(f'Expected exactly one probe, found {len(probes)}')
    return ConnectHelper.session_with_chosen_probe(
        blocking=False, unique_id=probes[0].unique_id, options={
            'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
            'auto_unlock': False, 'resume_on_disconnect': False, 'dap_protocol': 'swd',
            'no_config': True})


def snapshot(target, symbols):
    names = ['g_vehicle_stage', 'g_vehicle_state', 'g_vehicle_cause', 'g_vehicle_run_id',
             'g_vehicle_key', 'g_vehicle_alignment_state', 'g_vehicle_arm_state',
             'g_vehicle_arm_cause', 'g_source_prepare_state', 'g_source_prepare_cause',
             'g_source_prepare_done_mask', 'g_startup_axes_state', 'g_startup_axes_cause',
             'g_vm_test_request', 'g_chassis_auto_active', 'g_can_fault', 'g_rtos_fault',
             'g_safety_fault', 'g_safety_ready', 'g_dm_jog_request', 'g_dm_jog_state',
             'g_source_prepare_grip_deg']
    result = {}
    for name in names:
        if name in symbols:
            value = target.read32(symbols[name])
            result[name] = struct.unpack('<f', struct.pack('<I', value))[0] if name.endswith('_deg') else value
    result['core_state'] = target.get_state().name
    result['dbg_idcode'] = target.read32(0xE0042000)
    for name, address in {'TIM2_CR1':0x40000000, 'TIM2_CCMR1':0x40000018,
                          'TIM2_CCER':0x40000020, 'TIM2_PSC':0x40000028,
                          'TIM2_ARR':0x4000002C, 'TIM2_CCR1':0x40000034,
                          'GPIOA_MODER':0x40020000, 'GPIOA_AFRL':0x40020020}.items():
        result[name] = target.read32(address)
    return result


if __name__ == '__main__':
    symbols, segments = load_image()
    report = {'flash_image': str(IMAGE), 'flash_image_sha256': hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
              'actuator_writes': 0, 'reset_requested':False, 'halt_requested':False,
              'firmware_downloaded':False}
    session = attach()
    assert session is not None
    try:
        session.open()
        target = session.target
        assert target.get_state() == Target.State.RUNNING, 'Core was not running; no action'
        report['before'] = snapshot(target, symbols)
        compared = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                expected = data[offset:offset+4096]
                actual = bytes(target.read_memory_block8(address+offset, len(expected)))
                if actual != expected:
                    raise RuntimeError(f'Firmware mismatch at {address+offset:#x}; no action')
                compared += len(expected)
        report['flash_bytes_verified'] = compared
        report['flash_matches'] = True
        time.sleep(.3)
        report['after'] = snapshot(target, symbols)
    except Exception as e:
        report['error'] = repr(e)
        raise
    finally:
        session.close()
        (OUT/'attach-read.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(report))
