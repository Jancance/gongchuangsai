"""Read existing fault and boot evidence once; never halt/reset/write controls."""
from pathlib import Path
import datetime
import hashlib
import json
import sys
import time

E = Path(__file__).resolve().parent
R = E.parents[5]
sys.path[:0] = [str(R / '工创赛/tmp/turn_probe_deps'), str(R / '工创赛/scripts')]
from elftools.elf.elffile import ELFFile
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.target.pack.pack_target import PackTargets
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session

IMAGE = R / '工创赛/tmp/half-stop-observer-build-20261115/baseline40_integrated/Vehicle_Integrated.axf'
assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == '3966ed37a6c2dd85319884fbe22a2c54a610a9bb9e50f46240e79c006bf1113d'
assert json.loads((E / 'result_summary.json').read_text('utf8'))['status'] == 'ack_timeout'
with (E / 'failure_read_started.json').open('x', encoding='utf8') as stream:
    json.dump({'read_only': True, 'automatic_retry': False}, stream)
names = {'g_reset_flags', 'g_reset_iwdg', 'g_watchdog_started', 'g_watchdog_feed_count',
    'g_rtos_fault', 'g_rtos_assert_line', 'g_safety_fault', 'g_vm_test_request',
    'g_serial_motion_state', 'g_serial_motion_result', 'g_serial_motion_accepted',
    'g_serial_motion_rejected', 'g_serial_motion_link_lost', 'g_serial_rx_count',
    'g_serial_rx_last', 'g_test_tick', 'g_test_count_a', 'g_can_task_count',
    'g_chassis_auto_state', 'g_chassis_auto_active', 'g_chassis_auto_session',
    's_serial_tx_seq', 's_serial_tx_tick', 's_serial_tx_valid', 's_serial_dropped',
    'g_uart1_rx_errors', 'g_tx_length', 'g_tx_position'}
with IMAGE.open('rb') as stream:
    table = ELFFile(stream).get_section_by_name('.symtab')
    symbols = {s.name: (int(s['st_value']), int(s['st_size'])) for s in table.iter_symbols() if s.name in names}
for name, (address, size) in symbols.items():
    assert size in (1, 2, 4) and (0x10000000 <= address < 0x10010000 or 0x20000000 <= address < 0x20020000), name
receipt = dict(read_only=True, reset=False, halted=False, resumed=False,
    actuator_writes=0, VM_writes=0, retry=False, started_at=datetime.datetime.now().astimezone().isoformat(),
    missing_symbols=sorted(names-set(symbols)))
session = None
try:
    PackTargets.populate_targets_from_pack('D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8')
    probes = [p for p in ConnectHelper.get_all_connected_probes(blocking=False) if p.unique_id.lower() == '6afaae24']
    assert len(probes) == 1
    probe = probes[0]
    synchronous_interface(probe._link._interface)
    session = Session(probe, options={'target_override': 'stm32f407ve', 'connect_mode': 'attach',
        'frequency': 1000000, 'auto_unlock': False, 'resume_on_disconnect': False,
        'no_config': True, 'dap_protocol': 'swd', 'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True})
    session.open()
    target = session.target
    receipt['core_before'] = target.get_state().name
    def sample():
        return {n: target.read8(a) if size == 1 else target.read16(a) if size == 2 else target.read32(a) for n, (a, size) in symbols.items()}
    receipt['state_before'] = sample()
    receipt['registers'] = {name: f'0x{target.read32(address):08x}' for name, address in {
        'USART1_SR':0x40011000, 'USART1_BRR':0x40011008, 'USART1_CR1':0x4001100C, 'USART1_CR2':0x40011010, 'USART1_CR3':0x40011014, 'GPIOA_MODER':0x40020000, 'GPIOA_AFRH':0x40020024, 'RCC_AHB1ENR':0x40023830, 'RCC_APB2ENR':0x40023844, 'RCC_CSR_current_not_saved_cause':0x40023874, 'SCB_ICSR':0xE000ED04,
        'SCB_SHCSR':0xE000ED24, 'SCB_CFSR':0xE000ED28, 'SCB_HFSR':0xE000ED2C,
        'SCB_MMFAR':0xE000ED34, 'SCB_BFAR':0xE000ED38}.items()}
    receipt['pe2'] = (target.read32(0x40021010) >> 2) & 1
    time.sleep(0.03)
    receipt['state_after'] = sample()
    receipt['core_after'] = target.get_state().name
except Exception as exc:
    receipt['error'] = repr(exc)
    raise
finally:
    if session:
        session.close()
    (E / 'failure_read_receipt.json').write_text(json.dumps(receipt, indent=2), encoding='utf8')
    print(json.dumps(receipt))
