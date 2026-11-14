"""Read reset flags/state once after the failed attempt; no control writes."""
from pathlib import Path
import datetime
import hashlib
import json
import sys

E = Path(__file__).resolve().parent
R = E.parents[5]
sys.path.insert(0, str(R / '工创赛/tmp/turn_probe_deps'))
sys.path.insert(0, str(R / '工创赛/scripts'))
from elftools.elf.elffile import ELFFile
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.target.pack.pack_target import PackTargets
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

IMAGE = R / '工创赛/tmp/tail-snapshot-build-20261115/baseline40_integrated/Vehicle_Integrated.axf'
assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == '5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
result = json.loads((E / 'result_summary.json').read_text('utf8'))
assert result['status'] == 'telemetry_session_changed'
marker = E / 'reset_observation_started.json'
with marker.open('x', encoding='utf8') as stream:
    json.dump({'read_only': True, 'automatic_retry': False}, stream)
names = ['g_chassis_auto_session', 'g_chassis_auto_state', 'g_chassis_auto_active',
         'g_vm_test_request', 'g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched']
with IMAGE.open('rb') as stream:
    table = ELFFile(stream).get_section_by_name('.symtab')
    symbols = {s.name: (int(s['st_value']), int(s['st_size'])) for s in table.iter_symbols() if s.name in names}
assert set(symbols) == set(names) and all(size == 4 for _, size in symbols.values())
receipt = dict(read_only=True, reset=False, halted=False, resumed=False, VM_writes=0,
               automatic_retry=False, time=datetime.datetime.now().astimezone().isoformat(),
               limits='Reset flags are sticky; prior flags were not captured or cleared. Not a unique reset cause.')
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
    assert target.get_state() == Target.State.RUNNING
    receipt['core'] = target.get_state().name
    csr = target.read32(0x40023874)
    receipt['RCC_CSR'] = f'0x{csr:08x}'
    receipt['sticky_flags'] = {name: bool(csr & mask) for name, mask in {
        'brownout': 0x02000000, 'pin_reset': 0x04000000, 'power_on': 0x08000000,
        'software_reset': 0x10000000, 'independent_watchdog': 0x20000000,
        'window_watchdog': 0x40000000, 'low_power_reset': 0x80000000}.items()}
    receipt['ram_state'] = {name: target.read32(address) for name, (address, _) in symbols.items()}
    receipt['pe2'] = (target.read32(0x40021010) >> 2) & 1
except Exception as exc:
    receipt['error'] = repr(exc)
    raise
finally:
    if session:
        session.close()
    (E / 'reset_observation.json').write_text(json.dumps(receipt, indent=2), encoding='utf8')
    print(json.dumps(receipt))
