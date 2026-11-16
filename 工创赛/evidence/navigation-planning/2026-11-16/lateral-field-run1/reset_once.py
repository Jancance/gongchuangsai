from pathlib import Path
import datetime
import hashlib
import json
import sys

E = Path(__file__).parent
R = Path(__file__).resolve().parents[6]
from run_binding import R,S,IMAGE as image,EXPECTED as expected,validate
binding=validate('reset')
sys.path.insert(0,str(R/'工创赛/tmp/turn_probe_deps'))
sys.path.insert(0,str(R/'工创赛/scripts'))
from elftools.elf.elffile import ELFFile
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.target.pack.pack_target import PackTargets
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
receipt=E/'reset_once_receipt.json'
assert not receipt.exists()
names = ['g_chassis_auto_active', 'g_vm_test_request', 'g_safety_fault', 'g_rtos_fault',
         'g_can_ctrl_fault_latched', 'g_debug_plan_id', 'g_chassis_auto_cause', 's_distance_fault_reason', 'g_chassis_auto_state', 'g_chassis_auto_result']
with image.open('rb') as f:
    elf = ELFFile(f)
    symbols = {s.name: (int(s['st_value']), int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s.name in names}
assert set(symbols) == set(names)
r = dict(candidate_sha256=expected, flash_read=False, flashed=False, reset_attempted=False,
         reset=False, automatic_retry=False, actuator_requests_written=0,
         started_at=datetime.datetime.now().astimezone().isoformat())
with (E / 'reset_once_started.json').open('x', encoding='utf8') as stream:
    json.dump(dict(candidate_sha256=expected, attempts=1, reset_authorized=1, automatic_retry=False),stream)
session = None
try:
    PackTargets.populate_targets_from_pack('D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8')
    probes = [p for p in ConnectHelper.get_all_connected_probes(blocking=False) if p.unique_id.lower() == binding['probe_uid'].lower()]
    assert len(probes) == 1
    probe = probes[0]
    synchronous_interface(probe._link._interface)
    session = Session(probe, options={'target_override': 'stm32f407ve', 'connect_mode': 'attach',
        'frequency': 1000000, 'auto_unlock': False, 'resume_on_disconnect': False,
        'no_config': True, 'dap_protocol': 'swd', 'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True})
    session.open()
    t = session.target
    r['core_before'] = t.get_state().name
    assert t.get_state() == Target.State.RUNNING
    r['symbols_before'] = {}
    for name, (a, size) in symbols.items():
        assert size in (1, 2, 4) and (0x10000000 <= a < 0x10010000 or 0x20000000 <= a < 0x20020000)
        r['symbols_before'][name] = t.read8(a) if size == 1 else t.read16(a) if size == 2 else t.read32(a)
    r['pe2_before'] = (t.read32(0x40021010) >> 2) & 1
    assert r['symbols_before']['g_debug_plan_id'] in (8, 9)
    assert all(r['symbols_before'][n] == 0 for n in names if n not in ['g_debug_plan_id','g_vm_test_request','g_chassis_auto_state','g_chassis_auto_result'])
    assert (r['pe2_before']==0 and r['symbols_before']['g_vm_test_request']==0) or (r['pe2_before']==1 and r['symbols_before']['g_vm_test_request']==1 and r['symbols_before']['g_chassis_auto_state']==9 and r['symbols_before']['g_chassis_auto_result']==1)
    # Programming/Verify/Application-running receipt and current candidate binding establish identity.
    download=json.loads((E/'download_once_receipt.json').read_text('utf8'))
    assert download['success'] and download['candidate_sha256']==expected
    r['reset_attempted'] = True
    receipt.write_text(json.dumps(r, indent=2), 'utf8')
    t.reset()
    r['reset'] = True
except Exception as exc:
    r['error'] = repr(exc)
    raise
finally:
    if session:
        session.close()
    receipt.write_text(json.dumps(r, indent=2), 'utf8')
    print(json.dumps(r), flush=True)
