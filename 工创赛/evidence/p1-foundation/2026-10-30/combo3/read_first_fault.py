from pathlib import Path
import sys,json,struct
sys.path.insert(0,str(Path('工创赛/tmp/turn_probe_deps').resolve()))
from pyocd.core.helpers import ConnectHelper
from elftools.elf.elffile import ELFFile

axf=Path('工创赛/tmp/combo3_candidate_20261030/Objects/gongxun-f407.axf')
names=['g_forward_hold', 'g_combined_test', 'g_vm_test_request', 'g_heading_power_wait_state', 'g_heading_power_post_settle_mask', 'g_heading_power_on_age_ms', 'g_can_jog_request', 'g_can_enable_test_request', 'g_can_enable_test_state', 'g_can_enable_test_result', 'g_can_ctrl_fault_latched', 'g_safety_ready', 'g_can_guard_ready', 'g_can_poll_result', 'g_can_poll_flags', 'g_can_poll_count', 'g_can_poll_tick', 'g_chassis_auto_state', 'g_chassis_auto_result', 'g_chassis_auto_active', 'g_chassis_auto_cut_vm', 'g_chassis_auto_cause', 's_vm_command_active', 's_vm_enable_seen', 's_vm_enable_tick', 's_vm_enable_count', 's_can_enable_test', 's_can_jog_preparing', 'g_motion_fault_diag', 'g_motion_fault_diag_valid', 's_hold_wait_ms', 's_diagnostic']
with axf.open('rb') as f:
    table=ELFFile(f).get_section_by_name('.symtab')
    symbols={s.name:(s['st_value'],s['st_size']) for s in table.iter_symbols() if s.name in names}
probes=ConnectHelper.get_all_connected_probes(blocking=False)
print('probe_count',len(probes),flush=True)
if len(probes)!=1:raise RuntimeError('Expected exactly one probe; no target connection made')
report={'connect_mode':'attach','auto_unlock':False,'resume_on_disconnect':False,
        'reset_called':False,'halt_called':False,'firmware_loaded':False,'memory_written':False,
        'symbols':{}}
session=ConnectHelper.session_with_chosen_probe(unique_id=probes[0].unique_id,blocking=False,
    target_override='cortex_m',no_config=True,connect_mode='attach',auto_unlock=False,
    resume_on_disconnect=False,frequency=1000000)
if session is None:raise RuntimeError('No session')
with session:
    target=session.target
    report['core_state_before']=str(target.get_state())
    for name,(address,size) in symbols.items():
        if not(0x20000000<=address and address+size<=0x20020000 and size<=256):
            raise RuntimeError('Unexpected RAM symbol '+name)
        values=target.read_memory_block32(address,(size+3)//4)
        report['symbols'][name]={'address':hex(address),'size':size,'u32':values}
    report['core_state_after']=str(target.get_state())
path=Path('工创赛/evidence/p1-foundation/2026-10-30/combo3/ram_first_fault.json')
path.write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
