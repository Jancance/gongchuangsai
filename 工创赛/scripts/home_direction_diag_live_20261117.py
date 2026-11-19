"""Default local check; explicit --snapshot/--watch reads only, never resets/writes."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import struct
import sys
import time
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-17/home-direction-diag'
FIELDS='stage result frozen read_result fresh_valid after_valid ack_seen ack_extid ack_dlc write_tx_submit_count write_tx_ok_count packet offset last_mailbox last_status'.split()


def decode(data):
    assert len(data)==136
    d=dict(zip(FIELDS,struct.unpack('<15I',data[:60])))
    for i,n in enumerate(('before','fresh','expected','after')):d[n]=list(data[60+17*i:77+17*i])
    d['ack_data']=list(data[128:136]);return d


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--snapshot',action='store_true');ap.add_argument('--watch',type=float)
    ap.add_argument('--manifest',type=Path,default=E/'candidate.json')
    ap.add_argument('--compact',action='store_true',help='Keep full JSON on disk; print concise cycle events only')
    a=ap.parse_args();manifest=json.loads(a.manifest.read_text());im=manifest['images']['Vehicle_LiftRuler'];axf=Path(im['axf'])
    assert hashlib.sha256(axf.read_bytes()).hexdigest()==im['sha256'] and manifest['home_diag_version']==1
    assert decode(bytes(136))['frozen']==0
    if not a.snapshot and a.watch is None:
        print(json.dumps(dict(local_check=True,hardware_access=False,image_sha256=im['sha256'],flash_bytes=im['flash_bytes'])));return
    if a.watch is not None:assert 0<a.watch<=180
    sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
    from elftools.elf.elffile import ELFFile
    from source_body_probe_20261111 import attach
    with axf.open('rb') as f:
        elf=ELFFile(f);objects={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
        segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
    assert objects['g_home_direction_diag'][1]==136
    keys='g_source_cycle_state g_source_cycle_fault g_source_cycle_request g_startup_axes_state g_startup_axes_cause g_extension_home_state g_extension_home_result g_eh_first_reason g_eh_first_command g_eh_first_exchange g_eh_first_flags g_eh_first_step g_safety_fault g_rtos_fault g_can_ctrl_fault_latched g_boot_ready_init_mask g_boot_ready_link_ok g_boot_ready_beep_started g_it_state g_it_request g_lift_ruler_state g_lift_ruler_cause g_dm_jog_state g_dm_jog_first_fault g_vm_test_request'.split()
    keys+='g_source_cycle_run g_source_cycle_item g_source_cycle_pose g_source_cycle_body_steps g_low_state g_low_phase g_low_fault g_low_slot g_low_color g_low_region g_low_accepted_sequence g_low_done_sequence g_low_grip_command_ms g_low_release_command_ms g_low_group_count g_source_parallel_state g_source_parallel_cause g_eh_first_tick g_eh_first_service_gap g_eh_first_phase g_eh_speed_sign g_eh_speed_rpm'.split()
    keys+='g_source_cycle_prep g_source_cycle_beep_started g_source_cycle_beep_done g_source_cycle_pick_beeps g_source_cycle_pick_beep_pending g_source_parallel_start_skew_ms g_source_parallel_sync_tick g_source_parallel_started_mask g_it_axes g_it_action g_dm_parallel_group_go g_dm_stage2_sync'.split()
    floats='g_startup_extension_current_deg g_startup_extension_target_deg g_extension_home_reference_deg g_dm_can2_logical_position_deg g_low_target_extension_mm g_low_target_yaw'.split()
    samples=[];out=a.manifest.parent/('live-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(when=datetime.datetime.now().isoformat(),image_sha256=im['sha256'],target_writes=[],reset=False,verified_flash_bytes=0,samples=samples)
    try:
        with attach() as session:
            t=session.target
            for address,data in segments:
                for offset in range(0,len(data),2048):
                    chunk=data[offset:offset+2048]
                    assert bytes(t.read_memory_block8(address+offset,len(chunk)))==chunk,'Board image mismatch; no writes performed'
                report['verified_flash_bytes']+=len(data)
            start=time.monotonic();last=None
            while True:
                s={n:t.read32(objects[n][0]) for n in keys if n in objects}
                for n in floats:
                    if n in objects:s[n]=struct.unpack('<f',struct.pack('<I',t.read32(objects[n][0])))[0]
                for n in ('g_low_group_phase','g_low_group_axes','g_low_group_started_ms','g_low_group_done_ms','g_source_cycle_pick_beep_ms','g_source_cycle_colors','g_source_cycle_piece_ms'):
                    if n in objects:s[n]=list(t.read_memory_block32(objects[n][0],min(8,objects[n][1]//4)))
                s['diagnostic']=decode(bytes(t.read_memory_block8(objects['g_home_direction_diag'][0],136)))
                if 'g_home_after_write_probe' in objects:
                    address,size=objects['g_home_after_write_probe'];assert size==44
                    raw=bytes(t.read_memory_block8(address,size))
                    s['after_write_probe']=dict(zip(('called','status_result','status_flags','read_result','read_valid','wait_ms'),struct.unpack('<6I',raw[:24])))
                    s['after_write_probe']['after']=list(raw[24:41])
                s['core_state']=t.get_state().name
                serial=json.dumps(s,sort_keys=True)
                if serial!=last:
                    s['elapsed_s']=round(time.monotonic()-start,3);samples.append(s);last=serial
                    out.write_text(json.dumps(report,indent=2)+'\n')
                    if a.compact:
                        names=('elapsed_s','g_source_cycle_state','g_source_cycle_fault','g_source_cycle_item','g_source_cycle_prep','g_it_axes','g_low_phase','g_low_fault','g_low_done_sequence','g_source_cycle_pick_beeps','g_startup_axes_cause','g_dm_jog_first_fault')
                        print(json.dumps({n:s[n] for n in names if n in s}),flush=True)
                    else:print(json.dumps(s),flush=True)
                if a.snapshot or s['diagnostic']['frozen'] or s.get('g_source_cycle_state')==9 or (a.watch is not None and time.monotonic()-start>=a.watch):break
                time.sleep(.3)
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);raise
    finally:
        out.write_text(json.dumps(report,indent=2)+'\n');print(str(out),flush=True)


if __name__=='__main__':main()
