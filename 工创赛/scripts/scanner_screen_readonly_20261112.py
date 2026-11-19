"""Read scanner/display state without enabling, resetting or moving anything."""
from pathlib import Path
from datetime import datetime
import hashlib,json
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

R=Path(__file__).resolve().parents[1]
IMAGE=R/'tmp/source_body_hold_20261111/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'

def main():
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest()=='9e6d8b3972eda04937b054e128ae9f69b1ae4076de37b51e14ac24d2825ed5d4'
    live.IMAGE=IMAGE
    _,segments=live.load_image()
    with IMAGE.open('rb') as f:
        elf=live.ELFFile(f)
        symbols={s.name:int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
    names='''g_screen_init_done g_screen_command_count g_scan_display_result g_scan_display_count
g_saoma_rx_count g_saoma_rx_errors g_saoma_accept_count g_saoma_reject_count g_saoma_last_result g_saoma_locked
g_vis_task_valid g_vis_task_batch g_vis_task_item g_safety_fault g_rtos_fault g_vm_test_request
g_start_request g_vehicle_state g_vehicle_stage g_vehicle_run_id'''.split()
    out=R/'evidence/integration/2026-11-11/source-body-hold'/('scanner-screen-readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
    report=dict(read_only=True,reset=False,motion_commands=0,complete=False,
                supplied_task_code='215+213+512+321',task_code_source='user text, not scanner receipt')
    try:
        with attach() as session:
            t=session.target;report['core_state']=t.get_state().name
            compared=0
            for address,data in segments:
                for i in range(0,len(data),4096):
                    chunk=data[i:i+4096]
                    assert bytes(t.read_memory_block8(address+i,len(chunk)))==chunk,'image mismatch'
                    compared+=len(chunk)
            report['flash_bytes_verified']=compared
            report['values']={k:t.read32(symbols[k]) for k in names if k in symbols}
            for k in ['SAOMA_Data','s_vehicle_task_code','vis_task_code']:
                if k in symbols:report[k]=bytes(t.read_memory_block8(symbols[k],16)).hex()
            if 'SAOMA_DataReady' in symbols:report['SAOMA_DataReady']=t.read8(symbols['SAOMA_DataReady'])
            # Read CR1/BRR only; never consume scanner DR or trigger transmission.
            report['UART5_CR1']=t.read32(0x4000500c);report['UART5_BRR']=t.read32(0x40005008)
            report['UART4_CR1']=t.read32(0x40004c0c);report['UART4_BRR']=t.read32(0x40004c08)
            report['GPIOD_MODER']=t.read32(0x40020c00)
            report['GPIOD_AFRL']=t.read32(0x40020c20)
            report['GPIOD_IDR']=t.read32(0x40020c10)
            report['PD2_mode']=(report['GPIOD_MODER']>>4)&3
            report['PD2_AF']=(report['GPIOD_AFRL']>>8)&15
            report['NVIC_ISER1']=t.read32(0xe000e104)
            report['UART5_IRQ_enabled']=bool(report['NVIC_ISER1']&(1<<21))
            report['UART5_vector']=t.read32(0x08000000+4*(16+53))
            report['PE2']=(t.read32(0x40021010)>>2)&1
            report['complete']=True
    except BaseException as e:
        report['error']=repr(e)
        raise
    finally:
        out.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2));print(out)

if __name__=='__main__':main()
