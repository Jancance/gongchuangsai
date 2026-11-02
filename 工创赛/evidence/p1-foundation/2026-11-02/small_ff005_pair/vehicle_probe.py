"""Default inspect only. Flash A requires explicit --action flash-a after field readiness."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'工创赛/tmp/turn_probe_deps'))
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from elftools.elf.elffile import ELFFile

OUT=Path(__file__).resolve().parent
IMAGES=[('old_tm9',ROOT/'工创赛/tmp/x_perimeter_comp008_20261101/ObjectsVehicle/gongxun-f407.axf','6c8284e52e54bc6bda859a827a7b667ba491574fc889338edbae979f70353098'),
        ('A',OUT/'firmware/Objects_baseline_vehicle/gongxun-f407.axf','f8462206ca1fcdb22b5e18bd9980d3dc8dba11622fb3ae70b082fa816a0e3630'),
        ('B',OUT/'firmware/Objects_ff005_vehicle/gongxun-f407.axf','b36d68f26118381868ee9d56e99c2915dae13791f6691e58288b6bd8b5324304'),
        ('N',OUT.parent/'no_heading_once/firmware/Objects_no_heading_vehicle/gongxun-f407.axf','471472a73ee9340b4d94d407bcaefd51930a87b396a322ac4776cbdfc439b6b3')]

def parse(path):
    with path.open('rb') as f:
        elf=ELFFile(f)
        sy={s.name:int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols()}
        segs=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments()
              if s['p_type']=='PT_LOAD' and 0x08000000<=s['p_paddr']<0x08080000 and s['p_filesz']]
    return sy,segs

def main():
    p=argparse.ArgumentParser();p.add_argument('--action',choices=['inspect','flash-a','flash-b','restart-a'],default='inspect');p.add_argument('--probe-id');p.add_argument('--replace-unidentified',action='store_true');p.add_argument('--record-name');a=p.parse_args()
    if a.record_name:
        assert Path(a.record_name).name==a.record_name and a.record_name.endswith('.json'),'Record must be a JSON basename'
    ev=OUT/'vehicle';ev.mkdir(exist_ok=True)
    report=dict(action=a.action,reset_called=False,halt_called=False,programmed=False,motion_commands_sent=False)
    images=[]
    for tag,path,sha in IMAGES:
        assert hashlib.sha256(path.read_bytes()).hexdigest()==sha,'Image changed: '+tag
        sy,segs=parse(path);images.append((tag,path,sha,sy,segs))
    probes=ConnectHelper.get_all_connected_probes(blocking=False)
    if a.probe_id:
        probes=[probe for probe in probes if probe.unique_id==a.probe_id]
    assert len(probes)==1,'Expected one connected debug probe'
    report['probe_id']=probes[0].unique_id
    pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
    session=ConnectHelper.session_with_chosen_probe(unique_id=probes[0].unique_id,blocking=False,
        target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,
        'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
    assert session is not None,'Probe unavailable'
    with session:
        t=session.target;report['core_state']=str(t.get_state())
        matched=None
        for row in images:
            if all(bytes(t.read_memory_block8(addr,len(blob)))==blob for addr,blob in row[4]):matched=row;break
        report['matched_image']=matched[0] if matched else None
        report['gpioe_odr']=t.read32(0x40021014)
        if matched:
            sy=matched[3]
            names=['g_vm_test_request','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
                   'g_chassis_auto_active','g_reset_iwdg','g_reset_flags','g_chassis_auto_cause','g_chassis_auto_result']
            report['ram']={n:t.read32(sy[n]) for n in names}
        (ev/('inspect_'+str(time.time_ns())+'.json')).write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps(report,indent=2),flush=True)
        if a.action=='inspect':return
        assert not (report['gpioe_odr']&4),'VM active'
        if matched is None:
            assert a.action=='flash-a','B download requires verified A firmware'
            assert a.replace_unidentified,'Unknown firmware: preserve state, no download'
            # Explicit user instruction to replace this firmware. Preserve its
            # Flash/RAM before reset; do not interpret unknown symbols as healthy.
            report['prior_health']='unknown; raw state preserved before replacement'
            report['rcc_csr_before']=t.read32(0x40023874)
            stamp=str(time.time_ns());report['backups']={}
            for label,addr,size in [('flash',0x08000000,512*1024),('sram',0x20000000,128*1024),('ccm',0x10000000,64*1024)]:
                blob=bytes(t.read_memory_block8(addr,size));dest=ev/('before_a_'+stamp+'_'+label+'.bin');dest.write_bytes(blob)
                report['backups'][label]={'file':str(dest),'sha256':hashlib.sha256(blob).hexdigest(),'bytes':len(blob)}
            (ev/('replacement_before_'+stamp+'.json')).write_text(json.dumps(report,indent=2),encoding='utf8')
            assert not(t.read32(0x40021014)&4),'VM became active'
        else:
            health=report['ram']
            assert health['g_vm_test_request']==0 and health['g_chassis_auto_active']==0,'Motion active'
            assert all(health[n]==0 for n in ['g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg']),'Fault/reset present: preserve state'
        if a.action=='restart-a':
            assert matched is not None and matched[0]=='A','Restart requires verified A'
            assert report['ram']['g_chassis_auto_cause']==0,'Backend cause present: preserve state'
            t.reset();t.flush();report['reset_called']=True;time.sleep(2)
            report['after']={n:t.read32(matched[3][n]) for n in ['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg']}
            report['gpioe_odr_after']=t.read32(0x40021014)
            (ev/(a.record_name or ('restart_a_'+str(time.time_ns())+'.json'))).write_text(json.dumps(report,indent=2),encoding='utf8')
            print(json.dumps(report,indent=2))
            assert not(report['gpioe_odr_after']&4) and all(v==0 for v in report['after'].values()),'Restarted idle not healthy'
            return
        row=images[2] if a.action=='flash-b' else images[1]
        if a.action=='flash-b':
            assert matched is not None and matched[0]=='A','B requires verified A; no automatic reflash'
        else:
            assert matched is None or matched[0] in ('old_tm9','B','N'),'A already installed: no automatic reset or reflash'
        t.reset_and_halt();report['reset_called']=report['halt_called']=True
        FileProgrammer(session,smart_flash=False).program(str(row[1]));report['programmed']=True
        total=0
        for addr,blob in row[4]:
            assert bytes(t.read_memory_block8(addr,len(blob)))==blob,'Flash readback mismatch';total+=len(blob)
        t.reset();t.flush();time.sleep(2)
        report['new_image_sha256']=row[2];report['flash_bytes_verified']=total
        report['after']={n:t.read32(row[3][n]) for n in ['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg']}
        report['gpioe_odr_after']=t.read32(0x40021014)
        (ev/(a.record_name or ('download_b.json' if a.action=='flash-b' else 'download_a.json'))).write_text(json.dumps(report,indent=2),encoding='utf8')
        print(json.dumps(report,indent=2))
        assert not(report['gpioe_odr_after']&4) and all(v==0 for v in report['after'].values()),'New idle not healthy'

if __name__=='__main__':main()
