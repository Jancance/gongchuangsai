"""Download the reviewed no-heading-output trial; no motion commands."""
from pathlib import Path
import hashlib, json, sys, time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT, parse, ConnectHelper, FileProgrammer

old=OUT.parent/'small_ff005_pair/firmware/Objects_baseline_vehicle/gongxun-f407.axf'
new=OUT/'firmware/Objects_no_heading_vehicle/gongxun-f407.axf'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='f8462206ca1fcdb22b5e18bd9980d3dc8dba11622fb3ae70b082fa816a0e3630'
assert hashlib.sha256(new.read_bytes()).hexdigest()=='471472a73ee9340b4d94d407bcaefd51930a87b396a322ac4776cbdfc439b6b3'
oldsy,oldsegs=parse(old);newsy,newsegs=parse(new)
ev=OUT/'vehicle';ev.mkdir(exist_ok=True)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(blob)))==blob for addr,blob in oldsegs),'Not reviewed A: preserve state'
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 report={'before':{n:t.read32(oldsy[n]) for n in names},'gpioe_odr_before':t.read32(0x40021014),'old_image':'A','new_image':'N: forward distance heading output zero; FF off','motion_commands_sent':False,'programmed':False}
 (ev/'before_download.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 assert not(report['gpioe_odr_before']&4) and all(v==0 for v in report['before'].values()),'Active/fault: preserve state'
 t.reset_and_halt();FileProgrammer(session,smart_flash=False).program(str(new))
 report['programmed']=True;report['flash_bytes_verified']=0
 for addr,blob in newsegs:
  assert bytes(t.read_memory_block8(addr,len(blob)))==blob,'Readback mismatch'
  report['flash_bytes_verified']+=len(blob)
 t.reset();t.flush();time.sleep(2)
 report['sha256']=hashlib.sha256(new.read_bytes()).hexdigest()
 report['after']={n:t.read32(newsy[n]) for n in names};report['gpioe_odr_after']=t.read32(0x40021014)
 (ev/'download.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 print(json.dumps(report,indent=2))
 assert not(report['gpioe_odr_after']&4) and all(v==0 for v in report['after'].values()),'New idle not healthy'
