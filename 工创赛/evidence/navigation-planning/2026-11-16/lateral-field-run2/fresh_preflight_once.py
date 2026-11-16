from pathlib import Path
import datetime,hashlib,json,sys
from run_binding import E,R,S,IMAGE,EXPECTED,validate,start
binding=validate('fresh_preflight')
sys.path.insert(0,str(R/'工创赛/tmp/turn_probe_deps'));sys.path.insert(0,str(R/'工创赛/scripts'));sys.path.insert(0,str(S))
from elftools.elf.elffile import ELFFile
from read_capture_once import inspect_elf
from decode_capture import inspect_header
from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
from pyocd.target.pack.pack_target import PackTargets
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
symbols,flash=inspect_elf(IMAGE)
names=['g_chassis_auto_active','g_chassis_auto_session','g_vm_test_request','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_debug_plan_id','g_chassis_auto_state','g_chassis_auto_result','g_chassis_auto_cause','s_distance_fault_reason']
with IMAGE.open('rb') as f:
 elf=ELFFile(f);states={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s.name in names}
assert set(states)==set(names)
start('fresh_preflight')
receipt=dict(candidate_sha256=EXPECTED,probe_uid=binding['probe_uid'],accepted=False,reset_requested=False,halt_requested=False,resume_requested=False,target_memory_writes=0,flash_programmed=False,automatic_retry=False,flash_checks=[])
session=None
try:
 PackTargets.populate_targets_from_pack('D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8')
 probes=ConnectHelper.get_all_connected_probes(blocking=False)
 receipt['probe_uids']=[p.unique_id for p in probes]
 assert len(probes)==1 and probes[0].unique_id.lower()==binding['probe_uid'].lower()
 probe=probes[0];synchronous_interface(probe._link._interface)
 session=Session(probe,options={'target_override':'stm32f407ve','connect_mode':'attach','frequency':1000000,'auto_unlock':False,'resume_on_disconnect':False,'no_config':True,'dap_protocol':'swd','cmsis_dap.prefer_v1':False,'cmsis_dap.limit_packets':True})
 session.open();t=session.target
 receipt['core_before']=t.get_state().name;assert receipt['core_before']=='RUNNING'
 receipt['flash_identity_policy']='prior run1 full Flash match + same current candidate/source SHA; no Flash read in run2'
 receipt['states']={}
 for n,(a,z) in states.items():
  assert z in (1,2,4) and ((0x10000000<=a<a+z<=0x10010000) or (0x20000000<=a<a+z<=0x20020000))
  receipt['states'][n]=t.read8(a) if z==1 else t.read16(a) if z==2 else t.read32(a)
 receipt['pe2']=(t.read32(0x40021010)>>2)&1
 address,_=symbols['g_fusion_capture_a'];header=bytes(t.read_memory_block8(address,256))
 with (E/'fresh_capture_header.bin').open('xb') as f:f.write(header)
 receipt['capture_header']=inspect_header(header)
 receipt['core_after']=t.get_state().name;assert receipt['core_after']=='RUNNING'
 assert receipt['states']['g_chassis_auto_active']==0
 receipt['accepted']=True
except Exception as exc:
 receipt['error']=repr(exc)
 raise
finally:
 if session:
  try:session.close()
  except Exception as exc:receipt['disconnect_error']=repr(exc)
 receipt['finished_at']=datetime.datetime.now().astimezone().isoformat()
 with (E/'fresh_preflight_receipt.json').open('x',encoding='utf8') as f:json.dump(receipt,f,ensure_ascii=False,indent=2)
 print(json.dumps(receipt,ensure_ascii=False))
