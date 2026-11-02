"""Reviewed single servo1 0 -> 45 degree direction observation. No chassis commands."""
from pathlib import Path
import hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper,FileProgrammer
old=OUT.parent/'radar_servo_first_vehicle/firmware_vehicle.axf';new=OUT/'firmware_vehicle.axf'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='0287c747ac2a8dc9a68a9a0a1138d6b182089454173a52e35d385d712b5d1123'
sha='ce887b6b7382b15aec40e45cb7945537279efc362509d70c1bb010bff7dc2c9a'
assert hashlib.sha256(new.read_bytes()).hexdigest()==sha
oldsy,oldsegs=parse(old);sy,segs=parse(new)
probes=ConnectHelper.get_all_connected_probes(blocking=False);assert any(p.unique_id=='6afaae24' for p in probes)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in oldsegs),'Unexpected firmware: preserve state'
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 def health(symbols):return {n:t.read32(symbols[n]) for n in names}
 def healthy(symbols):return all(v==0 for v in health(symbols).values()) and not(t.read32(0x40021014)&4)
 r={'before':health(oldsy),'gpioe_odr_before':t.read32(0x40021014),'programmed':False,'servo_commands':[],'chassis_commands_sent':False}
 (OUT/'before.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 assert healthy(oldsy),'Active/fault: preserve state'
 assert t.read32(oldsy['g_radar_servo_test_request'])==0 and t.read32(oldsy['g_radar_servo_test_sequence'])==4,'Unexpected commissioning sequence'
 assert (t.read32(0x40000834),t.read32(0x40000838))==(500,500),'Servos not at expected initial command'
 t.reset_and_halt();FileProgrammer(session,smart_flash=False).program(str(new));r['programmed']=True;r['flash_bytes_verified']=0
 for addr,b in segs:
  assert bytes(t.read_memory_block8(addr,len(b)))==b,'Flash mismatch';r['flash_bytes_verified']+=len(b)
 t.reset();t.flush();time.sleep(2);r['image_sha256']=sha;r['after_download']=health(sy)
 (OUT/'download.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 assert healthy(sy) and t.read32(sy['g_radar_servo_state'])==1,'New idle not healthy'
 assert t.read32(sy['g_radar_servo_test_sequence'])==0 and t.read32(sy['g_radar_servo_test_request'])==0
 assert (t.read32(0x40000834),t.read32(0x40000838))==(500,500),'Unexpected boot pulse'
 t.write32(sy['g_radar_servo_test_id'],1)
 t.write32(sy['g_radar_servo_test_angle_deg'],struct.unpack('<I',struct.pack('<f',45.0))[0])
 t.write32(sy['g_radar_servo_test_request'],1);t.flush()
 deadline=time.monotonic()+1
 while t.read32(sy['g_radar_servo_test_sequence'])==0 and time.monotonic()<deadline:time.sleep(.02)
 r['result']=t.read32(sy['g_radar_servo_test_result']);r['sequence']=t.read32(sy['g_radar_servo_test_sequence'])
 r['servo_commands']=[{'id':1,'from_deg':0,'target_deg':45}];r['after']=health(sy);r['gpioe_odr_after']=t.read32(0x40021014)
 r['command_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
 r['pulse_us']=[t.read32(0x40000834),t.read32(0x40000838)]
 (OUT/'direction_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert r['result']==1 and r['sequence']==1 and r['command_angles_deg']==[45,0] and r['pulse_us']==[1000,500] and healthy(sy),'Command verification failed'
