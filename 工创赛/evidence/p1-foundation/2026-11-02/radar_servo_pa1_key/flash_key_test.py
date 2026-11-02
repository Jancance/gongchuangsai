"""Install key test, verify idle boot, then arm only the PC13 servo test."""
from pathlib import Path
import hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper,FileProgrammer
old=OUT.parent/'radar_servo_boot_profile/firmware_vehicle.axf';new=OUT/'firmware_vehicle.axf'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='0bbad57ffc261c36e268d52dba6b7cc42d70e352252d9baefcfff7e17a52889a'
sha='5cf9f977e96525867d77ad64cbe312ea866f12fde613c58f605fea3c87adab39'
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
 r={'before':health(oldsy),'programmed':False,'chassis_start_sent':False,'manual_angle_request_sent':False}
 (OUT/'before_download.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 assert all(v==0 for v in r['before'].values()) and not(t.read32(0x40021014)&4),'Active/fault state'
 assert t.read32(oldsy['g_radar_servo_test_request'])==0,'Manual request pending'
 t.reset_and_halt();FileProgrammer(session,smart_flash=False).program(str(new));r['programmed']=True;r['flash_bytes_verified']=0
 for addr,b in segs:
  assert bytes(t.read_memory_block8(addr,len(b)))==b,'Flash mismatch';r['flash_bytes_verified']+=len(b)
 t.reset();t.flush();time.sleep(2)
 r['image_sha256']=sha;r['after']=health(sy);r['VM_gpio']=bool(t.read32(0x40021014)&4)
 r['command_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
 r['pulse_us']=[t.read32(0x40000834),t.read32(0x40000838)]
 r['departure_done']=t.read32(sy['g_radar_servo_departure_done']);r['departure_id']=t.read32(sy['g_radar_servo_departure_id'])
 r['manual_sequence']=t.read32(sy['g_radar_servo_test_sequence'])
 (OUT/'download_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert all(v==0 for v in r['after'].values()) and not r['VM_gpio']
 assert r['command_angles_deg']==[0,180] and r['pulse_us']==[500,2500] and r['departure_done']==0 and r['departure_id']==0 and r['manual_sequence']==0,'Boot profile verification failed'

 assert t.read32(sy['g_radar_servo_key_test_enable'])==0
 assert t.read32(0x40020810)&(1<<13),'Release PC13 before arming'
 assert t.read32(sy['g_radar_zone_ready'])==1
 t.write32(sy['g_radar_servo_key_test_enable'],1);t.flush()
 r['key_test_enabled']=t.read32(sy['g_radar_servo_key_test_enable'])
 r['zone']=t.read32(sy['g_radar_zone_selected'])
 r['key_sequence']=t.read32(sy['g_radar_servo_key_sequence'])
 assert r['key_test_enabled']==1 and r['key_sequence']==0
 (OUT/'download_result.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 print(json.dumps({'key_test_enabled':r['key_test_enabled'],'zone':r['zone'],'key_sequence':r['key_sequence']}))
