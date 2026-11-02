"""Only command servo2 to 0 degrees; leave servo1 unchanged, no flash/reset."""
from pathlib import Path
import hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper
image=OUT/'firmware_vehicle.axf'
assert hashlib.sha256(image.read_bytes()).hexdigest()=='bf6fccb26966639f5b24a5eb142c99cddd47109f76bdd9485752954db3b6cd4f'
sy,segs=parse(image)
probes=ConnectHelper.get_all_connected_probes(blocking=False);assert any(p.unique_id=='6afaae24' for p in probes)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in segs),'Unexpected firmware: no command'
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 def health():return {n:t.read32(sy[n]) for n in names}
 assert all(v==0 for v in health().values()) and not(t.read32(0x40021014)&4),'Active/fault state'
 assert t.read32(sy['g_radar_servo_state'])==1 and t.read32(sy['g_radar_servo_test_request'])==0,'Servo not idle'
 before=[t.read32(0x40000834),t.read32(0x40000838)]
 r={'before_pulse_us':before,'commands':[],'reset':False,'download':False,'chassis_commands':False}
 if before[1]!=500:
  seq=t.read32(sy['g_radar_servo_test_sequence'])
  t.write32(sy['g_radar_servo_test_id'],2);t.write32(sy['g_radar_servo_test_angle_deg'],struct.unpack('<I',struct.pack('<f',0.0))[0])
  t.write32(sy['g_radar_servo_test_request'],1);t.flush();deadline=time.monotonic()+1
  while t.read32(sy['g_radar_servo_test_sequence'])==seq and time.monotonic()<deadline:time.sleep(.02)
  assert t.read32(sy['g_radar_servo_test_sequence'])==seq+1 and t.read32(sy['g_radar_servo_test_result'])==1,'Command not accepted'
  r['commands']=[{'servo_id':2,'target_deg':0}]
 r['after_pulse_us']=[t.read32(0x40000834),t.read32(0x40000838)]
 r['command_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
 r['after_health']=health()
 (OUT/'servo2_zero_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert r['after_pulse_us']==[before[0],500] and all(v==0 for v in r['after_health'].values()),'Channel verification failed'
