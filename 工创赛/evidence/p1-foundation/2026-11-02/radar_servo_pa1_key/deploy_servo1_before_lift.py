"""Deploy only servo1 to 180 before lift preparation; servo2 remains parked; no reset, flash or chassis start."""
from pathlib import Path
import hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper
image=OUT/'firmware_vehicle.axf'
assert hashlib.sha256(image.read_bytes()).hexdigest()=='5cf9f977e96525867d77ad64cbe312ea866f12fde613c58f605fea3c87adab39'
sy,segs=parse(image)
probes=ConnectHelper.get_all_connected_probes(blocking=False);assert any(p.unique_id=='6afaae24' for p in probes)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in segs),'Unexpected image: no command'
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 def health():return {n:t.read32(sy[n]) for n in names}
 assert all(v==0 for v in health().values()) and not(t.read32(0x40021014)&4),'Active/fault state'
 assert t.read32(sy['g_radar_servo_test_request'])==0,'Manual request pending'
 t.write32(sy['g_radar_servo_key_test_enable'],0);t.flush()
 assert [t.read32(0x40000834),t.read32(0x40000838)]==[500,2500],'Not at expected 0/180 initial commands'
 seq=t.read32(sy['g_radar_servo_test_sequence'])
 r={'before_pulse_us':[500,2500],'reset':False,'download':False,'chassis_start':False,'commands':[{'servo_id':1,'from_deg':0,'target_deg':180}]}
 t.write32(sy['g_radar_servo_test_id'],1);t.write32(sy['g_radar_servo_test_angle_deg'],struct.unpack('<I',struct.pack('<f',180.0))[0])
 t.write32(sy['g_radar_servo_test_request'],1);t.flush();deadline=time.monotonic()+1
 while t.read32(sy['g_radar_servo_test_sequence'])==seq and time.monotonic()<deadline:time.sleep(.02)
 r['result']=t.read32(sy['g_radar_servo_test_result']);r['sequence']=t.read32(sy['g_radar_servo_test_sequence'])
 r['pulse_us']=[t.read32(0x40000834),t.read32(0x40000838)]
 r['command_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
 r['after_health']=health();r['departure_done']=t.read32(sy['g_radar_servo_departure_done'])
 r['key_test_enabled']=t.read32(sy['g_radar_servo_key_test_enable'])
 assert r['key_test_enabled']==0
 (OUT/'deploy_servo1_before_lift_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert r['result']==1 and r['sequence']==seq+1 and r['pulse_us']==[2500,2500] and all(v==0 for v in r['after_health'].values()),'Command verification failed'
