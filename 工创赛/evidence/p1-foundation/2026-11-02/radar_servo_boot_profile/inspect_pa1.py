"""Read PA1, debounced zone, lock and servo commands. No writes/reset/download."""
from pathlib import Path
import argparse,hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper
p=argparse.ArgumentParser();p.add_argument('--label',default='initial');a=p.parse_args()
assert a.label in ('initial','zone1','zone2')
image=OUT/'firmware_vehicle.axf'
assert hashlib.sha256(image.read_bytes()).hexdigest()=='0bbad57ffc261c36e268d52dba6b7cc42d70e352252d9baefcfff7e17a52889a'
sy,segs=parse(image)
probes=ConnectHelper.get_all_connected_probes(blocking=False);assert any(p.unique_id=='6afaae24' for p in probes)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in segs),'Unexpected firmware'
 samples=[]
 for _ in range(5):
  r={'PA1_level':(t.read32(0x40020010)>>1)&1}
  for n in ['g_radar_zone_ready','g_radar_zone_selected','g_radar_zone_locked','g_radar_servo_departure_done','g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched']:
   r[n]=t.read32(sy[n])
  r['servo_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
  r['PWM_us']=[t.read32(0x40000834),t.read32(0x40000838)]
  samples.append(r);time.sleep(.1)
 report={'operator_label':a.label,'read_only':True,'reset':False,'download':False,'commands_sent':False,'samples':samples,'stable':all(x==samples[0] for x in samples)}
 (OUT/('pa1_'+a.label+'_'+str(time.time_ns())+'.json')).write_text(json.dumps(report,indent=2),encoding='utf8')
 print(json.dumps({'label':a.label,'stable':report['stable'],'last':samples[-1],'commands_sent':False},indent=2))
