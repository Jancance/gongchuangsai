"""Exactly one bounded 64-half-step direction probe, first rig only."""
from pathlib import Path
import json,hashlib,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper
image=OUT/'firmware_vehicle.axf';assert hashlib.sha256(image.read_bytes()).hexdigest()=='caec4a01193fdb29860544d88162065ad5df50609671fbcec237fed45f452b9b'
sy,segs=parse(image)
pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
assert session is not None
with session:
 t=session.target
 assert all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in segs),'Unexpected image'
 names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
 def health():return {n:t.read32(sy[n]) for n in names}
 assert all(v==0 for v in health().values()) and not(t.read32(0x40021014)&4)
 assert [t.read32(0x40000834),t.read32(0x40000838)]==[2500,2500],'Servo1 not deployed'
 assert t.read32(sy['g_radar_servo_key_test_enable'])==0
 assert t.read32(sy['g_radar_lift_state'])==2 and t.read32(sy['g_radar_lift_done'])==32
 assert t.read32(sy['g_radar_lift_sequence'])==1 and t.read32(sy['g_radar_lift_request'])==0,'Already tested; do not repeat'
 r={'steps_requested':64,'interval_ms':4,'direction_code':0,'commands_sent':1,'chassis_start':False,'before_health':health()}
 (OUT/'before_second64.json').write_text(json.dumps(r,indent=2),encoding='utf8')
 t.write32(sy['g_radar_lift_steps'],64);t.write32(sy['g_radar_lift_direction'],0)
 t.write32(sy['g_radar_lift_enable'],1);t.write32(sy['g_radar_lift_request'],1);t.flush()
 deadline=time.monotonic()+1.5
 while time.monotonic()<deadline:
  state=t.read32(sy['g_radar_lift_state'])
  if state in (2,3):break
  time.sleep(.02)
 # Disarm without releasing the final phase. No automatic reverse or homing.
 t.write32(sy['g_radar_lift_enable'],0);t.flush();time.sleep(.05)
 for n in ['state','result','sequence','done','phase_bits','enable']:
  r[n]=t.read32(sy['g_radar_lift_'+n])
 r['after_health']=health();r['VM_gpio']=bool(t.read32(0x40021014)&4)
 r['PD_ODR_mask']=t.read32(0x40020c14)&0x1c00;r['PB_ODR_mask']=t.read32(0x40020414)&0x100
 r['servo_pwm_us']=[t.read32(0x40000834),t.read32(0x40000838)]
 (OUT/'second64_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert r['state']==2 and r['result']==1 and r['sequence']==2 and r['done']==64 and r['phase_bits']==9
 assert r['enable']==0 and r['PD_ODR_mask']==0x400 and r['PB_ODR_mask']==0x100
 assert all(v==0 for v in r['after_health'].values()) and not r['VM_gpio'] and r['servo_pwm_us']==[2500,2500]
