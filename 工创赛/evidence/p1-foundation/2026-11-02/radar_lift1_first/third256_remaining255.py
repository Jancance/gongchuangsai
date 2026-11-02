"""User-requested total 256-half-step probe in four guarded 64-step segments."""
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
 assert t.read32(sy['g_radar_lift_state'])==3 and t.read32(sy['g_radar_lift_done'])==1
 assert t.read32(sy['g_radar_lift_sequence'])==3 and t.read32(sy['g_radar_lift_request'])==0,'Already tested; do not repeat'

 r={'remaining_half_steps_requested':255,'previous_aborted_output_half_steps':1,'segments':[],'interval_ms':4,'direction':0,'chassis_start':False,'before_health':health()}
 assert t.read32(sy['g_radar_lift_enable'])==0
 try:
  for index,count in enumerate([64,64,64,63]):
   assert all(v==0 for v in health().values()) and not(t.read32(0x40021014)&4),'Active/fault state; stop'
   assert [t.read32(0x40000834),t.read32(0x40000838)]==[2500,2500]
   assert t.read32(sy['g_radar_lift_sequence'])==3+index and t.read32(sy['g_radar_lift_request'])==0
   assert t.read32(sy['g_radar_lift_state'])==(3 if index==0 else 2)
   t.write32(sy['g_radar_lift_steps'],count);t.write32(sy['g_radar_lift_direction'],0)
   t.write32(sy['g_radar_lift_enable'],1);t.write32(sy['g_radar_lift_request'],1);t.flush()
   deadline=time.monotonic()+2.0;stable=0
   while time.monotonic()<deadline:
    # Read transaction sequence before state; require two complete samples.
    seq=t.read32(sy['g_radar_lift_sequence'])
    state=t.read32(sy['g_radar_lift_state']);done=t.read32(sy['g_radar_lift_done'])
    if seq==4+index and state==2 and done==count:stable+=1
    else:stable=0
    if stable>=2:break
    time.sleep(.02)
   segment={n:t.read32(sy['g_radar_lift_'+n]) for n in ['state','result','sequence','done','phase_bits']}
   r['segments'].append(segment)
   (OUT/'third256_remaining255_result.json').write_text(json.dumps(r,indent=2),encoding='utf8')
   assert segment['state']==2 and segment['result']==1 and segment['sequence']==4+index and segment['done']==count,'Incomplete segment; stop'
 finally:
  t.write32(sy['g_radar_lift_enable'],0);t.flush();time.sleep(.05)
  r['enable']=t.read32(sy['g_radar_lift_enable']);r['after_health']=health()
  r['VM_gpio']=bool(t.read32(0x40021014)&4)
  r['servo_pwm_us']=[t.read32(0x40000834),t.read32(0x40000838)]
  r['half_steps_completed_commands']=sum(seg['done'] for seg in r['segments'])
  (OUT/'third256_remaining255_result.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
 assert r['enable']==0 and r['half_steps_completed_commands']==255
 assert all(v==0 for v in r['after_health'].values()) and not r['VM_gpio']
