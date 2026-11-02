"""Read-only diagnosis after no visible physical movement; preserve all state."""
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

 r={'read_only':True,'health':health(),'servo_pwm_us':[t.read32(0x40000834),t.read32(0x40000838)]}
 for n in ['state','result','sequence','done','phase_bits','enable','request']:
  r[n]=t.read32(sy['g_radar_lift_'+n])
 for label,base,pins in [('PB',0x40020400,[8]),('PD',0x40020c00,[10,11,12])]:
  regs={n:t.read32(base+o) for n,o in [('MODER',0),('OTYPER',4),('PUPDR',12),('IDR',16),('ODR',20)]}
  r[label]={'registers':regs,'pins':{str(p):{'mode':(regs['MODER']>>(2*p))&3,'open_drain':(regs['OTYPER']>>p)&1,'input':(regs['IDR']>>p)&1,'output':(regs['ODR']>>p)&1} for p in pins}}
 r['VM_gpio']=bool(t.read32(0x40021014)&4)
 (OUT/'no_visible_move_readonly.json').write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
