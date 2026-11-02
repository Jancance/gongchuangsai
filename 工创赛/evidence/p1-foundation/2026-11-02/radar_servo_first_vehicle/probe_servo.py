"""Exact-image guarded first-servo commissioning. Never sends chassis commands."""
from pathlib import Path
import argparse,hashlib,json,struct,sys,time
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(OUT.parent/'small_ff005_pair'))
from vehicle_probe import ROOT,parse,ConnectHelper,FileProgrammer
A=OUT.parent/'small_ff005_pair/firmware/Objects_baseline_vehicle/gongxun-f407.axf'
S=OUT/'firmware_vehicle.axf'
SHA_A='f8462206ca1fcdb22b5e18bd9980d3dc8dba11622fb3ae70b082fa816a0e3630'
SHA_S='0287c747ac2a8dc9a68a9a0a1138d6b182089454173a52e35d385d712b5d1123'
def main():
 p=argparse.ArgumentParser();p.add_argument('--action',choices=['inspect','flash','step1','step2','park'],default='inspect');a=p.parse_args()
 assert hashlib.sha256(A.read_bytes()).hexdigest()==SHA_A
 assert hashlib.sha256(S.read_bytes()).hexdigest()==SHA_S
 images=[('A',*parse(A)),('S',*parse(S))]
 pack=ROOT/'工创赛/evidence/p1-foundation/2026-10-31/motor_menu_power/local_STM32F4xx_DFP.pack'
 probes=ConnectHelper.get_all_connected_probes(blocking=False)
 assert any(p.unique_id=='6afaae24' for p in probes),'Known vehicle probe not present'
 session=ConnectHelper.session_with_chosen_probe(unique_id='6afaae24',blocking=False,target_override='stm32f407ve',no_config=True,options={'pack':str(pack),'frequency':1000000,'resume_on_disconnect':False,'connect_mode':'attach','auto_unlock':False,'cache.enable_memory':False})
 assert session is not None
 with session:
  t=session.target;matched=None
  for tag,sy,segs in images:
   if all(bytes(t.read_memory_block8(addr,len(b)))==b for addr,b in segs):matched=(tag,sy,segs);break
  assert matched is not None,'Unknown firmware: preserve state'
  tag,sy,segs=matched
  names=['g_vm_test_request','g_chassis_auto_active','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched','g_reset_iwdg','g_chassis_auto_cause']
  def health(symbols):return {n:t.read32(symbols[n]) for n in names}
  r={'action':a.action,'matched_image':tag,'before':health(sy),'gpioe_odr_before':t.read32(0x40021014),'chassis_commands_sent':False,'programmed':False,'reset_called':False}
  (OUT/('before_'+a.action+'_'+str(time.time_ns())+'.json')).write_text(json.dumps(r,indent=2),encoding='utf8')
  assert all(v==0 for v in r['before'].values()) and not(r['gpioe_odr_before']&4),'Active/fault: preserve state'
  if a.action=='flash':
   assert tag=='A','Already commissioned or unexpected source: no automatic reset'
   t.reset_and_halt();r['reset_called']=True;FileProgrammer(session,smart_flash=False).program(str(S));r['programmed']=True
   r['flash_bytes_verified']=0
   for addr,b in images[1][2]:
    assert bytes(t.read_memory_block8(addr,len(b)))==b,'Readback mismatch';r['flash_bytes_verified']+=len(b)
   t.reset();t.flush();time.sleep(2);sy=images[1][1];r['new_sha256']=SHA_S
  if a.action in ('step1','step2','park'):
   assert tag=='S' and t.read32(sy['g_radar_servo_state'])==1,'Servo image/state not ready'
   assert t.read32(sy['g_radar_servo_test_request'])==0,'Pending request'
   seq=t.read32(sy['g_radar_servo_test_sequence'])
   assert seq=={'step1':0,'step2':1,'park':3}[a.action],'Unexpected test sequence: do not repeat'
   expected={'step1':(500,500),'step2':(611,500),'park':(500,611)}[a.action]
   assert (t.read32(0x40000834),t.read32(0x40000838))==expected,'Unexpected initial pulse'
   r['issued_servo_commands']=[]
   commands={'step1':[(1,10.0)],'step2':[(1,0.0),(2,10.0)],'park':[(2,0.0)]}[a.action]
   for servo_id,angle in commands:
    assert all(v==0 for v in health(sy).values()) and not(t.read32(0x40021014)&4),'State changed: no next command'
    t.write32(sy['g_radar_servo_test_id'],servo_id)
    t.write32(sy['g_radar_servo_test_angle_deg'],struct.unpack('<I',struct.pack('<f',angle))[0])
    t.write32(sy['g_radar_servo_test_request'],1);t.flush()
    deadline=time.monotonic()+1
    while t.read32(sy['g_radar_servo_test_sequence'])==seq and time.monotonic()<deadline:time.sleep(.02)
    r['test_result']=t.read32(sy['g_radar_servo_test_result']);r['test_sequence']=t.read32(sy['g_radar_servo_test_sequence'])
    assert r['test_result']==1 and r['test_sequence']==seq+1,'Mailbox rejected or no completion: preserve state'
    seq+=1;r['issued_servo_commands'].append({'id':servo_id,'angle_deg':angle});time.sleep(.3)
  r['after']=health(sy);r['gpioe_odr_after']=t.read32(0x40021014)
  if a.action=='flash' or tag=='S':
   r['servo_state']=t.read32(sy['g_radar_servo_state'])
   r['servo_pulse_us']=list(struct.unpack('<2I',bytes(t.read_memory_block8(sy['g_radar_servo_pulse_us'],8))))
   r['servo_angles_deg']=list(struct.unpack('<2f',bytes(t.read_memory_block8(sy['g_radar_servo_angle_deg'],8))))
   r['TIM4']={n:t.read32(0x40000800+offset) for n,offset in [('CR1',0),('PSC',0x28),('ARR',0x2c),('CCR1',0x34),('CCR2',0x38),('CCER',0x20)]}
  (OUT/(a.action+'_result.json')).write_text(json.dumps(r,indent=2),encoding='utf8');print(json.dumps(r,indent=2))
  assert all(v==0 for v in r['after'].values()) and not(r['gpioe_odr_after']&4),'After-state fault'
  if a.action=='flash':assert r['servo_state']==1 and r['servo_pulse_us']==[500,500],'Initialization not as expected'
  if a.action=='step1':assert r['servo_pulse_us']==[611,500] and r['TIM4']['CCR1']==611 and r['TIM4']['CCR2']==500,'Channel verification failed'
  if a.action=='step2':assert r['servo_pulse_us']==[500,611] and r['TIM4']['CCR1']==500 and r['TIM4']['CCR2']==611,'Channel verification failed'
  if a.action=='park':assert r['servo_pulse_us']==[500,500] and r['TIM4']['CCR1']==500 and r['TIM4']['CCR2']==500,'Park verification failed'
if __name__=='__main__':main()
